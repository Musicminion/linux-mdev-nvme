// SPDX-License-Identifier: GPL-2.0+
/*
 * Live migration of the virtual controller: the VFIO migration protocol
 * (v2, stop-copy) and the dirty tracking of the guest memory we write
 * (VFIO DMA logging).
 *
 * The guest's queues, commands and data are in guest memory, which the
 * VMM migrates.  What is ours is the controller state: the registers, the
 * configuration and positions of the queues, the doorbell values that
 * landed in our own page, the interrupt and feature settings, and the
 * Asynchronous Event Requests that nvmet holds.
 *
 * To stop (RUNNING -> STOP), the controller stops fetching commands and
 * waits for those in flight: their completion entries and interrupts are
 * posted before the device reaches STOP.  Commands the guest submitted but
 * we didn't fetch stay in its submission queues, in guest memory, and the
 * destination fetches them.  In the saved state, the only commands in
 * flight are the Asynchronous Event Requests: the destination submits them
 * to its own nvmet controller again.  An admin command nvmet completes
 * while the device is stopped (an event) is held back and saved; the
 * destination posts it when it starts running.
 *
 * The namespaces are not migrated, the storage has to be shared: the
 * destination's nvmet subsystem must have the same namespaces (NSIDs,
 * sizes, block sizes, NGUIDs and UUIDs, which the guest uses to identify
 * them).  The state describes them and the destination refuses a state
 * that doesn't match its own.  The controller ID is not preserved: nvmet
 * allocates a new one.
 *
 * Dirty tracking: the device writes guest memory directly, so the VMM's
 * own dirty logging doesn't see those writes (and with an emulated IOMMU
 * there is no IOMMU dirty tracking either).  While logging, the guest
 * memory ranges a command transfers data into are recorded when its data
 * pointer is translated, and set in a bitmap when the command completes,
 * i.e. after the data landed.  The completion queues, the shadow doorbell
 * buffer and the event index buffer, which we write all the time, are
 * reported dirty whenever the bitmap is read.
 */
#include <linux/kernel.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/interval_tree.h>
#include <linux/iova_bitmap.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/vfio.h>
#include <linux/vmalloc.h>
#include "priv.h"
#include "trace.h"

#define NVME_MDEV_MIG_MAGIC	0x4d564d4e	/* "NMVM" */
#define NVME_MDEV_MIG_VERSION	1
/* namespaces of the subsystem described in the state (nvmet's maximum) */
#define NVME_MDEV_MIG_MAX_NS	NVMET_MAX_NAMESPACES
/* admin completions held back while stopped (events: one per AER) */
#define NVME_MDEV_MIG_MAX_CQES	NVMET_ASYNC_EVENTS
#define NVME_MDEV_MIG_NO_IRQ	0xffff
/* dirty bitmaps, at most (else we track in larger pages) */
#define NVME_MDEV_DIRTY_MAX_BYTES	(64ULL << 20)

struct nvme_mdev_mig_ns {
	__le32 nsid;
	__le32 blksize_shift;
	__le64 size;
	u8 nguid[16];
	u8 uuid[16];
};

struct nvme_mdev_mig_sq {
	__le64 iova;
	__le16 size;
	__le16 head;
	__le16 cqid;
	u8 cont;
	u8 rsvd;
	__le32 db;		/* tail in our doorbell page */
	__le32 eidx;		/* event index in our page */
};

struct nvme_mdev_mig_cq {
	__le64 iova;
	__le16 size;
	__le16 head;
	__le16 tail;
	__le16 pending;
	__le16 irq;		/* NVME_MDEV_MIG_NO_IRQ: interrupts disabled */
	u8 cont;
	u8 phase;
	__le32 db;		/* head in our doorbell page */
	__le32 eidx;
};

struct nvme_mdev_mig_cqe {
	__le64 result;
	__le16 cid;
	__le16 status;
	u8 opcode;
	u8 rsvd[3];
};

struct nvme_mdev_mig_state {
	/* header */
	__le32 magic;
	__le32 version;
	__le32 size;			/* of this structure */
	__le32 flags;

	/* what the destination must have, too */
	__le16 max_queues;
	__le16 max_irqs;
	u8 shadow_db_supported;
	u8 has_ctrl;			/* an nvmet controller: ns[] valid */
	u8 rsvd0[2];
	__le32 nr_ns;
	char subsysnqn[NVMF_NQN_SIZE];
	char serial[NVMET_SN_MAX_SIZE];
	struct nvme_mdev_mig_ns ns[NVME_MDEV_MIG_MAX_NS];

	/* registers */
	__le32 cc;
	__le32 csts;
	__le32 aqa;
	__le32 rsvd1;
	__le64 asq;
	__le64 acq;

	/* shadow doorbells */
	u8 shadow_db_en;
	u8 rsvd2[7];
	__le64 sdb_iova;
	__le64 eidx_iova;

	/* queues */
	__le32 sq_en;			/* bitmaps of qids */
	__le32 cq_en;
	struct nvme_mdev_mig_sq sq[MAX_VIRTUAL_QUEUES];
	struct nvme_mdev_mig_cq cq[MAX_VIRTUAL_QUEUES];

	/* interrupts and features */
	__le32 irq_coalesc_time_us;
	__le32 irq_coalesc_en;		/* bitmap of vectors */
	u8 irq_coalesc_max;
	u8 arb_burst_shift;
	u8 write_atomic_dn;
	u8 rsvd3;
	__le32 aen_enabled;

	/* admin commands: AERs nvmet holds, completions held back */
	__le32 nr_aers;
	__le32 nr_cqes;
	struct nvme_command aers[NVMET_ASYNC_EVENTS];
	struct nvme_mdev_mig_cqe cqes[NVME_MDEV_MIG_MAX_CQES];
};

/* the data transfer file of a saving or resuming session */
struct nvme_mdev_mig_file {
	struct file *filp;
	struct nvme_mdev_vctrl *vctrl;
	struct mutex lock;		/* the state and the fields below */
	bool disabled;
	size_t filled;			/* bytes of state there */
	struct nvme_mdev_mig_state state;
};

/* ------------------------------------------------------------------ */
/* dirty tracking */

struct nvme_mdev_dirty_range {
	unsigned long start;		/* IOVA, page aligned */
	unsigned long npages;
	unsigned long *bitmap;
};

struct nvme_mdev_dirty_log {
	struct rcu_head rcu;
	unsigned int page_shift;
	unsigned int nr;
	struct nvme_mdev_dirty_range ranges[] __counted_by(nr);
};

static void nvme_mdev_dirty_log_free(struct nvme_mdev_dirty_log *log)
{
	unsigned int i;

	if (!log)
		return;
	for (i = 0; i < log->nr; i++)
		kvfree(log->ranges[i].bitmap);
	kfree(log);
}

bool nvme_mdev_dirty_logging(struct nvme_mdev_vctrl *vctrl)
{
	return !!rcu_access_pointer(vctrl->dirty_log);
}

/* The device wrote [@iova, @iova + @len) of guest memory (any context) */
void nvme_mdev_dirty_mark(struct nvme_mdev_vctrl *vctrl, u64 iova, u64 len)
{
	struct nvme_mdev_dirty_log *log;
	unsigned long first, last, lo, hi, p;
	unsigned int i;

	if (!len || iova + len < iova)
		return;

	rcu_read_lock();
	log = rcu_dereference(vctrl->dirty_log);
	if (!log)
		goto out;

	first = iova >> log->page_shift;
	last = (iova + len - 1) >> log->page_shift;
	for (i = 0; i < log->nr; i++) {
		struct nvme_mdev_dirty_range *r = &log->ranges[i];
		unsigned long rfirst = r->start >> log->page_shift;

		lo = max(first, rfirst);
		hi = min(last, rfirst + r->npages - 1);
		for (p = lo; p <= hi && lo <= hi; p++)
			set_bit(p - rfirst, r->bitmap);
	}
out:
	rcu_read_unlock();
}

/* A command's data landed: if it went to the guest, that memory is dirty */
void nvme_mdev_dirty_mark_cmd(struct nvme_mdev_vctrl *vctrl,
			      const struct nvme_command *cmd,
			      const struct nvme_mdev_iova_ranges *ranges)
{
	unsigned int i;

	/* data transfer direction: opcode bit 1, controller to host */
	if (!ranges || !(cmd->common.opcode & 2))
		return;
	for (i = 0; i < ranges->nr; i++)
		nvme_mdev_dirty_mark(vctrl, ranges->r[i].iova,
				     ranges->r[i].len);
}

/* Record the guest memory range of a command's data (coalescing) */
void nvme_mdev_iova_ranges_add(struct nvme_mdev_iova_ranges *ranges,
			       u64 iova, u64 len)
{
	if (ranges->nr) {
		struct nvme_mdev_iova_range *prev = &ranges->r[ranges->nr - 1];

		if (prev->iova + prev->len == iova) {
			prev->len += len;
			return;
		}
	}
	if (WARN_ON_ONCE(ranges->nr == ranges->max))
		return;
	ranges->r[ranges->nr].iova = iova;
	ranges->r[ranges->nr].len = len;
	ranges->nr++;
}

struct nvme_mdev_iova_ranges *nvme_mdev_iova_ranges_alloc(unsigned int max)
{
	struct nvme_mdev_iova_ranges *ranges;

	ranges = kmalloc(struct_size(ranges, r, max), GFP_KERNEL);
	if (ranges) {
		ranges->nr = 0;
		ranges->max = max;
	}
	return ranges;
}

/* report the part of [@start, @start + @len) within the requested range */
static void nvme_mdev_dirty_report(struct iova_bitmap *dirty,
				   unsigned long iova, unsigned long length,
				   u64 start, u64 len)
{
	u64 s = max_t(u64, start, iova);
	u64 e = min_t(u64, start + len, (u64)iova + length);

	if (s < e)
		iova_bitmap_set(dirty, s, e - s);
}

/*
 * A completion queue, or the shadow doorbell buffers, go away (or are
 * mapped again): we may have written them since the last report, which
 * reports them from the queues there are then.  (Any context with the
 * lock or the I/O thread paused.)
 */
void nvme_mdev_dirty_mark_vcq(struct nvme_mdev_vctrl *vctrl,
			      struct nvme_vcq *q)
{
	u64 bytes = (u64)q->size * sizeof(struct nvme_completion);
	unsigned int i;

	if (!nvme_mdev_dirty_logging(vctrl))
		return;
	if (q->cont) {
		nvme_mdev_dirty_mark(vctrl, q->iova, bytes);
		return;
	}
	if (q->page_iovas)
		for (i = 0; i < DIV_ROUND_UP(bytes, PAGE_SIZE); i++)
			nvme_mdev_dirty_mark(vctrl, q->page_iovas[i], PAGE_SIZE);
}

void nvme_mdev_dirty_mark_shadow_dbs(struct nvme_mdev_vctrl *vctrl)
{
	if (!vctrl->mmio.shadow_db_en || !nvme_mdev_dirty_logging(vctrl))
		return;
	nvme_mdev_dirty_mark(vctrl, vctrl->mmio.sdb_map.iova, PAGE_SIZE);
	nvme_mdev_dirty_mark(vctrl, vctrl->mmio.seidx_map.iova, PAGE_SIZE);
}

/*
 * Guest memory we write all the time, without recording it: the
 * completion queues and the shadow doorbell buffers (the admin queue's
 * doorbells in the shadow doorbell buffer, the event indexes).
 */
static void nvme_mdev_dirty_report_fixed(struct nvme_mdev_vctrl *vctrl,
					 unsigned long iova,
					 unsigned long length,
					 struct iova_bitmap *dirty)
{
	unsigned int i, npages;
	u16 qid;

	mutex_lock(&vctrl->lock);
	if (!(vctrl->mmio.csts & NVME_CSTS_RDY))
		goto out;

	for_each_set_bit(qid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES) {
		struct nvme_vcq *q = &vctrl->vcqs[qid];
		u64 bytes = (u64)q->size * sizeof(struct nvme_completion);

		if (q->cont) {
			nvme_mdev_dirty_report(dirty, iova, length, q->iova,
					       bytes);
			continue;
		}
		if (!q->page_iovas)
			continue;
		npages = DIV_ROUND_UP(bytes, PAGE_SIZE);
		for (i = 0; i < npages; i++)
			nvme_mdev_dirty_report(dirty, iova, length,
					       q->page_iovas[i], PAGE_SIZE);
	}

	if (vctrl->mmio.shadow_db_en) {
		nvme_mdev_dirty_report(dirty, iova, length,
				       vctrl->mmio.sdb_map.iova, PAGE_SIZE);
		nvme_mdev_dirty_report(dirty, iova, length,
				       vctrl->mmio.seidx_map.iova, PAGE_SIZE);
	}
out:
	mutex_unlock(&vctrl->lock);
}

static int nvme_mdev_log_start(struct vfio_device *vdev,
			       struct rb_root_cached *ranges, u32 nnodes,
			       u64 *page_size)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);
	struct nvme_mdev_dirty_log *log;
	struct interval_tree_node *node;
	unsigned int shift, i = 0;
	int ret = 0;

	if (!*page_size || !is_power_of_2(*page_size))
		return -EINVAL;
	shift = max_t(unsigned int, ilog2(*page_size), PAGE_SHIFT);
	/*
	 * Coarser pages if the bitmaps would get big (we may choose: the
	 * page size is a hint), e.g. for the huge ranges of a vIOMMU
	 */
	for (;;) {
		u64 bits = 0;

		for (node = interval_tree_iter_first(ranges, 0, ULONG_MAX);
		     node; node = interval_tree_iter_next(node, 0, ULONG_MAX))
			bits += ((node->last - ALIGN_DOWN(node->start,
							  1UL << shift)) >>
				 shift) + 1 + BITS_PER_LONG;
		if (bits / 8 <= NVME_MDEV_DIRTY_MAX_BYTES ||
		    shift >= BITS_PER_LONG - 1)
			break;
		shift++;
	}

	mutex_lock(&vctrl->dirty_lock);
	if (rcu_access_pointer(vctrl->dirty_log)) {
		ret = -EBUSY;
		goto out;
	}

	log = kzalloc(struct_size(log, ranges, nnodes), GFP_KERNEL_ACCOUNT);
	if (!log) {
		ret = -ENOMEM;
		goto out;
	}
	log->nr = nnodes;
	log->page_shift = shift;

	/* (in IOVA order, and they don't overlap) */
	for (node = interval_tree_iter_first(ranges, 0, ULONG_MAX);
	     node && i < nnodes;
	     node = interval_tree_iter_next(node, 0, ULONG_MAX), i++) {
		struct nvme_mdev_dirty_range *r = &log->ranges[i];

		r->start = ALIGN_DOWN(node->start, 1UL << shift);
		r->npages = ((node->last - r->start) >> shift) + 1;
		r->bitmap = kvcalloc(BITS_TO_LONGS(r->npages),
				     sizeof(unsigned long),
				     GFP_KERNEL_ACCOUNT | __GFP_NOWARN);
		if (!r->bitmap) {
			log->nr = i;
			nvme_mdev_dirty_log_free(log);
			ret = -ENOMEM;
			goto out;
		}
	}
	log->nr = i;
	*page_size = 1ULL << shift;

	/*
	 * The commands in flight had their data ranges recorded only if we
	 * were logging already: let them finish first.  (Admin commands are
	 * all done under the lock, except the AERs, which have no data.)
	 */
	mutex_lock(&vctrl->lock);
	nvme_mdev_io_pause(vctrl);
	rcu_assign_pointer(vctrl->dirty_log, log);
	nvme_mdev_io_resume(vctrl);
	mutex_unlock(&vctrl->lock);
	_DBG(vctrl, "MIG: dirty tracking started, %u ranges, page size %llu\n",
	     log->nr, *page_size);
out:
	mutex_unlock(&vctrl->dirty_lock);
	return ret;
}

static void __nvme_mdev_log_stop(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_dirty_log *log;

	lockdep_assert_held(&vctrl->dirty_lock);

	log = rcu_replace_pointer(vctrl->dirty_log, NULL,
				  lockdep_is_held(&vctrl->dirty_lock));
	if (!log)
		return;
	synchronize_rcu();
	nvme_mdev_dirty_log_free(log);
	_DBG(vctrl, "MIG: dirty tracking stopped\n");
}

static int nvme_mdev_log_stop(struct vfio_device *vdev)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);

	mutex_lock(&vctrl->dirty_lock);
	__nvme_mdev_log_stop(vctrl);
	mutex_unlock(&vctrl->dirty_lock);
	return 0;
}

static int nvme_mdev_log_read_and_clear(struct vfio_device *vdev,
					unsigned long iova,
					unsigned long length,
					struct iova_bitmap *dirty)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);
	struct nvme_mdev_dirty_log *log;
	unsigned long first, last, lo, hi, p;
	unsigned int i, shift;
	int ret = 0;

	if (!length || iova + length - 1 < iova)
		return -EINVAL;

	mutex_lock(&vctrl->dirty_lock);
	log = rcu_dereference_protected(vctrl->dirty_log,
					lockdep_is_held(&vctrl->dirty_lock));
	if (!log) {
		ret = -EINVAL;
		goto out;
	}

	shift = log->page_shift;
	first = iova >> shift;
	last = (iova + length - 1) >> shift;
	for (i = 0; i < log->nr; i++) {
		struct nvme_mdev_dirty_range *r = &log->ranges[i];
		unsigned long rfirst = r->start >> shift;

		lo = max(first, rfirst);
		hi = min(last, rfirst + r->npages - 1);
		if (lo > hi)
			continue;
		/* bit indexes in this range's bitmap */
		lo -= rfirst;
		hi -= rfirst;
		for (p = find_next_bit(r->bitmap, hi + 1, lo); p <= hi;
		     p = find_next_bit(r->bitmap, hi + 1, p + 1)) {
			u64 start = r->start + ((u64)p << shift);
			u64 end = start + (1ULL << shift);

			/*
			 * Report the part in this window; clear the bit only
			 * if the whole page is in it, else the next window
			 * (or report) gets it too
			 */
			nvme_mdev_dirty_report(dirty, iova, length, start,
					       1ULL << shift);
			if (start >= iova && end - 1 <= iova + length - 1)
				clear_bit(p, r->bitmap);
		}
	}

	nvme_mdev_dirty_report_fixed(vctrl, iova, length, dirty);
out:
	mutex_unlock(&vctrl->dirty_lock);
	return ret;
}

static const struct vfio_log_ops nvme_mdev_log_ops = {
	.log_start		= nvme_mdev_log_start,
	.log_stop		= nvme_mdev_log_stop,
	.log_read_and_clear	= nvme_mdev_log_read_and_clear,
};

/* ------------------------------------------------------------------ */
/* stopping and running */

/* RUNNING -> STOP: no more commands, interrupts or guest memory writes */
static void nvme_mdev_mig_stop(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->lock);
	vctrl->mig_stopped = true;
	/*
	 * The I/O thread finishes the commands in flight and posts their
	 * completions and interrupts; then the admin commands nvmet
	 * completed (the others are done under the lock anyway).
	 */
	nvme_mdev_io_pause(vctrl);
	nvme_mdev_nvmet_reap_adm(vctrl);
	mutex_unlock(&vctrl->lock);
	_DBG(vctrl, "MIG: stopped\n");
}

/* STOP -> RUNNING, on the destination or after a failed migration */
static void nvme_mdev_mig_run(struct nvme_mdev_vctrl *vctrl)
{
	u16 qid;

	mutex_lock(&vctrl->lock);
	vctrl->mig_stopped = false;
	if ((vctrl->mmio.csts & NVME_CSTS_RDY) &&
	    !nvme_mdev_vctrl_is_dead(vctrl)) {
		/* completions held back while stopped */
		nvme_mdev_nvmet_reap_adm(vctrl);
		/*
		 * Interrupts for the completions the guest hasn't seen yet:
		 * they may have been lost with the source (a spurious one
		 * does no harm).
		 */
		for_each_set_bit(qid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
			nvme_mdev_vcq_flush(vctrl, qid);
		/* admin commands the guest submitted in the meantime */
		nvme_mdev_adm_process_sq(vctrl);
		/* the I/O thread looks at all the queues again */
		WRITE_ONCE(vctrl->io_idle, false);
	}
	nvme_mdev_io_resume(vctrl);
	mutex_unlock(&vctrl->lock);
	_DBG(vctrl, "MIG: running\n");
}

/* ------------------------------------------------------------------ */
/* saving */

static int nvme_mdev_mig_save_ns(struct nvme_mdev_vctrl *vctrl,
				 struct nvme_mdev_mig_state *st)
{
	struct nvmet_subsys *subsys = vctrl->tctrl->subsys;
	struct nvmet_ns *ns;
	unsigned long idx;
	unsigned int n = 0;
	int ret = 0;

	strscpy(st->subsysnqn, subsys->subsysnqn, sizeof(st->subsysnqn));
	memcpy(st->serial, subsys->serial, sizeof(st->serial));

	mutex_lock(&subsys->lock);
	nvmet_for_each_enabled_ns(&subsys->namespaces, idx, ns) {
		struct nvme_mdev_mig_ns *m;

		if (n == NVME_MDEV_MIG_MAX_NS) {
			ret = -E2BIG;
			break;
		}
		m = &st->ns[n++];
		m->nsid = cpu_to_le32(ns->nsid);
		m->blksize_shift = cpu_to_le32(ns->blksize_shift);
		m->size = cpu_to_le64(ns->size);
		memcpy(m->nguid, ns->nguid, sizeof(m->nguid));
		memcpy(m->uuid, &ns->uuid, sizeof(m->uuid));
	}
	mutex_unlock(&subsys->lock);
	st->nr_ns = cpu_to_le32(n);
	return ret;
}

static int nvme_mdev_mig_save(struct nvme_mdev_vctrl *vctrl,
			      struct nvme_mdev_mig_state *st)
{
	struct nvmet_ctrl *tctrl = vctrl->tctrl;
	struct doorbell *dbs = vctrl->mmio.db_page_kmap;
	struct doorbell *eidxs = vctrl->mmio.fake_eidx_kmap;
	struct nvme_mdev_iod *iod;
	u32 sq_en = 0, cq_en = 0, coalesc = 0;
	unsigned int i, naer = 0, ncqe = 0;
	int ret = 0;
	u16 qid;

	lockdep_assert_held(&vctrl->lock);

	st->magic = cpu_to_le32(NVME_MDEV_MIG_MAGIC);
	st->version = cpu_to_le32(NVME_MDEV_MIG_VERSION);
	st->size = cpu_to_le32(sizeof(*st));
	st->max_queues = cpu_to_le16(MAX_VIRTUAL_QUEUES);
	st->max_irqs = cpu_to_le16(MAX_VIRTUAL_IRQS);
	st->shadow_db_supported = vctrl->mmio.shadow_db_supported;
	strscpy(st->subsysnqn, vctrl->subsysnqn, sizeof(st->subsysnqn));
	if (tctrl) {
		st->has_ctrl = 1;
		ret = nvme_mdev_mig_save_ns(vctrl, st);
		if (ret)
			return ret;
		st->aen_enabled = cpu_to_le32(READ_ONCE(tctrl->aen_enabled));
		st->write_atomic_dn = tctrl->write_atomic_dn;
	}

	st->cc = cpu_to_le32(vctrl->mmio.cc);
	st->csts = cpu_to_le32(vctrl->mmio.csts);
	st->aqa = cpu_to_le32(vctrl->mmio.aqa);
	st->asq = cpu_to_le64(vctrl->mmio.asql |
			      ((u64)vctrl->mmio.asqh << 32));
	st->acq = cpu_to_le64(vctrl->mmio.acql |
			      ((u64)vctrl->mmio.acqh << 32));

	st->shadow_db_en = vctrl->mmio.shadow_db_en;
	if (vctrl->mmio.shadow_db_en) {
		st->sdb_iova = cpu_to_le64(vctrl->mmio.sdb_map.iova);
		st->eidx_iova = cpu_to_le64(vctrl->mmio.seidx_map.iova);
	}

	for_each_set_bit(qid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES) {
		struct nvme_vcq *q = &vctrl->vcqs[qid];
		struct nvme_mdev_mig_cq *m = &st->cq[qid];

		cq_en |= BIT(qid);
		m->iova = cpu_to_le64(q->iova);
		m->size = cpu_to_le16(q->size);
		m->head = cpu_to_le16(q->head);
		m->tail = cpu_to_le16(q->tail);
		m->pending = cpu_to_le16(q->pending);
		m->irq = cpu_to_le16(q->irq == -1 ? NVME_MDEV_MIG_NO_IRQ :
				     q->irq);
		m->cont = q->cont;
		m->phase = q->phase;
		/* (our own pages; the shadow ones are guest memory) */
		m->db = READ_ONCE(dbs[qid].cqh);
		m->eidx = READ_ONCE(eidxs[qid].cqh);
	}
	for_each_set_bit(qid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES) {
		struct nvme_vsq *q = &vctrl->vsqs[qid];
		struct nvme_mdev_mig_sq *m = &st->sq[qid];

		sq_en |= BIT(qid);
		m->iova = cpu_to_le64(q->iova);
		m->size = cpu_to_le16(q->size);
		m->head = cpu_to_le16(q->head);
		m->cqid = cpu_to_le16(q->vcq->qid);
		m->cont = q->cont;
		m->db = READ_ONCE(dbs[qid].sqt);
		m->eidx = READ_ONCE(eidxs[qid].sqt);
	}
	st->sq_en = cpu_to_le32(sq_en);
	st->cq_en = cpu_to_le32(cq_en);

	for (i = 0; i < MAX_VIRTUAL_IRQS; i++)
		if (vctrl->irqs.vecs[i].irq_coalesc_en)
			coalesc |= BIT(i);
	st->irq_coalesc_en = cpu_to_le32(coalesc);
	st->irq_coalesc_time_us = cpu_to_le32(vctrl->irqs.irq_coalesc_time_us);
	st->irq_coalesc_max = vctrl->irqs.irq_coalesc_max;
	st->arb_burst_shift = vctrl->arb_burst_shift;

	/* the admin commands nvmet has: AERs, and completions held back */
	list_for_each_entry(iod, &vctrl->adm_list, link) {
		/* acquire: its result, see nvme_mdev_nvmet_queue_response() */
		if (smp_load_acquire(&iod->completed)) {
			struct nvme_mdev_mig_cqe *c;

			if (ncqe == NVME_MDEV_MIG_MAX_CQES)
				return -EBUSY;
			c = &st->cqes[ncqe++];
			c->result = iod->cqe.result.u64;
			c->cid = cpu_to_le16(nvme_mdev_cmd_cid(&iod->cmd));
			c->status = cpu_to_le16(le16_to_cpu(iod->cqe.status) >> 1);
			c->opcode = iod->cmd.common.opcode;
			continue;
		}
		/* only AERs stay in flight, see nvme_mdev_mig_stop() */
		if (iod->cmd.common.opcode != nvme_admin_async_event ||
		    naer == NVMET_ASYNC_EVENTS) {
			_WARN(vctrl, "MIG: admin command 0x%x in flight\n",
			      iod->cmd.common.opcode);
			return -EBUSY;
		}
		st->aers[naer++] = iod->cmd;
	}
	st->nr_aers = cpu_to_le32(naer);
	st->nr_cqes = cpu_to_le32(ncqe);

	_DBG(vctrl, "MIG: saved CC 0x%x CSTS 0x%x, queues sq 0x%x cq 0x%x, %u AERs, %u held completions\n",
	     vctrl->mmio.cc, vctrl->mmio.csts, sq_en, cq_en, naer, ncqe);
	return 0;
}

/* ------------------------------------------------------------------ */
/* loading */

static int nvme_mdev_mig_check_ns(struct nvme_mdev_vctrl *vctrl,
				  const struct nvme_mdev_mig_state *st)
{
	struct nvme_mdev_mig_state *mine;
	int ret;

	if (strncmp(st->subsysnqn, vctrl->subsysnqn, sizeof(st->subsysnqn))) {
		_INFO(vctrl, "MIG: state is for subsystem %.*s, we have %s\n",
		      (int)sizeof(st->subsysnqn), st->subsysnqn,
		      vctrl->subsysnqn);
		return -EINVAL;
	}
	if (!st->has_ctrl)
		return 0;
	/* (nvmet may have deleted ours: we'd get a new one with the queues) */
	ret = nvme_mdev_nvmet_ctrl_create(vctrl);
	if (ret)
		return ret;

	mine = kvzalloc(sizeof(*mine), GFP_KERNEL);
	if (!mine)
		return -ENOMEM;
	ret = nvme_mdev_mig_save_ns(vctrl, mine);
	if (!ret && (memcmp(st->serial, mine->serial, sizeof(st->serial)) ||
		     st->nr_ns != mine->nr_ns ||
		     memcmp(st->ns, mine->ns,
			    le32_to_cpu(st->nr_ns) * sizeof(st->ns[0])))) {
		_INFO(vctrl, "MIG: the subsystem's serial number or namespaces differ (%u namespaces, here %u)\n",
		      le32_to_cpu(st->nr_ns), le32_to_cpu(mine->nr_ns));
		ret = -EINVAL;
	}
	kvfree(mine);
	return ret;
}

/* everything in the state is user input: check it all first */
static int nvme_mdev_mig_check(struct nvme_mdev_vctrl *vctrl,
			       const struct nvme_mdev_mig_state *st,
			       size_t len)
{
	u32 sq_en = le32_to_cpu(st->sq_en), cq_en = le32_to_cpu(st->cq_en);
	unsigned long qids;
	unsigned int i;
	u16 qid;

	if (len != sizeof(*st) ||
	    le32_to_cpu(st->magic) != NVME_MDEV_MIG_MAGIC ||
	    le32_to_cpu(st->version) != NVME_MDEV_MIG_VERSION ||
	    le32_to_cpu(st->size) != sizeof(*st) || st->flags ||
	    le16_to_cpu(st->max_queues) != MAX_VIRTUAL_QUEUES ||
	    le16_to_cpu(st->max_irqs) != MAX_VIRTUAL_IRQS) {
		_INFO(vctrl, "MIG: not a state of this version (%zu bytes)\n",
		      len);
		return -EINVAL;
	}
	if (st->shadow_db_supported != vctrl->mmio.shadow_db_supported) {
		_INFO(vctrl, "MIG: settings/shadow_doorbell differs\n");
		return -EINVAL;
	}
	if (le32_to_cpu(st->nr_ns) > NVME_MDEV_MIG_MAX_NS ||
	    le32_to_cpu(st->nr_aers) > NVMET_ASYNC_EVENTS ||
	    le32_to_cpu(st->nr_cqes) > NVME_MDEV_MIG_MAX_CQES ||
	    (sq_en | cq_en) >> MAX_VIRTUAL_QUEUES ||
	    le32_to_cpu(st->irq_coalesc_en) >> MAX_VIRTUAL_IRQS ||
	    st->arb_burst_shift > 7)
		goto bad;

	/*
	 * Queues: the admin queues, or none at all; and there are queues
	 * exactly when the controller is enabled and running (not shut down
	 * or failed): that's what the teardown paths go by.
	 */
	if (!(sq_en & 1) != !(cq_en & 1) || (!(sq_en & 1) && (sq_en | cq_en)))
		goto bad;
	if (!!(sq_en & 1) !=
	    ((le32_to_cpu(st->csts) &
	      (NVME_CSTS_RDY | NVME_CSTS_CFS | NVME_CSTS_SHST_MASK)) ==
	     NVME_CSTS_RDY && (le32_to_cpu(st->cc) & NVME_CC_ENABLE)))
		goto bad;
	if (le32_to_cpu(st->csts) & ~(NVME_CSTS_RDY | NVME_CSTS_CFS |
				      NVME_CSTS_SHST_MASK))
		goto bad;
	/* shadow doorbells: only with queues, and page aligned */
	if (st->shadow_db_en > 1 ||
	    (st->shadow_db_en && (!st->shadow_db_supported || !(sq_en & 1) ||
				  OFFSET_IN_PAGE(le64_to_cpu(st->sdb_iova)) ||
				  OFFSET_IN_PAGE(le64_to_cpu(st->eidx_iova)))))
		goto bad;
	if (st->has_ctrl > 1 || st->write_atomic_dn > 1)
		goto bad;
	qids = cq_en;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_cq *m = &st->cq[qid];
		u16 size = le16_to_cpu(m->size), irq = le16_to_cpu(m->irq);

		u16 head = le16_to_cpu(m->head), tail = le16_to_cpu(m->tail);

		if (size < 2 || head >= size || tail >= size ||
		    le16_to_cpu(m->pending) >= size || m->phase > 1 ||
		    m->cont > 1 ||
		    (irq != NVME_MDEV_MIG_NO_IRQ && irq >= MAX_VIRTUAL_IRQS))
			goto bad;
		/* entries posted and not consumed yet are pending, at least */
		if (le16_to_cpu(m->pending) < (tail + size - head) % size)
			goto bad;
	}
	qids = sq_en;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_sq *m = &st->sq[qid];
		u16 size = le16_to_cpu(m->size), cqid = le16_to_cpu(m->cqid);

		if (size < 2 || le16_to_cpu(m->head) >= size || m->cont > 1 ||
		    cqid >= MAX_VIRTUAL_QUEUES || !(cq_en & BIT(cqid)) ||
		    (qid == 0) != (cqid == 0))
			goto bad;
	}
	for (i = 0; i < le32_to_cpu(st->nr_aers); i++)
		if (st->aers[i].common.opcode != nvme_admin_async_event)
			goto bad;

	return nvme_mdev_mig_check_ns(vctrl, st);
bad:
	_INFO(vctrl, "MIG: inconsistent state\n");
	return -EINVAL;
}

/* rebuild the queues, as the guest had them */
static int nvme_mdev_mig_load_queues(struct nvme_mdev_vctrl *vctrl,
				     const struct nvme_mdev_mig_state *st)
{
	u32 sq_en = le32_to_cpu(st->sq_en), cq_en = le32_to_cpu(st->cq_en);
	unsigned long qids;
	u16 qid, status;
	int ret;

	/* the admin queues; nvmet checks CC */
	if (!nvme_mdev_vctrl_enable(vctrl, le64_to_cpu(st->acq),
				    le64_to_cpu(st->asq), vctrl->mmio.aqa))
		return -EINVAL;
	if (le16_to_cpu(st->cq[0].size) != vctrl->vcqs[0].size ||
	    le16_to_cpu(st->sq[0].size) != vctrl->vsqs[0].size)
		return -EINVAL;

	qids = cq_en & ~1UL;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_cq *m = &st->cq[qid];
		u16 irq = le16_to_cpu(m->irq);
		u16 flags = m->cont ? NVME_QUEUE_PHYS_CONTIG : 0;

		if (irq != NVME_MDEV_MIG_NO_IRQ)
			flags |= NVME_CQ_IRQ_ENABLED;
		status = nvme_mdev_nvmet_create_cq(vctrl, qid, flags,
						   le16_to_cpu(m->size) - 1,
						   le64_to_cpu(m->iova),
						   irq == NVME_MDEV_MIG_NO_IRQ ?
						   0 : irq);
		if (status)
			return -EINVAL;
	}
	qids = sq_en & ~1UL;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_sq *m = &st->sq[qid];

		status = nvme_mdev_nvmet_create_sq(vctrl, qid,
						   le16_to_cpu(m->cqid),
						   m->cont ?
						   NVME_QUEUE_PHYS_CONTIG : 0,
						   le16_to_cpu(m->size) - 1,
						   le64_to_cpu(m->iova));
		if (status)
			return -EINVAL;
	}

	/* where they are, and our doorbell pages (mmio.dbs is ours now) */
	qids = cq_en;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_cq *m = &st->cq[qid];
		struct nvme_vcq *q = &vctrl->vcqs[qid];

		q->head = le16_to_cpu(m->head);
		q->tail = le16_to_cpu(m->tail);
		q->pending = le16_to_cpu(m->pending);
		q->phase = m->phase;
		vctrl->mmio.dbs[qid].cqh = m->db;
		vctrl->mmio.eidxs[qid].cqh = m->eidx;
	}
	qids = sq_en;
	for_each_set_bit(qid, &qids, MAX_VIRTUAL_QUEUES) {
		const struct nvme_mdev_mig_sq *m = &st->sq[qid];

		vctrl->vsqs[qid].head = le16_to_cpu(m->head);
		vctrl->mmio.dbs[qid].sqt = m->db;
		vctrl->mmio.eidxs[qid].sqt = m->eidx;
	}

	/* the guest's shadow doorbells: as they are in its memory */
	if (st->shadow_db_en) {
		ret = nvme_mdev_mmio_enable_dbs_shadow(vctrl,
						       le64_to_cpu(st->sdb_iova),
						       le64_to_cpu(st->eidx_iova),
						       false);
		if (ret)
			return ret;
	}
	return 0;
}

static int nvme_mdev_mig_load(struct nvme_mdev_vctrl *vctrl,
			      const struct nvme_mdev_mig_state *st,
			      size_t len)
{
	u64 asq = le64_to_cpu(st->asq), acq = le64_to_cpu(st->acq);
	unsigned int i;
	int ret;

	lockdep_assert_held(&vctrl->lock);

	ret = nvme_mdev_mig_check(vctrl, st, len);
	if (ret)
		return ret;

	/* from a controller that was just reset */
	nvme_mdev_io_pause(vctrl);
	__nvme_mdev_vctrl_reset(vctrl, true);

	vctrl->mmio.cc = le32_to_cpu(st->cc);
	vctrl->mmio.aqa = le32_to_cpu(st->aqa);
	vctrl->mmio.asql = lower_32_bits(asq);
	vctrl->mmio.asqh = upper_32_bits(asq);
	vctrl->mmio.acql = lower_32_bits(acq);
	vctrl->mmio.acqh = upper_32_bits(acq);

	if (le32_to_cpu(st->sq_en) & 1) {
		ret = nvme_mdev_mig_load_queues(vctrl, st);
		if (ret)
			goto fail;
		if (vctrl->tctrl) {
			WRITE_ONCE(vctrl->tctrl->aen_enabled,
				   le32_to_cpu(st->aen_enabled));
			vctrl->tctrl->write_atomic_dn = !!st->write_atomic_dn;
		}
	}
	vctrl->mmio.csts = le32_to_cpu(st->csts);

	vctrl->arb_burst_shift = st->arb_burst_shift;
	vctrl->irqs.irq_coalesc_max = st->irq_coalesc_max;
	vctrl->irqs.irq_coalesc_time_us = le32_to_cpu(st->irq_coalesc_time_us);
	for (i = 0; i < MAX_VIRTUAL_IRQS; i++)
		vctrl->irqs.vecs[i].irq_coalesc_en =
			!!(le32_to_cpu(st->irq_coalesc_en) & BIT(i));

	/* the admin commands: completions to post, AERs for nvmet */
	if (le32_to_cpu(st->sq_en) & 1) {
		for (i = 0; i < le32_to_cpu(st->nr_cqes); i++) {
			const struct nvme_mdev_mig_cqe *c = &st->cqes[i];

			ret = nvme_mdev_nvmet_hold_adm_cqe(vctrl, c->opcode,
							   le16_to_cpu(c->cid),
							   le16_to_cpu(c->status),
							   c->result);
			if (ret)
				goto fail;
		}
		for (i = 0; i < le32_to_cpu(st->nr_aers); i++) {
			ret = nvme_mdev_nvmet_resubmit_adm(vctrl, &st->aers[i]);
			if (ret)
				goto fail;
		}
	}

	_DBG(vctrl, "MIG: loaded CC 0x%x CSTS 0x%x, queues sq 0x%x cq 0x%x, %u AERs, %u held completions\n",
	     vctrl->mmio.cc, vctrl->mmio.csts, le32_to_cpu(st->sq_en),
	     le32_to_cpu(st->cq_en), le32_to_cpu(st->nr_aers),
	     le32_to_cpu(st->nr_cqes));
	return 0;
fail:
	_INFO(vctrl, "MIG: can't load the state (%d)\n", ret);
	/* whatever got created */
	nvme_mdev_vctrl_disable(vctrl);
	nvme_mdev_mmio_reset(vctrl, true);
	return ret ? ret : -EINVAL;
}

/* ------------------------------------------------------------------ */
/* the data transfer files */

static void nvme_mdev_mig_file_disable(struct nvme_mdev_mig_file *migf)
{
	mutex_lock(&migf->lock);
	migf->disabled = true;
	migf->filled = 0;
	migf->filp->f_pos = 0;
	mutex_unlock(&migf->lock);
}

static void nvme_mdev_mig_disable_files(struct nvme_mdev_vctrl *vctrl)
{
	if (vctrl->saving_migf) {
		nvme_mdev_mig_file_disable(vctrl->saving_migf);
		fput(vctrl->saving_migf->filp);
		vctrl->saving_migf = NULL;
	}
	if (vctrl->resuming_migf) {
		nvme_mdev_mig_file_disable(vctrl->resuming_migf);
		fput(vctrl->resuming_migf->filp);
		vctrl->resuming_migf = NULL;
	}
}

static int nvme_mdev_mig_release(struct inode *inode, struct file *filp)
{
	struct nvme_mdev_mig_file *migf = filp->private_data;

	mutex_destroy(&migf->lock);
	kvfree(migf);
	return 0;
}

static ssize_t nvme_mdev_mig_read(struct file *filp, char __user *buf,
				  size_t len, loff_t *pos)
{
	struct nvme_mdev_mig_file *migf = filp->private_data;
	ssize_t ret;

	if (pos)
		return -ESPIPE;
	pos = &filp->f_pos;

	mutex_lock(&migf->lock);
	if (migf->disabled) {
		ret = -ENODEV;
		goto out;
	}
	if (*pos < 0 || *pos > migf->filled) {
		ret = -EINVAL;
		goto out;
	}
	len = min_t(size_t, migf->filled - *pos, len);
	if (copy_to_user(buf, (u8 *)&migf->state + *pos, len)) {
		ret = -EFAULT;
		goto out;
	}
	*pos += len;
	ret = len;
out:
	mutex_unlock(&migf->lock);
	return ret;
}

static ssize_t nvme_mdev_mig_write(struct file *filp, const char __user *buf,
				   size_t len, loff_t *pos)
{
	struct nvme_mdev_mig_file *migf = filp->private_data;
	loff_t end;
	ssize_t ret;

	if (pos)
		return -ESPIPE;
	pos = &filp->f_pos;

	/* (f_pos too: writers aren't serialized on a stream) */
	mutex_lock(&migf->lock);
	if (migf->disabled) {
		ret = -ENODEV;
		goto out;
	}
	if (*pos < 0 || check_add_overflow((loff_t)len, *pos, &end)) {
		ret = -EINVAL;
		goto out;
	}
	if (end > sizeof(migf->state)) {
		ret = -ENOMEM;
		goto out;
	}
	if (copy_from_user((u8 *)&migf->state + *pos, buf, len)) {
		ret = -EFAULT;
		goto out;
	}
	*pos += len;
	migf->filled = max_t(size_t, migf->filled, *pos);
	ret = len;

	/*
	 * All there: check it now, so that a state we can't take fails the
	 * VMM's loading of the migration stream, while the source can still
	 * go on (we load it only when leaving RESUMING, when the destination
	 * VM starts).
	 */
	if (migf->filled == sizeof(migf->state)) {
		struct nvme_mdev_vctrl *vctrl = migf->vctrl;
		int err;

		mutex_lock(&vctrl->lock);
		err = nvme_mdev_mig_check(vctrl, &migf->state, migf->filled);
		mutex_unlock(&vctrl->lock);
		if (err)
			ret = err;
	}
out:
	mutex_unlock(&migf->lock);
	return ret;
}

static const struct file_operations nvme_mdev_mig_save_fops = {
	.owner		= THIS_MODULE,
	.read		= nvme_mdev_mig_read,
	.release	= nvme_mdev_mig_release,
};

static const struct file_operations nvme_mdev_mig_resume_fops = {
	.owner		= THIS_MODULE,
	.write		= nvme_mdev_mig_write,
	.release	= nvme_mdev_mig_release,
};

static struct nvme_mdev_mig_file *
nvme_mdev_mig_file_alloc(struct nvme_mdev_vctrl *vctrl,
			 const struct file_operations *fops, int flags)
{
	struct nvme_mdev_mig_file *migf;

	migf = kvzalloc(sizeof(*migf), GFP_KERNEL_ACCOUNT);
	if (!migf)
		return ERR_PTR(-ENOMEM);

	migf->filp = anon_inode_getfile("nvme_mdev_mig", fops, migf, flags);
	if (IS_ERR(migf->filp)) {
		int ret = PTR_ERR(migf->filp);

		kvfree(migf);
		return ERR_PTR(ret);
	}
	stream_open(migf->filp->f_inode, migf->filp);
	mutex_init(&migf->lock);
	migf->vctrl = vctrl;
	return migf;
}

/* STOP -> STOP_COPY: the state, read from the returned file */
static struct file *nvme_mdev_mig_save_file(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_mig_file *migf;
	int ret;

	migf = nvme_mdev_mig_file_alloc(vctrl, &nvme_mdev_mig_save_fops,
					O_RDONLY);
	if (IS_ERR(migf))
		return ERR_CAST(migf);

	mutex_lock(&vctrl->lock);
	ret = nvme_mdev_mig_save(vctrl, &migf->state);
	mutex_unlock(&vctrl->lock);
	if (ret) {
		fput(migf->filp);
		return ERR_PTR(ret);
	}
	migf->filled = sizeof(migf->state);

	get_file(migf->filp);
	vctrl->saving_migf = migf;
	return migf->filp;
}

/* STOP -> RESUMING: the state is written to the returned file */
static struct file *nvme_mdev_mig_resume_file(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_mig_file *migf;

	migf = nvme_mdev_mig_file_alloc(vctrl, &nvme_mdev_mig_resume_fops,
					O_WRONLY);
	if (IS_ERR(migf))
		return ERR_CAST(migf);

	get_file(migf->filp);
	vctrl->resuming_migf = migf;
	return migf->filp;
}

/* RESUMING -> STOP: take over the state written */
static int nvme_mdev_mig_resume(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_mig_file *migf = vctrl->resuming_migf;
	int ret;

	if (WARN_ON(!migf))
		return -EINVAL;
	mutex_lock(&migf->lock);
	mutex_lock(&vctrl->lock);
	ret = nvme_mdev_mig_load(vctrl, &migf->state, migf->filled);
	mutex_unlock(&vctrl->lock);
	mutex_unlock(&migf->lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/* the state machine */

static struct file *nvme_mdev_mig_step(struct nvme_mdev_vctrl *vctrl,
				       enum vfio_device_mig_state new)
{
	enum vfio_device_mig_state cur = vctrl->mig_state;
	int ret;

	_DBG(vctrl, "MIG: state %d -> %d\n", cur, new);

	if (cur == VFIO_DEVICE_STATE_RUNNING &&
	    new == VFIO_DEVICE_STATE_STOP) {
		nvme_mdev_mig_stop(vctrl);
		return NULL;
	}
	if (cur == VFIO_DEVICE_STATE_STOP &&
	    new == VFIO_DEVICE_STATE_RUNNING) {
		nvme_mdev_mig_run(vctrl);
		return NULL;
	}
	if (cur == VFIO_DEVICE_STATE_STOP &&
	    new == VFIO_DEVICE_STATE_STOP_COPY)
		return nvme_mdev_mig_save_file(vctrl);
	if (cur == VFIO_DEVICE_STATE_STOP_COPY &&
	    new == VFIO_DEVICE_STATE_STOP) {
		nvme_mdev_mig_disable_files(vctrl);
		return NULL;
	}
	if (cur == VFIO_DEVICE_STATE_STOP &&
	    new == VFIO_DEVICE_STATE_RESUMING)
		return nvme_mdev_mig_resume_file(vctrl);
	if (cur == VFIO_DEVICE_STATE_RESUMING &&
	    new == VFIO_DEVICE_STATE_STOP) {
		/* (failing, we stay in RESUMING with the file: reset gets out) */
		ret = nvme_mdev_mig_resume(vctrl);
		if (ret)
			return ERR_PTR(ret);
		nvme_mdev_mig_disable_files(vctrl);
		return NULL;
	}

	/* vfio_mig_get_next_state() doesn't give us anything else */
	WARN_ON(true);
	return ERR_PTR(-EINVAL);
}

static struct file *
nvme_mdev_mig_set_state(struct vfio_device *vdev,
			enum vfio_device_mig_state new_state)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);
	enum vfio_device_mig_state next;
	struct file *res = NULL;
	int ret;

	mutex_lock(&vctrl->mig_lock);
	while (new_state != vctrl->mig_state) {
		ret = vfio_mig_get_next_state(vdev, vctrl->mig_state,
					      new_state, &next);
		if (ret) {
			res = ERR_PTR(ret);
			break;
		}
		res = nvme_mdev_mig_step(vctrl, next);
		trace_nvme_mdev_mig_state(vctrl, vctrl->mig_state, next,
					  IS_ERR(res) ? PTR_ERR(res) : 0);
		if (IS_ERR(res))
			break;
		vctrl->mig_state = next;
		if (WARN_ON(res && new_state != vctrl->mig_state)) {
			fput(res);
			res = ERR_PTR(-EINVAL);
			break;
		}
	}
	mutex_unlock(&vctrl->mig_lock);
	return res;
}

static int nvme_mdev_mig_get_state(struct vfio_device *vdev,
				   enum vfio_device_mig_state *state)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);

	mutex_lock(&vctrl->mig_lock);
	*state = vctrl->mig_state;
	mutex_unlock(&vctrl->mig_lock);
	return 0;
}

static int nvme_mdev_mig_get_data_size(struct vfio_device *vdev,
				       unsigned long *stop_copy_length)
{
	*stop_copy_length = sizeof(struct nvme_mdev_mig_state);
	return 0;
}

static const struct vfio_migration_ops nvme_mdev_mig_ops = {
	.migration_set_state	= nvme_mdev_mig_set_state,
	.migration_get_state	= nvme_mdev_mig_get_state,
	.migration_get_data_size = nvme_mdev_mig_get_data_size,
};

/* ------------------------------------------------------------------ */

void nvme_mdev_mig_init(struct nvme_mdev_vctrl *vctrl)
{
	mutex_init(&vctrl->mig_lock);
	mutex_init(&vctrl->dirty_lock);
	vctrl->mig_state = VFIO_DEVICE_STATE_RUNNING;
	vctrl->vdev.migration_flags = VFIO_MIGRATION_STOP_COPY;
	vctrl->vdev.mig_ops = &nvme_mdev_mig_ops;
	vctrl->vdev.log_ops = &nvme_mdev_log_ops;
}

/*
 * VFIO_DEVICE_RESET: back to RUNNING, the only way out of RESUMING or
 * ERROR; the controller itself is reset by the caller
 */
void nvme_mdev_mig_reset(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->mig_lock);
	nvme_mdev_mig_disable_files(vctrl);
	if (vctrl->mig_state != VFIO_DEVICE_STATE_RUNNING)
		_DBG(vctrl, "MIG: reset in state %d\n", vctrl->mig_state);
	vctrl->mig_state = VFIO_DEVICE_STATE_RUNNING;
	mutex_lock(&vctrl->lock);
	vctrl->mig_stopped = false;
	nvme_mdev_io_resume(vctrl);
	mutex_unlock(&vctrl->lock);
	mutex_unlock(&vctrl->mig_lock);
}

/* the device is closed: back to RUNNING for the next user */
void nvme_mdev_mig_close(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->dirty_lock);
	__nvme_mdev_log_stop(vctrl);
	mutex_unlock(&vctrl->dirty_lock);

	mutex_lock(&vctrl->mig_lock);
	nvme_mdev_mig_disable_files(vctrl);
	vctrl->mig_state = VFIO_DEVICE_STATE_RUNNING;
	mutex_unlock(&vctrl->mig_lock);
}

void nvme_mdev_mig_cleanup(struct nvme_mdev_vctrl *vctrl)
{
	mutex_destroy(&vctrl->mig_lock);
	mutex_destroy(&vctrl->dirty_lock);
}
