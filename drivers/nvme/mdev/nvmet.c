// SPDX-License-Identifier: GPL-2.0+
/*
 * nvme-mdev as an NVMe target (nvmet) transport
 *
 * The virtual controller's registers, doorbells, queues and interrupts
 * are emulated by this driver (mmio.c, vsq.c, vcq.c, irq.c); the commands
 * are executed by nvmet, like for the nvmet PCI endpoint target
 * (pci-epf.c).  The admin command set, the namespaces and their backends
 * (block devices, files) are nvmet's, configured through its configfs
 * interface: an instance is tied to an nvmet port of type "pci" and to a
 * subsystem by its nvmet/portid and nvmet/subsysnqn attributes, and gets
 * an nvmet controller of its own when the device is opened.
 *
 * Unlike for pci-epf, the guest's memory is ours to access: command data
 * is not bounced, the scatterlist nvmet gets points to the guest's pages.
 *
 * Contexts: admin commands are fetched and executed with vctrl->lock held
 * and the I/O thread paused (nvme_mdev_adm_process_sq()), which is also
 * where nvmet calls our queue and feature operations from.  I/O commands
 * are fetched and started by the I/O thread.  nvmet completes commands
 * from any context, e.g. a bio's end_io: ->queue_response() only puts the
 * command on a lock-less list; the completion entry is written by the I/O
 * thread (I/O queues) or under vctrl->lock (the admin queue).
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include "priv.h"
#include "trace.h"

static const struct nvmet_fabrics_ops nvme_mdev_nvmet_ops;
static struct kmem_cache *nvme_mdev_iod_cache;

/* nvmet ports of our type ("pci") that are enabled */
static LIST_HEAD(nvme_mdev_ports);
static DEFINE_MUTEX(nvme_mdev_ports_mutex);

/* ------------------------------------------------------------------ */
/* commands */

static struct nvme_mdev_iod *nvme_mdev_iod_alloc(struct nvme_mdev_vctrl *vctrl,
						 u16 sqid)
{
	struct nvme_mdev_iod *iod;

	iod = kmem_cache_zalloc(nvme_mdev_iod_cache, GFP_KERNEL);
	if (!iod)
		return NULL;

	iod->req.cmd = &iod->cmd;
	iod->req.cqe = &iod->cqe;
	iod->req.port = vctrl->tctrl->port;
	iod->vctrl = vctrl;
	iod->sqid = sqid;
	return iod;
}

static void nvme_mdev_iod_free(struct nvme_mdev_iod *iod)
{
	nvme_mdev_udata_unmap_sg(&iod->sgt, iod->inline_sg);
	kfree(iod->dirty);
	kmem_cache_free(nvme_mdev_iod_cache, iod);
}

/* nvmet is done with a command (any context) */
static void nvme_mdev_nvmet_queue_response(struct nvmet_req *req)
{
	struct nvme_mdev_iod *iod = container_of(req, struct nvme_mdev_iod, req);
	struct nvme_mdev_vctrl *vctrl = iod->vctrl;

	/* release: the result is in, for nvme_mdev_mig_save() */
	smp_store_release(&iod->completed, true);
	if (iod->sqid) {
		/* the I/O thread polls while it has commands in flight */
		llist_add(&iod->done, &vctrl->io_done);
		return;
	}

	/*
	 * Admin commands normally complete right away and the submitter
	 * writes their completion entry; the others (e.g. Asynchronous
	 * Event Requests) complete later, from nvmet's work items.
	 */
	if (llist_add(&iod->done, &vctrl->adm_done))
		queue_work(system_wq, &vctrl->adm_done_work);
}

/* Complete a command that nvmet didn't execute */
static void nvme_mdev_iod_fail(struct nvme_mdev_iod *iod, u16 status)
{
	iod->cqe.status = cpu_to_le16(status << 1);
	iod->cqe.result.u64 = 0;
	nvme_mdev_nvmet_queue_response(&iod->req);
}

/* Hand a command (copied out of the guest's queue) to nvmet */
static void nvme_mdev_iod_start(struct nvme_mdev_iod *iod)
{
	struct nvme_mdev_vctrl *vctrl = iod->vctrl;
	struct nvmet_req *req = &iod->req;
	u16 status;
	u32 len;

	/* on failure nvmet already completed it (->queue_response) */
	if (!nvmet_req_init(req, &vctrl->tsqs[iod->sqid], &nvme_mdev_nvmet_ops))
		return;

	len = nvmet_req_transfer_len(req);
	if (len) {
		if (len > NVME_MDEV_MAX_XFER)
			status = NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		else
			status = nvme_mdev_udata_map_sg(&vctrl->viommu,
							&iod->cmd, len,
							&iod->sgt,
							iod->inline_sg,
							ARRAY_SIZE(iod->inline_sg),
							nvme_mdev_dirty_logging(vctrl) ?
							&iod->dirty : NULL);
		if (status) {
			nvmet_req_uninit(req);
			nvme_mdev_iod_fail(iod, status);
			return;
		}
		req->sg = iod->sgt.sgl;
		req->sg_cnt = iod->sgt.nents;
		req->transfer_len = len;
	}

	req->execute(req);
}

static inline u16 nvme_mdev_iod_status(struct nvme_mdev_iod *iod)
{
	return le16_to_cpu(iod->cqe.status) >> 1;
}

/* Write the completion entries of the finished admin commands */
void nvme_mdev_nvmet_reap_adm(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_iod *iod, *tmp;
	struct llist_node *list;

	lockdep_assert_held(&vctrl->lock);

	list = llist_reverse_order(llist_del_all(&vctrl->adm_done));
	llist_for_each_entry_safe(iod, tmp, list, done) {
		u16 status = nvme_mdev_iod_status(iod);
		u16 cid = nvme_mdev_cmd_cid(&iod->cmd);

		if (status && iod->cmd.common.opcode != nvme_admin_async_event)
			_DBG(vctrl, "ADMINQ: CID 0x%x opcode 0x%x: status 0x%x\n",
			     cid, iod->cmd.common.opcode, status);

		/* (not if the queue went away meanwhile) */
		if (test_bit(0, vctrl->vsq_en)) {
			nvme_mdev_dirty_mark_cmd(vctrl, &iod->cmd, iod->dirty);
			nvme_mdev_vsq_cmd_done_adm(vctrl, iod->cqe.result.u64,
						   cid, status);
		}
		list_del(&iod->link);
		nvme_mdev_iod_free(iod);
	}
}

static void nvme_mdev_nvmet_adm_done_work(struct work_struct *work)
{
	struct nvme_mdev_vctrl *vctrl =
		container_of(work, struct nvme_mdev_vctrl, adm_done_work);

	mutex_lock(&vctrl->lock);
	/* stopped for migration: posted when it runs again, or saved */
	if (!vctrl->mig_stopped)
		nvme_mdev_nvmet_reap_adm(vctrl);
	mutex_unlock(&vctrl->lock);
}

/* Process all pending admin commands */
void nvme_mdev_adm_process_sq(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_iod *iod;

	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->tctrl)
		return;

	nvme_mdev_io_pause(vctrl);

	while (!nvme_mdev_vctrl_is_dead(vctrl) && vctrl->tctrl) {
		if (!nvme_mdev_vsq_has_data(vctrl, &vctrl->vsqs[0]))
			break;

		iod = nvme_mdev_iod_alloc(vctrl, 0);
		if (!iod)
			break;

		if (!nvme_mdev_vsq_get_cmd(vctrl, &vctrl->vsqs[0], &iod->cmd)) {
			nvme_mdev_iod_free(iod);
			break;
		}

		_DBG(vctrl, "ADMINQ: opcode 0x%x\n", iod->cmd.common.opcode);
		list_add_tail(&iod->link, &vctrl->adm_list);
		nvme_mdev_iod_start(iod);
		nvme_mdev_nvmet_reap_adm(vctrl);
	}

	nvme_mdev_io_resume(vctrl);
}

/*
 * Migration, on the destination: give nvmet an admin command again that
 * the source's nvmet had in flight (an Asynchronous Event Request).
 */
int nvme_mdev_nvmet_resubmit_adm(struct nvme_mdev_vctrl *vctrl,
				 const struct nvme_command *cmd)
{
	struct nvme_mdev_iod *iod;

	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->tctrl)
		return -ENODEV;
	iod = nvme_mdev_iod_alloc(vctrl, 0);
	if (!iod)
		return -ENOMEM;
	iod->cmd = *cmd;
	list_add_tail(&iod->link, &vctrl->adm_list);
	nvme_mdev_iod_start(iod);
	return 0;
}

/*
 * Migration, on the destination: an admin command the source's nvmet
 * completed while stopped; its completion entry is written when we run.
 */
int nvme_mdev_nvmet_hold_adm_cqe(struct nvme_mdev_vctrl *vctrl, u8 opcode,
				 u16 cid, u16 status, __le64 result)
{
	struct nvme_mdev_iod *iod;

	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->tctrl)
		return -ENODEV;
	iod = nvme_mdev_iod_alloc(vctrl, 0);
	if (!iod)
		return -ENOMEM;
	iod->cmd.common.opcode = opcode;
	iod->cmd.common.command_id = (__force u16)cpu_to_le16(cid);
	iod->cqe.status = cpu_to_le16(status << 1);
	iod->cqe.result.u64 = result;
	iod->completed = true;
	list_add_tail(&iod->link, &vctrl->adm_list);
	llist_add(&iod->done, &vctrl->adm_done);
	return 0;
}

/*
 * I/O thread: fetch one command from a guest submission queue and start
 * it.  Returns false if there was nothing to do (or no memory).
 */
bool nvme_mdev_nvmet_submit_io(struct nvme_mdev_vctrl *vctrl, u16 sqid)
{
	struct nvme_vsq *vsq = &vctrl->vsqs[sqid];
	struct nvme_mdev_iod *iod;

	if (!nvme_mdev_vsq_has_data(vctrl, vsq))
		return false;

	iod = nvme_mdev_iod_alloc(vctrl, sqid);
	if (!iod)
		return false;

	if (!nvme_mdev_vsq_get_cmd(vctrl, vsq, &iod->cmd)) {
		nvme_mdev_iod_free(iod);
		return false;
	}

	atomic_inc(&vctrl->io_inflight);
	list_add_tail(&iod->link, &vctrl->io_list);
	trace_nvme_mdev_io_submit(vctrl, sqid, &iod->cmd);
	nvme_mdev_iod_start(iod);
	return true;
}

/*
 * I/O thread (or with the I/O thread paused): write the completion
 * entries of the finished I/O commands.  Returns how many there were.
 */
unsigned int nvme_mdev_nvmet_reap_io(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_iod *iod, *tmp;
	struct llist_node *list;
	unsigned int n = 0;

	list = llist_del_all(&vctrl->io_done);
	if (!list)
		return 0;

	list = llist_reverse_order(list);
	llist_for_each_entry_safe(iod, tmp, list, done) {
		u16 status = nvme_mdev_iod_status(iod);
		u16 cid = nvme_mdev_cmd_cid(&iod->cmd);

		if (status)
			_DBG(vctrl, "IOQ: QID %d CID %d opcode 0x%x: status 0x%x\n",
			     iod->sqid, cid, iod->cmd.common.opcode, status);

		trace_nvme_mdev_io_complete(vctrl, iod->sqid, cid, status);
		/* the data landed: dirty, before the guest learns about it */
		nvme_mdev_dirty_mark_cmd(vctrl, &iod->cmd, iod->dirty);
		if (test_bit(iod->sqid, vctrl->vsq_en))
			nvme_mdev_vsq_cmd_done_io(vctrl, iod->sqid, cid, status,
						  iod->cqe.result.u64);
		list_del(&iod->link);
		nvme_mdev_iod_free(iod);
		atomic_dec(&vctrl->io_inflight);
		n++;
	}
	return n;
}

/*
 * I/O thread: poll for the completion of the I/O commands in flight that
 * nvmet submitted as polled bios (NVMF_POLLED_IO), which completes them
 * (->queue_response).  One poll reaps everything done on the host queue it
 * looks at; our bios go to the poll queue of the CPU we run on, so the
 * newest command covers almost all of them, the oldest the ones submitted
 * before the thread moved to another CPU.
 */
void nvme_mdev_nvmet_poll_io(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_iod *first, *last;

	if (list_empty(&vctrl->io_list))
		return;

	last = list_last_entry(&vctrl->io_list, struct nvme_mdev_iod, link);
	nvmet_req_poll(&last->req, BLK_POLL_ONESHOT);

	first = list_first_entry(&vctrl->io_list, struct nvme_mdev_iod, link);
	if (first != last)
		nvmet_req_poll(&first->req, BLK_POLL_ONESHOT);
}

/* ------------------------------------------------------------------ */
/* queues (also nvmet's ->create_sq() etc.: lock held, I/O paused) */

u16 nvme_mdev_nvmet_create_cq(struct nvme_mdev_vctrl *vctrl, u16 cqid,
			      u16 flags, u16 qsize, u64 iova, u16 vector)
{
	int irq = -1, ret;
	u16 status;

	lockdep_assert_held(&vctrl->lock);
	nvme_mdev_assert_io_not_running(vctrl);

	if (cqid >= MAX_VIRTUAL_QUEUES || test_bit(cqid, vctrl->vcq_en))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	if (flags & ~(NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED))
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;

	if (flags & NVME_CQ_IRQ_ENABLED) {
		if (vector >= MAX_VIRTUAL_IRQS)
			return NVME_SC_INVALID_VECTOR | NVME_STATUS_DNR;
		irq = vector;
	}

	status = nvmet_cq_create(vctrl->tctrl, &vctrl->tcqs[cqid], cqid,
				 qsize + 1);
	if (status)
		return status;

	ret = nvme_mdev_vcq_init(vctrl, cqid, iova,
				 flags & NVME_QUEUE_PHYS_CONTIG, qsize + 1, irq);
	if (ret) {
		nvmet_cq_put(&vctrl->tcqs[cqid]);
		return nvme_mdev_translate_error(ret);
	}
	return NVME_SC_SUCCESS;
}

void nvme_mdev_nvmet_delete_cq(struct nvme_mdev_vctrl *vctrl, u16 cqid)
{
	lockdep_assert_held(&vctrl->lock);

	if (!test_bit(cqid, vctrl->vcq_en))
		return;
	nvme_mdev_vcq_delete(vctrl, cqid);
	nvmet_cq_put(&vctrl->tcqs[cqid]);
}

u16 nvme_mdev_nvmet_create_sq(struct nvme_mdev_vctrl *vctrl, u16 sqid,
			      u16 cqid, u16 flags, u16 qsize, u64 iova)
{
	u16 status;
	int ret;

	lockdep_assert_held(&vctrl->lock);
	nvme_mdev_assert_io_not_running(vctrl);

	if (sqid >= MAX_VIRTUAL_QUEUES || test_bit(sqid, vctrl->vsq_en))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;

	if (cqid >= MAX_VIRTUAL_QUEUES || !test_bit(cqid, vctrl->vcq_en))
		return NVME_SC_CQ_INVALID | NVME_STATUS_DNR;

	if (flags & ~(NVME_QUEUE_PHYS_CONTIG | NVME_SQ_PRIO_MASK))
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;

	status = nvmet_sq_create(vctrl->tctrl, &vctrl->tsqs[sqid],
				 &vctrl->tcqs[cqid], sqid, qsize + 1);
	if (status)
		return status;

	ret = nvme_mdev_vsq_init(vctrl, sqid, iova,
				 flags & NVME_QUEUE_PHYS_CONTIG, qsize + 1, cqid);
	if (ret) {
		nvmet_sq_destroy(&vctrl->tsqs[sqid]);
		return nvme_mdev_translate_error(ret);
	}
	return NVME_SC_SUCCESS;
}

/*
 * Delete a submission queue.  nvmet_sq_destroy() waits for the commands
 * nvmet has in flight on it: the I/O thread is paused, which waited for
 * the I/O commands; on the admin queue, it fails the Asynchronous Event
 * Requests.
 */
void nvme_mdev_nvmet_delete_sq(struct nvme_mdev_vctrl *vctrl, u16 sqid)
{
	lockdep_assert_held(&vctrl->lock);

	if (!test_bit(sqid, vctrl->vsq_en))
		return;
	nvme_mdev_vsq_delete(vctrl, sqid);
	nvmet_sq_destroy(&vctrl->tsqs[sqid]);
}

static u16 nvme_mdev_nvmet_op_create_cq(struct nvmet_ctrl *tctrl, u16 cqid,
					u16 flags, u16 qsize, u64 prp1,
					u16 vector)
{
	return nvme_mdev_nvmet_create_cq(tctrl->drvdata, cqid, flags, qsize,
					 prp1, vector);
}

static u16 nvme_mdev_nvmet_op_delete_cq(struct nvmet_ctrl *tctrl, u16 cqid)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;

	if (cqid >= MAX_VIRTUAL_QUEUES || !test_bit(cqid, vctrl->vcq_en))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	nvme_mdev_nvmet_delete_cq(vctrl, cqid);
	return NVME_SC_SUCCESS;
}

static u16 nvme_mdev_nvmet_op_create_sq(struct nvmet_ctrl *tctrl, u16 sqid,
					u16 cqid, u16 flags, u16 qsize,
					u64 prp1)
{
	return nvme_mdev_nvmet_create_sq(tctrl->drvdata, sqid, cqid, flags,
					 qsize, prp1);
}

static u16 nvme_mdev_nvmet_op_delete_sq(struct nvmet_ctrl *tctrl, u16 sqid)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;

	if (sqid >= MAX_VIRTUAL_QUEUES || !test_bit(sqid, vctrl->vsq_en))
		return NVME_SC_QID_INVALID | NVME_STATUS_DNR;
	nvme_mdev_nvmet_delete_sq(vctrl, sqid);
	return NVME_SC_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* other nvmet operations */

static u16 nvme_mdev_nvmet_op_get_feature(const struct nvmet_ctrl *tctrl,
					  u8 feat, void *data)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;
	struct nvmet_feat_arbitration *arb;
	struct nvmet_feat_irq_coalesce *irqc;
	struct nvmet_feat_irq_config *irqcfg;

	switch (feat) {
	case NVME_FEAT_ARBITRATION:
		arb = data;
		arb->ab = vctrl->arb_burst_shift & 0x7;
		return NVME_SC_SUCCESS;
	case NVME_FEAT_IRQ_COALESCE:
		irqc = data;
		irqc->thr = vctrl->irqs.irq_coalesc_max ?
			    vctrl->irqs.irq_coalesc_max - 1 : 0;
		irqc->time = vctrl->irqs.irq_coalesc_time_us / 100;
		return NVME_SC_SUCCESS;
	case NVME_FEAT_IRQ_CONFIG:
		irqcfg = data;
		if (irqcfg->iv >= MAX_VIRTUAL_IRQS)
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		/* CD: coalescing disabled */
		irqcfg->cd = !vctrl->irqs.vecs[irqcfg->iv].irq_coalesc_en;
		return NVME_SC_SUCCESS;
	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
}

static u16 nvme_mdev_nvmet_op_set_feature(const struct nvmet_ctrl *tctrl,
					  u8 feat, void *data)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;
	struct nvmet_feat_arbitration *arb;
	struct nvmet_feat_irq_coalesce *irqc;
	struct nvmet_feat_irq_config *irqcfg;

	lockdep_assert_held(&vctrl->lock);

	switch (feat) {
	case NVME_FEAT_ARBITRATION:
		arb = data;
		vctrl->arb_burst_shift = arb->ab & 0x7;
		return NVME_SC_SUCCESS;
	case NVME_FEAT_IRQ_COALESCE:
		irqc = data;
		vctrl->irqs.irq_coalesc_max = irqc->thr + 1;
		vctrl->irqs.irq_coalesc_time_us = irqc->time * 100;
		return NVME_SC_SUCCESS;
	case NVME_FEAT_IRQ_CONFIG:
		irqcfg = data;
		if (irqcfg->iv >= MAX_VIRTUAL_IRQS)
			return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
		vctrl->irqs.vecs[irqcfg->iv].irq_coalesc_en = !irqcfg->cd;
		return NVME_SC_SUCCESS;
	default:
		return NVME_SC_INVALID_FIELD | NVME_STATUS_DNR;
	}
}

/* Doorbell Buffer Config: switch to the guest's shadow doorbells */
static u16 nvme_mdev_nvmet_op_dbbuf_config(struct nvmet_ctrl *tctrl,
					   u64 dbbuf_addr, u64 eventidx_addr)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;
	int ret;

	lockdep_assert_held(&vctrl->lock);

	/* (the setting can't be changed while the device is open) */
	if (!vctrl->mmio.shadow_db_supported)
		return NVME_SC_INVALID_OPCODE | NVME_STATUS_DNR;

	/* only once per controller enable */
	if (vctrl->mmio.shadow_db_en)
		return NVME_SC_CMD_SEQ_ERROR | NVME_STATUS_DNR;

	ret = nvme_mdev_mmio_enable_dbs_shadow(vctrl, dbbuf_addr,
					       eventidx_addr, true);
	return nvme_mdev_translate_error(ret);
}

static u8 nvme_mdev_nvmet_op_get_mdts(const struct nvmet_ctrl *tctrl)
{
	/* in units of the minimum memory page size, 4k */
	return NVME_MDEV_MDTS;
}

static int nvme_mdev_nvmet_op_add_port(struct nvmet_port *port)
{
	mutex_lock(&nvme_mdev_ports_mutex);
	list_add_tail(&port->entry, &nvme_mdev_ports);
	mutex_unlock(&nvme_mdev_ports_mutex);
	return 0;
}

static void nvme_mdev_nvmet_op_remove_port(struct nvmet_port *port)
{
	mutex_lock(&nvme_mdev_ports_mutex);
	list_del_init(&port->entry);
	mutex_unlock(&nvme_mdev_ports_mutex);
}

static struct nvmet_port *nvme_mdev_nvmet_find_port(u16 portid)
{
	struct nvmet_port *p, *port = NULL;

	mutex_lock(&nvme_mdev_ports_mutex);
	list_for_each_entry(p, &nvme_mdev_ports, entry) {
		if (le16_to_cpu(p->disc_addr.portid) == portid) {
			port = p;
			break;
		}
	}
	mutex_unlock(&nvme_mdev_ports_mutex);
	return port;
}

/*
 * nvmet wants the controller gone (the subsystem was removed or unlinked
 * from the port, or a fatal error).  Called with nvmet locks held: the
 * work does it.
 */
static void nvme_mdev_nvmet_op_delete_ctrl(struct nvmet_ctrl *tctrl)
{
	struct nvme_mdev_vctrl *vctrl = tctrl->drvdata;

	if (READ_ONCE(vctrl->tctrl) != tctrl)
		return;
	set_bit(NVME_MDEV_NVMET_DELETE, &vctrl->nvmet_flags);
	queue_work(system_wq, &vctrl->nvmet_delete_work);
}

static void nvme_mdev_nvmet_delete_work(struct work_struct *work)
{
	struct nvme_mdev_vctrl *vctrl =
		container_of(work, struct nvme_mdev_vctrl, nvmet_delete_work);

	mutex_lock(&vctrl->lock);
	if (test_and_clear_bit(NVME_MDEV_NVMET_DELETE, &vctrl->nvmet_flags) &&
	    vctrl->tctrl) {
		_INFO(vctrl, "nvmet removed the controller\n");
		/* the guest sees a controller fatal error and resets it */
		nvme_mdev_mmio_fatal_error(vctrl);
		nvme_mdev_nvmet_ctrl_destroy(vctrl);
	}
	mutex_unlock(&vctrl->lock);
}

static const struct nvmet_fabrics_ops nvme_mdev_nvmet_ops = {
	.owner		= THIS_MODULE,
	.type		= NVMF_TRTYPE_PCI,
	/* the I/O thread polls (nvme_mdev_nvmet_poll_io()) */
	.flags		= NVMF_POLLED_IO,
	.add_port	= nvme_mdev_nvmet_op_add_port,
	.remove_port	= nvme_mdev_nvmet_op_remove_port,
	.queue_response	= nvme_mdev_nvmet_queue_response,
	.delete_ctrl	= nvme_mdev_nvmet_op_delete_ctrl,
	.get_mdts	= nvme_mdev_nvmet_op_get_mdts,
	.create_cq	= nvme_mdev_nvmet_op_create_cq,
	.delete_cq	= nvme_mdev_nvmet_op_delete_cq,
	.create_sq	= nvme_mdev_nvmet_op_create_sq,
	.delete_sq	= nvme_mdev_nvmet_op_delete_sq,
	.get_feature	= nvme_mdev_nvmet_op_get_feature,
	.set_feature	= nvme_mdev_nvmet_op_set_feature,
	.dbbuf_config	= nvme_mdev_nvmet_op_dbbuf_config,
};

/* ------------------------------------------------------------------ */
/* the nvmet controller */

/*
 * Create the nvmet controller for the configured port and subsystem.  The
 * host NQN and ID are the instance's UUID, so that subsystems can be
 * restricted to particular instances (allowed_hosts).
 */
int nvme_mdev_nvmet_ctrl_create(struct nvme_mdev_vctrl *vctrl)
{
	struct nvmet_alloc_ctrl_args args = {};
	struct nvmet_ctrl *tctrl;
	struct nvmet_port *port;
	uuid_t hostid;

	lockdep_assert_held(&vctrl->lock);

	if (vctrl->tctrl)
		return 0;

	if (!vctrl->subsysnqn[0]) {
		_INFO(vctrl, "no nvmet subsystem set (nvmet/subsysnqn)\n");
		return -EINVAL;
	}

	port = nvme_mdev_nvmet_find_port(vctrl->portid);
	if (!port) {
		_INFO(vctrl, "nvmet port %u: not enabled for the pci transport\n",
		      vctrl->portid);
		return -ENODEV;
	}

	/* the UUID part of the host NQN, i.e. the instance's UUID */
	if (uuid_parse(strrchr(vctrl->hostnqn, ':') + 1, &hostid))
		uuid_gen(&hostid);
	args.port = port;
	args.subsysnqn = vctrl->subsysnqn;
	args.hostnqn = vctrl->hostnqn;
	args.hostid = &hostid;
	args.ops = &nvme_mdev_nvmet_ops;

	tctrl = nvmet_alloc_ctrl(&args);
	if (!tctrl) {
		_INFO(vctrl, "can't create an nvmet controller for %s (status 0x%x)\n",
		      vctrl->subsysnqn, args.status);
		return -ENODEV;
	}

	if (tctrl->pi_support) {
		_INFO(vctrl, "protection information is not supported\n");
		nvmet_ctrl_put(tctrl);
		return -EOPNOTSUPP;
	}

	tctrl->drvdata = vctrl;
	/* the queues the guest can create (Number of Queues feature) */
	tctrl->max_qid = min_t(u16, tctrl->max_qid, MAX_VIRTUAL_QUEUES - 1);

	clear_bit(NVME_MDEV_NVMET_DELETE, &vctrl->nvmet_flags);
	WRITE_ONCE(vctrl->tctrl, tctrl);
	nvme_mdev_mmio_set_cap(vctrl, tctrl->cap);
	return 0;
}

/* Drop the nvmet controller; its queues must be gone already */
void nvme_mdev_nvmet_ctrl_destroy(struct nvme_mdev_vctrl *vctrl)
{
	struct nvmet_ctrl *tctrl = vctrl->tctrl;

	lockdep_assert_held(&vctrl->lock);

	if (!tctrl)
		return;

	WARN_ON(test_bit(0, vctrl->vsq_en) || test_bit(0, vctrl->vcq_en));
	nvmet_update_cc(tctrl, 0);
	WRITE_ONCE(vctrl->tctrl, NULL);
	nvmet_ctrl_put(tctrl);
}

/*
 * The guest enables the controller (CC.EN 0 -> 1): get an nvmet
 * controller if nvmet deleted ours, and enable it, which checks CC.
 */
int nvme_mdev_nvmet_enable(struct nvme_mdev_vctrl *vctrl)
{
	int ret;

	lockdep_assert_held(&vctrl->lock);

	ret = nvme_mdev_nvmet_ctrl_create(vctrl);
	if (ret)
		return ret;

	nvmet_update_cc(vctrl->tctrl, vctrl->mmio.cc);
	if (!(vctrl->tctrl->csts & NVME_CSTS_RDY)) {
		_DBG(vctrl, "nvmet refused CC 0x%x\n", vctrl->mmio.cc);
		nvmet_update_cc(vctrl->tctrl, 0);
		return -EINVAL;
	}
	return 0;
}

/* The controller was disabled, all its queues deleted */
void nvme_mdev_nvmet_disable(struct nvme_mdev_vctrl *vctrl)
{
	lockdep_assert_held(&vctrl->lock);

	/* commands that completed while their queue went away */
	nvme_mdev_nvmet_reap_adm(vctrl);
	nvme_mdev_nvmet_reap_io(vctrl);
	WARN_ON(atomic_read(&vctrl->io_inflight));

	if (vctrl->tctrl)
		nvmet_update_cc(vctrl->tctrl, 0);
}

void nvme_mdev_nvmet_vctrl_init(struct nvme_mdev_vctrl *vctrl)
{
	init_llist_head(&vctrl->adm_done);
	init_llist_head(&vctrl->io_done);
	atomic_set(&vctrl->io_inflight, 0);
	INIT_LIST_HEAD(&vctrl->io_list);
	INIT_LIST_HEAD(&vctrl->adm_list);
	INIT_WORK(&vctrl->adm_done_work, nvme_mdev_nvmet_adm_done_work);
	INIT_WORK(&vctrl->nvmet_delete_work, nvme_mdev_nvmet_delete_work);
	/* the instance's UUID, as the mdev core shows it */
	snprintf(vctrl->hostnqn, sizeof(vctrl->hostnqn),
		 "nqn.2014-08.org.nvmexpress:uuid:%pUl", &vctrl->mdev->uuid);
}

/* The device is going away: no nvmet controller, no queues */
void nvme_mdev_nvmet_vctrl_cleanup(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_iod *iod, *tmp;
	struct llist_node *list;

	cancel_work_sync(&vctrl->nvmet_delete_work);
	cancel_work_sync(&vctrl->adm_done_work);

	/* e.g. the Asynchronous Event Requests failed at the end */
	list = llist_del_all(&vctrl->adm_done);
	llist_for_each_entry_safe(iod, tmp, list, done) {
		list_del(&iod->link);
		nvme_mdev_iod_free(iod);
	}
	WARN_ON(!list_empty(&vctrl->adm_list));
	WARN_ON(!llist_empty(&vctrl->io_done));
}

int nvme_mdev_nvmet_init(void)
{
	int ret;

	nvme_mdev_iod_cache = KMEM_CACHE(nvme_mdev_iod, 0);
	if (!nvme_mdev_iod_cache)
		return -ENOMEM;

	ret = nvmet_register_transport(&nvme_mdev_nvmet_ops);
	if (ret) {
		pr_err("nvme_mdev: can't register the nvmet pci transport (nvmet-pci-epf loaded?)\n");
		kmem_cache_destroy(nvme_mdev_iod_cache);
	}
	return ret;
}

void nvme_mdev_nvmet_exit(void)
{
	nvmet_unregister_transport(&nvme_mdev_nvmet_ops);
	kmem_cache_destroy(nvme_mdev_iod_cache);
}
