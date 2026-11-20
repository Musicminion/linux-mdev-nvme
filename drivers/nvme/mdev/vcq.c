// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtual NVMe completion queue implementation
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "priv.h"

/* Create new virtual completion queue */
int nvme_mdev_vcq_init(struct nvme_mdev_vctrl *vctrl, u16 qid,
		       dma_addr_t iova, bool cont, u16 size, int irq)
{
	struct nvme_vcq *q = &vctrl->vcqs[qid];
	int ret;

	lockdep_assert_held(&vctrl->lock);

	q->iova = iova;
	q->cont = cont;
	q->data = NULL;
	q->qid = qid;
	q->size = size;
	q->tail = 0;
	q->phase = true;
	q->irq = irq;
	q->pending = 0;
	q->head = 0;

	ret = nvme_mdev_vcq_viommu_update(&vctrl->viommu, q);
	if (ret && (ret != -EFAULT))
		return ret;

	_DBG(vctrl, "VCQ: create qid=%d contig=%d depth=%d irq=%d\n",
	     qid, cont, size, irq);

	set_bit(qid, vctrl->vcq_en);

	WRITE_ONCE(vctrl->mmio.dbs[q->qid].cqh, 0);
	WRITE_ONCE(vctrl->mmio.eidxs[q->qid].cqh, 0);
	return 0;
}

/* Update the kernel mapping of the queue */
int nvme_mdev_vcq_viommu_update(struct nvme_mdev_viommu *viommu,
				struct nvme_vcq *q)
{
	void *data;

	if (q->data)
		vunmap(q->data);

	data = nvme_mdev_udata_queue_vmap(viommu, q->iova,
					  (unsigned int)q->size *
					  sizeof(struct nvme_completion),
					  q->cont);

	q->data = IS_ERR(data) ? NULL : data;
	return IS_ERR(data) ? PTR_ERR(data) : 0;
}

/* Delete a virtual completion queue */
void nvme_mdev_vcq_delete(struct nvme_mdev_vctrl *vctrl, u16 qid)
{
	struct nvme_vcq *q = &vctrl->vcqs[qid];

	lockdep_assert_held(&vctrl->lock);

	if (q->data)
		vunmap(q->data);
	q->data = NULL;

	_DBG(vctrl, "VCQ: delete qid=%d\n", q->qid);
	clear_bit(qid, vctrl->vcq_en);
}

/* Move queue tail one item forward */
static void nvme_mdev_vcq_advance_tail(struct nvme_vcq *q)
{
	if (++q->tail == q->size) {
		q->tail = 0;
		q->phase = !q->phase;
	}
}

/* Move queue head one item forward */
static void nvme_mdev_vcq_advance_head(struct nvme_vcq *q)
{
	q->head++;
	if (q->head == q->size)
		q->head = 0;
}

/* Process a virtual completion queue*/
void nvme_mdev_vcq_process(struct nvme_mdev_vctrl *vctrl, u16 qid,
			   bool trigger_irqs)
{
	struct nvme_vcq *q = &vctrl->vcqs[qid];
	u16 new_head;
	u32 eidx;

	if (!vctrl->mmio.dbs || !vctrl->mmio.eidxs)
		return;

	/*
	 * Acquire: the entries the guest just released get overwritten by
	 * later completions, those stores must come after this load.
	 */
	new_head = le32_to_cpu(smp_load_acquire(&vctrl->mmio.dbs[qid].cqh));

	if (new_head != q->head) {
		/* bad tail - can't process*/
		if (!nvme_mdev_mmio_db_check(vctrl, q->qid, q->size, new_head))
			return;

		while (q->head != new_head) {
			nvme_mdev_vcq_advance_head(q);
			WARN_ON_ONCE(q->pending == 0);
			if (q->pending > 0)
				q->pending--;
		}

		eidx = q->head + (q->size >> 1);
		if (eidx >= q->size)
			eidx -= q->size;
		WRITE_ONCE(vctrl->mmio.eidxs[q->qid].cqh, cpu_to_le32(eidx));
	}

	if (q->irq != -1 && trigger_irqs) {
		if (q->tail != new_head)
			nvme_mdev_irq_cond_trigger(vctrl, q->irq);
		else
			nvme_mdev_irq_clear(vctrl, q->irq);
	}
}

/* flush interrupts on a completion queue */
bool nvme_mdev_vcq_flush(struct nvme_mdev_vctrl *vctrl, u16 qid)
{
	struct nvme_vcq *q = &vctrl->vcqs[qid];
	u16 new_head = le32_to_cpu(READ_ONCE(vctrl->mmio.dbs[qid].cqh));

	if (new_head == q->tail || q->irq == -1)
		return false;

	nvme_mdev_irq_trigger(vctrl, q->irq);
	nvme_mdev_irq_clear(vctrl, q->irq);
	return true;
}

/* Reserve space for one completion entry, that will be added later */
bool nvme_mdev_vcq_reserve_space(struct nvme_vcq *q)
{
	/* TODOLATER: track passed through commmands
	 * If we pass through a command to host and never receive a response
	 * we will keep space for response in CQ forever, eventually stalling
	 * the CQ forever.
	 * In this case, the guest is still expected to recover by resetting
	 * our controller
	 * This can be fixed by tracking all the commands that we send
	 * to the host
	 */

	if (q->pending == q->size - 1)
		return false;
	q->pending++;
	return true;
}

/* Write a new item into the completion queue (IO version) */
void nvme_mdev_vcq_write_io(struct nvme_mdev_vctrl *vctrl,
			    struct nvme_vcq *q, u16 sq_head,
			    u16 sqid, u16 cid, u16 status)
{
	__le64 *qw = (__le64 *)(&q->data[q->tail]);

	u64 phase = q->phase ? (0x1ULL << 48) : 0;
	u64 qw1 =
		((u64)sq_head) |
		((u64)sqid << 16) |
		((u64)cid << 32) |
		((u64)status << 49) | phase;

	/*
	 * Only the second qword is written: status and phase are in it, so a
	 * single store publishes the entry.  The data the device wrote for
	 * the command was ordered before this by the acquire load of the
	 * request status in the I/O backend.
	 */
	WRITE_ONCE(qw[1], cpu_to_le64(qw1));

	nvme_mdev_vcq_advance_tail(q);
	if (q->irq != -1)
		nvme_mdev_irq_raise(vctrl, q->irq);
}

/* Write a new item into the completion queue (ADMIN version) */
void nvme_mdev_vcq_write_adm(struct nvme_mdev_vctrl *vctrl,
			     struct nvme_vcq *q, u32 dw0,
			     u16 sq_head, u16 cid, u16 status)
{
	__le64 *qw = (__le64 *)(&q->data[q->tail]);

	u64 phase = q->phase ? (0x1ULL << 48) : 0;
	u64 qw1 =
		((u64)sq_head) |
		((u64)cid << 32) |
		((u64)status << 49) | phase;

	WRITE_ONCE(qw[0], cpu_to_le64(dw0));
	/*
	 * The guest (a CPU, not a device) must see DW0 and any data we
	 * copied into its buffers before the phase bit flips.
	 */
	smp_wmb();
	WRITE_ONCE(qw[1], cpu_to_le64(qw1));

	nvme_mdev_vcq_advance_tail(q);
	if (q->irq != -1)
		nvme_mdev_irq_trigger(vctrl, q->irq);
}
