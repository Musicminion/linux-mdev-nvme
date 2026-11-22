// SPDX-License-Identifier: GPL-2.0+
/*
 * NVMe IO command translation and polling IO thread
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/sched/vhost_task.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include <linux/timekeeping.h>
#include <linux/ktime.h>
#include "priv.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

struct io_ctx {
	struct nvme_mdev_hctrl *hctrl;
	struct nvme_mdev_vctrl *vctrl;

	const struct nvme_command *in;	/* points to in_cmd */
	struct nvme_command in_cmd;	/* the command, copied from the guest */
	struct nvme_mdev_vns *ns;
	struct nvme_ext_data_iter udatait;

	struct nvme_command out;
	struct nvme_ext_data_iter *kdatait;

	ktime_t last_io_t;
	ktime_t last_admin_poll_time;
	unsigned int idle_timeout_ms;
	unsigned int arb_burst;
};

/* Check if we need to read a command from the admin queue */
static bool nvme_mdev_adm_needs_processing(struct io_ctx *ctx)
{
	if (!timeout(ctx->last_admin_poll_time,
		     ctx->vctrl->now, NVME_MDEV_ADMIN_POLL_MS))
		return false;

	if (nvme_mdev_vsq_has_data(ctx->vctrl, &ctx->vctrl->vsqs[0]))
		return true;

	ctx->last_admin_poll_time = ctx->vctrl->now;
	return false;
}

/* Handle read/write command.*/
static int nvme_mdev_io_translate_rw(struct io_ctx *ctx)
{
	int ret;
	const struct nvme_rw_command *in = &ctx->in->rw;

	u64 slba = le64_to_cpu(in->slba);
	u64 length = le16_to_cpu(in->length) + 1;
	u16 control = le16_to_cpu(in->control);

	_DBG(ctx->vctrl, "IOQ: READ/WRITE\n");

	if (!check_reserved_dwords(nvme_mdev_cmd_dwords(ctx->in), 16,
				   RSRV_DW23 | RSRV_MPTR | RSRV_DW14_15))
		return DNR(NVME_SC_INVALID_FIELD);

	if (!check_reserved_dwords(nvme_mdev_cmd_dwords(ctx->in), 16, 0b1100000000111100))
		return DNR(NVME_SC_INVALID_FIELD);

	if (in->opcode == nvme_cmd_write && ctx->ns->readonly)
		return DNR(NVME_SC_READ_ONLY);

	if (!check_range(slba, length, ctx->ns->ns_size))
		return DNR(NVME_SC_LBA_RANGE);

	ctx->out.rw.slba = in->slba;
	ctx->out.rw.length = in->length;

	/* data transfer direction: opcode bit 1, controller to host */
	ret = nvme_mdev_udata_iter_set_dptr(&ctx->udatait, &in->dptr,
					    length << ctx->ns->blksize_shift,
					    in->opcode & 2);
	if (ret)
		return nvme_mdev_translate_error(ret);

	ctx->kdatait = &ctx->udatait;
	if (control & ~(NVME_RW_LR | NVME_RW_FUA))
		return DNR(NVME_SC_INVALID_FIELD);

	ctx->out.rw.control = in->control;
	return -1;
}

/*Handle flush command */
static int nvme_mdev_io_translate_flush(struct io_ctx *ctx)
{
	ctx->kdatait = NULL;

	_DBG(ctx->vctrl, "IOQ: FLUSH\n");

	if (!check_reserved_dwords(nvme_mdev_cmd_dwords(ctx->in), 16,
				   RSRV_DW23 | RSRV_DPTR |
				   RSRV_MPTR | RSRV_DW10_15))
		return DNR(NVME_SC_INVALID_FIELD);

	if (ctx->ns->readonly)
		return DNR(NVME_SC_READ_ONLY);

	return -1;
}

/* Handle write zeros command */
static int nvme_mdev_io_translate_write_zeros(struct io_ctx *ctx)
{
	const struct nvme_write_zeroes_cmd *in = &ctx->in->write_zeroes;
	u64 slba = le64_to_cpu(in->slba);
	u64 length = le16_to_cpu(in->length) + 1;
	u16 control = le16_to_cpu(in->control);

	_DBG(ctx->vctrl, "IOQ: WRITE_ZEROS\n");

	if (!check_reserved_dwords(nvme_mdev_cmd_dwords(ctx->in), 16,
				   RSRV_DW23 | RSRV_DPTR |
				   RSRV_MPTR | RSRV_DW13_15))
		return DNR(NVME_SC_INVALID_FIELD);

	if (!nvme_mdev_hctrl_hq_check_op(ctx->hctrl, in->opcode))
		return DNR(NVME_SC_INVALID_OPCODE);

	if (ctx->ns->readonly)
		return DNR(NVME_SC_READ_ONLY);
	ctx->kdatait = NULL;

	if (!check_range(slba, length, ctx->ns->ns_size))
		return DNR(NVME_SC_LBA_RANGE);

	ctx->out.write_zeroes.slba = in->slba;
	ctx->out.write_zeroes.length = in->length;

	if (control & ~(NVME_RW_LR | NVME_RW_FUA | NVME_WZ_DEAC))
		return DNR(NVME_SC_INVALID_FIELD);

	ctx->out.write_zeroes.control = in->control;
	return -1;
}

/* Handle dataset management command */
static int nvme_mdev_io_translate_dsm(struct io_ctx *ctx)
{
	unsigned int size, i, nr;
	int ret;
	const struct nvme_dsm_cmd *in = &ctx->in->dsm;
	struct nvme_dsm_range *data_ptr;

	_DBG(ctx->vctrl, "IOQ: DSM_MANAGEMENT\n");

	if (!check_reserved_dwords(nvme_mdev_cmd_dwords(ctx->in), 16,
				   RSRV_DW23 | RSRV_MPTR | RSRV_DW12_15))
		return DNR(NVME_SC_INVALID_FIELD);

	if (le32_to_cpu(in->nr) & 0xFFFFFF00)
		return DNR(NVME_SC_INVALID_FIELD);

	if (!nvme_mdev_hctrl_hq_check_op(ctx->hctrl, in->opcode))
		return DNR(NVME_SC_INVALID_OPCODE);

	if (ctx->ns->readonly)
		return DNR(NVME_SC_READ_ONLY);

	nr = le32_to_cpu(in->nr) + 1;
	size = nr * sizeof(struct nvme_dsm_range);

	ctx->out.dsm.nr = in->nr;
	ret = nvme_mdev_udata_iter_set_dptr(&ctx->udatait, &in->dptr, size,
					    false);
	if (ret)
		goto error;

	ctx->kdatait = nvme_mdev_kdata_iter_alloc(&ctx->vctrl->viommu, size);
	if (!ctx->kdatait)
		return NVME_SC_INTERNAL;

	_DBG(ctx->vctrl, "IOQ: DSM_MANAGEMENT: NR=%d\n", nr);

	ret = nvme_mdev_read_from_udata(ctx->kdatait->kmem.data, &ctx->udatait,
					size);
	if (ret)
		goto error2;

	data_ptr = (struct nvme_dsm_range *)ctx->kdatait->kmem.data;

	for (i = 0 ; i < nr; i++) {
		u64 slba = le64_to_cpu(data_ptr[i].slba);
		/* looks like not zero based value*/
		u32 nlb = le32_to_cpu(data_ptr[i].nlb);

		if (!check_range(slba, nlb, ctx->ns->ns_size))
			goto error2;

		_DBG(ctx->vctrl, "IOQ: DSM_MANAGEMENT: RANGE 0x%llx-0x%x\n",
		     slba, nlb);
	}

	ctx->out.dsm.attributes = in->attributes;
	return -1;
error2:
	ctx->kdatait->release(ctx->kdatait);
error:
	return nvme_mdev_translate_error(ret);
}

/* Process one new command in the io queue*/
static int nvme_mdev_io_translate_cmd(struct io_ctx *ctx)
{
	memset(&ctx->out, 0, sizeof(ctx->out));
	/* translate opcode */
	ctx->out.common.opcode = ctx->in->common.opcode;

	/* check flags */
	if (ctx->in->common.flags != 0)
		return DNR(NVME_SC_INVALID_FIELD);

	/* namespace*/
	ctx->ns = nvme_mdev_vns_from_vnsid(ctx->vctrl,
					   le32_to_cpu(ctx->in->rw.nsid));
	if (!ctx->ns || READ_ONCE(ctx->ns->dead)) {
		_DBG(ctx->vctrl, "IOQ: invalid NSID\n");
		return DNR(NVME_SC_INVALID_NS);
	}

	if (!ctx->ns->readonly && bdev_read_only(ctx->ns->host_part))
		ctx->ns->readonly = true;

	switch (ctx->in->common.opcode) {
	case nvme_cmd_flush:
		return nvme_mdev_io_translate_flush(ctx);
	case nvme_cmd_read:
		return nvme_mdev_io_translate_rw(ctx);
	case nvme_cmd_write:
		return nvme_mdev_io_translate_rw(ctx);
	case nvme_cmd_write_zeroes:
		return nvme_mdev_io_translate_write_zeros(ctx);
	case nvme_cmd_dsm:
		return nvme_mdev_io_translate_dsm(ctx);
	default:
		return DNR(NVME_SC_INVALID_OPCODE);
	}
}

/* process a user submission queue */
static bool nvme_mdev_io_process_sq(struct io_ctx *ctx, u16 sqid)
{
	struct nvme_vsq *vsq = &ctx->vctrl->vsqs[sqid];
	u16 ucid;
	int ret;

	/* If host queue is full, we can't process a command
	 * as a command will likely result in passthrough
	 */
	if (!nvme_mdev_hctrl_hq_can_submit(ctx->hctrl, vsq->hsq))
		return false;

	/* read the command */
	if (!nvme_mdev_vsq_get_cmd(ctx->vctrl, vsq, &ctx->in_cmd))
		return false;
	ctx->in = &ctx->in_cmd;
	ucid = nvme_mdev_cmd_cid(ctx->in);

	/* translate the command */
	ret = nvme_mdev_io_translate_cmd(ctx);
	if (ret != -1) {
		_DBG(ctx->vctrl,
		     "IOQ: QID %d CID %d FAILED: status 0x%x (translate)\n",
		     sqid, ucid, ret);
		nvme_mdev_vsq_cmd_done_io(ctx->vctrl, sqid, ucid, ret);
		return true;
	}

	/*passthrough*/
	trace_nvme_mdev_io_submit(ctx->vctrl, sqid, ctx->in);
	ret = nvme_mdev_hctrl_hq_submit(ctx->hctrl,
					ctx->ns,
					vsq->hsq,
					(((u32)vsq->qid) << 16) | ((u32)ucid),
					&ctx->out,
					ctx->kdatait);
	if (ret) {
		ret = nvme_mdev_translate_error(ret);

		_DBG(ctx->vctrl,
		     "IOQ: QID %d CID %d FAILED: status 0x%x (host submit)\n",
		     sqid, ucid, ret);

		nvme_mdev_vsq_cmd_done_io(ctx->vctrl, sqid, ucid, ret);
	}
	return true;
}

/* process a user completion queue */
static void nvme_mdev_io_process_cq(struct io_ctx *ctx, u16 cqid)
{
	nvme_mdev_vcq_process(ctx->vctrl, cqid, true);
}

/* see vctrl->irq_mode */
bool nvme_mdev_io_irq_mode(struct nvme_mdev_vctrl *vctrl)
{
	return READ_ONCE(vctrl->irq_mode) && vctrl->mmio.shadow_db_supported;
}

/*
 * Host I/O completed (any context, after its status is published): in
 * interrupt mode, wake the I/O thread up if it sleeps.  Under the lock,
 * so that the thread can't go away meanwhile (nvme_mdev_io_free()).
 */
void nvme_mdev_io_completed(struct nvme_mdev_vctrl *vctrl)
{
	unsigned long flags;

	if (!READ_ONCE(vctrl->irq_mode))
		return;
	WRITE_ONCE(vctrl->io_kicked, true);
	/* io_kicked before io_sleeping: pairs with nvme_mdev_io_wait() */
	smp_mb();
	if (!READ_ONCE(vctrl->io_sleeping))
		return;
	spin_lock_irqsave(&vctrl->wake_lock, flags);
	if (READ_ONCE(vctrl->io_sleeping) && vctrl->iothread)
		vhost_task_wake(vctrl->iothread);
	spin_unlock_irqrestore(&vctrl->wake_lock, flags);
}

/* the guest has commands for us */
static bool nvme_mdev_io_sqs_have_data(struct nvme_mdev_vctrl *vctrl)
{
	u16 sqid = 1;

	for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
		if (nvme_mdev_vsq_has_data(vctrl, &vctrl->vsqs[sqid]))
			return true;
	return false;
}

/*
 * Interrupt mode, with host I/O in flight and nothing else to do: sleep
 * until some completes (nvme_mdev_io_completed() wakes us up), the guest
 * rings a doorbell (the event indexes ask for that now; the MMIO write
 * wakes us up through nvme_mdev_io_resume()), or we are paused.
 */
static void nvme_mdev_io_wait(struct io_ctx *ctx)
{
	struct nvme_mdev_vctrl *vctrl = ctx->vctrl;
	u16 sqid = 1;

	for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
		if (!nvme_mdev_vsq_suspend_io(vctrl, sqid))
			return;		/* there are new commands already */

	/* no interrupt waits for its coalescing time while we sleep */
	nvme_mdev_irqs_flush_pending(vctrl);

	WRITE_ONCE(vctrl->io_sleeping, true);
	/*
	 * A full barrier: a completion after it sees io_sleeping and wakes
	 * us (or we see io_kicked); a doorbell write after it wakes us (or
	 * we see the new tail).
	 */
	set_current_state(TASK_INTERRUPTIBLE);
	if (!READ_ONCE(vctrl->io_kicked) && !READ_ONCE(vctrl->io_pause_req) &&
	    !nvme_mdev_io_sqs_have_data(vctrl))
		schedule();
	__set_current_state(TASK_RUNNING);
	WRITE_ONCE(vctrl->io_sleeping, false);
}

/* process hardware completion queue */
static int nvme_mdev_io_process_hwq(struct io_ctx *ctx, u16 hwq)
{
	int n, i;
	struct nvme_ext_cmd_result res[16];

	/* process the completions from the hardware */
	n = nvme_mdev_hctrl_hq_poll(ctx->hctrl, hwq, res, 16);
	if (n == -1)
		return -1;

	for (i = 0; i < n; i++) {
		u16 qid = res[i].tag >> 16;
		u16 cid = res[i].tag & 0xFFFF;
		u16 status = res[i].status;

		if (status != 0)
			_DBG(ctx->vctrl,
			     "IOQ: QID %d CID %d FAILED: status 0x%x (host response)\n",
			     qid, cid, status);

		trace_nvme_mdev_io_complete(ctx->vctrl, qid, cid, status);
		nvme_mdev_vsq_cmd_done_io(ctx->vctrl, qid, cid, status);
	}

	return n;
}

/* do polling till one of events stops it */
static void nvme_mdev_io_polling_loop(struct io_ctx *ctx)
{
	struct nvme_mdev_vctrl *vctrl = ctx->vctrl;
	u16 i, cqid, sqid, hsqcnt;
	u16 hsqs[MAX_HOST_QUEUES];
	struct blk_plug plug;
	bool idle = false;
	bool irq_mode = nvme_mdev_io_irq_mode(vctrl);
	bool pending_io;
	unsigned int work;
	int n;

	hsqcnt = nvme_mdev_vctrl_hqs_list(vctrl, hsqs);
	ctx->arb_burst = 1 << ctx->vctrl->arb_burst_shift;

	/* can't stop polling when shadow db not enabled */
	ctx->idle_timeout_ms = vctrl->mmio.shadow_db_en ?
			       READ_ONCE(vctrl->poll_timeout_ms) : 0;

	vctrl->now = ktime_get();
	ctx->last_admin_poll_time = vctrl->now;
	ctx->last_io_t = vctrl->now;

	/* main loop, until paused or killed */
	while (!READ_ONCE(vctrl->io_pause_req) && !signal_pending(current)) {
		vctrl->now = ktime_get();

		/* check if we have to exit to support admin polling */
		if (!vctrl->mmio.shadow_db_supported)
			if (nvme_mdev_adm_needs_processing(ctx))
				break;

		/*
		 * process the submission queues; plugged, so that the block
		 * layer hands the batch to the driver at once (->queue_rqs)
		 */
		work = 0;
		/* completions from now on show (interrupt mode) */
		if (irq_mode)
			WRITE_ONCE(vctrl->io_kicked, false);
		blk_start_plug(&plug);
		sqid = 1;
		for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
			for (i = 0 ; i < ctx->arb_burst ; i++) {
				if (!nvme_mdev_io_process_sq(ctx, sqid))
					break;
				work++;
			}
		blk_finish_plug(&plug);

		/* process the completions from the guest*/
		cqid = 1;
		for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
			nvme_mdev_io_process_cq(ctx, cqid);

		/* process the completions from the hardware*/
		pending_io = false;
		for (i = 0 ; i < hsqcnt ; i++) {
			n = nvme_mdev_io_process_hwq(ctx, hsqs[i]);
			if (n > 0) {
				ctx->last_io_t = vctrl->now;
				work++;
			}
			if (n != -1)
				pending_io = true;
		}

		/*
		 * Interrupt mode: go on while there's work; idle as soon as
		 * no host I/O is in flight; else sleep until something
		 * happens
		 */
		if (irq_mode && !work) {
			if (!pending_io) {
				idle = true;
				break;
			}
			nvme_mdev_io_wait(ctx);
			continue;
		}

		/* Check if we need to stop polling*/
		if (ctx->idle_timeout_ms) {
			if (timeout(ctx->last_io_t,
				    vctrl->now, ctx->idle_timeout_ms)) {
				idle = true;
				break;
			}
		}
		cond_resched();
	}

	/* Drain the host IO */
	for (;;) {
		bool pending_io = false;

		vctrl->now = ktime_get();

		/* nothing to wait for if the guest killed the controller */
		if (nvme_mdev_vctrl_is_dead(vctrl)) {
			idle = false;
			break;
		}

		for (i = 0; i < hsqcnt; i++) {
			int n = nvme_mdev_io_process_hwq(ctx, hsqs[i]);

			if (n != -1)
				pending_io = true;
			if (n > 0)
				ctx->last_io_t = vctrl->now;
		}

		if (!pending_io)
			break;

		cond_resched();

		if (!timeout(ctx->last_io_t, vctrl->now,
			     NVME_MDEV_IO_TIMEOUT_MS))
			continue;

		_WARN(ctx->vctrl, "IO: skipping flush - host IO timeout\n");
		idle = false;
		break;
	}

	/*
	 * Drain all the pending completion interrupts to the guest; while it
	 * hasn't consumed the entries, keep polling (interrupt mode: no,
	 * that's the point, it rings the doorbell when it has more for us)
	 */
	cqid = 1;
	for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
		if (nvme_mdev_vcq_flush(vctrl, cqid) && !irq_mode)
			idle = false;

	/* Park IO thread if IO is truly idle*/
	if (idle) {
		/* don't bother going idle if someone holds the vctrl
		 * lock. It might try to park us, and thus
		 * cause a deadlock
		 */
		if (!mutex_trylock(&vctrl->lock))
			return;

		sqid = 1;
		for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
			if (!nvme_mdev_vsq_suspend_io(vctrl, sqid)) {
				idle = false;
				break;
			}

		if (idle) {
			/* sleep in nvme_mdev_io_polling_thread() until kicked */
			_DBG(ctx->vctrl, "IO: going idle\n");
			WRITE_ONCE(vctrl->io_idle, true);
		}

		mutex_unlock(&vctrl->lock);
	}

	/* Admin poll for cases when shadow doorbell is not supported */
	if (!vctrl->mmio.shadow_db_supported) {
		if (mutex_trylock(&vctrl->lock)) {
			nvme_mdev_vcq_process(vctrl, 0, false);
			nvme_mdev_adm_process_sq(ctx->vctrl);
			ctx->last_admin_poll_time = vctrl->now;
			mutex_unlock(&ctx->vctrl->lock);
		}
	}
}

/*
 * One round of the I/O thread.  vhost_task calls this in a loop with the
 * task state already set to TASK_INTERRUPTIBLE: returning false sleeps
 * until we are woken by nvme_mdev_io_resume()/_pause() or a signal.
 */
static bool nvme_mdev_io_thread_fn(void *data)
{
	struct io_ctx *ctx = data;
	struct nvme_mdev_vctrl *vctrl = ctx->vctrl;

	WRITE_ONCE(vctrl->iothread_task, current);

	/* pairs with smp_store_release() in nvme_mdev_io_pause() */
	if (smp_load_acquire(&vctrl->io_pause_req)) {
		/* all our work is done: tell nvme_mdev_io_pause() */
		smp_store_release(&vctrl->io_pause_ack, true);
		wake_up(&vctrl->io_pause_wq);
		return false;
	}

	/* idle: wait for a doorbell write.  io_idle is only cleared under
	 * vctrl->lock, by nvme_mdev_io_resume()'s callers.
	 */
	if (READ_ONCE(vctrl->io_idle))
		return false;

	__set_current_state(TASK_RUNNING);
	nvme_mdev_io_polling_loop(ctx);
	return true;
}

/*
 * The process the thread belongs to got SIGKILL: the thread exits without
 * being stopped.  It drained its host I/O before (the polling loop stops on
 * a pending signal), so just let nvme_mdev_io_pause() callers go.
 */
static void nvme_mdev_io_thread_killed(void *data)
{
	struct io_ctx *ctx = data;

	_DBG(ctx->vctrl, "IO: iothread killed\n");
	WRITE_ONCE(ctx->vctrl->io_dead, true);
	wake_up(&ctx->vctrl->io_pause_wq);
}

static bool nvme_mdev_io_is_self(struct nvme_mdev_vctrl *vctrl)
{
	return current == READ_ONCE(vctrl->iothread_task);
}

/*
 * Undo nvme_mdev_io_pause(), and wake the thread up if it has work (it
 * goes back to sleep by itself if io_idle is still set).
 */
void nvme_mdev_io_resume(struct nvme_mdev_vctrl *vctrl)
{
	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->iothread || nvme_mdev_io_is_self(vctrl) ||
	    vctrl->vctrl_paused || READ_ONCE(vctrl->io_dead))
		return;

	if (vctrl->iothread_parked) {
		vctrl->iothread_parked = false;
		WRITE_ONCE(vctrl->io_pause_req, false);
	}
	if (!READ_ONCE(vctrl->io_idle))
		vhost_task_wake(vctrl->iothread);
}

static bool nvme_mdev_io_paused(struct nvme_mdev_vctrl *vctrl)
{
	/*
	 * Acquire: what the thread did before it acknowledged the pause
	 * (completion entries, its view of the queues) is visible to us
	 * afterwards; pairs with its smp_store_release().
	 */
	return smp_load_acquire(&vctrl->io_pause_ack) ||
	       READ_ONCE(vctrl->io_dead);
}

/*
 * Pause the IO thread: returns once it finished what it was doing (its
 * host I/O drained) and sleeps until nvme_mdev_io_resume().  Not from the
 * thread itself, which runs nothing else while it is in here anyway.
 */
void nvme_mdev_io_pause(struct nvme_mdev_vctrl *vctrl)
{
	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->iothread || nvme_mdev_io_is_self(vctrl) ||
	    vctrl->iothread_parked)
		return;

	vctrl->iothread_parked = true;
	WRITE_ONCE(vctrl->io_pause_ack, false);
	/* the thread must not see the request with a stale ack */
	smp_store_release(&vctrl->io_pause_req, true);
	vhost_task_wake(vctrl->iothread);
	wait_event(vctrl->io_pause_wq, nvme_mdev_io_paused(vctrl));
}

/*
 * Set up the IO thread.  Called when the device is opened, from the
 * context of the process that opens it (QEMU): the thread becomes one of
 * its threads, see vhost_task_create().
 */
int nvme_mdev_io_create(struct nvme_mdev_vctrl *vctrl)
{
	struct vhost_task *vtsk;
	struct io_ctx *ctx;

	_DBG(vctrl, "IO: creating the polling iothread\n");

	lockdep_assert_held(&vctrl->lock);
	if (WARN_ON(vctrl->iothread))
		return -EINVAL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->vctrl = vctrl;
	ctx->hctrl = vctrl->hctrl;
	nvme_mdev_udata_iter_setup(&vctrl->viommu, &ctx->udatait);

	vctrl->iothread_parked = false;
	vctrl->io_pause_req = false;
	vctrl->io_pause_ack = false;
	vctrl->io_dead = false;
	vctrl->iothread_task = NULL;
	WRITE_ONCE(vctrl->io_idle, true);

	vtsk = vhost_task_create(nvme_mdev_io_thread_fn,
				 nvme_mdev_io_thread_killed, ctx,
				 "nvme-mdev-io");
	if (IS_ERR(vtsk)) {
		kfree(ctx);
		return PTR_ERR(vtsk);
	}

	vctrl->io_ctx = ctx;
	vctrl->iothread = vtsk;
	/* it goes to sleep right away: io_idle is set */
	vhost_task_start(vtsk);
	return 0;
}

/* End the main IO thread */
void nvme_mdev_io_free(struct nvme_mdev_vctrl *vctrl)
{
	_DBG(vctrl, "IO: destroying the polling iothread\n");

	lockdep_assert_held(&vctrl->lock);
	if (!vctrl->iothread)
		return;

	/* let it finish its host I/O first */
	nvme_mdev_io_pause(vctrl);
	/* (no completion is waking it up any more, see io_completed()) */
	spin_lock_irq(&vctrl->wake_lock);
	spin_unlock_irq(&vctrl->wake_lock);
	vhost_task_stop(vctrl->iothread);

	vctrl->iothread = NULL;
	vctrl->iothread_task = NULL;
	vctrl->iothread_parked = false;
	kfree(vctrl->io_ctx);
	vctrl->io_ctx = NULL;
}

/*
 * The I/O thread must not be working: paused, idle (it doesn't touch
 * anything while io_idle is set, and that is only cleared under the lock),
 * or gone.
 */
void nvme_mdev_assert_io_not_running(struct nvme_mdev_vctrl *vctrl)
{
	if (WARN_ON(vctrl->iothread && !nvme_mdev_io_is_self(vctrl) &&
		    !vctrl->iothread_parked && !READ_ONCE(vctrl->io_idle) &&
		    !READ_ONCE(vctrl->io_dead)))
		nvme_mdev_io_pause(vctrl);
}
