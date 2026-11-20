// SPDX-License-Identifier: GPL-2.0+
/*
 * NVMe IO command translation and polling IO thread
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/kernel.h>
#include <linux/kthread.h>
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

	ret = nvme_mdev_udata_iter_set_dptr(&ctx->udatait, &in->dptr,
					    length << ctx->ns->blksize_shift);
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
	ret = nvme_mdev_udata_iter_set_dptr(&ctx->udatait, &in->dptr, size);
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
	ucid = le16_to_cpu(ctx->in->common.command_id);

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

	hsqcnt = nvme_mdev_vctrl_hqs_list(vctrl, hsqs);
	ctx->arb_burst = 1 << ctx->vctrl->arb_burst_shift;

	/* can't stop polling when shadow db not enabled */
	ctx->idle_timeout_ms = vctrl->mmio.shadow_db_en ?
			       READ_ONCE(vctrl->poll_timeout_ms) : 0;

	vctrl->now = ktime_get();
	ctx->last_admin_poll_time = vctrl->now;
	ctx->last_io_t = vctrl->now;

	/* main loop */
	while (!kthread_should_park()) {
		vctrl->now = ktime_get();

		/* check if we have to exit to support admin polling */
		if (!vctrl->mmio.shadow_db_supported)
			if (nvme_mdev_adm_needs_processing(ctx))
				break;

		/*
		 * process the submission queues; plugged, so that the block
		 * layer hands the batch to the driver at once (->queue_rqs)
		 */
		blk_start_plug(&plug);
		sqid = 1;
		for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
			for (i = 0 ; i < ctx->arb_burst ; i++)
				if (!nvme_mdev_io_process_sq(ctx, sqid))
					break;
		blk_finish_plug(&plug);

		/* process the completions from the guest*/
		cqid = 1;
		for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
			nvme_mdev_io_process_cq(ctx, cqid);

		/* process the completions from the hardware*/
		for (i = 0 ; i < hsqcnt ; i++)
			if (nvme_mdev_io_process_hwq(ctx, hsqs[i]) > 0)
				ctx->last_io_t = vctrl->now;

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

	/* Drain all the pending completion interrupts to the guest*/
	cqid = 1;
	for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
		if (nvme_mdev_vcq_flush(vctrl, cqid))
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

/* the main IO thread */
static int nvme_mdev_io_polling_thread(void *data)
{
	struct io_ctx ctx;

	if (kthread_should_stop())
		return 0;

	memset(&ctx, 0, sizeof(struct io_ctx));
	ctx.vctrl = (struct nvme_mdev_vctrl *)data;
	ctx.hctrl = ctx.vctrl->hctrl;
	nvme_mdev_udata_iter_setup(&ctx.vctrl->viommu, &ctx.udatait);

	_DBG(ctx.vctrl, "IO: iothread started\n");

	for (;;) {
		if (kthread_should_park()) {
			_DBG(ctx.vctrl, "IO: iothread parked\n");
			kthread_parkme();
		}

		if (kthread_should_stop())
			break;

		/*
		 * Idle: sleep until a doorbell write kicks us (or we are parked
		 * or stopped).  io_idle is only cleared under vctrl->lock.
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		if (READ_ONCE(ctx.vctrl->io_idle) && !kthread_should_park() &&
		    !kthread_should_stop()) {
			schedule();
			continue;
		}
		__set_current_state(TASK_RUNNING);

		nvme_mdev_io_polling_loop(&ctx);
	}

	_DBG(ctx.vctrl, "IO: iothread stopped\n");
	return 0;
}

/*
 * Undo nvme_mdev_io_pause(), and wake the thread up if it has work (it
 * goes back to sleep by itself if io_idle is still set).
 *
 * Only ever park/unpark the thread from other threads: kthread_park()
 * on itself leaves the "parked" completion signalled, and a later
 * kthread_park() from another thread then returns before the thread
 * actually parked (2019 parked itself when idle).  The I/O thread runs
 * nothing else while it is in here anyway.
 */
void nvme_mdev_io_resume(struct nvme_mdev_vctrl *vctrl)
{
	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->iothread || current == vctrl->iothread ||
	    vctrl->vctrl_paused)
		return;

	if (vctrl->iothread_parked) {
		vctrl->iothread_parked = false;
		kthread_unpark(vctrl->iothread);
	}
	if (!READ_ONCE(vctrl->io_idle))
		wake_up_process(vctrl->iothread);
}

/* Pause the IO thread (waits until it is parked) */
void nvme_mdev_io_pause(struct nvme_mdev_vctrl *vctrl)
{
	lockdep_assert_held(&vctrl->lock);

	if (!vctrl->iothread || current == vctrl->iothread ||
	    vctrl->iothread_parked)
		return;

	vctrl->iothread_parked = true;
	kthread_park(vctrl->iothread);
}

/* setup the main IO thread */
int nvme_mdev_io_create(struct nvme_mdev_vctrl *vctrl, unsigned int cpu)
{
	/*TODOLATER: IO: Better thread name*/
	char name[TASK_COMM_LEN];

	_DBG(vctrl, "IO: creating the polling iothread\n");

	if (WARN_ON(vctrl->iothread))
		return -EINVAL;

	snprintf(name, sizeof(name), "nvme%d_poll_io", vctrl->hctrl->id);

	vctrl->iothread_cpu = cpu;
	vctrl->iothread_parked = false;
	WRITE_ONCE(vctrl->io_idle, true);

	vctrl->iothread = kthread_create_on_node(nvme_mdev_io_polling_thread,
						 vctrl,
						 vctrl->hctrl->node,
						 name);
	if (IS_ERR(vctrl->iothread)) {
		int ret = PTR_ERR(vctrl->iothread);

		vctrl->iothread = NULL;
		return ret;
	}

	kthread_bind(vctrl->iothread, cpu);
	/* it goes to sleep right away: io_idle is set */
	wake_up_process(vctrl->iothread);
	return 0;
}

/* End the  main IO thread */
void nvme_mdev_io_free(struct nvme_mdev_vctrl *vctrl)
{
	int ret;

	_DBG(vctrl, "IO: destroying the polling iothread\n");

	lockdep_assert_held(&vctrl->lock);
	if (!vctrl->iothread)
		return;
	nvme_mdev_io_pause(vctrl);
	ret = kthread_stop(vctrl->iothread);
	WARN_ON(ret);
	vctrl->iothread = NULL;
}

/*
 * The I/O thread must not be working: parked, or idle (it doesn't touch
 * anything while io_idle is set, and that is only cleared under the lock).
 */
void nvme_mdev_assert_io_not_running(struct nvme_mdev_vctrl *vctrl)
{
	if (WARN_ON(vctrl->iothread && current != vctrl->iothread &&
		    !vctrl->iothread_parked && !READ_ONCE(vctrl->io_idle)))
		nvme_mdev_io_pause(vctrl);
}
