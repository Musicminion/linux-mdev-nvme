// SPDX-License-Identifier: GPL-2.0+
/*
 * The polling IO thread
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
	struct nvme_mdev_vctrl *vctrl;

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

/* process a user submission queue: up to a burst of commands to nvmet */
static void nvme_mdev_io_process_sq(struct io_ctx *ctx, u16 sqid)
{
	unsigned int i;

	for (i = 0; i < ctx->arb_burst; i++)
		if (!nvme_mdev_nvmet_submit_io(ctx->vctrl, sqid))
			break;
}

/* process a user completion queue */
static void nvme_mdev_io_process_cq(struct io_ctx *ctx, u16 cqid)
{
	nvme_mdev_vcq_process(ctx->vctrl, cqid, true);
}

/* do polling till one of events stops it */
static void nvme_mdev_io_polling_loop(struct io_ctx *ctx)
{
	struct nvme_mdev_vctrl *vctrl = ctx->vctrl;
	struct blk_plug plug;
	u16 cqid, sqid;
	bool idle = false;
	ktime_t wait_start;

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
		 * process the submission queues; plugged, so that a block
		 * device backend gets the batch at once (->queue_rqs)
		 */
		blk_start_plug(&plug);
		sqid = 1;
		for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
			nvme_mdev_io_process_sq(ctx, sqid);
		blk_finish_plug(&plug);

		/* the commands nvmet completed */
		nvme_mdev_nvmet_poll_io(vctrl);
		if (nvme_mdev_nvmet_reap_io(vctrl))
			ctx->last_io_t = vctrl->now;

		/* process the completions from the guest*/
		cqid = 1;
		for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
			nvme_mdev_io_process_cq(ctx, cqid);

		/* Check if we need to stop polling */
		if (ctx->idle_timeout_ms && !atomic_read(&vctrl->io_inflight) &&
		    timeout(ctx->last_io_t, vctrl->now, ctx->idle_timeout_ms)) {
			idle = true;
			break;
		}
		cond_resched();
	}

	/*
	 * Wait for the commands in flight: the guest's pages they use must
	 * stay pinned, and their queues must stay, until they are done.
	 */
	wait_start = ktime_get();
	while (atomic_read(&vctrl->io_inflight)) {
		vctrl->now = ktime_get();
		nvme_mdev_nvmet_poll_io(vctrl);
		if (!nvme_mdev_nvmet_reap_io(vctrl)) {
			if (timeout(wait_start, vctrl->now,
				    NVME_MDEV_IO_TIMEOUT_MS)) {
				_WARN(vctrl, "IO: still waiting for %d commands\n",
				      atomic_read(&vctrl->io_inflight));
				wait_start = vctrl->now;
			}
			cond_resched();
		}
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
			/* sleep in nvme_mdev_io_thread_fn() until kicked */
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
