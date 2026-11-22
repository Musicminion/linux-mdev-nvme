// SPDX-License-Identifier: GPL-2.0+
/*
 * NVMe mediated device: the host side
 * Copyright (c) 2019 - Maxim Levitsky
 *
 * The virtual controllers send their I/O through the block layer, as
 * polled bios, to whatever block devices their namespaces are backed by;
 * nothing here depends on the NVMe host driver.  All instances hang off
 * one parent device, /sys/class/nvme_mdev/nvme_mdev.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include <linux/mdev.h>
#include <linux/module.h>
#include <linux/log2.h>
#include "priv.h"
#include "trace.h"

/* ------------------------------------------------------------------ */
/* the parent device and the instance type */

static const struct class nvme_mdev_class = {
	.name	= "nvme_mdev",
};

static void nvme_mdev_parent_release(struct device *dev)
{
}

static struct device nvme_mdev_parent = {
	.class		= &nvme_mdev_class,
	.release	= nvme_mdev_parent_release,
};

static struct nvme_mdev_hctrl *nvme_mdev_host;

ssize_t nvme_mdev_type_show_description(struct mdev_type *mtype, char *buf)
{
	return sysfs_emit(buf,
			  "NVMe virtual controller, I/O through the block layer\n");
}

static void nvme_mdev_hctrl_free(struct kref *ref)
{
	kfree(container_of(ref, struct nvme_mdev_hctrl, ref));
}

void nvme_mdev_hctrl_get(struct nvme_mdev_hctrl *hctrl)
{
	kref_get(&hctrl->ref);
}

void nvme_mdev_hctrl_put(struct nvme_mdev_hctrl *hctrl)
{
	kref_put(&hctrl->ref, nvme_mdev_hctrl_free);
}

/* Check if IO passthrough is supported for given IO optcode */
bool nvme_mdev_hctrl_hq_check_op(struct nvme_mdev_hctrl *hctrl, u8 optcode)
{
	switch (optcode) {
	case nvme_cmd_flush:
	case nvme_cmd_read:
	case nvme_cmd_write:
		/* these are mandatory*/
		return true;
	case nvme_cmd_write_zeroes:
		return (hctrl->oncs & NVME_CTRL_ONCS_WRITE_ZEROES);
	case nvme_cmd_dsm:
		return (hctrl->oncs & NVME_CTRL_ONCS_DSM);
	default:
		return false;
	}
}

/* Allocate a 'host' queue - here the queues are virtual*/
int nvme_mdev_hctrl_hq_alloc(struct nvme_mdev_hctrl *hctrl)
{
	int qid, ret;
	struct hw_mbio_queue *hwq;

	hwq = kzalloc_node(sizeof(*hwq), GFP_KERNEL, hctrl->node);
	if (!hwq)
		return -ENOMEM;

	INIT_LIST_HEAD(&hwq->bios_in_flight);

	ret = bioset_init(&hwq->bioset, MDEV_NVME_BIO_QUEUE_SIZE,
			  offsetof(struct mbio, bio), BIOSET_NEED_BVECS);
	if (ret < 0) {
		kfree(hwq);
		return ret;
	}

	/* the slots are shared by all virtual controllers */
	mutex_lock(&hctrl->lock);
	for (qid = 0 ; qid < MDEV_NVME_NUM_BIO_QUEUES ; qid++)
		if (!hctrl->hw_queues[qid])
			break;
	if (qid < MDEV_NVME_NUM_BIO_QUEUES)
		hctrl->hw_queues[qid] = hwq;
	mutex_unlock(&hctrl->lock);

	if (qid == MDEV_NVME_NUM_BIO_QUEUES) {
		bioset_exit(&hwq->bioset);
		kfree(hwq);
		return -ENOSPC;
	}
	return qid + 1;
}

/* Free a 'host' queue - here the queues are virtual*/
void nvme_mdev_hctrl_hq_free(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	struct hw_mbio_queue *hwq = hctrl->hw_queues[qid - 1];

	if (WARN_ON(!hwq))
		return;

	WARN_ON(!list_empty(&hwq->bios_in_flight));
	WARN_ON(hwq->inflight);

	mutex_lock(&hctrl->lock);
	hctrl->hw_queues[qid - 1] = NULL;
	mutex_unlock(&hctrl->lock);
	bioset_exit(&hwq->bioset);
	kfree(hwq);
}

/*
 * Check if the host queue has space for submission - also our limit
 * not related to the block layer
 */
bool nvme_mdev_hctrl_hq_can_submit(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	struct hw_mbio_queue *hwq = hctrl->hw_queues[qid - 1];

	if (WARN_ON(!hwq))
		return false;
	return hwq->inflight < MDEV_NVME_BIO_QUEUE_SIZE;
}

/*
 * Callback we get from the block layer.  Polled bios complete from
 * bio_poll() in our thread, others (flush, or a queue without poll
 * queues) from interrupt context.
 */
static struct workqueue_struct *nvme_mdev_wq;
static void nvme_mdev_hctrl_queue_work(struct mbio *mbio, u8 op);

static void nvme_mdev_hctrl_bio_done(struct bio *bio)
{
	struct mbio *mbio = container_of(bio, struct mbio, bio);
	struct nvme_mdev_vctrl *vctrl;
	u16 status;

	/*
	 * A polled bio (REQ_NOWAIT) had to wait: the host queue is frozen,
	 * has no free tag, or the bio has to be split.  This runs in our
	 * thread, from submit_bio(), which must not wait: it is the one that
	 * polls for the completions that end the wait (a controller reset
	 * waits for them before it unfreezes the queue).  Submit it again
	 * from the work item, without polling.  (Not polled from now on, so
	 * our thread won't look at the bio while the work item resets it.)
	 */
	if (bio->bi_status == BLK_STS_AGAIN && (bio->bi_opf & REQ_NOWAIT)) {
		mbio->polled = false;
		nvme_mdev_hctrl_queue_work(mbio, MBIO_WORK_RESUBMIT);
		return;
	}

	status = nvme_mdev_translate_error_block(bio->bi_status);

	/*
	 * this marks the bio as done for the polling thread, which may free
	 * it right away: vctrl first
	 */
	vctrl = mbio->vctrl;
	smp_store_release(&mbio->status, status);
	nvme_mdev_io_completed(vctrl);
}

/*
 * Completion of a discard / write zeroes chain built by the block layer
 * helpers; bi_private is the (never submitted) mbio tracking the command.
 */
static void nvme_mdev_hctrl_chain_done(struct bio *bio)
{
	struct mbio *mbio = bio->bi_private;
	struct nvme_mdev_vctrl *vctrl = mbio->vctrl;
	u16 status = nvme_mdev_translate_error_block(bio->bi_status);

	bio_put(bio);
	/* publishes the completion, pairs with the acquire in hq_poll() */
	smp_store_release(&mbio->status, status);
	nvme_mdev_io_completed(vctrl);
}

/*
 * DSM: deallocate the ranges as discards (the ranges were checked and
 * copied in by the translation); without the AD attribute, or if the
 * device can't discard, it is only a hint and succeeds as a no-op.
 */
static void nvme_mdev_hctrl_dsm(struct mbio *mbio, struct bio **chain)
{
	const struct nvme_dsm_range *range = mbio->ranges;
	unsigned int i;

	for (i = 0; i < mbio->nr_ranges; i++) {
		sector_t nr_sects = (sector_t)le32_to_cpu(range[i].nlb) <<
				    mbio->shift;

		if (nr_sects)
			__blkdev_issue_discard(mbio->bdev,
					       le64_to_cpu(range[i].slba) <<
					       mbio->shift,
					       nr_sects, GFP_NOIO, chain);
	}
}

/*
 * The work item: what our thread must not do because it may sleep in the
 * block layer (in submit_bio() while the host queue is frozen, waiting for
 * a free tag, or the discard and zeroing helpers, which submit as they
 * go).  Our thread may be the one that has to poll for the completions
 * that end the wait.
 */
static void nvme_mdev_hctrl_work(struct work_struct *w)
{
	struct mbio *mbio = container_of(w, struct mbio, work);
	struct nvme_mdev_vctrl *vctrl = mbio->vctrl;
	struct bio *bio = &mbio->bio;
	struct bio *chain = NULL;
	blk_opf_t opf;
	u16 status;
	int ret;

	switch (mbio->work_op) {
	case MBIO_WORK_RESUBMIT:
		/* (the bio's sector may be remapped: from the saved iter) */
		opf = bio->bi_opf & ~(REQ_POLLED | REQ_NOWAIT);
		bio_reset(bio, mbio->bdev, opf);
		bio->bi_vcnt = mbio->vcnt;
		bio->bi_iter = mbio->iter;
		bio->bi_end_io = nvme_mdev_hctrl_bio_done;
		break;
	case MBIO_WORK_FLUSH:
		bio_reset(bio, mbio->bdev, REQ_OP_WRITE | REQ_PREFLUSH);
		bio->bi_end_io = nvme_mdev_hctrl_bio_done;
		break;
	case MBIO_WORK_CHAIN:
		ret = 0;
		if (mbio->ranges) {
			nvme_mdev_hctrl_dsm(mbio, &chain);
			kfree(mbio->ranges);
			mbio->ranges = NULL;
		} else if (mbio->nr_sects) {
			ret = __blkdev_issue_zeroout(mbio->bdev, mbio->sector,
						     mbio->nr_sects, GFP_NOIO,
						     &chain, mbio->zero_flags);
		}
		if (chain && !ret) {
			chain->bi_private = mbio;
			chain->bi_end_io = nvme_mdev_hctrl_chain_done;
			submit_bio(chain);
			return;
		}
		/* nothing to do (a DSM hint only), or an error */
		status = ret ? nvme_mdev_translate_error(ret) : NVME_SC_SUCCESS;
		/* release: pairs with the acquire in nvme_mdev_hctrl_hq_poll() */
		smp_store_release(&mbio->status, status);
		nvme_mdev_io_completed(vctrl);
		return;
	}
	/* (it may complete and be freed right away: mbio is gone after) */
	submit_bio(bio);
}

static void nvme_mdev_hctrl_queue_work(struct mbio *mbio, u8 op)
{
	mbio->work_op = op;
	INIT_WORK(&mbio->work, nvme_mdev_hctrl_work);
	queue_work(nvme_mdev_wq, &mbio->work);
}

/* Submit a IO passthrough command */
int nvme_mdev_hctrl_hq_submit(struct nvme_mdev_hctrl *hctrl,
			      struct nvme_mdev_vns *vns,
			      u16 qid, u32 tag,
			      struct nvme_command *cmd,
			      struct nvme_ext_data_iter *datait)
{
	struct hw_mbio_queue *hwq = hctrl->hw_queues[qid - 1];
	struct bio *bio = NULL;
	struct request_queue *q;
	bool polled_q = false;
	struct mbio *mbio;
	bool premap = false;
	u8 opcode = cmd->common.opcode;
	blk_opf_t opf;
	int retval;

	if (WARN_ON(!hwq))
		return -EINVAL;
	if (WARN_ON(hwq->inflight >= MDEV_NVME_BIO_QUEUE_SIZE))
		return -EBUSY;

	/* read/write buffer processing */
	if (opcode == nvme_cmd_read || opcode == nvme_cmd_write) {
		unsigned long datalength =
			(le16_to_cpu(cmd->rw.length) + 1) << vns->blksize_shift;

		if (opcode == nvme_cmd_read) {
			opf = REQ_OP_READ;
		} else {
			opf = REQ_OP_WRITE | REQ_SYNC | REQ_IDLE;
			if (cmd->rw.control & cpu_to_le16(NVME_RW_FUA))
				opf |= REQ_FUA;
		}
		/*
		 * Completion is reaped with bio_poll(), on devices that can
		 * poll (e.g. NVMe with poll queues); on others it comes from
		 * interrupt context through ->bi_end_io, like for flushes.
		 * REQ_NOWAIT with it: our thread must not wait in submit_bio()
		 * (see nvme_mdev_hctrl_bio_done()).  Only a bio the block layer
		 * needn't split (it would fail it, as REQ_NOWAIT): e.g. with an
		 * IOMMU, NVMe takes 128K at most (dma_opt_mapping_size); a
		 * larger one goes to the work item as it is.
		 */
		q = bdev_get_queue(vns->host_part);
		polled_q = !nvme_mdev_io_irq_mode(vns->vctrl) &&
			   (q->limits.features & BLK_FEAT_POLL) &&
			   bdev_nowait(vns->host_part);
		if (polled_q &&
		    datalength <= queue_max_hw_sectors(q) << SECTOR_SHIFT &&
		    datait->count <= queue_max_segments(q))
			opf |= REQ_POLLED | REQ_NOWAIT;

		if (WARN_ON(datait->count > BIO_MAX_VECS))
			return -EINVAL;

		/* the guest's pages DMA pre-mapped for the host device? */
		premap = vns->premap && !vns->premap_off;
		bio = bio_alloc_bioset(vns->host_part, datait->count, opf,
				       GFP_KERNEL, &hwq->bioset);
		if (WARN_ON(!bio))
			return -ENOMEM;

		/* starting sector */
		bio->bi_iter.bi_sector = le64_to_cpu(cmd->rw.slba) <<
				(vns->blksize_shift - SECTOR_SHIFT);

		/* Data. First page might start at an offset, last be partial */
		while (datait->count) {
			struct page *page = pfn_to_page(PHYS_PFN(datait->physical));
			unsigned int offset = offset_in_page(datait->physical);
			unsigned int chunk = min_t(unsigned long,
						   PAGE_SIZE - offset, datalength);

			if (WARN_ON(datalength == 0))
				break;

			/* pre-mapped: one bio_vec per page, see bio_set_premap() */
			if (premap) {
				__bio_add_page(bio, page, chunk, offset);
			} else if (bio_add_page(bio, page, chunk, offset) !=
				   chunk) {
				WARN_ON(1);
				retval = -ENOMEM;
				goto error;
			}

			/* (the next PRP: the guest's, checked only now) */
			retval = datait->next(datait);
			if (retval)
				goto error;
			datalength -= chunk;
		}
		/* (a pin during the translation may have failed to add) */
		if (premap && !vns->premap_off &&
		    bio_set_premap(bio, vns->premap))
			trace_nvme_mdev_io_premapped(vns->vctrl, vns->nsid,
						     bio->bi_iter.bi_size);

	/* flush request */
	} else if (opcode == nvme_cmd_flush) {
		/* flushes go through the flush state machine: no polling */
		opf = REQ_OP_WRITE | REQ_PREFLUSH;
		bio = bio_alloc_bioset(vns->host_part, 0, opf, GFP_KERNEL,
				       &hwq->bioset);
		if (WARN_ON(!bio))
			return -ENOMEM;

	/* discard / write zeroes: chains built by the block layer helpers */
	} else if (opcode == nvme_cmd_dsm || opcode == nvme_cmd_write_zeroes) {
		unsigned int shift = vns->blksize_shift - SECTOR_SHIFT;

		/* tracks the command; never submitted itself (no bdev) */
		bio = bio_alloc_bioset(NULL, 0, 0, GFP_KERNEL, &hwq->bioset);
		if (WARN_ON(!bio)) {
			if (datait && datait->release)
				datait->release(datait);
			return -ENOMEM;
		}
		mbio = container_of(bio, struct mbio, bio);
		mbio->tag = tag;
		mbio->status = NVME_STATUS_PENDING;
		mbio->flush_after = false;
		mbio->bdev = vns->host_part;
		mbio->vctrl = vns->vctrl;
		mbio->polled = false;
		mbio->ranges = NULL;
		mbio->nr_ranges = 0;
		mbio->nr_sects = 0;
		mbio->shift = shift;

		/* (the ranges were checked and copied in by the translation) */
		if (opcode == nvme_cmd_dsm) {
			/*
			 * Without the AD attribute, or if the device can't
			 * discard, only a hint: a no-op
			 */
			if ((le32_to_cpu(cmd->dsm.attributes) & NVME_DSMGMT_AD) &&
			    bdev_max_discard_sectors(vns->host_part)) {
				mbio->nr_ranges = le32_to_cpu(cmd->dsm.nr) + 1;
				mbio->ranges = kmemdup(datait->kmem.data,
						       mbio->nr_ranges *
						       sizeof(*mbio->ranges),
						       GFP_KERNEL);
			}
			datait->release(datait);
			if (mbio->nr_ranges && !mbio->ranges) {
				retval = -ENOMEM;
				goto error;
			}
		} else {
			u16 control = le16_to_cpu(cmd->write_zeroes.control);

			mbio->sector = le64_to_cpu(cmd->write_zeroes.slba) <<
				       shift;
			mbio->nr_sects = (sector_t)(le16_to_cpu(
					cmd->write_zeroes.length) + 1) << shift;
			mbio->zero_flags = (control & NVME_WZ_DEAC) ?
					   0 : BLKDEV_ZERO_NOUNMAP;
			mbio->flush_after = control & NVME_RW_FUA;
		}

		list_add_tail(&mbio->link, &hwq->bios_in_flight);
		hwq->inflight++;

		if (mbio->ranges || mbio->nr_sects)
			nvme_mdev_hctrl_queue_work(mbio, MBIO_WORK_CHAIN);
		else
			/* nothing to do (DSM hint only) */
			mbio->status = NVME_SC_SUCCESS;
		return 0;
	} else {
		return -EINVAL;
	}

	bio->bi_end_io = nvme_mdev_hctrl_bio_done;

	/* setup our portion of the bio*/
	mbio = container_of(bio, struct mbio, bio);
	mbio->tag = tag;
	mbio->status = NVME_STATUS_PENDING;
	mbio->flush_after = false;
	mbio->bdev = vns->host_part;
	mbio->vctrl = vns->vctrl;
	mbio->polled = bio->bi_opf & REQ_POLLED;
	/* to submit it again (BLK_STS_AGAIN, see bio_done) */
	mbio->iter = bio->bi_iter;
	mbio->vcnt = bio->bi_vcnt;

	list_add_tail(&mbio->link, &hwq->bios_in_flight);
	hwq->inflight++;

	/*
	 * A flush may wait (see nvme_mdev_hctrl_work()), so may a read or
	 * write that isn't polled: from the work item on a queue we poll
	 * (elsewhere nothing depends on our polling: interrupt mode, or a
	 * device that can't poll).
	 */
	if (opcode == nvme_cmd_flush || (polled_q && !(opf & REQ_POLLED))) {
		nvme_mdev_hctrl_queue_work(mbio, MBIO_WORK_SUBMIT);
		return 0;
	}
	submit_bio(bio);
	return 0;
error:
	bio_put(bio);
	return retval;
}

/* Poll for completion of IO passthrough commands */
int nvme_mdev_hctrl_hq_poll(struct nvme_mdev_hctrl *hctrl,
			    u32 qid,
			    struct nvme_ext_cmd_result *results,
			    unsigned int max_len)
{
	struct hw_mbio_queue *hwq = hctrl->hw_queues[qid - 1];
	struct mbio *mbio, *tmp;
	int i = 0;

	if (!hwq->inflight)
		return -1;

	list_for_each_entry_safe(mbio, tmp, &hwq->bios_in_flight, link) {
		/*
		 * Acquire, pairs with the release in the completion handlers:
		 * the data of a completed read is visible once we see it.
		 */
		u16 status = smp_load_acquire(&mbio->status);

		/*
		 * (only polled bios: others have no poll cookie, or 0 after a
		 * bio_reset(), i.e. hardware queue 0, not a poll queue)
		 */
		if (status == NVME_STATUS_PENDING && mbio->polled) {
			bio_poll(&mbio->bio, NULL, BLK_POLL_ONESHOT);
			/* as above */
			status = smp_load_acquire(&mbio->status);
		}
		if (status == NVME_STATUS_PENDING)
			continue;

		/* Write Zeroes with FUA: make the zeroes durable first */
		if (mbio->flush_after && status == NVME_SC_SUCCESS) {
			mbio->flush_after = false;
			mbio->status = NVME_STATUS_PENDING;
			nvme_mdev_hctrl_queue_work(mbio, MBIO_WORK_FLUSH);
			continue;
		}

		results[i].tag = mbio->tag;
		results[i].status = status;

		hwq->inflight--;
		list_del(&mbio->link);
		bio_put(&mbio->bio);

		if (++i == max_len)
			break;
	}
	return i;
}

static int __init nvme_mdev_init(void)
{
	struct nvme_mdev_hctrl *hctrl;
	int ret;

	hctrl = kzalloc(sizeof(*hctrl), GFP_KERNEL);
	if (!hctrl)
		return -ENOMEM;

	kref_init(&hctrl->ref);
	mutex_init(&hctrl->lock);
	hctrl->node = NUMA_NO_NODE;
	/*
	 * 128 pages: stays within the usual max_segments (polled bios that
	 * the block layer has to split lose REQ_POLLED)
	 */
	hctrl->mdts = 7;
	/*
	 * ONCS is per controller, namespaces come and go: advertise both.
	 * DSM is a hint (a no-op on devices that can't discard); Write
	 * Zeroes falls back to writing zero pages where there is no native
	 * support, so it is correct for every block device.
	 */
	hctrl->oncs = NVME_CTRL_ONCS_DSM | NVME_CTRL_ONCS_WRITE_ZEROES;

	/*
	 * One instance type: every virtual controller takes the same
	 * resources (an I/O thread and a bio pool).  "V1" leaves room for
	 * other kinds later, e.g. controllers sharing an I/O thread.
	 */
	hctrl->type.max_hw_queues = MAX_HOST_QUEUES;
	strscpy(hctrl->type.name, "V1", sizeof(hctrl->type.name));
	hctrl->type.type.sysfs_name = hctrl->type.name;
	hctrl->type.type.pretty_name = "NVMe virtual controller";
	hctrl->type_ptrs[0] = &hctrl->type.type;

	/* (drained when we unload: no work item outlives the module) */
	nvme_mdev_wq = alloc_workqueue("nvme-mdev", WQ_UNBOUND | WQ_MEM_RECLAIM,
				       0);
	if (!nvme_mdev_wq) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = class_register(&nvme_mdev_class);
	if (ret)
		goto err_wq;

	dev_set_name(&nvme_mdev_parent, "nvme_mdev");
	ret = device_register(&nvme_mdev_parent);
	if (ret)
		goto err_class;

	ret = mdev_register_driver(&nvme_mdev_driver);
	if (ret)
		goto err_device;

	hctrl->parent_dev = &nvme_mdev_parent;
	ret = mdev_register_parent(&hctrl->mparent, &nvme_mdev_parent,
				   &nvme_mdev_driver, hctrl->type_ptrs, 1);
	if (ret)
		goto err_driver;

	nvme_mdev_host = hctrl;
	pr_info("nvme_mdev " NVME_MDEV_FIRMWARE_VERSION " loaded\n");
	return 0;

err_driver:
	mdev_unregister_driver(&nvme_mdev_driver);
err_device:
	device_unregister(&nvme_mdev_parent);
err_class:
	class_unregister(&nvme_mdev_class);
err_wq:
	destroy_workqueue(nvme_mdev_wq);
err_free:
	nvme_mdev_hctrl_put(hctrl);
	return ret;
}

static void __exit nvme_mdev_exit(void)
{
	struct nvme_mdev_hctrl *hctrl = nvme_mdev_host;

	WRITE_ONCE(hctrl->removing, true);
	/* removes all instances, waiting for their users to let go */
	mdev_unregister_parent(&hctrl->mparent);
	mdev_unregister_driver(&nvme_mdev_driver);
	device_unregister(&nvme_mdev_parent);
	class_unregister(&nvme_mdev_class);
	destroy_workqueue(nvme_mdev_wq);
	nvme_mdev_hctrl_put(hctrl);
	pr_info("nvme_mdev unloaded\n");
}

MODULE_AUTHOR("Maxim Levitsky <mlevitsk@redhat.com>");
MODULE_DESCRIPTION("NVMe mediated VFIO virtual controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(NVME_MDEV_FIRMWARE_VERSION);

module_init(nvme_mdev_init)
module_exit(nvme_mdev_exit)
