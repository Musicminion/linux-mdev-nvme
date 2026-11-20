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
static void nvme_mdev_hctrl_bio_done(struct bio *bio)
{
	struct mbio *mbio = container_of(bio, struct mbio, bio);
	u16 status;

	if (bio->bi_status == BLK_STS_AGAIN)
		status = NVME_SC_INTERNAL;	/* retryable */
	else
		status = nvme_mdev_translate_error_block(bio->bi_status);

	/* this marks the bio as done for the polling thread */
	smp_store_release(&mbio->status, status);
}

/*
 * Completion of a discard / write zeroes chain built by the block layer
 * helpers; bi_private is the (never submitted) mbio tracking the command.
 */
static void nvme_mdev_hctrl_chain_done(struct bio *bio)
{
	struct mbio *mbio = bio->bi_private;
	u16 status = nvme_mdev_translate_error_block(bio->bi_status);

	bio_put(bio);
	/* publishes the completion, pairs with the acquire in hq_poll() */
	smp_store_release(&mbio->status, status);
}

/*
 * DSM: deallocate the ranges as discards (the ranges were checked and
 * copied in by the translation); without the AD attribute, or if the
 * device can't discard, it is only a hint and succeeds as a no-op.
 */
static struct bio *nvme_mdev_hctrl_dsm(struct nvme_mdev_vns *vns,
				       struct nvme_command *cmd,
				       struct nvme_ext_data_iter *datait)
{
	const struct nvme_dsm_range *range = datait->kmem.data;
	unsigned int i, nr = le32_to_cpu(cmd->dsm.nr) + 1;
	unsigned int shift = vns->blksize_shift - SECTOR_SHIFT;
	struct bio *chain = NULL;

	if (!(le32_to_cpu(cmd->dsm.attributes) & NVME_DSMGMT_AD) ||
	    !bdev_max_discard_sectors(vns->host_part))
		return NULL;

	for (i = 0; i < nr; i++) {
		sector_t nr_sects = (sector_t)le32_to_cpu(range[i].nlb) << shift;

		if (nr_sects)
			__blkdev_issue_discard(vns->host_part,
					       le64_to_cpu(range[i].slba) << shift,
					       nr_sects, GFP_NOIO, &chain);
	}
	return chain;
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
	struct mbio *mbio;
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
		 * Completion is reaped with bio_poll().  No REQ_NOWAIT: while
		 * the host queue is frozen (e.g. during a controller reset) it
		 * fails every bio at once, and the guest runs out of retries
		 * before the reset is over.  Waiting here is fine, nothing
		 * completes during that time anyway.
		 */
		opf |= REQ_POLLED;

		if (WARN_ON(datait->count > BIO_MAX_VECS))
			return -EINVAL;

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

			if (bio_add_page(bio, page, chunk, offset) != chunk) {
				WARN_ON(1);
				retval = -ENOMEM;
				goto error;
			}

			retval = datait->next(datait);
			if (WARN_ON(retval))
				goto error;
			datalength -= chunk;
		}

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
		struct bio *chain = NULL;

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

		if (opcode == nvme_cmd_dsm) {
			chain = nvme_mdev_hctrl_dsm(vns, cmd, datait);
			datait->release(datait);
		} else {
			u16 control = le16_to_cpu(cmd->write_zeroes.control);
			sector_t sector, nr_sects;

			sector = le64_to_cpu(cmd->write_zeroes.slba) << shift;
			nr_sects = (sector_t)(le16_to_cpu(cmd->write_zeroes.length)
					      + 1) << shift;
			retval = __blkdev_issue_zeroout(vns->host_part, sector,
							nr_sects, GFP_NOIO, &chain,
							(control & NVME_WZ_DEAC) ?
							0 : BLKDEV_ZERO_NOUNMAP);
			if (retval)
				goto error;
			mbio->flush_after = control & NVME_RW_FUA;
		}

		list_add_tail(&mbio->link, &hwq->bios_in_flight);
		hwq->inflight++;

		if (chain) {
			chain->bi_private = mbio;
			chain->bi_end_io = nvme_mdev_hctrl_chain_done;
			submit_bio(chain);
		} else {
			/* nothing to do (DSM hint only) */
			mbio->status = NVME_SC_SUCCESS;
		}
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

	list_add_tail(&mbio->link, &hwq->bios_in_flight);
	hwq->inflight++;

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

		if (status == NVME_STATUS_PENDING) {
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
			bio_reset(&mbio->bio, mbio->bdev,
				  REQ_OP_WRITE | REQ_PREFLUSH);
			mbio->bio.bi_end_io = nvme_mdev_hctrl_bio_done;
			submit_bio(&mbio->bio);
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

	ret = class_register(&nvme_mdev_class);
	if (ret)
		goto err_free;

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
	nvme_mdev_hctrl_put(hctrl);
	pr_info("nvme_mdev unloaded\n");
}

MODULE_AUTHOR("Maxim Levitsky <mlevitsk@redhat.com>");
MODULE_DESCRIPTION("NVMe mediated VFIO virtual controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(NVME_MDEV_FIRMWARE_VERSION);

module_init(nvme_mdev_init)
module_exit(nvme_mdev_exit)
