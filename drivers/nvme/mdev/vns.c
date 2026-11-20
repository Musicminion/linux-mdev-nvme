// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtual NVMe namespace implementation
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include "priv.h"

/* Reset the changed namespace log */
void nvme_mdev_vns_log_reset(struct nvme_mdev_vctrl *vctrl)
{
	vctrl->ns_log_size = 0;
}

/* This adds entry to NS changed log and sends to the user a notification */
static void nvme_mdev_vns_send_event(struct nvme_mdev_vctrl *vctrl, u32 ns)
{
	unsigned int i;
	unsigned int log_size = vctrl->ns_log_size;

	lockdep_assert_held(&vctrl->lock);

	_INFO(vctrl, "host namespace list rescanned\n");

	if (WARN_ON(ns == 0 || ns > MAX_VIRTUAL_NAMESPACES))
		return;

	// check if the namespace ID is alredy in the log
	if (log_size == MAX_VIRTUAL_NAMESPACES)
		return;

	for (i = 0; i < log_size; i++)
		if (vctrl->ns_log[i] == cpu_to_le32(ns))
			return;

	vctrl->ns_log[log_size++] = cpu_to_le32(ns);
	vctrl->ns_log_size++;
	nvme_mdev_event_send(vctrl, NVME_AER_TYPE_NOTICE,
			     NVME_AER_INFO_NS_CHANGED);
}

/* Read host NS/partition parameters to update our virtual NS */
static int nvme_mdev_vns_read_host_properties(struct nvme_mdev_vctrl *vctrl,
					      struct nvme_mdev_vns *vns,
					      unsigned int lba_shift)
{
	struct block_device *bdev = vns->host_part;
	unsigned int sector_to_lba_shift;
	u64 host_ns_size, start, nr, align_mask, chunk;

	lockdep_assert_held(&vctrl->lock);

	/* read the namespace block size */
	vns->blksize_shift = lba_shift;

	if (WARN_ON(vns->blksize_shift < SECTOR_SHIFT)) {
		_WARN(vctrl, "NS/create: device block size is bad\n");
		return -EINVAL;
	}

	sector_to_lba_shift = vns->blksize_shift - SECTOR_SHIFT;
	align_mask = (1ULL << sector_to_lba_shift) - 1;

	/* read the partition start and size*/
	start = bdev_is_partition(bdev) ? get_start_sect(bdev) : 0;
	nr = bdev_nr_sectors(bdev);

	/* check that partition is aligned on LBA size*/
	if ((start & align_mask) || (nr & align_mask)) {
		_WARN(vctrl, "NS/create: partition not aligned\n");
		return -EINVAL;
	}

	vns->ns_size = nr >> sector_to_lba_shift;
	host_ns_size = get_capacity(bdev->bd_disk) >> sector_to_lba_shift;

#ifdef CONFIG_NVME_MDEV_VFIO_HW_QUEUES
	/* we talk to the namespace directly: add the partition offset */
	vns->host_lba_offset = start >> sector_to_lba_shift;
#else
	/* the block layer remaps partitions for us */
	vns->host_lba_offset = 0;
#endif

	if (vns->ns_size == 0) {
		_WARN(vctrl, "NS/create: host namespace has size 0\n");
		return -EINVAL;
	}

	/* sanity check that partition doesn't extend beyond the namespace */
	if (!check_range(start >> sector_to_lba_shift, vns->ns_size,
			 host_ns_size)) {
		_WARN(vctrl, "NS/create: host namespace size mismatch\n");
		return -EINVAL;
	}

	/* check if namespace is readonly*/
	if (!vns->readonly)
		vns->readonly = bdev_read_only(bdev);

	/* optimal IO boundary: the host chunk size, in LBAs */
	chunk = bdev_get_queue(bdev)->limits.chunk_sectors >>
		sector_to_lba_shift;
	vns->noiob = chunk <= U16_MAX ? chunk : 0;
	if (vns->noiob != 0) {
		u64 tmp = start >> sector_to_lba_shift;

		if (do_div(tmp, vns->noiob)) {
			_WARN(vctrl,
			      "NS/create: host partition is not aligned on host optimum IO boundary, performance might suffer");
			vns->noiob = 0;
		}
	}
	return 0;
}

/*
 * The host block device is going away (namespace removed, controller
 * gone).  Called with bdev->bd_holder_lock held, which we must drop, and
 * possibly from deep inside del_gendisk(): only mark the namespace and
 * let the work item do the rest.
 */
static void nvme_mdev_vns_mark_dead(struct block_device *bdev, bool surprise)
{
	struct nvme_mdev_vns *vns = bdev->bd_holder;

	if (vns) {
		WRITE_ONCE(vns->dead, true);
		schedule_work(&vns->vctrl->ns_gc_work);
	}
	mutex_unlock(&bdev->bd_holder_lock);
}

static const struct blk_holder_ops nvme_mdev_vns_holder_ops = {
	.mark_dead = nvme_mdev_vns_mark_dead,
};

/* Open a host block device (e.g. /dev/nvme0n1p5) as a new namespace */
int nvme_mdev_vns_open(struct nvme_mdev_vctrl *vctrl, const char *path)
{
	struct nvme_mdev_vns *vns;
	unsigned int lba_shift;
	struct file *bdev_file;
	u32 user_nsid;
	int ret;

	_INFO(vctrl, "open host block device %s\n", path);

	mutex_lock(&vctrl->lock);
	ret = -ENODEV;
	if (nvme_mdev_vctrl_is_dead(vctrl))
		goto out;

	/* create the namespace object */
	ret = -ENOMEM;
	vns = kzalloc_node(sizeof(*vns), GFP_KERNEL, vctrl->hctrl->node);
	if (!vns)
		goto out;

	vns->vctrl = vctrl;
	uuid_gen(&vns->uuid); // TODOLATER: NS: non random NS UUID

	/* get exclusive access to the block device, read-only if need be */
	bdev_file = bdev_file_open_by_path(path, BLK_OPEN_READ | BLK_OPEN_WRITE,
					   vns, &nvme_mdev_vns_holder_ops);
	if (IS_ERR(bdev_file) &&
	    (PTR_ERR(bdev_file) == -EACCES || PTR_ERR(bdev_file) == -EROFS)) {
		vns->readonly = true;
		bdev_file = bdev_file_open_by_path(path, BLK_OPEN_READ, vns,
						   &nvme_mdev_vns_holder_ops);
	}
	if (IS_ERR(bdev_file)) {
		ret = PTR_ERR(bdev_file);
		goto error1;
	}
	vns->bdev_file = bdev_file;
	vns->host_part = file_bdev(bdev_file);

	/* must belong to the host controller we pass I/O to */
	ret = nvme_mdev_hctrl_check_bdev(vctrl->hctrl, vns->host_part,
					 &vns->host_nsid, &lba_shift);
	if (ret) {
		_WARN(vctrl, "NS/create: %s can't be used with this controller (%d)\n",
		      path, ret);
		goto error2;
	}

	/* read properties of the host namespace */
	ret = nvme_mdev_vns_read_host_properties(vctrl, vns, lba_shift);
	if (ret)
		goto error2;

	/* Allocate a user namespace ID for this namespace */
	ret = -ENOSPC;
	for (user_nsid = 1; user_nsid <= MAX_VIRTUAL_NAMESPACES; user_nsid++)
		if (!nvme_mdev_vns_from_vnsid(vctrl, user_nsid))
			break;

	if (user_nsid > MAX_VIRTUAL_NAMESPACES)
		goto error2;

	nvme_mdev_io_pause(vctrl);

	vctrl->namespaces[user_nsid - 1] = vns;
	vns->nsid = user_nsid;

	/* Announce the new namespace to the user */
	nvme_mdev_vns_send_event(vctrl, user_nsid);
	nvme_mdev_io_resume(vctrl);
	ret = 0;
	goto out;
error2:
	bdev_fput(vns->bdev_file);
error1:
	kfree(vns);
out:
	mutex_unlock(&vctrl->lock);
	return ret;
}

/* Destroy a virtual namespace*/
static int __nvme_mdev_vns_destroy(struct nvme_mdev_vctrl *vctrl, u32 user_nsid)
{
	struct nvme_mdev_vns *vns;

	lockdep_assert_held(&vctrl->lock);

	vns = nvme_mdev_vns_from_vnsid(vctrl, user_nsid);
	if (!vns)
		return -ENODEV;

	nvme_mdev_vns_send_event(vctrl, user_nsid);
	nvme_mdev_io_pause(vctrl);

	vctrl->namespaces[user_nsid - 1] = NULL;
	bdev_fput(vns->bdev_file);
	kfree(vns);
	nvme_mdev_io_resume(vctrl);
	return 0;
}

/* Destroy a virtual namespace (external interface) */
int nvme_mdev_vns_destroy(struct nvme_mdev_vctrl *vctrl, u32 user_nsid)
{
	int ret;

	mutex_lock(&vctrl->lock);
	nvme_mdev_io_pause(vctrl);
	ret = __nvme_mdev_vns_destroy(vctrl, user_nsid);
	nvme_mdev_io_resume(vctrl);
	mutex_unlock(&vctrl->lock);

	return ret;
}

/* Destroy all virtual namespaces */
void nvme_mdev_vns_destroy_all(struct nvme_mdev_vctrl *vctrl)
{
	u32 user_nsid;

	lockdep_assert_held(&vctrl->lock);

	for (user_nsid = 1 ; user_nsid <= MAX_VIRTUAL_NAMESPACES ; user_nsid++)
		__nvme_mdev_vns_destroy(vctrl, user_nsid);
}

/* Remove the namespaces whose host block device went away */
void nvme_mdev_vns_gc_work(struct work_struct *work)
{
	struct nvme_mdev_vctrl *vctrl =
		container_of(work, struct nvme_mdev_vctrl, ns_gc_work);
	u32 user_nsid;

	mutex_lock(&vctrl->lock);
	for (user_nsid = 1; user_nsid <= MAX_VIRTUAL_NAMESPACES; user_nsid++) {
		struct nvme_mdev_vns *vns =
			nvme_mdev_vns_from_vnsid(vctrl, user_nsid);

		if (vns && READ_ONCE(vns->dead)) {
			_INFO(vctrl, "host device of namespace %u is gone\n",
			      user_nsid);
			__nvme_mdev_vns_destroy(vctrl, user_nsid);
		}
	}
	mutex_unlock(&vctrl->lock);
}

/* Get a virtual namespace */
struct nvme_mdev_vns *nvme_mdev_vns_from_vnsid(struct nvme_mdev_vctrl *vctrl,
					       u32 user_ns_id)
{
	if (user_ns_id == 0 || user_ns_id > MAX_VIRTUAL_NAMESPACES)
		return NULL;
	return vctrl->namespaces[user_ns_id - 1];
}

/* Print description off all virtual namespaces */
int nvme_mdev_vns_print_description(struct nvme_mdev_vctrl *vctrl,
				    char *buf, unsigned int size)
{
	int nsid, ret = 0;

	mutex_lock(&vctrl->lock);

	for (nsid = 1; nsid <= MAX_VIRTUAL_NAMESPACES; nsid++) {
		int n;
		struct nvme_mdev_vns *vns = nvme_mdev_vns_from_vnsid(vctrl,
				nsid);
		if (!vns)
			continue;

		n = snprintf(buf, size, "VNS%d: %pg%s\n", nsid, vns->host_part,
			     vns->readonly ? " (ro)" : "");
		if (n >= size) {
			/* (the 2019 version returned here with the lock held) */
			ret = -ENOMEM;
			break;
		}
		buf += n;
		size -= n;
		ret += n;
	}
	mutex_unlock(&vctrl->lock);
	return ret;
}
