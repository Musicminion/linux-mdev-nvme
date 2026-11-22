// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtual NVMe controller implementation
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mdev.h>
#include <linux/nvme.h>
#include "priv.h"

bool nvme_mdev_vctrl_is_dead(struct nvme_mdev_vctrl *vctrl)
{
	return (vctrl->mmio.csts & (NVME_CSTS_CFS | NVME_CSTS_SHST_MASK)) != 0;
}

/* Setup the controller guid and serial */
static void nvme_mdev_vctrl_init_id(struct nvme_mdev_vctrl *vctrl)
{
	const guid_t *guid = &vctrl->mdev->uuid;

	snprintf(vctrl->subnqn, sizeof(vctrl->subnqn),
		 "nqn.2014-08.org.nvmexpress:uuid:%pUl", guid->b);

	snprintf(vctrl->serial, sizeof(vctrl->serial), "%pUl", guid->b);
}

/* Change the status of support for shadow doorbell */
int nvme_mdev_vctrl_set_shadow_doorbell_supported(struct nvme_mdev_vctrl *vctrl,
						  bool enable)
{
	if (vctrl->inuse)
		return -EBUSY;
	vctrl->mmio.shadow_db_supported = enable;
	return 0;
}

/* Called when memory mapping are changed. Propagate this to all kmap users */
static void nvme_mdev_vctrl_viommu_update(struct nvme_mdev_vctrl *vctrl)
{
	u16 qid;

	lockdep_assert_held(&vctrl->lock);

	if (!(vctrl->mmio.csts & NVME_CSTS_RDY))
		return;

	/* update mappings for submission and completion queues */
	for_each_set_bit(qid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
		nvme_mdev_vsq_viommu_update(&vctrl->viommu, &vctrl->vsqs[qid]);

	for_each_set_bit(qid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
		nvme_mdev_vcq_viommu_update(&vctrl->viommu, &vctrl->vcqs[qid]);

	/* update mapping for the shadow doorbells */
	nvme_mdev_mmio_viommu_update(vctrl);
}

/*
 * Create a new virtual controller.  @vctrl is zeroed memory owned by the
 * vfio core (see nvme_mdev_ops_init()).
 */
int nvme_mdev_vctrl_create(struct nvme_mdev_vctrl *vctrl,
			   struct mdev_device *mdev,
			   struct nvme_mdev_hctrl *hctrl,
			   unsigned int max_host_queues)
{
	int ret;

	/* Basic init */
	vctrl->hctrl = hctrl;
	vctrl->mdev = mdev;
	vctrl->max_host_hw_queues = max_host_queues;
	vctrl->viommu.vctrl = vctrl;
	xa_init(&vctrl->viommu.chunks);
	spin_lock_init(&vctrl->viommu.unmap_lock);

	mutex_init(&vctrl->lock);
	init_waitqueue_head(&vctrl->io_pause_wq);
	spin_lock_init(&vctrl->wake_lock);
	INIT_LIST_HEAD(&vctrl->link);
	INIT_LIST_HEAD(&vctrl->host_hw_queues);
	INIT_WORK(&vctrl->ns_gc_work, nvme_mdev_vns_gc_work);
	nvme_mdev_vctrl_init_id(vctrl);

	/* default feature values*/
	vctrl->arb_burst_shift = 3;
	vctrl->mmio.shadow_db_supported = true;
	vctrl->poll_timeout_ms = NVME_MDEV_POLL_TIMEOUT_MS;

	ret = nvme_mdev_pci_create(vctrl);
	if (ret)
		return ret;

	ret = nvme_mdev_mmio_create(vctrl);
	if (ret)
		goto error2;

	nvme_mdev_irqs_setup(vctrl);

	nvme_mdev_hctrl_get(hctrl);
	vctrl->created = true;
	_INFO(vctrl, "device created using %d host queues\n", max_host_queues);
	return 0;
error2:
	nvme_mdev_pci_free(vctrl);
	return ret;
}

/*
 * Destroy a virtual controller.  Called when the last reference to the
 * vfio device goes away, i.e. after the user closed it.
 */
void nvme_mdev_vctrl_destroy(struct nvme_mdev_vctrl *vctrl)
{
	if (!vctrl->created)
		return;

	_INFO(vctrl, "device is destroying\n");

	mutex_lock(&nvme_mdev_vctrl_list_mutex);
	list_del_init(&vctrl->link);
	mutex_unlock(&nvme_mdev_vctrl_list_mutex);

	mutex_lock(&vctrl->lock);
	nvme_mdev_io_free(vctrl);
	nvme_mdev_vns_destroy_all(vctrl);
	mutex_unlock(&vctrl->lock);

	/* no more host block devices: nothing can queue the gc work now */
	cancel_work_sync(&vctrl->ns_gc_work);

	mutex_lock(&vctrl->lock);
	__nvme_mdev_vctrl_reset(vctrl, true);
	nvme_mdev_pci_free(vctrl);
	nvme_mdev_mmio_free(vctrl);
	mutex_unlock(&vctrl->lock);

	xa_destroy(&vctrl->viommu.chunks);
	nvme_mdev_hctrl_put(vctrl->hctrl);
	vctrl->created = false;
	_INFO(vctrl, "device is destroyed\n");
}

/* Suspends a running virtual controller
 * Called when host needs to regain full control of the device
 */
void nvme_mdev_vctrl_pause(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->lock);
	if (!vctrl->vctrl_paused) {
		_INFO(vctrl, "pausing the virtual controller\n");
		if (vctrl->mmio.csts & NVME_CSTS_RDY)
			nvme_mdev_io_pause(vctrl);
		vctrl->vctrl_paused = true;
	}
	mutex_unlock(&vctrl->lock);
}

/* Resumes a virtual controller
 * Called when host done with exclusive access and allows us
 * again to attach to the controller
 */
void nvme_mdev_vctrl_resume(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->lock);
	nvme_mdev_assert_io_not_running(vctrl);

	if (vctrl->vctrl_paused) {
		_INFO(vctrl, "resuming the virtual controller\n");

		/* must be cleared first: nvme_mdev_io_resume() checks it */
		vctrl->vctrl_paused = false;

		if (vctrl->mmio.csts & NVME_CSTS_RDY) {
			/* handle all pending admin commands*/
			nvme_mdev_adm_process_sq(vctrl);
			/* start the IO thread again if it was stopped or
			 * if we had doorbell writes during the pause
			 */
			nvme_mdev_io_resume(vctrl);
		}
	}
	mutex_unlock(&vctrl->lock);
}

/* Called when emulator opens the virtual device */
int nvme_mdev_vctrl_open(struct nvme_mdev_vctrl *vctrl)
{
	int ret = 0;

	mutex_lock(&vctrl->lock);

	if (vctrl->hctrl->removing) {
		ret = -ENODEV;
		goto out;
	}

	if (vctrl->inuse) {
		ret = -EBUSY;
		goto out;
	}

	_INFO(vctrl, "device is opened\n");

	nvme_mdev_viommu_init(&vctrl->viommu, &vctrl->vdev);

	/* in the context of the process opening the device, see there */
	ret = nvme_mdev_io_create(vctrl);
	if (ret)
		goto out;

	nvme_mdev_mmio_open(vctrl);
	vctrl->inuse = true;

out:
	mutex_unlock(&vctrl->lock);
	return ret;
}

/* Called when emulator closes the virtual device */
void nvme_mdev_vctrl_release(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->lock);
	/* drains its host I/O and stops it; the next user gets a new one */
	nvme_mdev_io_free(vctrl);

	/*
	 * Reset the controller to a clean state for a new user.  This deletes
	 * the queues (and their mappings of guest memory), so it has to come
	 * before the guest memory is unpinned below.
	 */
	__nvme_mdev_vctrl_reset(vctrl, false);
	nvme_mdev_irqs_reset(vctrl);

	/* Remove the guest DMA mappings - new user that will open the
	 * device might be a different guest
	 */
	nvme_mdev_viommu_reset(&vctrl->viommu);
	vctrl->inuse = false;
	mutex_unlock(&vctrl->lock);

	WARN_ON(!list_empty(&vctrl->host_hw_queues));

	_INFO(vctrl, "device is released\n");
}

/* Called each time the controller is reset (CC.EN <= 0 or VM level reset) */
void __nvme_mdev_vctrl_reset(struct nvme_mdev_vctrl *vctrl, bool pci_reset)
{
	lockdep_assert_held(&vctrl->lock);

	if ((vctrl->mmio.csts & NVME_CSTS_RDY) &&
	    !(vctrl->mmio.csts & NVME_CSTS_SHST_MASK)) {
		_DBG(vctrl, "unsafe reset (CSTS.RDY==1)\n");
		nvme_mdev_io_pause(vctrl);
		nvme_mdev_vctrl_disable(vctrl);
	}
	nvme_mdev_mmio_reset(vctrl, pci_reset);
}

/* setups initial admin queues and doorbells */
bool nvme_mdev_vctrl_enable(struct nvme_mdev_vctrl *vctrl,
			    dma_addr_t cqiova, dma_addr_t sqiova, u32 sizes)
{
	int ret;
	u16 cqentries, sqentries;

	nvme_mdev_assert_io_not_running(vctrl);

	lockdep_assert_held(&vctrl->lock);

	sqentries = (sizes & 0xFFFF) + 1;
	cqentries = (sizes >> 16) + 1;

	if (cqentries > 4096 || cqentries < 2)
		return false;
	if (sqentries > 4096 || sqentries < 2)
		return false;

	ret = nvme_mdev_mmio_enable_dbs(vctrl);
	if (ret)
		goto error0;

	ret = nvme_mdev_vcq_init(vctrl, 0, cqiova, true, cqentries, 0);
	if (ret)
		goto error1;

	ret = nvme_mdev_vsq_init(vctrl, 0, sqiova, true, sqentries, 0);
	if (ret)
		goto error2;

	nvme_mdev_events_init(vctrl);

	if (!vctrl->mmio.shadow_db_supported) {
		/* start polling right away to support admin queue */
		WRITE_ONCE(vctrl->io_idle, false);
		nvme_mdev_io_resume(vctrl);
	}

	return true;
error2:
	nvme_mdev_mmio_disable_dbs(vctrl);
error1:
	nvme_mdev_vcq_delete(vctrl, 0);
error0:
	return false;
}

/* destroy all io/admin queues on the controller  */
void nvme_mdev_vctrl_disable(struct nvme_mdev_vctrl *vctrl)
{
	u16 sqid, cqid;

	nvme_mdev_assert_io_not_running(vctrl);

	lockdep_assert_held(&vctrl->lock);

	nvme_mdev_events_reset(vctrl);
	nvme_mdev_vns_log_reset(vctrl);

	sqid = 1;
	for_each_set_bit_from(sqid, vctrl->vsq_en, MAX_VIRTUAL_QUEUES)
		nvme_mdev_vsq_delete(vctrl, sqid);

	cqid = 1;
	for_each_set_bit_from(cqid, vctrl->vcq_en, MAX_VIRTUAL_QUEUES)
		nvme_mdev_vcq_delete(vctrl, cqid);

	nvme_mdev_vsq_delete(vctrl, 0);
	nvme_mdev_vcq_delete(vctrl, 0);

	nvme_mdev_mmio_disable_dbs(vctrl);
	WRITE_ONCE(vctrl->io_idle, true);
}

/* External reset */
void nvme_mdev_vctrl_reset(struct nvme_mdev_vctrl *vctrl)
{
	mutex_lock(&vctrl->lock);
	_INFO(vctrl, "reset\n");
	__nvme_mdev_vctrl_reset(vctrl, true);
	mutex_unlock(&vctrl->lock);
}

/* Add IO region*/
void nvme_mdev_vctrl_add_region(struct nvme_mdev_vctrl *vctrl,
				unsigned int index, unsigned int size,
				region_access_fn access_fn)
{
	struct nvme_mdev_io_region *region = &vctrl->regions[index];

	region->size = size;
	region->rw = access_fn;
	region->mmap_ops = NULL;
}

/* Enable mmap window on an IO region */
void nvme_mdev_vctrl_region_set_mmap(struct nvme_mdev_vctrl *vctrl,
				     unsigned int index,
				     unsigned int offset,
				     unsigned int size,
				     const struct vm_operations_struct *ops)
{
	struct nvme_mdev_io_region *region = &vctrl->regions[index];

	region->mmap_area_start = offset;
	region->mmap_area_size = size;
	region->mmap_ops = ops;
}

/* Disable mmap window on an IO region */
void nvme_mdev_vctrl_region_disable_mmap(struct nvme_mdev_vctrl *vctrl,
					 unsigned int index)
{
	struct nvme_mdev_io_region *region = &vctrl->regions[index];

	region->mmap_area_start = 0;
	region->mmap_area_size = 0;
	region->mmap_ops = NULL;
}

/* Allocate a host IO queue */
int nvme_mdev_vctrl_hq_alloc(struct nvme_mdev_vctrl *vctrl)
{
	struct nvme_mdev_hq *hq = NULL, *tmp;
	int hwqcount = 0, ret;

	lockdep_assert_held(&vctrl->lock);

	nvme_mdev_assert_io_not_running(vctrl);

	list_for_each_entry(tmp, &vctrl->host_hw_queues, link) {
		if (!hq || tmp->usecount < hq->usecount)
			hq = tmp;
		hwqcount++;
	}

	if (hwqcount < vctrl->max_host_hw_queues) {
		ret = nvme_mdev_hctrl_hq_alloc(vctrl->hctrl);
		if (ret < 0)
			return ret;

		hq = kzalloc_node(sizeof(*hq), GFP_KERNEL, vctrl->hctrl->node);
		if (!hq) {
			nvme_mdev_hctrl_hq_free(vctrl->hctrl, ret);
			return -ENOMEM;
		}

		hq->hqid = ret;
		hq->usecount = 1;
		list_add_tail(&hq->link, &vctrl->host_hw_queues);
	} else {
		hq->usecount++;
	}
	return hq->hqid;
}

/* Free a host IO queue */
void nvme_mdev_vctrl_hq_free(struct nvme_mdev_vctrl *vctrl, u16 hqid)
{
	struct nvme_mdev_hq *hq;

	lockdep_assert_held(&vctrl->lock);
	nvme_mdev_assert_io_not_running(vctrl);

	list_for_each_entry(hq, &vctrl->host_hw_queues, link)
		if (hq->hqid == hqid) {
			if (--hq->usecount > 0)
				return;
			nvme_mdev_hctrl_hq_free(vctrl->hctrl, hq->hqid);
			list_del(&hq->link);
			kfree(hq);
			return;
		}
	WARN_ON(1);
}

/* get current list of host queues */
unsigned int nvme_mdev_vctrl_hqs_list(struct nvme_mdev_vctrl *vctrl, u16 *out)
{
	struct nvme_mdev_hq *q;
	unsigned int i = 0;

	list_for_each_entry(q, &vctrl->host_hw_queues, link) {
		out[i++] = q->hqid;
		if (WARN_ON(i > MAX_HOST_QUEUES))
			break;
	}
	return i;
}

/*
 * VFIO dma_unmap callback: drop our pins on [iova, iova + size) before
 * returning, as both VFIO backends require (type1 BUG()s if it has to call
 * us more than a few times for the same mapping).
 *
 * The I/O thread is parked first, which waits for the host I/O in flight
 * to complete: the device must not DMA into pages we unpin.  The range is
 * blocked while we re-translate the queues so that this doesn't pin it
 * again.
 *
 * Note: with the type1 backend this runs under its device_list_lock, and
 * type1 itself takes iommu->lock (in vfio_unpin_pages) inside it while
 * device registration nests them the other way round; lockdep reports
 * that for every mdev driver that pins pages.  iommufd calls us without
 * any lock held.
 */
void nvme_mdev_vctrl_viommu_unmap(struct nvme_mdev_vctrl *vctrl,
				  dma_addr_t iova, u64 size)
{
	struct nvme_mdev_viommu *viommu = &vctrl->viommu;

	mutex_lock(&vctrl->lock);
	nvme_mdev_viommu_block_range(viommu, iova, size);
	nvme_mdev_io_pause(vctrl);
	nvme_mdev_viommu_remove(viommu, iova, size);
	nvme_mdev_vctrl_viommu_update(vctrl);
	nvme_mdev_viommu_unblock_range(viommu);
	nvme_mdev_io_resume(vctrl);
	mutex_unlock(&vctrl->lock);
}
