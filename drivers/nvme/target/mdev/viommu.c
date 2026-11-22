// SPDX-License-Identifier: GPL-2.0+
/*
 * Virtual IOMMU - mapping user memory to the real device
 * Copyright (c) 2019 - Maxim Levitsky
 *
 * How guest memory gets to the device:
 *
 * The VMM (QEMU) registers the guest RAM with VFIO (VFIO_IOMMU_MAP_DMA or
 * IOMMU_IOAS_MAP): "guest address X is user address Y".  For an emulated
 * IOMMU device like this one nothing is pinned at that point.  We pin the
 * pages we actually touch with vfio_pin_pages(), which also accounts them
 * against the user's locked memory limit (the block layer DMA maps them
 * per I/O).  When the VMM removes a mapping, VFIO calls our dma_unmap
 * callback and we must unpin everything in that range before returning.
 *
 * Pinning is done in naturally aligned chunks of NVME_MDEV_CHUNK_PAGES
 * (2MB).  A chunk is pinned with a single vfio_pin_pages() call if the
 * whole chunk is mapped, otherwise page by page.  Every unpin uses exactly
 * the range of the matching pin, as required by iommufd.
 *
 * (The 2019 version instead pinned all of guest memory up front from a
 * VFIO "map" notifier that never went upstream.)
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/mdev.h>
#include <linux/vmalloc.h>
#include <linux/nvme.h>
#include <linux/iommu.h>
#include "priv.h"

struct nvme_mdev_chunk {
	dma_addr_t iova;		/* first guest address of the chunk */
	bool whole;			/* pinned with one vfio_pin_pages() */
	DECLARE_BITMAP(pinned, NVME_MDEV_CHUNK_PAGES);
	struct page *pages[NVME_MDEV_CHUNK_PAGES];
};

#define CHUNK_PROT (IOMMU_READ | IOMMU_WRITE)

static inline unsigned long chunk_index(dma_addr_t iova)
{
	return iova >> NVME_MDEV_CHUNK_SHIFT;
}

static inline unsigned int chunk_page(dma_addr_t iova)
{
	return (iova >> PAGE_SHIFT) & (NVME_MDEV_CHUNK_PAGES - 1);
}

static inline dma_addr_t chunk_page_iova(struct nvme_mdev_chunk *chunk,
					 unsigned int i)
{
	return chunk->iova + ((dma_addr_t)i << PAGE_SHIFT);
}

/* Unpin everything of a chunk and free it */
static void nvme_mdev_chunk_free(struct nvme_mdev_viommu *viommu,
				 struct nvme_mdev_chunk *chunk)
{
	unsigned int i;

	if (chunk->whole)
		vfio_unpin_pages(viommu->vdev, chunk->iova,
				 NVME_MDEV_CHUNK_PAGES);
	else
		for_each_set_bit(i, chunk->pinned, NVME_MDEV_CHUNK_PAGES)
			vfio_unpin_pages(viommu->vdev, chunk_page_iova(chunk, i),
					 1);
	kvfree(chunk);
}

/* Try to pin the whole chunk with one call */
static void nvme_mdev_chunk_pin_whole(struct nvme_mdev_viommu *viommu,
				      struct nvme_mdev_chunk *chunk)
{
	int ret;

	ret = vfio_pin_pages(viommu->vdev, chunk->iova, NVME_MDEV_CHUNK_PAGES,
			     CHUNK_PROT, chunk->pages);
	if (ret != NVME_MDEV_CHUNK_PAGES) {
		/* not all of it is mapped: fall back to single pages */
		if (WARN_ON_ONCE(ret > 0))
			vfio_unpin_pages(viommu->vdev, chunk->iova, ret);
		return;
	}
	chunk->whole = true;
	/* publish the pages before the bits, see nvme_mdev_viommu_translate */
	smp_wmb();
	bitmap_fill(chunk->pinned, NVME_MDEV_CHUNK_PAGES);
}

/* Is (part of) [iova, iova + size) being unmapped by VFIO right now? */
static bool nvme_mdev_viommu_blocked(struct nvme_mdev_viommu *viommu,
				     dma_addr_t iova, u64 size)
{
	bool blocked;

	spin_lock(&viommu->unmap_lock);
	blocked = iova < viommu->unmap_end &&
		  iova + size > viommu->unmap_start;
	spin_unlock(&viommu->unmap_lock);
	return blocked;
}

/*
 * Slow path of the translation: pin the page that contains @iova.
 * No lock is held while calling into VFIO: its dma_unmap callback may run
 * concurrently and must never wait for us (VFIO lock ordering).
 */
static int nvme_mdev_viommu_pin(struct nvme_mdev_viommu *viommu,
				dma_addr_t iova)
{
	unsigned long idx = chunk_index(iova);
	unsigned int i = chunk_page(iova);
	struct nvme_mdev_chunk *chunk;
	int ret;

	/* never pin inside a range VFIO is currently unmapping */
	if (nvme_mdev_viommu_blocked(viommu, iova, PAGE_SIZE))
		return -EFAULT;

	chunk = xa_load(&viommu->chunks, idx);
	if (!chunk) {
		chunk = kvzalloc(sizeof(*chunk), GFP_KERNEL);
		if (!chunk)
			return -ENOMEM;
		chunk->iova = (dma_addr_t)idx << NVME_MDEV_CHUNK_SHIFT;

		if (!nvme_mdev_viommu_blocked(viommu, chunk->iova,
					      NVME_MDEV_CHUNK_PAGES << PAGE_SHIFT))
			nvme_mdev_chunk_pin_whole(viommu, chunk);

		ret = xa_err(xa_store(&viommu->chunks, idx, chunk,
				      GFP_KERNEL));
		if (ret) {
			nvme_mdev_chunk_free(viommu, chunk);
			return ret;
		}
	}

	if (test_bit(i, chunk->pinned))
		return 0;

	ret = vfio_pin_pages(viommu->vdev, chunk_page_iova(chunk, i), 1,
			     CHUNK_PROT, &chunk->pages[i]);
	if (ret != 1) {
		_DBG(viommu->vctrl, "vIOMMU: pin of IOVA %pad failed (%d)\n",
		     &iova, ret);
		return ret < 0 ? ret : -EFAULT;
	}

	/* the page before the bit, see nvme_mdev_viommu_translate() */
	smp_wmb();
	set_bit(i, chunk->pinned);
	return 0;
}

/* User memory init code*/
void nvme_mdev_viommu_init(struct nvme_mdev_viommu *viommu,
			   struct vfio_device *vdev)
{
	viommu->vdev = vdev;
	nvme_mdev_viommu_unblock_range(viommu);
	WARN_ON(!xa_empty(&viommu->chunks));
}

/* Drop all pins (device close) */
void nvme_mdev_viommu_reset(struct nvme_mdev_viommu *viommu)
{
	nvme_mdev_viommu_remove(viommu, 0, ~0ULL);
	nvme_mdev_viommu_unblock_range(viommu);
	WARN_ON(!xa_empty(&viommu->chunks));
}

/*
 * Removes a range of user memory.  The caller makes sure nobody is
 * translating concurrently (I/O thread parked, vctrl lock held).
 * Returns the number of chunks dropped.
 */
int nvme_mdev_viommu_remove(struct nvme_mdev_viommu *viommu,
			    dma_addr_t iova,
			    u64 size)
{
	dma_addr_t last = size ? iova + size - 1 : iova;
	struct nvme_mdev_chunk *chunk;
	unsigned long idx;
	int count = 0;

	if (last < iova)
		last = ~(dma_addr_t)0;

	xa_for_each_range(&viommu->chunks, idx, chunk,
			  chunk_index(iova), chunk_index(last)) {
		xa_erase(&viommu->chunks, idx);
		nvme_mdev_chunk_free(viommu, chunk);
		count++;
	}
	return count;
}

/* Stop pinning in [iova, iova + size) (merged with what is blocked) */
void nvme_mdev_viommu_block_range(struct nvme_mdev_viommu *viommu,
				  dma_addr_t iova, u64 size)
{
	dma_addr_t end = iova + size < iova ? ~(dma_addr_t)0 : iova + size;

	spin_lock(&viommu->unmap_lock);
	if (viommu->unmap_start == viommu->unmap_end) {
		viommu->unmap_start = iova;
		viommu->unmap_end = end;
	} else {
		viommu->unmap_start = min(viommu->unmap_start, iova);
		viommu->unmap_end = max(viommu->unmap_end, end);
	}
	spin_unlock(&viommu->unmap_lock);
}

void nvme_mdev_viommu_unblock_range(struct nvme_mdev_viommu *viommu)
{
	spin_lock(&viommu->unmap_lock);
	viommu->unmap_start = 0;
	viommu->unmap_end = 0;
	spin_unlock(&viommu->unmap_lock);
}

/* Translate an IOVA to a physical address and read device bus address */
int nvme_mdev_viommu_translate(struct nvme_mdev_viommu *viommu,
			       dma_addr_t iova,
			       dma_addr_t *physical,
			       dma_addr_t *host_iova)
{
	struct nvme_mdev_chunk *chunk;
	unsigned int i = chunk_page(iova);
	int ret;

	if (WARN_ON_ONCE(OFFSET_IN_PAGE(iova) != 0))
		return -EINVAL;

	chunk = xa_load(&viommu->chunks, chunk_index(iova));
	if (unlikely(!chunk || !test_bit(i, chunk->pinned))) {
		ret = nvme_mdev_viommu_pin(viommu, iova);
		if (ret)
			return ret;
		chunk = xa_load(&viommu->chunks, chunk_index(iova));
	}
	/* pairs with the smp_wmb() before setting the pinned bit */
	smp_rmb();

	*physical = page_to_phys(chunk->pages[i]);
	*host_iova = 0;		/* unused: the block layer maps the pages */
	return 0;
}

/* map an IOVA to kernel address space  */
int nvme_mdev_viommu_create_kmap(struct nvme_mdev_viommu *viommu,
				 dma_addr_t iova, struct page_map *page)
{
	dma_addr_t host_iova;
	phys_addr_t physical;
	struct page *new_page;
	int ret;

	page->iova = iova;

	ret = nvme_mdev_viommu_translate(viommu, iova, &physical, &host_iova);
	if (ret)
		return ret;

	new_page = pfn_to_page(PHYS_PFN(physical));

	/* long lived (shadow doorbells, PRP lists): 64-bit, no highmem */
	page->kmap = page_address(new_page);

	page->page = new_page;
	return 0;
}

/* update IOVA <-> kernel mapping. If fails, removes the previous mapping */
void nvme_mdev_viommu_update_kmap(struct nvme_mdev_viommu *viommu,
				  struct page_map *page)
{
	dma_addr_t host_iova;
	phys_addr_t physical;
	struct page *new_page;
	int ret;

	ret = nvme_mdev_viommu_translate(viommu, page->iova,
					 &physical, &host_iova);
	if (ret) {
		nvme_mdev_viommu_free_kmap(viommu, page);
		return;
	}

	new_page = pfn_to_page(PHYS_PFN(physical));
	if (new_page == page->page)
		return;

	nvme_mdev_viommu_free_kmap(viommu, page);

	page->kmap = page_address(new_page);
	page->page = new_page;
}

/* unmap an IOVA to kernel address space  */
void nvme_mdev_viommu_free_kmap(struct nvme_mdev_viommu *viommu,
				struct page_map *page)
{
	page->page = NULL;
	page->kmap = NULL;
}
