// SPDX-License-Identifier: GPL-2.0+
/*
 * User (guest) data access routines
 * Implementation of PRP iterator in user memory
 * Copyright (c) 2019 - Maxim Levitsky
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/mdev.h>
#include <linux/nvme.h>
#include "priv.h"

#define MAX_PRP ((PAGE_SIZE / sizeof(__le64)) - 1)

/* Setup up a new PRP iterator */
void nvme_mdev_udata_iter_setup(struct nvme_mdev_viommu *viommu,
				struct nvme_ext_data_iter *iter)
{
	iter->viommu = viommu;
	iter->write = false;
	iter->count = 0;
	iter->next = NULL;
	iter->release = NULL;
}

/* Load a new prp list into the iterator. Internal*/
static int nvme_mdev_udata_iter_load_prplist(struct nvme_ext_data_iter *iter,
					     dma_addr_t iova)
{
	dma_addr_t  data_iova;
	int ret;
	__le64 *map;

	/* map the prp list (the device only reads it) */
	ret = nvme_mdev_viommu_create_kmap(iter->viommu,
					   PAGE_ADDRESS(iova), false,
					   &iter->uprp.page);
	if (ret)
		return ret;

	iter->uprp.index = OFFSET_IN_PAGE(iova) / (sizeof(__le64));

	/* read its first entry and check its alignment */
	map = iter->uprp.page.kmap;
	data_iova = le64_to_cpu(map[iter->uprp.index]);

	if (OFFSET_IN_PAGE(data_iova) != 0) {
		nvme_mdev_viommu_free_kmap(iter->viommu, &iter->uprp.page);
		return -EINVAL;
	}

	/* translate the entry to complete the setup*/
	iter->iova = data_iova;
	ret =  nvme_mdev_viommu_translate(iter->viommu, data_iova, iter->write,
					  &iter->physical, &iter->host_iova);
	if (ret)
		nvme_mdev_viommu_free_kmap(iter->viommu, &iter->uprp.page);

	return ret;
}

/* ->next function when iterator points to prp list*/
static int nvme_mdev_udata_iter_next_prplist(struct nvme_ext_data_iter *iter)
{
	dma_addr_t iova;
	int ret;
	__le64 *map = iter->uprp.page.kmap;

	if (WARN_ON(iter->count <= 0))
		return 0;

	if (--iter->count == 0) {
		nvme_mdev_viommu_free_kmap(iter->viommu, &iter->uprp.page);
		return 0;
	}

	iter->uprp.index++;

	if (iter->uprp.index < MAX_PRP || iter->count == 1) {
		// advance over next pointer in current prp list
		// these pointers must be page aligned
		iova = le64_to_cpu(map[iter->uprp.index]);
		if (OFFSET_IN_PAGE(iova) != 0)
			return -EINVAL;

		iter->iova = iova;
		ret  = nvme_mdev_viommu_translate(iter->viommu, iova,
						  iter->write,
						  &iter->physical,
						  &iter->host_iova);
		if (ret)
			nvme_mdev_viommu_free_kmap(iter->viommu,
						   &iter->uprp.page);
		return ret;
	}

	/* switch to next prp list. it must be page aligned as well*/
	iova = le64_to_cpu(map[MAX_PRP]);

	if (OFFSET_IN_PAGE(iova) != 0)
		return -EINVAL;

	nvme_mdev_viommu_free_kmap(iter->viommu, &iter->uprp.page);
	return nvme_mdev_udata_iter_load_prplist(iter, iova);
}

/* ->next function when iterator points to user data pointer*/
static int nvme_mdev_udata_iter_next_dptr(struct nvme_ext_data_iter *iter)
{
	dma_addr_t  iova;

	if (WARN_ON(iter->count <= 0))
		return 0;

	if (--iter->count == 0)
		return 0;

	/* we will be called only once to deal with the second
	 * pointer in the data pointer
	 */
	iova = le64_to_cpu(iter->dptr->prp2);

	if (iter->count == 1) {
		/* only need to read one more entry, meaning
		 * the 2nd entry of the dptr.
		 * It must be page aligned
		 */
		if (OFFSET_IN_PAGE(iova) != 0)
			return -EINVAL;
		iter->iova = iova;
		return nvme_mdev_viommu_translate(iter->viommu, iova,
						  iter->write,
						  &iter->physical,
						  &iter->host_iova);
	} else {
		/*
		 * Second dptr entry is prp pointer, and it might not
		 * be page aligned (but QWORD aligned at least)
		 */
		if (iova & 0x7ULL)
			return -EINVAL;
		iter->next = nvme_mdev_udata_iter_next_prplist;
		return nvme_mdev_udata_iter_load_prplist(iter, iova);
	}
}

/*
 * Set prp list iterator to point to data pointer found in NVME command;
 * @write: the device writes the data (controller to host)
 */
int nvme_mdev_udata_iter_set_dptr(struct nvme_ext_data_iter *it,
				  const union nvme_data_ptr *dptr, u64 size,
				  bool write)
{
	int ret;
	u64 prp1 = le64_to_cpu(dptr->prp1);
	dma_addr_t iova = PAGE_ADDRESS(prp1);
	unsigned int page_offset = OFFSET_IN_PAGE(prp1);

	/* first dptr pointer must be at least DWORD aligned*/
	if (page_offset & 0x3)
		return -EINVAL;

	it->dptr = dptr;
	it->write = write;
	it->next = nvme_mdev_udata_iter_next_dptr;
	it->count = DIV_ROUND_UP_ULL(size + page_offset, PAGE_SIZE);

	ret = nvme_mdev_viommu_translate(it->viommu, iova, write,
					 &it->physical, &it->host_iova);
	if (ret)
		return ret;

	it->physical += page_offset;
	it->host_iova += page_offset;
	it->iova = prp1;
	return 0;
}

/* Set prp list iterator to point to prp list found in create queue command */
static int
nvme_mdev_udata_iter_set_queue_prplist(struct nvme_mdev_viommu *viommu,
				       struct nvme_ext_data_iter *iter,
				       dma_addr_t iova, unsigned int size,
				       bool write)
{
	if (iova & ~PAGE_MASK)
		return -EINVAL;

	nvme_mdev_udata_iter_setup(viommu, iter);
	iter->write = write;
	iter->count = DIV_ROUND_UP(size, PAGE_SIZE);
	iter->next = nvme_mdev_udata_iter_next_prplist;
	return nvme_mdev_udata_iter_load_prplist(iter, iova);
}

/* Map an SQ/CQ queue (contiguous in guest physical memory) */
static int nvme_mdev_queue_getpages_contiguous(struct nvme_mdev_viommu *viommu,
					       dma_addr_t iova, bool write,
					       struct page **pages,
					       unsigned int npages,
					       dma_addr_t *iovas)
{
	int ret;
	unsigned int i;

	dma_addr_t host_page_iova;
	phys_addr_t physical;

	for (i = 0 ; i < npages; i++) {
		ret = nvme_mdev_viommu_translate(viommu, iova + (PAGE_SIZE * i),
						 write, &physical,
						 &host_page_iova);
		if (ret)
			return ret;
		pages[i] = pfn_to_page(PHYS_PFN(physical));
		if (iovas)
			iovas[i] = iova + PAGE_SIZE * i;
	}
	return 0;
}

/* Map an SQ/CQ queue (non contiguous in guest physical memory) */
static int nvme_mdev_queue_getpages_prplist(struct nvme_mdev_viommu *viommu,
					    dma_addr_t iova, bool write,
					    struct page **pages,
					    unsigned int npages,
					    dma_addr_t *iovas)
{
	int ret, i = 0;
	struct nvme_ext_data_iter uprpit;

	ret = nvme_mdev_udata_iter_set_queue_prplist(viommu,
						     &uprpit, iova,
						     npages * PAGE_SIZE, write);
	if (ret)
		return ret;

	while (uprpit.count && i < npages) {
		if (iovas)
			iovas[i] = uprpit.iova;
		pages[i++] = pfn_to_page(PHYS_PFN(uprpit.physical));
		ret = uprpit.next(&uprpit);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * map a SQ/CQ queue to host physical memory; if it isn't contiguous,
 * @page_iovas (if not NULL) gets the IOVA of each page.  @write: the
 * device writes the queue (a CQ)
 */
void *nvme_mdev_udata_queue_vmap(struct nvme_mdev_viommu *viommu,
				 dma_addr_t iova,
				 unsigned int size,
				 bool cont, bool write,
				 dma_addr_t *page_iovas)
{
	int ret;
	unsigned int npages;
	void *map;
	struct page **pages;

	// queue must be page aligned
	if (OFFSET_IN_PAGE(iova) != 0)
		return ERR_PTR(-EINVAL);

	npages = DIV_ROUND_UP(size, PAGE_SIZE);
	pages = kcalloc(npages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	ret = cont ?
		nvme_mdev_queue_getpages_contiguous(viommu, iova, write, pages,
						    npages, page_iovas)
		: nvme_mdev_queue_getpages_prplist(viommu, iova, write, pages,
						   npages, page_iovas);

	if (ret) {
		map = ERR_PTR(ret);
		goto out;
	}

	map =  vmap(pages, npages, VM_MAP, PAGE_KERNEL);
	if (!map)
		map = ERR_PTR(-ENOMEM);
out:
	kfree(pages);
	return map;
}

/* ------------------------------------------------------------------ */
/* command data as a scatterlist of the guest's pages, for nvmet */

/* SGL data block descriptors per command, at most */
#define NVME_MDEV_MAX_SGL_DESCS		256

struct nvme_mdev_sgl_block {
	u64 addr;
	u32 len;
};

static int nvme_mdev_sgt_alloc(struct sg_table *sgt, unsigned int nents,
			       struct scatterlist *inline_sg,
			       unsigned int inline_nents)
{
	if (nents <= inline_nents) {
		sg_init_table(inline_sg, nents);
		sgt->sgl = inline_sg;
		sgt->nents = nents;
		sgt->orig_nents = nents;
		return 0;
	}
	return sg_alloc_table(sgt, nents, GFP_KERNEL);
}

void nvme_mdev_udata_unmap_sg(struct sg_table *sgt,
			      struct scatterlist *inline_sg)
{
	if (sgt->sgl && sgt->sgl != inline_sg)
		sg_free_table(sgt);
	sgt->sgl = NULL;
}

/* The pages of the data buffer a PRP pair (and list) describes */
static u16 nvme_mdev_udata_map_prps(struct nvme_mdev_viommu *viommu,
				    const union nvme_data_ptr *dptr, u32 len,
				    bool write, struct sg_table *sgt,
				    struct scatterlist *inline_sg,
				    unsigned int inline_nents,
				    struct nvme_mdev_iova_ranges **rangesp)
{
	struct nvme_mdev_iova_ranges *ranges = NULL;
	struct nvme_ext_data_iter it;
	struct scatterlist *sg;
	unsigned int nents, i;
	int ret;

	nvme_mdev_udata_iter_setup(viommu, &it);
	ret = nvme_mdev_udata_iter_set_dptr(&it, dptr, len, write);
	if (ret)
		return nvme_mdev_translate_error(ret);

	nents = it.count;
	if (rangesp) {
		ranges = nvme_mdev_iova_ranges_alloc(nents);
		if (!ranges)
			return NVME_SC_INTERNAL;
	}
	if (nvme_mdev_sgt_alloc(sgt, nents, inline_sg, inline_nents)) {
		kfree(ranges);
		return NVME_SC_INTERNAL;
	}

	for_each_sg(sgt->sgl, sg, nents, i) {
		unsigned int offset = OFFSET_IN_PAGE(it.physical);
		unsigned int seg = min_t(u32, len, PAGE_SIZE - offset);

		sg_set_page(sg, pfn_to_page(PHYS_PFN(it.physical)), seg,
			    offset);
		if (ranges)
			nvme_mdev_iova_ranges_add(ranges, it.iova, seg);
		len -= seg;

		ret = it.next(&it);
		if (ret) {
			nvme_mdev_udata_unmap_sg(sgt, inline_sg);
			kfree(ranges);
			return nvme_mdev_translate_error(ret);
		}
	}
	WARN_ON_ONCE(len);
	if (rangesp)
		*rangesp = ranges;
	return NVME_SC_SUCCESS;
}

/* Copy @len bytes of guest memory at @iova (an SGL segment; read only) */
static int nvme_mdev_udata_read(struct nvme_mdev_viommu *viommu,
				dma_addr_t iova, void *buf, unsigned int len)
{
	while (len) {
		unsigned int offset = OFFSET_IN_PAGE(iova);
		unsigned int chunk = min_t(unsigned int, len,
					   PAGE_SIZE - offset);
		dma_addr_t physical, host_iova;
		void *src;
		int ret;

		ret = nvme_mdev_viommu_translate(viommu, PAGE_ADDRESS(iova),
						 false, &physical, &host_iova);
		if (ret)
			return ret;

		src = kmap_local_page(pfn_to_page(PHYS_PFN(physical)));
		memcpy(buf, src + offset, chunk);
		kunmap_local(src);

		buf += chunk;
		iova += chunk;
		len -= chunk;
	}
	return 0;
}

static u16 nvme_mdev_sgl_add_block(struct nvme_mdev_sgl_block *blocks,
				   unsigned int *nr,
				   const struct nvme_sgl_desc *desc)
{
	u32 len = le32_to_cpu(desc->length);

	if (desc->type != (NVME_SGL_FMT_DATA_DESC << 4))
		return NVME_SC_SGL_INVALID_TYPE | NVME_STATUS_DNR;
	if (!len)
		return NVME_SC_SUCCESS;
	if (*nr == NVME_MDEV_MAX_SGL_DESCS)
		return NVME_SC_SGL_INVALID_COUNT | NVME_STATUS_DNR;

	blocks[*nr].addr = le64_to_cpu(desc->addr);
	blocks[*nr].len = len;
	(*nr)++;
	return NVME_SC_SUCCESS;
}

/*
 * The data blocks an SGL describes: one data block descriptor in the
 * command, or a chain of segments (each one's last descriptor may point
 * to the next, unless it is the last segment).  No bit buckets, no keyed
 * descriptors.
 */
static u16 nvme_mdev_udata_parse_sgl(struct nvme_mdev_viommu *viommu,
				     const struct nvme_sgl_desc *sgl,
				     struct nvme_mdev_sgl_block *blocks,
				     unsigned int *nr)
{
	struct nvme_sgl_desc seg = *sgl, *descs;
	u16 status = NVME_SC_SUCCESS;

	*nr = 0;
	if (seg.type == (NVME_SGL_FMT_DATA_DESC << 4))
		return nvme_mdev_sgl_add_block(blocks, nr, &seg);

	descs = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!descs)
		return NVME_SC_INTERNAL;

	for (;;) {
		u32 seglen = le32_to_cpu(seg.length);
		bool last = seg.type == (NVME_SGL_FMT_LAST_SEG_DESC << 4);
		unsigned int n, i;

		if (!last && seg.type != (NVME_SGL_FMT_SEG_DESC << 4)) {
			status = NVME_SC_SGL_INVALID_TYPE | NVME_STATUS_DNR;
			break;
		}
		if (!seglen || seglen % sizeof(*descs) || seglen > PAGE_SIZE) {
			status = NVME_SC_SGL_INVALID_DATA | NVME_STATUS_DNR;
			break;
		}
		n = seglen / sizeof(*descs);

		if (nvme_mdev_udata_read(viommu, le64_to_cpu(seg.addr), descs,
					 seglen)) {
			status = NVME_SC_DATA_XFER_ERROR | NVME_STATUS_DNR;
			break;
		}

		for (i = 0; i < n - 1 && !status; i++)
			status = nvme_mdev_sgl_add_block(blocks, nr, &descs[i]);
		if (status)
			break;

		/* the last one: data, or (if allowed) the next segment */
		if (descs[n - 1].type == (NVME_SGL_FMT_DATA_DESC << 4)) {
			status = nvme_mdev_sgl_add_block(blocks, nr,
							 &descs[n - 1]);
			break;
		}
		if (last) {
			status = NVME_SC_SGL_INVALID_LAST | NVME_STATUS_DNR;
			break;
		}
		seg = descs[n - 1];
	}

	kfree(descs);
	return status;
}

static u16 nvme_mdev_udata_map_sgl(struct nvme_mdev_viommu *viommu,
				   const struct nvme_sgl_desc *sgl, u32 len,
				   bool write, struct sg_table *sgt,
				   struct scatterlist *inline_sg,
				   unsigned int inline_nents,
				   struct nvme_mdev_iova_ranges **rangesp)
{
	struct nvme_mdev_iova_ranges *ranges = NULL;
	struct nvme_mdev_sgl_block *blocks;
	unsigned int nr, nents = 0, i;
	struct scatterlist *sg;
	u64 total = 0;
	u16 status;

	blocks = kvmalloc_array(NVME_MDEV_MAX_SGL_DESCS, sizeof(*blocks),
				GFP_KERNEL);
	if (!blocks)
		return NVME_SC_INTERNAL;

	status = nvme_mdev_udata_parse_sgl(viommu, sgl, blocks, &nr);
	if (status)
		goto out;

	for (i = 0; i < nr; i++) {
		total += blocks[i].len;
		nents += DIV_ROUND_UP(OFFSET_IN_PAGE(blocks[i].addr) +
				      blocks[i].len, PAGE_SIZE);
	}

	/* the blocks must describe exactly the data the command transfers */
	if (total != len) {
		status = NVME_SC_SGL_INVALID_DATA | NVME_STATUS_DNR;
		goto out;
	}

	if (rangesp) {
		ranges = nvme_mdev_iova_ranges_alloc(nents);
		if (!ranges) {
			status = NVME_SC_INTERNAL;
			goto out;
		}
	}
	if (nvme_mdev_sgt_alloc(sgt, nents, inline_sg, inline_nents)) {
		status = NVME_SC_INTERNAL;
		goto out;
	}

	/* one entry per guest page: they need not be contiguous on the host */
	sg = sgt->sgl;
	for (i = 0; i < nr; i++) {
		u64 addr = blocks[i].addr;
		u32 left = blocks[i].len;

		while (left) {
			unsigned int offset = OFFSET_IN_PAGE(addr);
			unsigned int seg = min_t(u32, left, PAGE_SIZE - offset);
			dma_addr_t physical, host_iova;
			int ret;

			ret = nvme_mdev_viommu_translate(viommu,
							 PAGE_ADDRESS(addr),
							 write, &physical,
							 &host_iova);
			if (ret) {
				nvme_mdev_udata_unmap_sg(sgt, inline_sg);
				status = NVME_SC_DATA_XFER_ERROR |
					 NVME_STATUS_DNR;
				goto out;
			}
			sg_set_page(sg, pfn_to_page(PHYS_PFN(physical)), seg,
				    offset);
			if (ranges)
				nvme_mdev_iova_ranges_add(ranges, addr, seg);
			sg = sg_next(sg);
			addr += seg;
			left -= seg;
		}
	}
out:
	kvfree(blocks);
	if (status)
		kfree(ranges);
	else if (rangesp)
		*rangesp = ranges;
	return status;
}

/*
 * Build the scatterlist of the guest pages that hold the @len bytes of a
 * command's data (PRPs or SGL data blocks).  The pages stay pinned until
 * the VFIO dma_unmap callback, which waits for the commands in flight
 * (nvme_mdev_io_pause()).  Returns an NVMe status.
 */
u16 nvme_mdev_udata_map_sg(struct nvme_mdev_viommu *viommu,
			   const struct nvme_command *cmd, u32 len,
			   struct sg_table *sgt, struct scatterlist *inline_sg,
			   unsigned int inline_nents,
			   struct nvme_mdev_iova_ranges **ranges)
{
	/* data transfer direction: opcode bit 1, controller to host */
	bool write = cmd->common.opcode & 2;

	/* @ranges: dirty logging, the guest memory ranges of the data */
	if (cmd->common.flags & NVME_CMD_SGL_ALL)
		return nvme_mdev_udata_map_sgl(viommu, &cmd->common.dptr.sgl,
					       len, write, sgt, inline_sg,
					       inline_nents, ranges);
	return nvme_mdev_udata_map_prps(viommu, &cmd->common.dptr, len, write,
					sgt, inline_sg, inline_nents, ranges);
}
