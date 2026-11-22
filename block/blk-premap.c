// SPDX-License-Identifier: GPL-2.0-only
/*
 * DMA pre-mapping of pages for a block device.
 *
 * A submitter that does I/O to the same pages over and over (e.g. to the
 * memory of a guest, which stays pinned) maps them for DMA once, for the
 * device that does the DMA of the block device's requests, instead of the
 * driver mapping and unmapping them for each request.  With an IOMMU that
 * saves the IOMMU page table updates and IOTLB invalidations per I/O.
 *
 * The bios tagged with bio_set_premap() are mapped page by page from here
 * by blk_rq_dma_map_iter_start(), and not unmapped after the I/O.  The
 * submitter keeps their pages in the mapping until their I/O is done.
 *
 * Only for coherent DMA without bounce buffering: a mapping that lasts
 * can't sync or bounce data at each I/O.
 */
#include <linux/blk-mq.h>
#include <linux/blk-mq-dma.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/xarray.h>

struct blk_dma_premap {
	struct device	*dma_dev;
	/*
	 * PFN -> DMA address of the page >> PAGE_SHIFT << REFS_BITS | how
	 * many times it was added, as an xarray value: lookups are lock-less
	 */
	struct xarray	pages;
	struct mutex	lock;		/* adding and removing */
};

#define REFS_BITS	8
#define REFS_MASK	((1UL << REFS_BITS) - 1)

static unsigned long premap_entry(dma_addr_t dma, unsigned long refs)
{
	return ((unsigned long)(dma >> PAGE_SHIFT) << REFS_BITS) | refs;
}

static dma_addr_t premap_dma(unsigned long entry)
{
	return (dma_addr_t)(entry >> REFS_BITS) << PAGE_SHIFT;
}

/**
 * blk_dma_premap_alloc - set up DMA pre-mapping for a block device
 * @bdev: the block device
 *
 * Returns the (empty) mapping, or ERR_PTR(-EOPNOTSUPP) if the device's
 * driver doesn't support it (it must set the tag set's dma_dev and map
 * its requests with blk_rq_dma_map_iter_start()) or its DMA isn't
 * coherent.
 */
struct blk_dma_premap *blk_dma_premap_alloc(struct block_device *bdev)
{
	struct request_queue *q = bdev_get_queue(bdev);
	struct blk_dma_premap *pm;
	struct device *dma_dev;

	if (!queue_is_mq(q) || !q->tag_set || !q->tag_set->dma_dev)
		return ERR_PTR(-EOPNOTSUPP);
	dma_dev = q->tag_set->dma_dev;
	if (!dev_is_dma_coherent(dma_dev))
		return ERR_PTR(-EOPNOTSUPP);

	pm = kzalloc(sizeof(*pm), GFP_KERNEL);
	if (!pm)
		return ERR_PTR(-ENOMEM);
	pm->dma_dev = get_device(dma_dev);
	xa_init(&pm->pages);
	mutex_init(&pm->lock);
	return pm;
}
EXPORT_SYMBOL_GPL(blk_dma_premap_alloc);

static void blk_dma_premap_unmap(struct blk_dma_premap *pm, unsigned long entry)
{
	dma_unmap_page_attrs(pm->dma_dev, premap_dma(entry), PAGE_SIZE,
			     DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
}

/**
 * blk_dma_premap_free - unmap all the pages and free the mapping
 * @pm: the mapping (nothing in flight uses it any more)
 */
void blk_dma_premap_free(struct blk_dma_premap *pm)
{
	unsigned long pfn;
	void *entry;

	if (IS_ERR_OR_NULL(pm))
		return;
	xa_for_each(&pm->pages, pfn, entry)
		blk_dma_premap_unmap(pm, xa_to_value(entry));
	xa_destroy(&pm->pages);
	put_device(pm->dma_dev);
	mutex_destroy(&pm->lock);
	kfree(pm);
}
EXPORT_SYMBOL_GPL(blk_dma_premap_free);

static void __blk_dma_premap_remove(struct blk_dma_premap *pm,
		struct page *page)
{
	unsigned long pfn = page_to_pfn(page);
	unsigned long entry;
	void *e;

	e = xa_load(&pm->pages, pfn);
	if (WARN_ON_ONCE(!e))
		return;
	entry = xa_to_value(e);
	if ((entry & REFS_MASK) > 1) {
		xa_store(&pm->pages, pfn, xa_mk_value(entry - 1), GFP_KERNEL);
		return;
	}
	xa_erase(&pm->pages, pfn);
	blk_dma_premap_unmap(pm, entry);
}

/**
 * blk_dma_premap_add - map pages for DMA
 * @pm: the mapping
 * @pages: the pages (whole pages are mapped)
 * @nr: how many
 *
 * A page added several times stays mapped until it was removed as many
 * times.  Either all the pages are added, or none (an error is returned).
 */
int blk_dma_premap_add(struct blk_dma_premap *pm, struct page **pages,
		unsigned int nr)
{
	unsigned int i;
	int ret = 0;

	mutex_lock(&pm->lock);
	for (i = 0; i < nr; i++) {
		unsigned long pfn = page_to_pfn(pages[i]);
		unsigned long entry;
		dma_addr_t dma;
		void *e;

		e = xa_load(&pm->pages, pfn);
		if (e) {
			entry = xa_to_value(e);
			if ((entry & REFS_MASK) == REFS_MASK) {
				ret = -EOVERFLOW;
				break;
			}
			ret = xa_err(xa_store(&pm->pages, pfn,
					      xa_mk_value(entry + 1), GFP_KERNEL));
			if (ret)
				break;
			continue;
		}

		dma = dma_map_page_attrs(pm->dma_dev, pages[i], 0, PAGE_SIZE,
					 DMA_BIDIRECTIONAL,
					 DMA_ATTR_SKIP_CPU_SYNC);
		if (dma_mapping_error(pm->dma_dev, dma)) {
			ret = -ENOMEM;
			break;
		}
		/* a lasting mapping can't bounce or sync at each I/O */
		if (dma_need_sync(pm->dma_dev, dma) || offset_in_page(dma)) {
			dma_unmap_page_attrs(pm->dma_dev, dma, PAGE_SIZE,
					     DMA_BIDIRECTIONAL,
					     DMA_ATTR_SKIP_CPU_SYNC);
			ret = -EOPNOTSUPP;
			break;
		}
		ret = xa_err(xa_store(&pm->pages, pfn,
				      xa_mk_value(premap_entry(dma, 1)),
				      GFP_KERNEL));
		if (ret) {
			dma_unmap_page_attrs(pm->dma_dev, dma, PAGE_SIZE,
					     DMA_BIDIRECTIONAL,
					     DMA_ATTR_SKIP_CPU_SYNC);
			break;
		}
	}
	/* all or nothing */
	while (ret && i--)
		__blk_dma_premap_remove(pm, pages[i]);
	mutex_unlock(&pm->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(blk_dma_premap_add);

/**
 * blk_dma_premap_remove - unmap pages added with blk_dma_premap_add()
 * @pm: the mapping
 * @pages: the pages
 * @nr: how many
 */
void blk_dma_premap_remove(struct blk_dma_premap *pm, struct page **pages,
		unsigned int nr)
{
	unsigned int i;

	mutex_lock(&pm->lock);
	for (i = 0; i < nr; i++)
		__blk_dma_premap_remove(pm, pages[i]);
	mutex_unlock(&pm->lock);
}
EXPORT_SYMBOL_GPL(blk_dma_premap_remove);

/**
 * bio_set_premap - tag a bio whose data pages are pre-mapped
 * @bio: the bio, with all its pages in @pm (until it completes)
 * @pm: the mapping
 *
 * The pages are DMA mapped one by one, their DMA addresses needn't be
 * contiguous: the bio's segments must not cross pages, i.e. one bio_vec per
 * page (__bio_add_page()), so that the segments the block layer counts are
 * the DMA segments.  Returns false (and the bio is mapped as usual) if they
 * do.
 */
bool bio_set_premap(struct bio *bio, struct blk_dma_premap *pm)
{
	unsigned short i;

	for (i = 0; i < bio->bi_vcnt; i++)
		if (WARN_ON_ONCE(bio->bi_io_vec[i].bv_offset +
				 bio->bi_io_vec[i].bv_len > PAGE_SIZE))
			return false;
	bio->bi_premap = pm;
	/* one bio per request: their pages may be in different mappings */
	bio->bi_opf |= REQ_NOMERGE;
	return true;
}
EXPORT_SYMBOL_GPL(bio_set_premap);

/**
 * blk_dma_premap_lookup - the DMA address of a physical address
 * @pm: the mapping
 * @paddr: a physical address in one of its pages
 * @dma: its DMA address
 *
 * Returns false if the page isn't in the mapping.  Any context.
 */
bool blk_dma_premap_lookup(struct blk_dma_premap *pm, phys_addr_t paddr,
		dma_addr_t *dma)
{
	void *e = xa_load(&pm->pages, PHYS_PFN(paddr));

	if (!e)
		return false;
	*dma = premap_dma(xa_to_value(e)) + offset_in_page(paddr);
	return true;
}
EXPORT_SYMBOL_GPL(blk_dma_premap_lookup);
