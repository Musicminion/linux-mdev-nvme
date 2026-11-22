/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NVME VFIO mediated driver - the guest data (PRP) iterator
 * Copyright (c) 2019 - Maxim Levitsky
 */

#ifndef _MDEV_NVME_MDEV_H
#define _MDEV_NVME_MDEV_H

#include <linux/kernel.h>
#include <linux/nvme.h>

struct nvme_mdev_viommu;

struct page_map {
	void *kmap;
	struct page *page;
	dma_addr_t iova;
	bool write;		/* the device writes it */
};

struct user_prplist {
	/* used by user data iterator*/
	struct page_map page;
	unsigned int index;	/* index of current entry */
};

struct nvme_ext_data_iter {
	/* private */
	struct nvme_mdev_viommu *viommu;
	bool write;		/* the device writes the data pages */
	union {
		const union nvme_data_ptr *dptr;
		struct user_prplist uprp;
	};

	/* user interface */
	u64		count;	/* number of data pages, yet to be covered */

	phys_addr_t	physical; /* iterator physical address value*/
	dma_addr_t	iova;	/* the guest's address of it */
	dma_addr_t	host_iova; /* iterator dma address value*/

	/* moves iterator to the next item */
	int (*next)(struct nvme_ext_data_iter *data_iter);

	/* if != NULL, user should call this when it done with data
	 * pointed by the iterator
	 */
	void (*release)(struct nvme_ext_data_iter *data_iter);
};

#endif
