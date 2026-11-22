/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NVMe specification values used by the virtual controller that are not
 * (or not under these names) in <linux/nvme.h>.  Kept local so that the
 * mediated driver needs no change to the shared NVMe header.
 */
#ifndef _NVME_MDEV_SPEC_H
#define _NVME_MDEV_SPEC_H

#include <linux/nvme.h>

#define NVME_CSTS_SHST_SHIFT		2

/* Asynchronous event types (NVMe 1.3, Figure 146) */
enum nvme_async_event_type {
	NVME_AER_TYPE_ERROR		= NVME_AER_ERROR,
	NVME_AER_TYPE_SMART		= NVME_AER_SMART,
	NVME_AER_TYPE_NOTICE		= NVME_AER_NOTICE,
	NVME_AER_TYPE_MAX		= 7,
};

/* Asynchronous event information (NVMe 1.3, Figures 147 - 149) */
enum nvme_async_event {
	NVME_AER_ERROR_INVALID_DB_REG		= 0,
	NVME_AER_ERROR_INVALID_DB_VALUE		= 1,
	NVME_AER_ERROR_DIAG_FAILURE		= 2,
	NVME_AER_ERROR_PERSISTENT_INT_ERR	= 3,
	NVME_AER_ERROR_TRANSIENT_INT_ERR	= 4,
	NVME_AER_ERROR_FW_IMAGE_LOAD_ERR	= 5,

	NVME_AER_SMART_SUBSYS_RELIABILITY	= 0,
	NVME_AER_SMART_TEMP_THRESH		= 1,
	NVME_AER_SMART_SPARE_BELOW_THRESH	= 2,

	NVME_AER_INFO_NS_CHANGED		= NVME_AER_NOTICE_NS_CHANGED,
};

#define NVME_RW_PRINFO	(NVME_RW_PRINFO_PRCHK_REF | NVME_RW_PRINFO_PRCHK_APP | \
			 NVME_RW_PRINFO_PRCHK_GUARD | NVME_RW_PRINFO_PRACT)

#define NVME_SQ_PRIO_MASK		(3 << 1)

/* renamed upstream */
#define NVME_SC_PRP_OFFSET_INVALID	NVME_SC_PRP_INVALID_OFFSET
#define NVME_SC_DNR			NVME_STATUS_DNR

/* the 16 dwords of a submission queue entry */
static inline const u32 *nvme_mdev_cmd_dwords(const struct nvme_command *cmd)
{
	BUILD_BUG_ON(sizeof(*cmd) != 16 * sizeof(u32));
	return (const u32 *)cmd;
}

#endif /* _NVME_MDEV_SPEC_H */
