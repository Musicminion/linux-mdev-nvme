/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Tracepoints of the NVMe mediated device: the guest I/O commands as they
 * are passed to the host, their completion and the guest interrupts.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM nvme_mdev

#if !defined(_TRACE_NVME_MDEV_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NVME_MDEV_H

#include <linux/tracepoint.h>
#include "priv.h"

TRACE_EVENT(nvme_mdev_io_submit,
	TP_PROTO(struct nvme_mdev_vctrl *vctrl, u16 sqid,
		 const struct nvme_command *cmd),
	TP_ARGS(vctrl, sqid, cmd),
	TP_STRUCT__entry(
		__string(dev, dev_name(mdev_dev(vctrl->mdev)))
		__field(u16, sqid)
		__field(u16, cid)
		__field(u32, nsid)
		__field(u8, opcode)
		__field(u64, slba)
		__field(u16, length)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->sqid = sqid;
		__entry->cid = nvme_mdev_cmd_cid(cmd);
		__entry->nsid = le32_to_cpu(cmd->common.nsid);
		__entry->opcode = cmd->common.opcode;
		__entry->slba = le64_to_cpu(cmd->rw.slba);
		__entry->length = le16_to_cpu(cmd->rw.length);
	),
	TP_printk("%s sqid=%u cid=%u nsid=%u opcode=0x%x slba=%llu length=%u",
		  __get_str(dev), __entry->sqid, __entry->cid, __entry->nsid,
		  __entry->opcode, __entry->slba, __entry->length)
);

TRACE_EVENT(nvme_mdev_io_complete,
	TP_PROTO(struct nvme_mdev_vctrl *vctrl, u16 sqid, u16 cid, u16 status),
	TP_ARGS(vctrl, sqid, cid, status),
	TP_STRUCT__entry(
		__string(dev, dev_name(mdev_dev(vctrl->mdev)))
		__field(u16, sqid)
		__field(u16, cid)
		__field(u16, status)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->sqid = sqid;
		__entry->cid = cid;
		__entry->status = status;
	),
	TP_printk("%s sqid=%u cid=%u status=0x%x", __get_str(dev),
		  __entry->sqid, __entry->cid, __entry->status)
);

/* A guest read or write went to the host with DMA pre-mapped pages */
TRACE_EVENT(nvme_mdev_io_premapped,
	TP_PROTO(struct nvme_mdev_vctrl *vctrl, u32 nsid, u32 bytes),
	TP_ARGS(vctrl, nsid, bytes),
	TP_STRUCT__entry(
		__string(dev, dev_name(mdev_dev(vctrl->mdev)))
		__field(u32, nsid)
		__field(u32, bytes)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->nsid = nsid;
		__entry->bytes = bytes;
	),
	TP_printk("%s nsid=%u bytes=%u", __get_str(dev), __entry->nsid,
		  __entry->bytes)
);

TRACE_EVENT(nvme_mdev_irq,
	TP_PROTO(struct nvme_mdev_vctrl *vctrl, unsigned int vector),
	TP_ARGS(vctrl, vector),
	TP_STRUCT__entry(
		__string(dev, dev_name(mdev_dev(vctrl->mdev)))
		__field(unsigned int, vector)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->vector = vector;
	),
	TP_printk("%s vector=%u", __get_str(dev), __entry->vector)
);

#endif /* _TRACE_NVME_MDEV_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

/* This part must be outside protection */
#include <trace/define_trace.h>
