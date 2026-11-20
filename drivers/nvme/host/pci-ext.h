/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NVMe PCIe external IO queue interface.
 *
 * Lets an in-kernel client (nvme-mdev) own a few IO queue pairs of a PCIe
 * controller that are kept out of blk-mq.  The client builds complete NVMe
 * commands itself (PRPs and command ids included) and reaps the raw
 * completion entries; nvme-pci only creates the queues, rings the
 * doorbells and tells the client when the controller goes away and comes
 * back.  Nothing in the nvme core is involved.
 *
 * The queues are reserved with the nvme module parameter "ext_queues" and
 * created as polled queues after the default/read/poll queues.
 */
#ifndef _NVME_PCI_EXT_H
#define _NVME_PCI_EXT_H

#include <linux/notifier.h>
#include <linux/nvme.h>

struct nvme_ctrl;

/* notifier actions; the notifier data is the struct nvme_ctrl */
enum nvme_pci_ext_event {
	/*
	 * The controller finished probing or a reset.  The external queues
	 * are (again) usable and empty.  Also sent for every live controller
	 * when a notifier is registered.
	 */
	NVME_PCI_EXT_CTRL_LIVE,
	/*
	 * The controller is about to be disabled (reset, shutdown, error).
	 * Stop using the external queues; commands outstanding on them will
	 * never complete.  Called from the reset path: must not wait for I/O.
	 */
	NVME_PCI_EXT_CTRL_STOP,
	/* The controller is being removed: free all external queues now. */
	NVME_PCI_EXT_CTRL_REMOVE,
};

int nvme_pci_ext_register_notifier(struct notifier_block *nb);
void nvme_pci_ext_unregister_notifier(struct notifier_block *nb);

unsigned int nvme_pci_ext_queues_available(struct nvme_ctrl *ctrl);
int nvme_pci_ext_queue_alloc(struct nvme_ctrl *ctrl, u16 *qid, u32 *depth);
void nvme_pci_ext_queue_free(struct nvme_ctrl *ctrl, u16 qid);

/* Copy one command to the submission queue and ring the doorbell. */
int nvme_pci_ext_queue_submit(struct nvme_ctrl *ctrl, u16 qid,
			      struct nvme_command *cmd);
/* Reap up to @max completion entries.  Returns the number reaped. */
int nvme_pci_ext_queue_poll(struct nvme_ctrl *ctrl, u16 qid,
			    struct nvme_completion *cqes, unsigned int max);

#endif /* _NVME_PCI_EXT_H */
