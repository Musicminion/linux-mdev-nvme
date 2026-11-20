// SPDX-License-Identifier: GPL-2.0+
/*
 * NVMe parent (host) device abstraction
 * Copyright (c) 2019 - Maxim Levitsky
 *
 * A "host controller" (hctrl) is what the mediated devices hang off and
 * where their I/O ends up.  There are two backends:
 *
 *  - dedicated hardware queues: one hctrl per NVMe PCIe controller that
 *    reserved external queues (nvme.ext_queues=N).  Each virtual
 *    controller owns whole host queue pairs; we build the PRP lists and
 *    pick the command ids ourselves.  The only interface to the NVMe
 *    driver is drivers/nvme/host/pci-ext.h.
 *
 *  - block layer: a single hctrl hanging off a virtual "nvme_mdev"
 *    device; I/O is sent as polled bios to any block device.  Needs no
 *    support from the NVMe driver at all.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include <linux/mdev.h>
#include <linux/module.h>
#include <linux/dmapool.h>
#include <linux/log2.h>
#include "priv.h"

static LIST_HEAD(nvme_mdev_hctrl_list);
static DEFINE_MUTEX(nvme_mdev_hctrl_list_mutex);

unsigned int io_timeout_ms = 30000;
module_param_named(io_timeout, io_timeout_ms, uint, 0644);
MODULE_PARM_DESC(io_timeout,
		 "Maximum I/O command completion timeout (in msec)");

unsigned int poll_timeout_ms = 500;
module_param_named(poll_timeout, poll_timeout_ms, uint, 0644);
MODULE_PARM_DESC(poll_timeout,
		 "Maximum idle time to keep polling (in msec) (0 - poll forever)");

unsigned int admin_poll_rate_ms = 100;
module_param_named(admin_poll_rate, admin_poll_rate_ms, uint, 0644);
MODULE_PARM_DESC(admin_poll_rate,
		 "Admin queue polling rate (in msec) (used only when shadow doorbell is disabled)");

bool use_shadow_doorbell = true;
module_param(use_shadow_doorbell, bool, 0644);
MODULE_PARM_DESC(use_shadow_doorbell,
		 "Enable the shadow doorbell NVMe extension");

/* ------------------------------------------------------------------ */
/* mdev instance types: "nQ_V1" = virtual controller using n host queues */

static void nvme_mdev_hctrl_init_types(struct nvme_mdev_hctrl *hctrl)
{
	unsigned int i;

	for (i = 0; i < MAX_HOST_QUEUES; i++) {
		struct nvme_mdev_inst_type *type = &hctrl->types[i];

		snprintf(type->name, sizeof(type->name), "%uQ_V1", i + 1);
		type->max_hw_queues = i + 1;
		type->type.sysfs_name = type->name;
		type->type.pretty_name = type->name;
		hctrl->type_ptrs[i] = &type->type;
	}
}

static struct nvme_mdev_hctrl *mtype_to_hctrl(struct mdev_type *mtype)
{
	return container_of(mtype->parent, struct nvme_mdev_hctrl, mparent);
}

unsigned int nvme_mdev_type_get_available(struct mdev_type *mtype)
{
	struct nvme_mdev_inst_type *type =
		container_of(mtype, struct nvme_mdev_inst_type, type);

	return nvme_mdev_hctrl_hqs_available(mtype_to_hctrl(mtype)) /
		type->max_hw_queues;
}

ssize_t nvme_mdev_type_show_description(struct mdev_type *mtype, char *buf)
{
	struct nvme_mdev_inst_type *type =
		container_of(mtype, struct nvme_mdev_inst_type, type);

	return sysfs_emit(buf,
			  "MDEV nvme device, using maximum %u hw submission queues\n",
			  type->max_hw_queues);
}

/* ------------------------------------------------------------------ */
/* host controller life cycle */

static struct nvme_mdev_hctrl *nvme_mdev_hctrl_alloc(int node)
{
	struct nvme_mdev_hctrl *hctrl;

	hctrl = kzalloc_node(sizeof(*hctrl), GFP_KERNEL, node);
	if (!hctrl)
		return NULL;

	kref_init(&hctrl->ref);
	mutex_init(&hctrl->lock);
	INIT_LIST_HEAD(&hctrl->link);
	hctrl->node = node;
	nvme_mdev_hctrl_init_types(hctrl);
#ifdef CONFIG_NVME_MDEV_VFIO_HW_QUEUES
	xa_init(&hctrl->hw_queues);
#endif
	return hctrl;
}

static void nvme_mdev_hctrl_free(struct kref *ref)
{
	struct nvme_mdev_hctrl *hctrl =
		container_of(ref, struct nvme_mdev_hctrl, ref);

#ifdef CONFIG_NVME_MDEV_VFIO_HW_QUEUES
	WARN_ON(!xa_empty(&hctrl->hw_queues));
	xa_destroy(&hctrl->hw_queues);
	dma_pool_destroy(hctrl->prp_pool);
	if (hctrl->nvme_ctrl)
		nvme_put_ctrl(hctrl->nvme_ctrl);
#endif
	kfree(hctrl);
}

void nvme_mdev_hctrl_get(struct nvme_mdev_hctrl *hctrl)
{
	kref_get(&hctrl->ref);
}

void nvme_mdev_hctrl_put(struct nvme_mdev_hctrl *hctrl)
{
	kref_put(&hctrl->ref, nvme_mdev_hctrl_free);
}

/* Expose the instance types under @parent and make the hctrl visible */
static int nvme_mdev_hctrl_register(struct nvme_mdev_hctrl *hctrl,
				    struct device *parent)
{
	int ret;

	hctrl->parent_dev = parent;
	ret = mdev_register_parent(&hctrl->mparent, parent, &nvme_mdev_driver,
				   hctrl->type_ptrs, MAX_HOST_QUEUES);
	if (ret)
		return ret;

	mutex_lock(&nvme_mdev_hctrl_list_mutex);
	list_add_tail(&hctrl->link, &nvme_mdev_hctrl_list);
	mutex_unlock(&nvme_mdev_hctrl_list_mutex);

	dev_info(parent,
		 "mediated nvme support enabled, using up to %u host queues\n",
		 hctrl->nr_host_queues);
	return 0;
}

/*
 * Remove all mediated devices of this hctrl and drop the list reference.
 * mdev_unregister_parent() waits for the users to close their devices.
 */
static void nvme_mdev_hctrl_destroy(struct nvme_mdev_hctrl *hctrl)
{
	mutex_lock(&nvme_mdev_hctrl_list_mutex);
	list_del_init(&hctrl->link);
	mutex_unlock(&nvme_mdev_hctrl_list_mutex);

	WRITE_ONCE(hctrl->removing, true);
	mdev_unregister_parent(&hctrl->mparent);
	dev_info(hctrl->parent_dev, "mediated nvme support disabled\n");
	nvme_mdev_hctrl_put(hctrl);
}

/* Check how many host queues are still available */
int nvme_mdev_hctrl_hqs_available(struct nvme_mdev_hctrl *hctrl)
{
	int ret;

	mutex_lock(&hctrl->lock);
	ret =  hctrl->nr_host_queues;
	mutex_unlock(&hctrl->lock);
	return ret;
}

/* Reserve N host IO queues, for later allocation to a specific user*/
bool nvme_mdev_hctrl_hqs_reserve(struct nvme_mdev_hctrl *hctrl,
				 unsigned int n)
{
	mutex_lock(&hctrl->lock);

	if (n > hctrl->nr_host_queues) {
		mutex_unlock(&hctrl->lock);
		return false;
	}

	hctrl->nr_host_queues -= n;
	mutex_unlock(&hctrl->lock);
	return true;
}

/* Free N host IO queues, for allocation for other users*/
void nvme_mdev_hctrl_hqs_unreserve(struct nvme_mdev_hctrl *hctrl,
				   unsigned int n)
{
	mutex_lock(&hctrl->lock);
	hctrl->nr_host_queues += n;
	mutex_unlock(&hctrl->lock);
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

#ifdef CONFIG_NVME_MDEV_VFIO_HW_QUEUES
/* ------------------------------------------------------------------ */
/* backend: dedicated NVMe hardware queues */

MODULE_IMPORT_NS("NVME_PCI_EXT");

#define NVME_MDEV_POLL_BATCH	16
#define NVME_MDEV_PRPS_PER_LIST	(NVME_CTRL_PAGE_SIZE / sizeof(__le64))
/* PRP1 + full lists chained through their last entry */
#define NVME_MDEV_MAX_PRP_PAGES	\
	(1 + NVME_MDEV_MAX_PRP_LISTS * (NVME_MDEV_PRPS_PER_LIST - 1) + 1)

/*
 * Is @bdev a namespace (or a partition of one) of this controller?
 * Multipath head nodes are rejected: they have no tag set of their own.
 */
int nvme_mdev_hctrl_check_bdev(struct nvme_mdev_hctrl *hctrl,
			       struct block_device *bdev,
			       u32 *host_nsid, unsigned int *lba_shift)
{
	struct request_queue *q = bdev_get_queue(bdev);
	struct nvme_ns *ns;

	if (!q->tag_set || q->tag_set != hctrl->nvme_ctrl->tagset)
		return -EXDEV;

	ns = q->queuedata;
	if (ns->head->ms)
		return -EOPNOTSUPP;	/* no metadata support yet */

	*host_nsid = ns->head->ns_id;
	*lba_shift = ns->head->lba_shift;
	return 0;
}

static void nvme_mdev_hw_iod_release(struct nvme_mdev_hctrl *hctrl,
				     struct nvme_mdev_hw_iod *iod)
{
	unsigned int i;

	for (i = 0; i < iod->nr_lists; i++)
		dma_pool_free(hctrl->prp_pool, iod->lists[i], iod->list_dma[i]);
	iod->nr_lists = 0;

	if (iod->iter) {
		iod->iter->release(iod->iter);
		iod->iter = NULL;
	}
	iod->inuse = false;
}

/*
 * Fill in the data pointer of @cmd from the data iterator.
 *
 * PRP1 takes the first (possibly unaligned) page.  PRP2 is either the
 * second page, or points to a PRP list; when a list is full its last
 * entry is moved to a new list and replaced by the pointer to it.
 *
 * (The 2019 version advanced the list index twice per allocated list,
 * which broke every transfer that needed more than one PRP list page.)
 */
static int nvme_mdev_hw_setup_prps(struct nvme_mdev_hctrl *hctrl,
				   struct nvme_mdev_hw_iod *iod,
				   struct nvme_command *cmd,
				   struct nvme_ext_data_iter *iter)
{
	__le64 *list = NULL;
	unsigned int i = 0;
	int ret;

	iod->nr_lists = 0;
	iod->iter = (iter && iter->release) ? iter : NULL;
	cmd->common.dptr.prp1 = 0;
	cmd->common.dptr.prp2 = 0;

	if (!iter || !iter->count)
		return 0;

	cmd->common.dptr.prp1 = cpu_to_le64(iter->host_iova);
	ret = iter->next(iter);
	if (ret)
		goto error;
	if (!iter->count)
		return 0;

	if (iter->count == 1) {
		cmd->common.dptr.prp2 = cpu_to_le64(iter->host_iova);
		ret = iter->next(iter);
		if (ret)
			goto error;
		return 0;
	}

	while (iter->count) {
		if (!list || i == NVME_MDEV_PRPS_PER_LIST) {
			dma_addr_t dma;
			__le64 *new_list;

			if (iod->nr_lists == NVME_MDEV_MAX_PRP_LISTS) {
				ret = -ENOSPC;
				goto error;
			}

			new_list = dma_pool_alloc(hctrl->prp_pool, GFP_NOIO,
						  &dma);
			if (!new_list) {
				ret = -ENOMEM;
				goto error;
			}
			iod->lists[iod->nr_lists] = new_list;
			iod->list_dma[iod->nr_lists] = dma;
			iod->nr_lists++;

			if (!list) {
				cmd->common.dptr.prp2 = cpu_to_le64(dma);
				i = 0;
			} else {
				new_list[0] = list[i - 1];
				list[i - 1] = cpu_to_le64(dma);
				i = 1;
			}
			list = new_list;
		}

		list[i++] = cpu_to_le64(iter->host_iova);
		ret = iter->next(iter);
		if (ret)
			goto error;
	}
	return 0;
error:
	nvme_mdev_hw_iod_release(hctrl, iod);
	return ret;
}

/* Allocate a host IO queue */
int nvme_mdev_hctrl_hq_alloc(struct nvme_mdev_hctrl *hctrl)
{
	struct nvme_mdev_hw_queue *hq;
	u32 depth;
	u16 qid;
	int ret, i;

	ret = nvme_pci_ext_queue_alloc(hctrl->nvme_ctrl, &qid, &depth);
	if (ret)
		return ret;

	/* drop completions a previous owner of the queue may have left */
	for (i = 0; i < depth; i++) {
		struct nvme_completion stale[NVME_MDEV_POLL_BATCH];

		if (nvme_pci_ext_queue_poll(hctrl->nvme_ctrl, qid, stale,
					    NVME_MDEV_POLL_BATCH) <= 0)
			break;
	}

	ret = -ENOMEM;
	hq = kzalloc_node(sizeof(*hq), GFP_KERNEL, hctrl->node);
	if (!hq)
		goto err_free_queue;

	hq->iods = kvcalloc(depth, sizeof(*hq->iods), GFP_KERNEL);
	hq->free_cids = kvcalloc(depth, sizeof(*hq->free_cids), GFP_KERNEL);
	if (!hq->iods || !hq->free_cids)
		goto err_free_hq;

	hq->qid = qid;
	hq->depth = depth;
	for (i = depth - 1; i >= 0; i--)
		hq->free_cids[hq->nr_free++] = i;

	ret = xa_insert(&hctrl->hw_queues, qid, hq, GFP_KERNEL);
	if (ret)
		goto err_free_hq;
	return qid;

err_free_hq:
	kvfree(hq->free_cids);
	kvfree(hq->iods);
	kfree(hq);
err_free_queue:
	nvme_pci_ext_queue_free(hctrl->nvme_ctrl, qid);
	return ret;
}

/* Free an host IO queue */
void nvme_mdev_hctrl_hq_free(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	struct nvme_mdev_hw_queue *hq = xa_erase(&hctrl->hw_queues, qid);
	bool device_quiet;
	u16 cid;

	if (WARN_ON(!hq))
		return;

	/*
	 * Commands still outstanding at this point were not drained.  Their
	 * PRP lists may only be freed if the device can no longer touch them.
	 */
	device_quiet = hq->lost || READ_ONCE(hctrl->host_stopped);
	for (cid = 0; cid < hq->depth; cid++) {
		struct nvme_mdev_hw_iod *iod = &hq->iods[cid];

		if (!iod->inuse)
			continue;
		if (device_quiet)
			nvme_mdev_hw_iod_release(hctrl, iod);
		else
			WARN_ONCE(1, "nvme-mdev: leaking PRP lists of a busy queue\n");
	}

	nvme_pci_ext_queue_free(hctrl->nvme_ctrl, qid);
	kvfree(hq->free_cids);
	kvfree(hq->iods);
	kfree(hq);
}

/* Check if we can submit another IO passthrough command */
bool nvme_mdev_hctrl_hq_can_submit(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	struct nvme_mdev_hw_queue *hq = xa_load(&hctrl->hw_queues, qid);

	if (unlikely(!hq || hq->lost || READ_ONCE(hctrl->host_stopped)))
		return false;
	/* a queue of depth N can hold at most N - 1 commands */
	return hq->inflight < hq->depth - 1;
}

/* The host controller was reset: outstanding commands will never complete */
void nvme_mdev_hctrl_hq_mark_lost(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	struct nvme_mdev_hw_queue *hq = xa_load(&hctrl->hw_queues, qid);

	if (hq && hq->inflight)
		hq->lost = true;
}

/* Submit a IO passthrough command */
int nvme_mdev_hctrl_hq_submit(struct nvme_mdev_hctrl *hctrl,
			      struct nvme_mdev_vns *vns,
			      u16 qid, u32 tag,
			      struct nvme_command *cmd,
			      struct nvme_ext_data_iter *datait)
{
	struct nvme_mdev_hw_queue *hq = xa_load(&hctrl->hw_queues, qid);
	struct nvme_mdev_hw_iod *iod;
	u16 cid;
	int ret;

	if (WARN_ON_ONCE(!hq))
		return -EINVAL;
	if (unlikely(!hq->nr_free || hq->inflight >= hq->depth - 1)) {
		if (datait && datait->release)
			datait->release(datait);
		return -EBUSY;
	}

	cid = hq->free_cids[--hq->nr_free];
	iod = &hq->iods[cid];

	ret = nvme_mdev_hw_setup_prps(hctrl, iod, cmd, datait);
	if (ret)
		goto err_put_cid;

	cmd->common.command_id = cid;
	iod->tag = tag;
	iod->inuse = true;

	ret = nvme_pci_ext_queue_submit(hctrl->nvme_ctrl, qid, cmd);
	if (ret) {
		nvme_mdev_hw_iod_release(hctrl, iod);
		goto err_put_cid;
	}

	hq->inflight++;
	return 0;

err_put_cid:
	hq->free_cids[hq->nr_free++] = cid;
	return ret;
}

/*
 * Poll for completion of IO passthrough commands.
 * Returns the number of results, or -1 if nothing is outstanding.
 */
int nvme_mdev_hctrl_hq_poll(struct nvme_mdev_hctrl *hctrl,
			    u32 qid,
			    struct nvme_ext_cmd_result *results,
			    unsigned int max_len)
{
	struct nvme_mdev_hw_queue *hq = xa_load(&hctrl->hw_queues, qid);
	struct nvme_completion cqes[NVME_MDEV_POLL_BATCH];
	int i, n, done = 0;

	if (WARN_ON_ONCE(!hq))
		return -1;
	if (!hq->inflight)
		return -1;

	/* complete everything a host reset took away from us */
	if (unlikely(hq->lost)) {
		u16 cid;

		for (cid = 0; cid < hq->depth && done < max_len; cid++) {
			struct nvme_mdev_hw_iod *iod = &hq->iods[cid];

			if (!iod->inuse)
				continue;
			results[done].tag = iod->tag;
			results[done].status = NVME_SC_ABORT_REQ;
			done++;
			nvme_mdev_hw_iod_release(hctrl, iod);
			hq->free_cids[hq->nr_free++] = cid;
			hq->inflight--;
		}
		if (!hq->inflight)
			hq->lost = false;
		return done;
	}

	if (READ_ONCE(hctrl->host_stopped))
		return 0;

	n = nvme_pci_ext_queue_poll(hctrl->nvme_ctrl, qid, cqes,
				    min_t(unsigned int, max_len,
					  NVME_MDEV_POLL_BATCH));
	for (i = 0; i < n; i++) {
		u16 cid = cqes[i].command_id;
		struct nvme_mdev_hw_iod *iod;

		if (WARN_ON_ONCE(cid >= hq->depth || !hq->iods[cid].inuse))
			continue;

		iod = &hq->iods[cid];
		results[done].tag = iod->tag;
		results[done].status = le16_to_cpu(cqes[i].status) >> 1;
		done++;

		nvme_mdev_hw_iod_release(hctrl, iod);
		hq->free_cids[hq->nr_free++] = cid;
		hq->inflight--;
	}
	return done;
}

/*
 * Events from nvme-pci.  They are serialized by nvme_mdev_event_mutex,
 * except for the (possibly long) teardown on removal.
 */
static DEFINE_MUTEX(nvme_mdev_event_mutex);

/* Find the hctrl of a host controller, with a reference */
static struct nvme_mdev_hctrl *nvme_mdev_hctrl_find(struct nvme_ctrl *ctrl)
{
	struct nvme_mdev_hctrl *hctrl = NULL, *tmp;

	mutex_lock(&nvme_mdev_hctrl_list_mutex);
	list_for_each_entry(tmp, &nvme_mdev_hctrl_list, link) {
		if (tmp->nvme_ctrl == ctrl) {
			hctrl = tmp;
			nvme_mdev_hctrl_get(hctrl);
			break;
		}
	}
	mutex_unlock(&nvme_mdev_hctrl_list_mutex);
	return hctrl;
}

/* A controller with external queues showed up */
static void nvme_mdev_hctrl_add(struct nvme_ctrl *ctrl)
{
	struct nvme_mdev_hctrl *hctrl;
	unsigned int nr, max_pages;

	if (PAGE_SIZE != NVME_CTRL_PAGE_SIZE) {
		dev_info(ctrl->device, "no mdev support: page size mismatch\n");
		return;
	}

	nr = nvme_pci_ext_queues_available(ctrl);
	if (!nr)
		return;

	hctrl = nvme_mdev_hctrl_alloc(dev_to_node(ctrl->dev));
	if (!hctrl)
		return;

	nvme_get_ctrl(ctrl);
	hctrl->nvme_ctrl = ctrl;
	hctrl->id = ctrl->instance;
	/*
	 * A LIVE replayed at registration can be stale if a reset started
	 * meanwhile (its STOP found no hctrl yet): then the next LIVE must
	 * take the restart path.
	 */
	hctrl->host_stopped = nvme_ctrl_state(ctrl) != NVME_CTRL_LIVE;
	hctrl->nr_host_queues = nr;
	hctrl->oncs = ctrl->oncs &
		(NVME_CTRL_ONCS_DSM | NVME_CTRL_ONCS_WRITE_ZEROES);
	/* honour the host driver's quirks for broken devices */
	if (ctrl->quirks & NVME_QUIRK_DISABLE_WRITE_ZEROES)
		hctrl->oncs &= ~NVME_CTRL_ONCS_WRITE_ZEROES;

	max_pages = min_t(unsigned int,
			  ctrl->max_hw_sectors >> (PAGE_SHIFT - SECTOR_SHIFT),
			  NVME_MDEV_MAX_PRP_PAGES);
	hctrl->mdts = ilog2(rounddown_pow_of_two(max(max_pages, 1U)));

	hctrl->prp_pool = dma_pool_create("nvme-mdev-prp", ctrl->dev,
					  NVME_CTRL_PAGE_SIZE,
					  NVME_CTRL_PAGE_SIZE, 0);
	if (!hctrl->prp_pool)
		goto error;

	if (nvme_mdev_hctrl_register(hctrl, ctrl->dev))
		goto error;
	return;
error:
	dev_warn(ctrl->device, "failed to enable mediated nvme support\n");
	nvme_mdev_hctrl_put(hctrl);
}

/*
 * The host controller came back from a reset.  Park the I/O threads, mark
 * what was outstanding on the host queues as lost (it is completed to the
 * guest as aborted), and let the threads use the (empty) queues again.
 */
static void nvme_mdev_hctrl_restart(struct nvme_mdev_hctrl *hctrl)
{
	struct nvme_mdev_vctrl *vctrl;

	mutex_lock(&nvme_mdev_vctrl_list_mutex);
	list_for_each_entry(vctrl, &nvme_mdev_vctrl_list, link)
		if (vctrl->hctrl == hctrl)
			nvme_mdev_vctrl_pause(vctrl);
	list_for_each_entry(vctrl, &nvme_mdev_vctrl_list, link)
		if (vctrl->hctrl == hctrl)
			nvme_mdev_vctrl_host_restart(vctrl);

	WRITE_ONCE(hctrl->host_stopped, false);

	list_for_each_entry(vctrl, &nvme_mdev_vctrl_list, link)
		if (vctrl->hctrl == hctrl)
			nvme_mdev_vctrl_resume(vctrl);
	mutex_unlock(&nvme_mdev_vctrl_list_mutex);
}

static void nvme_mdev_ctrl_live(struct nvme_ctrl *ctrl)
{
	struct nvme_mdev_hctrl *hctrl;

	mutex_lock(&nvme_mdev_event_mutex);
	hctrl = nvme_mdev_hctrl_find(ctrl);
	if (!hctrl) {
		nvme_mdev_hctrl_add(ctrl);
	} else {
		if (READ_ONCE(hctrl->host_stopped))
			nvme_mdev_hctrl_restart(hctrl);
		nvme_mdev_hctrl_put(hctrl);
	}
	mutex_unlock(&nvme_mdev_event_mutex);
}

static void nvme_mdev_ctrl_stop(struct nvme_ctrl *ctrl)
{
	struct nvme_mdev_hctrl *hctrl;

	mutex_lock(&nvme_mdev_event_mutex);
	hctrl = nvme_mdev_hctrl_find(ctrl);
	if (hctrl) {
		/*
		 * The I/O threads stop submitting and polling as soon as they
		 * see this.  nvme-pci itself guards the queue memory
		 * (NVMEQ_ENABLED under the queue locks), so no need to wait.
		 */
		WRITE_ONCE(hctrl->host_stopped, true);
		nvme_mdev_hctrl_put(hctrl);
	}
	mutex_unlock(&nvme_mdev_event_mutex);
}

static void nvme_mdev_ctrl_remove(struct nvme_ctrl *ctrl)
{
	struct nvme_mdev_hctrl *hctrl;

	mutex_lock(&nvme_mdev_event_mutex);
	hctrl = nvme_mdev_hctrl_find(ctrl);
	mutex_unlock(&nvme_mdev_event_mutex);

	if (hctrl) {
		/* waits for the users of the mediated devices to let go */
		nvme_mdev_hctrl_destroy(hctrl);
		nvme_mdev_hctrl_put(hctrl);
	}
}

static int nvme_mdev_pci_event(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct nvme_ctrl *ctrl = data;

	switch (event) {
	case NVME_PCI_EXT_CTRL_LIVE:
		nvme_mdev_ctrl_live(ctrl);
		break;
	case NVME_PCI_EXT_CTRL_STOP:
		nvme_mdev_ctrl_stop(ctrl);
		break;
	case NVME_PCI_EXT_CTRL_REMOVE:
		nvme_mdev_ctrl_remove(ctrl);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block nvme_mdev_pci_nb = {
	.notifier_call = nvme_mdev_pci_event,
};

static int nvme_mdev_backend_init(void)
{
	return nvme_pci_ext_register_notifier(&nvme_mdev_pci_nb);
}

static void nvme_mdev_backend_exit(void)
{
	struct nvme_mdev_hctrl *hctrl;

	/* waits for running callbacks, no new ones after this */
	nvme_pci_ext_unregister_notifier(&nvme_mdev_pci_nb);

	for (;;) {
		mutex_lock(&nvme_mdev_hctrl_list_mutex);
		hctrl = list_first_entry_or_null(&nvme_mdev_hctrl_list,
						 struct nvme_mdev_hctrl, link);
		mutex_unlock(&nvme_mdev_hctrl_list_mutex);
		if (!hctrl)
			break;
		nvme_mdev_hctrl_destroy(hctrl);
	}
}

#else /* CONFIG_NVME_MDEV_VFIO_GENERIC_IO */
/* ------------------------------------------------------------------ */
/* backend: block layer, polled bios */

static struct device *nvme_mdev_root_dev;
static struct nvme_mdev_hctrl *nvme_mdev_block_hctrl;

int nvme_mdev_hctrl_check_bdev(struct nvme_mdev_hctrl *hctrl,
			       struct block_device *bdev,
			       u32 *host_nsid, unsigned int *lba_shift)
{
	*host_nsid = 0;
	*lba_shift = ilog2(bdev_logical_block_size(bdev));
	return 0;
}

/* Allocate a 'host' queue - here the queues are virtual*/
int nvme_mdev_hctrl_hq_alloc(struct nvme_mdev_hctrl *hctrl)
{
	int qid, ret;
	struct hw_mbio_queue *hwq;

	for (qid = 0 ; qid < MDEV_NVME_NUM_BIO_QUEUES ; qid++)
		if (!hctrl->hw_queues[qid])
			break;

	if (qid == MDEV_NVME_NUM_BIO_QUEUES)
		return -ENOSPC;

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

	hctrl->hw_queues[qid] = hwq;
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

	hctrl->hw_queues[qid - 1] = NULL;
	bioset_exit(&hwq->bioset);
	kfree(hwq);
}

void nvme_mdev_hctrl_hq_mark_lost(struct nvme_mdev_hctrl *hctrl, u16 qid)
{
	/* the block layer handles host controller resets itself */
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
		/* completion is reaped with bio_poll() */
		opf |= REQ_POLLED | REQ_NOWAIT;

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

			retval = __blkdev_issue_zeroout(vns->host_part,
				le64_to_cpu(cmd->write_zeroes.slba) << shift,
				(sector_t)(le16_to_cpu(cmd->write_zeroes.length) + 1)
					<< shift,
				GFP_NOIO, &chain,
				(control & NVME_WZ_DEAC) ? 0 : BLKDEV_ZERO_NOUNMAP);
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
		u16 status = smp_load_acquire(&mbio->status);

		if (status == NVME_STATUS_PENDING) {
			bio_poll(&mbio->bio, NULL, BLK_POLL_ONESHOT);
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

static int nvme_mdev_backend_init(void)
{
	struct nvme_mdev_hctrl *hctrl;
	int ret;

	nvme_mdev_root_dev = root_device_register("nvme_mdev");
	if (IS_ERR(nvme_mdev_root_dev))
		return PTR_ERR(nvme_mdev_root_dev);

	ret = -ENOMEM;
	hctrl = nvme_mdev_hctrl_alloc(NUMA_NO_NODE);
	if (!hctrl)
		goto err_root;

	/*
	 * 128 pages: stays within the usual max_segments (polled bios that
	 * the block layer has to split lose REQ_POLLED)
	 */
	hctrl->mdts = 7;
	hctrl->nr_host_queues = MDEV_NVME_NUM_BIO_QUEUES;
	/*
	 * ONCS is per controller, namespaces come and go: advertise both.
	 * DSM is a hint (a no-op on devices that can't discard); Write
	 * Zeroes falls back to writing zero pages where there is no native
	 * support, so it is correct for every block device.
	 */
	hctrl->oncs = NVME_CTRL_ONCS_DSM | NVME_CTRL_ONCS_WRITE_ZEROES;

	ret = nvme_mdev_hctrl_register(hctrl, nvme_mdev_root_dev);
	if (ret)
		goto err_hctrl;

	nvme_mdev_block_hctrl = hctrl;
	pr_info("nvme_mdev: using block layer polled IO\n");
	return 0;

err_hctrl:
	nvme_mdev_hctrl_put(hctrl);
err_root:
	root_device_unregister(nvme_mdev_root_dev);
	return ret;
}

static void nvme_mdev_backend_exit(void)
{
	nvme_mdev_hctrl_destroy(nvme_mdev_block_hctrl);
	root_device_unregister(nvme_mdev_root_dev);
}
#endif

static int __init nvme_mdev_init(void)
{
	int ret;

	ret = mdev_register_driver(&nvme_mdev_driver);
	if (ret)
		return ret;

	ret = nvme_mdev_backend_init();
	if (ret) {
		mdev_unregister_driver(&nvme_mdev_driver);
		return ret;
	}

	pr_info("nvme_mdev " NVME_MDEV_FIRMWARE_VERSION " loaded\n");
	return 0;
}

static void __exit nvme_mdev_exit(void)
{
	nvme_mdev_backend_exit();
	mdev_unregister_driver(&nvme_mdev_driver);
	pr_info("nvme_mdev unloaded\n");
}

MODULE_AUTHOR("Maxim Levitsky <mlevitsk@redhat.com>");
MODULE_DESCRIPTION("NVMe mediated VFIO virtual controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(NVME_MDEV_FIRMWARE_VERSION);

module_init(nvme_mdev_init)
module_exit(nvme_mdev_exit)
