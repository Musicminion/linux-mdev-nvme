/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Driver private data structures and helper macros
 * Copyright (c) 2019 - Maxim Levitsky
 */

#ifndef _MDEV_NVME_PRIV_H
#define _MDEV_NVME_PRIV_H

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/xarray.h>
#include <linux/vfio.h>
#include <linux/mdev.h>
#include <linux/pci.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/eventfd.h>
#include <linux/workqueue.h>
#include <linux/dmapool.h>
#include <linux/byteorder/generic.h>
#include "spec.h"
#include "mdev.h"

#define NVME_MDEV_NVME_VER  NVME_VS(0x01, 0x03, 0x00)
#define NVME_MDEV_FIRMWARE_VERSION "1.0"

#define NVME_MDEV_PCI_VENDOR_ID		PCI_VENDOR_ID_REDHAT_QUMRANET
#define NVME_MDEV_PCI_DEVICE_ID		0x1234
#define NVME_MDEV_PCI_SUBVENDOR_ID	PCI_SUBVENDOR_ID_REDHAT_QUMRANET
#define NVME_MDEV_PCI_SUBDEVICE_ID	0
#define NVME_MDEV_PCI_REVISION		0x0

#define DB_STRIDE_SHIFT 4 /*4 = 1 cacheline */
#define MAX_VIRTUAL_QUEUES 16
#define MAX_VIRTUAL_QUEUE_DEPTH 0xFFFF
#define MAX_VIRTUAL_NAMESPACES 16 /* NSID = 1..16*/
#define MAX_VIRTUAL_IRQS 16

/*
 * Each virtual controller has one I/O thread and one "host queue": the
 * list of the bios it has in flight (all its guest queues share it).
 */
#define MAX_HOST_QUEUES 1

/* how many virtual controllers can be created */
#define NVME_MDEV_MAX_INSTANCES 16

#define MAX_AER_COMMANDS 16
#define MAX_LOG_PAGES 16

/*
 * How long to wait for the host I/O to complete when the I/O thread is
 * paused; longer than the block layer's own timeouts, so that this only
 * triggers if the host device stopped answering altogether.
 */
#define NVME_MDEV_IO_TIMEOUT_MS		60000
/* admin queue polling period, only without shadow doorbells */
#define NVME_MDEV_ADMIN_POLL_MS		100
/* default idle time before the I/O thread stops polling */
#define NVME_MDEV_POLL_TIMEOUT_MS	500
#define NVME_MDEV_POLL_TIMEOUT_MAX_MS	60000

/* virtual submission queue*/
struct  nvme_vsq {
	u16 qid;
	u16 size;
	u16 head;	/*next item to read */

	struct nvme_command *data; /*the queue*/
	struct nvme_vcq *vcq; /* completion queue*/

	dma_addr_t iova;
	bool cont;

	u16 hsq;
};

/* virtual completion queue*/
struct nvme_vcq {
	/* basic queue settings */
	u16 qid;
	u16 size;
	u16 head;
	u16 tail;
	bool phase; /* current queue phase */

	/* in guest memory: written with WRITE_ONCE(), see vcq.c */
	struct nvme_completion *data;

	/* number of items pending*/
	u16 pending;

	/* IRQ settings */
	int irq /* -1 if disabled*/;

	dma_addr_t iova;
	bool cont;
};

/*A virtual namespace */
struct nvme_mdev_vns {
	struct nvme_mdev_vctrl *vctrl;

	/*
	 * The host block device (a namespace or a partition of it), opened
	 * exclusively with this vns as the holder.
	 */
	struct file *bdev_file;
	struct block_device *host_part;

	/* set by the block layer when the host device goes away */
	bool dead;

	u32 nsid;

	/* size of each block on the real namespace, same for host and guest */
	u8 blksize_shift;

	/* size of the namespace in lbas*/
	u64 ns_size;

	/* is the namespace read only?*/
	bool readonly;

	/* UUID of this namespace */
	uuid_t uuid;

	/* Optimal IO boundary*/
	u16 noiob;
};

/*
 * Virtual IOMMU: guest IOVA -> pinned host page (+ DMA address on the
 * host NVMe device).  Guest memory is pinned lazily through
 * vfio_pin_pages() in chunks of NVME_MDEV_CHUNK_PAGES and released from
 * the VFIO dma_unmap callback.
 */
#define NVME_MDEV_CHUNK_SHIFT	21
#define NVME_MDEV_CHUNK_PAGES	(1U << (NVME_MDEV_CHUNK_SHIFT - PAGE_SHIFT))

struct nvme_mdev_viommu {
	struct vfio_device *vdev;	/* the VFIO device we pin for */

	/*
	 * Translations (and thus pinning) are only done by the I/O thread or
	 * with vctrl->lock held and the I/O thread parked, never in parallel;
	 * the chunk table is changed only under the same exclusion.
	 */
	struct xarray chunks;		/* chunk index -> nvme_mdev_chunk */
	struct nvme_mdev_vctrl *vctrl;

	/* guest memory VFIO is removing right now: must not be pinned again */
	spinlock_t unmap_lock;
	dma_addr_t unmap_start;		/* [unmap_start, unmap_end) */
	dma_addr_t unmap_end;
};

/*
 * Doorbells and event indexes live in memory shared with the guest (the
 * shadow doorbell buffers, or our page the doorbell writes land in): all
 * accesses are READ_ONCE()/WRITE_ONCE() or acquire loads, see vsq.c/vcq.c.
 */
struct doorbell {
	__le32 sqt;
	u8 rsvd1[(4 << DB_STRIDE_SHIFT) - sizeof(__le32)];
	__le32 cqh;
	u8 rsvd2[(4 << DB_STRIDE_SHIFT) - sizeof(__le32)];
};

/* MMIO state */
struct nvme_mdev_user_ctrl_mmio {
	u32 cc;		/* controller configuration */
	u32 csts;	/* controller status */
	u64 cap		/* controller capabilities*/;

	/* admin queue location & size */
	u32 aqa;
	u32 asql;
	u32 asqh;
	u32 acql;
	u32 acqh;

	bool shadow_db_supported;
	bool shadow_db_en;

	/* Regular doorbells */
	struct page *dbs_page;
	struct page *fake_eidx_page;
	void *db_page_kmap;
	void *fake_eidx_kmap;

	/* Shadow doorbells */
	struct page_map sdb_map;
	struct page_map seidx_map;

	/* Current doorbell mappings */
	struct doorbell *dbs;
	struct doorbell *eidxs;
};

/* pci configuration space of the device*/
#define PCI_CFG_SIZE 4096
struct nvme_mdev_pci_cfg_space {
	u8 *values;
	u8 *wmask;

	u8 pmcap;
	u8 pciecap;
	u8 msixcap;
	u8 end;
};

/*IRQ state of the controller */
struct nvme_mdev_user_irq {
	struct eventfd_ctx *trigger;
	/* IRQ coalescing */
	bool irq_coalesc_en;
	ktime_t irq_time;
	unsigned int irq_pending_cnt;
};

enum nvme_mdev_irq_mode {
	NVME_MDEV_IMODE_NONE,
	NVME_MDEV_IMODE_INTX,
	NVME_MDEV_IMODE_MSIX,
};

struct nvme_mdev_user_irqs {
	/* one of VFIO_PCI_{INTX|MSI|MSIX}_IRQ_INDEX */
	enum nvme_mdev_irq_mode mode;

	struct nvme_mdev_user_irq vecs[MAX_VIRTUAL_IRQS];
	/* user interrupt coalescing settings */
	u8 irq_coalesc_max;
	unsigned int irq_coalesc_time_us;
	/* device removal trigger*/
	struct eventfd_ctx *request_trigger;
};

/*AER state */
struct nvme_mdev_user_events {
	/* async event request CIDs*/
	u16 aer_cids[MAX_AER_COMMANDS];
	unsigned int aer_cid_count;

	/* events that are enabled */
	unsigned long events_enabled[BITS_TO_LONGS(MAX_LOG_PAGES)];

	/* events that are masked till next log page read*/
	unsigned long events_masked[BITS_TO_LONGS(MAX_LOG_PAGES)];

	/* events that are pending to be sent when user gives us an AER*/
	unsigned long  events_pending[BITS_TO_LONGS(MAX_LOG_PAGES)];
	u32 event_values[MAX_LOG_PAGES];
};

/* host IO queue */
struct nvme_mdev_hq {
	unsigned int usecount;
	struct list_head link;
	unsigned int hqid;
};

/* IO region abstraction (BARs, the PCI config space */
struct nvme_mdev_vctrl;
typedef int (*region_access_fn) (struct nvme_mdev_vctrl *vctrl,
				 u16 offset, char *buf,
				 u32 size, bool is_write);

struct nvme_mdev_io_region {
	unsigned int size;
	region_access_fn rw;

	/* IF != NULL, the mmap_area_start/size specify the mmaped window
	 * of this region
	 */
	const struct vm_operations_struct *mmap_ops;
	unsigned int mmap_area_start;
	unsigned int mmap_area_size;
};

struct io_ctx;
struct vhost_task;

/*Virtual NVME controller state */
struct nvme_mdev_vctrl {
	struct vfio_device vdev;	/* must be first, see vfio_alloc_device */
	struct mutex lock;
	struct list_head link;

	struct mdev_device *mdev;
	struct nvme_mdev_hctrl *hctrl;
	bool inuse;
	bool created;			/* vfio init completed */

	struct nvme_mdev_io_region regions[VFIO_PCI_NUM_REGIONS];

	/* virtual controller state */
	struct nvme_mdev_user_ctrl_mmio mmio;
	struct nvme_mdev_pci_cfg_space pcicfg;
	struct nvme_mdev_user_irqs irqs;
	struct nvme_mdev_user_events events;

	/* emulated namespaces */
	struct nvme_mdev_vns *namespaces[MAX_VIRTUAL_NAMESPACES];
	__le32 ns_log[MAX_VIRTUAL_NAMESPACES];
	unsigned int ns_log_size;

	/* emulated submission queues*/
	struct nvme_vsq vsqs[MAX_VIRTUAL_QUEUES];
	unsigned long vsq_en[BITS_TO_LONGS(MAX_VIRTUAL_QUEUES)];

	/* emulated completion queues*/
	unsigned long vcq_en[BITS_TO_LONGS(MAX_VIRTUAL_QUEUES)];
	struct nvme_vcq vcqs[MAX_VIRTUAL_QUEUES];

	/* Host IO queues*/
	int max_host_hw_queues;
	struct list_head host_hw_queues;

	/* Interface to access user memory */
	struct nvme_mdev_viommu viommu;

	/* removes namespaces whose host block device went away */
	struct work_struct ns_gc_work;

	/*
	 * The I/O thread: a vhost_task, i.e. a thread of the process that
	 * opened the device (QEMU), so that its CPU time and the bios it
	 * submits are accounted to that process and its cgroups.  It exists
	 * while the device is open.
	 */
	struct vhost_task *iothread;
	struct io_ctx *io_ctx;
	struct task_struct *iothread_task;	/* set by the thread itself */
	bool iothread_parked;		/* paused, under lock */
	bool io_pause_req;		/* set/cleared under lock */
	bool io_pause_ack;		/* the thread saw io_pause_req */
	bool io_dead;			/* the thread was killed (SIGKILL) */
	wait_queue_head_t io_pause_wq;
	bool io_idle;
	ktime_t now;

	/* Settings */
	unsigned int arb_burst_shift;
	bool write_atomicity_dn;	/* feature 0x0a, Disable Normal */
	u8 worload_hint;
	unsigned int poll_timeout_ms;	/* 0: never stop polling */
	/*
	 * Interrupt mode (set while the device isn't open): the I/O thread
	 * never waits by polling, it sleeps as soon as there's nothing to do;
	 * doorbell writes (the event indexes ask for them) and host I/O
	 * completions wake it up.  No polled bios then.  Only with shadow
	 * doorbells, which make the doorbell writes trap.
	 */
	bool irq_mode;
	bool io_sleeping;		/* sleeping with host I/O in flight */
	bool io_kicked;			/* host I/O completed since it looked */
	/* completions waking the thread up vs. its end (nvme_mdev_io_free()) */
	spinlock_t wake_lock;

	/* Identification*/
	char subnqn[256];
	char serial[9];

	bool vctrl_paused; /* true when the host device paused our IO */

};

/* mdev instance type*/
struct nvme_mdev_inst_type {
	struct mdev_type type;
	unsigned int max_hw_queues;
	char name[16];
};

#define MDEV_NVME_BIO_QUEUE_SIZE 128
#define NVME_STATUS_PENDING 0xFFFF
#define MDEV_NVME_NUM_BIO_QUEUES NVME_MDEV_MAX_INSTANCES

struct mbio {
	/* link in a list of pending bios*/
	struct list_head link;

	/* tag from the translation (user cid + user qid) */
	u32 tag;

	/* result NVME status, written by the completion handler */
	u16 status;

	/* Write Zeroes with FUA: flush once the zeroing is done */
	bool flush_after;
	struct block_device *bdev;

	/* the controller it is for (interrupt mode wake-ups) */
	struct nvme_mdev_vctrl *vctrl;

	/* must be last for bioset allocation*/
	struct bio bio;
};

struct hw_mbio_queue {
	int inflight;
	struct list_head bios_in_flight;
	struct bio_set bioset;
};

/*
 * The "host": where the virtual controllers' I/O goes.  With the block
 * layer backend there is one for the whole module; it is the mdev parent
 * and hangs off the "nvme_mdev" class device.
 */
struct nvme_mdev_hctrl {
	struct mutex lock;	/* protects hw_queues[] */

	/* numa node of the host controller*/
	int node;

	struct kref ref;
	bool removing;

	/* the device the mdev instances hang off */
	struct device *parent_dev;
	struct mdev_parent mparent;
	struct nvme_mdev_inst_type type;
	struct mdev_type *type_ptrs[1];

	/* Host area*/
	u16 oncs;
	u8 mdts;
	unsigned int id;

	/* one bio list per virtual controller (MAX_HOST_QUEUES each) */
	struct hw_mbio_queue *hw_queues[MDEV_NVME_NUM_BIO_QUEUES];
};

/* vctrl.c*/
int nvme_mdev_vctrl_create(struct nvme_mdev_vctrl *vctrl,
			   struct mdev_device *mdev,
			   struct nvme_mdev_hctrl *hctrl,
			   unsigned int max_host_queues);

void nvme_mdev_vctrl_destroy(struct nvme_mdev_vctrl *vctrl);

int nvme_mdev_vctrl_open(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_vctrl_release(struct nvme_mdev_vctrl *vctrl);

void nvme_mdev_vctrl_pause(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_vctrl_resume(struct nvme_mdev_vctrl *vctrl);

bool nvme_mdev_vctrl_enable(struct nvme_mdev_vctrl *vctrl,
			    dma_addr_t cqiova, dma_addr_t sqiova, u32 sizes);

void nvme_mdev_vctrl_disable(struct nvme_mdev_vctrl *vctrl);

void nvme_mdev_vctrl_reset(struct nvme_mdev_vctrl *vctrl);
void __nvme_mdev_vctrl_reset(struct nvme_mdev_vctrl *vctrl, bool pci_reset);

void nvme_mdev_vctrl_add_region(struct nvme_mdev_vctrl *vctrl,
				unsigned int index, unsigned int size,
				region_access_fn access_fn);

void nvme_mdev_vctrl_region_set_mmap(struct nvme_mdev_vctrl *vctrl,
				     unsigned int index,
				     unsigned int offset,
				     unsigned int size,
				     const struct vm_operations_struct *ops);

void nvme_mdev_vctrl_region_disable_mmap(struct nvme_mdev_vctrl *vctrl,
					 unsigned int index);

int nvme_mdev_vctrl_set_shadow_doorbell_supported(struct nvme_mdev_vctrl *vctrl,
						  bool enable);

int nvme_mdev_vctrl_hq_alloc(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_vctrl_hq_free(struct nvme_mdev_vctrl *vctrl, u16 qid);
unsigned int nvme_mdev_vctrl_hqs_list(struct nvme_mdev_vctrl *vctrl, u16 *out);
bool nvme_mdev_vctrl_is_dead(struct nvme_mdev_vctrl *vctrl);

void nvme_mdev_vctrl_viommu_unmap(struct nvme_mdev_vctrl *vctrl,
				  dma_addr_t iova, u64 size);

/* host.c*/
ssize_t nvme_mdev_type_show_description(struct mdev_type *mtype, char *buf);
void nvme_mdev_hctrl_get(struct nvme_mdev_hctrl *hctrl);
void nvme_mdev_hctrl_put(struct nvme_mdev_hctrl *hctrl);

int nvme_mdev_hctrl_hq_alloc(struct nvme_mdev_hctrl *hctrl);
void nvme_mdev_hctrl_hq_free(struct nvme_mdev_hctrl *hctrl, u16 qid);
bool nvme_mdev_hctrl_hq_can_submit(struct nvme_mdev_hctrl *hctrl, u16 qid);
bool nvme_mdev_hctrl_hq_check_op(struct nvme_mdev_hctrl *hctrl, u8 optcode);

int nvme_mdev_hctrl_hq_submit(struct nvme_mdev_hctrl *hctrl,
			      struct nvme_mdev_vns *vns,
			      u16 qid, u32 tag,
			      struct nvme_command *cmd,
			      struct nvme_ext_data_iter *datait);

int nvme_mdev_hctrl_hq_poll(struct nvme_mdev_hctrl *hctrl,
			    u32 qid,
			    struct nvme_ext_cmd_result *results,
			    unsigned int max_len);

/* instance.c */
extern struct mdev_driver nvme_mdev_driver;
extern struct list_head nvme_mdev_vctrl_list;
extern struct mutex nvme_mdev_vctrl_list_mutex;

/* io.c */
int nvme_mdev_io_create(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_io_free(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_io_pause(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_io_resume(struct nvme_mdev_vctrl *vctrl);
bool nvme_mdev_io_irq_mode(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_io_completed(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_assert_io_not_running(struct nvme_mdev_vctrl *vctrl);

/* mmio.c*/
int nvme_mdev_mmio_create(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_mmio_open(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_mmio_reset(struct nvme_mdev_vctrl *vctrl, bool pci_reset);
void nvme_mdev_mmio_free(struct nvme_mdev_vctrl *vctrl);

int nvme_mdev_mmio_enable_dbs(struct nvme_mdev_vctrl *vctrl);
int nvme_mdev_mmio_enable_dbs_shadow(struct nvme_mdev_vctrl *vctrl,
				     dma_addr_t sdb_iova, dma_addr_t eidx_iova);

void nvme_mdev_mmio_viommu_update(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_mmio_disable_dbs(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_mmio_db_invalid(struct nvme_mdev_vctrl *vctrl,
			       u16 qid, u16 size, u16 db);
bool nvme_mdev_mmio_db_check(struct nvme_mdev_vctrl *vctrl,
			     u16 qid, u16 size, u16 db);

/* pci.c*/
int nvme_mdev_pci_create(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_pci_free(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_pci_setup_bar(struct nvme_mdev_vctrl *vctrl,
			     u8 bar, unsigned int size,
			     region_access_fn access_fn);
/* adm.c*/
void nvme_mdev_adm_process_sq(struct nvme_mdev_vctrl *vctrl);

/* events.c */
void nvme_mdev_events_init(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_events_reset(struct nvme_mdev_vctrl *vctrl);

int nvme_mdev_event_request_receive(struct nvme_mdev_vctrl *vctrl, u16 cid);
void nvme_mdev_event_process_ack(struct nvme_mdev_vctrl *vctrl, u8 log_page);

void nvme_mdev_event_send(struct nvme_mdev_vctrl *vctrl,
			  enum nvme_async_event_type type,
			  enum nvme_async_event info);

u32 nvme_mdev_event_read_aen_config(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_event_set_aen_config(struct nvme_mdev_vctrl *vctrl, u32 value);

/* irq.c*/
void nvme_mdev_irqs_setup(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_irqs_reset(struct nvme_mdev_vctrl *vctrl);

int nvme_mdev_irqs_enable(struct nvme_mdev_vctrl *vctrl,
			  enum nvme_mdev_irq_mode mode);
void nvme_mdev_irqs_disable(struct nvme_mdev_vctrl *vctrl,
			    enum nvme_mdev_irq_mode mode);

int nvme_mdev_irqs_set_triggers(struct nvme_mdev_vctrl *vctrl,
				int start, int count, int32_t *fds);
int nvme_mdev_irqs_set_unplug_trigger(struct nvme_mdev_vctrl *vctrl,
				      int32_t fd);

void nvme_mdev_irq_raise_unplug_event(struct nvme_mdev_vctrl *vctrl,
				      unsigned int count);
void nvme_mdev_irq_raise(struct nvme_mdev_vctrl *vctrl,
			 unsigned int index);
void nvme_mdev_irq_trigger(struct nvme_mdev_vctrl *vctrl,
			   unsigned int index);
void nvme_mdev_irq_cond_trigger(struct nvme_mdev_vctrl *vctrl,
				unsigned int index);
void nvme_mdev_irqs_flush_pending(struct nvme_mdev_vctrl *vctrl);
void nvme_mdev_irq_clear(struct nvme_mdev_vctrl *vctrl,
			 unsigned int index);

/* vns.c*/
int nvme_mdev_vns_open(struct nvme_mdev_vctrl *vctrl, const char *path);
int nvme_mdev_vns_destroy(struct nvme_mdev_vctrl *vctrl,
			  u32 user_nsid);
void nvme_mdev_vns_destroy_all(struct nvme_mdev_vctrl *vctrl);

struct nvme_mdev_vns *nvme_mdev_vns_from_vnsid(struct nvme_mdev_vctrl *vctrl,
					       u32 user_ns_id);

int nvme_mdev_vns_print_description(struct nvme_mdev_vctrl *vctrl,
				    char *buf, unsigned int size);
void nvme_mdev_vns_gc_work(struct work_struct *work);

void nvme_mdev_vns_log_reset(struct nvme_mdev_vctrl *vctrl);

/* vcq.c */
int nvme_mdev_vcq_init(struct nvme_mdev_vctrl *vctrl, u16 qid,
		       dma_addr_t iova, bool cont, u16 size, int irq);

int nvme_mdev_vcq_viommu_update(struct nvme_mdev_viommu *viommu,
				struct nvme_vcq *q);

void nvme_mdev_vcq_delete(struct nvme_mdev_vctrl *vctrl, u16 qid);
void nvme_mdev_vcq_process(struct nvme_mdev_vctrl *vctrl, u16 qid,
			   bool trigger_irqs);

bool nvme_mdev_vcq_flush(struct nvme_mdev_vctrl *vctrl, u16 qid);
bool nvme_mdev_vcq_reserve_space(struct nvme_vcq *q);

void nvme_mdev_vcq_write_io(struct nvme_mdev_vctrl *vctrl,
			    struct nvme_vcq *q, u16 sq_head,
			    u16 sqid, u16 cid, u16 status);

void nvme_mdev_vcq_write_adm(struct nvme_mdev_vctrl *vctrl,
			     struct nvme_vcq *q, u32 dw0,
			     u16 sq_head, u16 cid, u16 status);
/* vsq.c*/
int nvme_mdev_vsq_init(struct nvme_mdev_vctrl *vctrl, u16 qid,
		       dma_addr_t iova, bool cont, u16 size, u16 cqid);

int nvme_mdev_vsq_viommu_update(struct nvme_mdev_viommu *viommu,
				struct nvme_vsq *q);

void nvme_mdev_vsq_delete(struct nvme_mdev_vctrl *vctrl, u16 qid);

bool nvme_mdev_vsq_has_data(struct nvme_mdev_vctrl *vctrl,
			    struct nvme_vsq *q);

bool nvme_mdev_vsq_get_cmd(struct nvme_mdev_vctrl *vctrl, struct nvme_vsq *q,
			   struct nvme_command *cmd);

void nvme_mdev_vsq_cmd_done_io(struct nvme_mdev_vctrl *vctrl,
			       u16 sqid, u16 cid, u16 status);
void nvme_mdev_vsq_cmd_done_adm(struct nvme_mdev_vctrl *vctrl,
				u32 dw0, u16 cid, u16 status);
bool nvme_mdev_vsq_suspend_io(struct nvme_mdev_vctrl *vctrl, u16 sqid);

/* udata.c*/
void nvme_mdev_udata_iter_setup(struct nvme_mdev_viommu *viommu,
				struct nvme_ext_data_iter *iter);

int nvme_mdev_udata_iter_set_dptr(struct nvme_ext_data_iter *it,
				  const union nvme_data_ptr *dptr, u64 size,
				  bool write);

struct nvme_ext_data_iter *
nvme_mdev_kdata_iter_alloc(struct nvme_mdev_viommu *viommu, unsigned int size);

int nvme_mdev_read_from_udata(void *dst, struct nvme_ext_data_iter *srcit,
			      u64 size);

int nvme_mdev_write_to_udata(struct nvme_ext_data_iter *dstit, void *src,
			     u64 size);

void *nvme_mdev_udata_queue_vmap(struct nvme_mdev_viommu *viommu,
				 dma_addr_t iova,
				 unsigned int size, bool cont, bool write);
/* viommu.c */
void nvme_mdev_viommu_init(struct nvme_mdev_viommu *viommu,
			   struct vfio_device *vdev);

int nvme_mdev_viommu_remove(struct nvme_mdev_viommu *viommu,
			    dma_addr_t iova, u64 size);

void nvme_mdev_viommu_block_range(struct nvme_mdev_viommu *viommu,
				  dma_addr_t iova, u64 size);
void nvme_mdev_viommu_unblock_range(struct nvme_mdev_viommu *viommu);

int nvme_mdev_viommu_translate(struct nvme_mdev_viommu *viommu,
			       dma_addr_t iova, bool write,
			       dma_addr_t *physical,
			       dma_addr_t *host_iova);

int nvme_mdev_viommu_create_kmap(struct nvme_mdev_viommu *viommu,
				 dma_addr_t iova, bool write,
				 struct page_map *page);

void nvme_mdev_viommu_free_kmap(struct nvme_mdev_viommu *viommu,
				struct page_map *page);

void nvme_mdev_viommu_update_kmap(struct nvme_mdev_viommu *viommu,
				  struct page_map *page);

void nvme_mdev_viommu_reset(struct nvme_mdev_viommu *viommu);

/* some utilities*/

#define store_le64(address, value) (*((__le64 *)(address)) = cpu_to_le64(value))
#define store_le32(address, value) (*((__le32 *)(address)) = cpu_to_le32(value))
#define store_le16(address, value) (*((__le16 *)(address)) = cpu_to_le16(value))
#define store_le8(address, value)  (*((u8 *)(address)) = (value))

#define load_le16(address) le16_to_cpu(*(__le16 *)(address))
#define load_le32(address) le32_to_cpu(*(__le32 *)(address))

#define store_strsp(dst, src) \
	memcpy_and_pad(dst, sizeof(dst), src, sizeof(src) - 1, ' ')

#define DNR(e) ((e) | NVME_SC_DNR)

#define PAGE_ADDRESS(address) ((address) & PAGE_MASK)
#define OFFSET_IN_PAGE(address) ((address) & ~(PAGE_MASK))

#define _DBG(vctrl, fmt, ...) \
	dev_dbg(mdev_dev((vctrl)->mdev), fmt, ##__VA_ARGS__)

#define _INFO(vctrl, fmt, ...) \
	dev_info(mdev_dev((vctrl)->mdev), fmt, ##__VA_ARGS__)

#define _WARN(vctrl, fmt, ...) \
	dev_warn(mdev_dev((vctrl)->mdev), fmt, ##__VA_ARGS__)

#define vdev_to_vctrl(vd) container_of(vd, struct nvme_mdev_vctrl, vdev)

/* the mdev device's drvdata is set at probe time */
#define dev_to_vctrl(dev) \
	((struct nvme_mdev_vctrl *)dev_get_drvdata(dev))

#define RSRV_NSID (BIT(1))
#define RSRV_DW23 (BIT(2) | BIT(3))
#define RSRV_MPTR (BIT(4) | BIT(5))

#define RSRV_DPTR (BIT(6) | BIT(7) | BIT(8) | BIT(9))
#define RSRV_DPTR_PRP2 (BIT(8) | BIT(9))

#define RSRV_DW10_15 (BIT(10) | BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(15))
#define RSRV_DW11_15 (BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(15))
#define RSRV_DW12_15 (BIT(12) | BIT(13) | BIT(14) | BIT(15))
#define RSRV_DW13_15 (BIT(13) | BIT(14) | BIT(15))
#define RSRV_DW14_15 (BIT(14) | BIT(15))

static inline bool check_reserved_dwords(const u32 *dwords,
					 int count, unsigned long bitmask)
{
	int bit;

	if (WARN_ON(count > BITS_PER_TYPE(long)))
		return false;

	for_each_set_bit(bit, &bitmask, count)
		if (dwords[bit])
			return false;
	return true;
}

static inline bool check_range(u64 start, u64 size, u64 end)
{
	u64 test = start + size;

	/* check for overflow */
	if (test < start || test < size)
		return false;
	return test <= end;
}

/* Rough translation of internal errors to the NVME errors */
static inline int nvme_mdev_translate_error(int error)
{
	// nvme status, including no error (NVME_SC_SUCCESS)
	if (error >= 0)
		return error;

	switch (error) {
	case -ENOMEM:
		/*no memory - truly an internal error*/
		return NVME_SC_INTERNAL;
	case -ENOSPC:
		/* Happens when user sends to large PRP list
		 * User shoudn't do this since the maximum transfer size
		 * is specified in the controller caps
		 */
		return DNR(NVME_SC_DATA_XFER_ERROR);
	case -EFAULT:
	case -ENOENT:
	case -EOVERFLOW:
	case -EACCES:
		/*
		 * Bad memory pointers in the command or the prp lists:
		 * nothing mapped there (-ENOENT from iommufd), a range that
		 * wraps around, memory the device may not use
		 */
		return DNR(NVME_SC_DATA_XFER_ERROR);
	case -EINVAL:
		/* Bad prp offsets in the prp lists/command*/
		return DNR(NVME_SC_PRP_OFFSET_INVALID);
	case -EPERM:
		/* backing device is read-only */
		return DNR(NVME_SC_READ_ONLY);
	case -ENODEV:
	case -EBUSY:
	case -EAGAIN:
		/* host queue (temporarily) unusable, e.g. host reset */
		return NVME_SC_INTERNAL;
	default:
		/* (the guest's addresses get us here: no WARN) */
		pr_debug("nvme-mdev: unexpected error %d\n", error);
		return NVME_SC_INTERNAL;
	}
}

static inline int nvme_mdev_translate_error_block(blk_status_t blk_sts)
{
	switch (blk_sts) {
	case BLK_STS_OK:
		return NVME_SC_SUCCESS;
	case BLK_STS_NOSPC:
		return DNR(NVME_SC_CAP_EXCEEDED);
	case BLK_STS_TARGET:
		return DNR(NVME_SC_LBA_RANGE);
	case BLK_STS_NOTSUPP:
		return DNR(NVME_SC_INVALID_OPCODE);
	case BLK_STS_MEDIUM:
		return DNR(NVME_SC_ACCESS_DENIED);
	default:
		return DNR(NVME_SC_INTERNAL);
	}
}

static inline bool timeout(ktime_t event, ktime_t now, unsigned long timeout_ms)
{
	return ktime_ms_delta(now, event) > (long)timeout_ms;
}

#endif // _MDEV_NVME_H
