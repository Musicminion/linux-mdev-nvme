// SPDX-License-Identifier: GPL-2.0+
/*
 * NVMe mediated device: the module and the mdev parent
 * Copyright (c) 2019 - Maxim Levitsky
 *
 * The virtual controllers' commands are executed by nvmet (see nvmet.c);
 * nothing here depends on the NVMe host driver.  All instances hang off
 * one parent device, /sys/class/nvme_mdev/nvme_mdev.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/nvme.h>
#include <linux/mdev.h>
#include <linux/module.h>
#include <linux/log2.h>
#include "priv.h"

/* ------------------------------------------------------------------ */
/* the parent device and the instance type */

static const struct class nvme_mdev_class = {
	.name	= "nvme_mdev",
};

static void nvme_mdev_parent_release(struct device *dev)
{
}

static struct device nvme_mdev_parent = {
	.class		= &nvme_mdev_class,
	.release	= nvme_mdev_parent_release,
};

static struct nvme_mdev_hctrl *nvme_mdev_host;

ssize_t nvme_mdev_type_show_description(struct mdev_type *mtype, char *buf)
{
	return sysfs_emit(buf,
			  "NVMe virtual controller, commands executed by nvmet\n");
}

static void nvme_mdev_hctrl_free(struct kref *ref)
{
	kfree(container_of(ref, struct nvme_mdev_hctrl, ref));
}

void nvme_mdev_hctrl_get(struct nvme_mdev_hctrl *hctrl)
{
	kref_get(&hctrl->ref);
}

void nvme_mdev_hctrl_put(struct nvme_mdev_hctrl *hctrl)
{
	kref_put(&hctrl->ref, nvme_mdev_hctrl_free);
}

static int __init nvme_mdev_init(void)
{
	struct nvme_mdev_hctrl *hctrl;
	int ret;

	hctrl = kzalloc(sizeof(*hctrl), GFP_KERNEL);
	if (!hctrl)
		return -ENOMEM;

	kref_init(&hctrl->ref);
	hctrl->node = NUMA_NO_NODE;

	/*
	 * One instance type: every virtual controller takes the same
	 * resources (an I/O thread, an nvmet controller).  "V1" leaves room
	 * for other kinds later, e.g. controllers sharing an I/O thread.
	 */
	strscpy(hctrl->type.name, "V1", sizeof(hctrl->type.name));
	hctrl->type.type.sysfs_name = hctrl->type.name;
	hctrl->type.type.pretty_name = "NVMe virtual controller";
	hctrl->type_ptr = &hctrl->type.type;

	ret = nvme_mdev_nvmet_init();
	if (ret)
		goto err_free;

	ret = class_register(&nvme_mdev_class);
	if (ret)
		goto err_nvmet;

	dev_set_name(&nvme_mdev_parent, "nvme_mdev");
	ret = device_register(&nvme_mdev_parent);
	if (ret)
		goto err_class;

	ret = mdev_register_driver(&nvme_mdev_driver);
	if (ret)
		goto err_device;

	hctrl->parent_dev = &nvme_mdev_parent;
	ret = mdev_register_parent(&hctrl->mparent, &nvme_mdev_parent,
				   &nvme_mdev_driver, &hctrl->type_ptr, 1);
	if (ret)
		goto err_driver;

	nvme_mdev_host = hctrl;
	pr_info("nvme_mdev " NVME_MDEV_FIRMWARE_VERSION " loaded\n");
	return 0;

err_driver:
	mdev_unregister_driver(&nvme_mdev_driver);
err_device:
	device_unregister(&nvme_mdev_parent);
err_class:
	class_unregister(&nvme_mdev_class);
err_nvmet:
	nvme_mdev_nvmet_exit();
err_free:
	nvme_mdev_hctrl_put(hctrl);
	return ret;
}

static void __exit nvme_mdev_exit(void)
{
	struct nvme_mdev_hctrl *hctrl = nvme_mdev_host;

	WRITE_ONCE(hctrl->removing, true);
	/* removes all instances, waiting for their users to let go */
	mdev_unregister_parent(&hctrl->mparent);
	mdev_unregister_driver(&nvme_mdev_driver);
	device_unregister(&nvme_mdev_parent);
	class_unregister(&nvme_mdev_class);
	nvme_mdev_nvmet_exit();
	nvme_mdev_hctrl_put(hctrl);
	pr_info("nvme_mdev unloaded\n");
}

MODULE_AUTHOR("Maxim Levitsky <mlevitsk@redhat.com>");
MODULE_DESCRIPTION("NVMe mediated VFIO virtual controller");
MODULE_LICENSE("GPL");
/* iova_bitmap_set(), for the dirty tracking (migration.c) */
MODULE_IMPORT_NS("IOMMUFD");
MODULE_VERSION(NVME_MDEV_FIRMWARE_VERSION);

module_init(nvme_mdev_init)
module_exit(nvme_mdev_exit)
