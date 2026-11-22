// SPDX-License-Identifier: GPL-2.0+
/*
 * Mediated NVMe instance VFIO code
 * Copyright (c) 2019 - Maxim Levitsky
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/vfio.h>
#include <linux/sysfs.h>
#include <linux/mdev.h>
#include "priv.h"

#define OFFSET_TO_REGION(offset) ((offset) >> 20)
#define REGION_TO_OFFSET(nr) (((u64)nr) << 20)

LIST_HEAD(nvme_mdev_vctrl_list);
/*protects the list */
DEFINE_MUTEX(nvme_mdev_vctrl_list_mutex);

static const struct vfio_device_ops nvme_mdev_vfio_ops;

struct mdev_nvme_vfio_region_info {
	struct vfio_region_info base;
	struct vfio_region_info_cap_sparse_mmap mmap_cap;
};

/* Called from vfio_alloc_device(): create the virtual controller */
static int nvme_mdev_ops_init(struct vfio_device *vdev)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);
	struct mdev_device *mdev = to_mdev_device(vdev->dev);
	struct nvme_mdev_inst_type *type =
		container_of(mdev->type, struct nvme_mdev_inst_type, type);
	struct nvme_mdev_hctrl *hctrl =
		container_of(mdev->type->parent, struct nvme_mdev_hctrl,
			     mparent);
	int ret;

	ret = nvme_mdev_vctrl_create(vctrl, mdev, hctrl, type->max_hw_queues);
	if (ret)
		return ret;

	mutex_lock(&nvme_mdev_vctrl_list_mutex);
	list_add_tail(&vctrl->link, &nvme_mdev_vctrl_list);
	mutex_unlock(&nvme_mdev_vctrl_list_mutex);
	return 0;
}

/* Called when the last reference to the vfio device is dropped */
static void nvme_mdev_ops_release(struct vfio_device *vdev)
{
	nvme_mdev_vctrl_destroy(vdev_to_vctrl(vdev));
}

/* Called when new mediated device is created */
static int nvme_mdev_probe(struct mdev_device *mdev)
{
	struct nvme_mdev_vctrl *vctrl;
	int ret;

	vctrl = vfio_alloc_device(nvme_mdev_vctrl, vdev, &mdev->dev,
				  &nvme_mdev_vfio_ops);
	if (IS_ERR(vctrl))
		return PTR_ERR(vctrl);

	ret = vfio_register_emulated_iommu_dev(&vctrl->vdev);
	if (ret) {
		vfio_put_device(&vctrl->vdev);
		return ret;
	}
	dev_set_drvdata(&mdev->dev, vctrl);
	return 0;
}

/*
 * Called when a mediated device is removed.  vfio_unregister_group_dev()
 * asks the user to release the device and waits until it did.
 */
static void nvme_mdev_remove(struct mdev_device *mdev)
{
	struct nvme_mdev_vctrl *vctrl = dev_get_drvdata(&mdev->dev);

	vfio_unregister_group_dev(&vctrl->vdev);
	vfio_put_device(&vctrl->vdev);
}

/* Called when the device is opened by a user (first open) */
static int nvme_mdev_ops_open(struct vfio_device *vdev)
{
	return nvme_mdev_vctrl_open(vdev_to_vctrl(vdev));
}

/* Called on the last close of the device */
static void nvme_mdev_ops_close(struct vfio_device *vdev)
{
	nvme_mdev_vctrl_release(vdev_to_vctrl(vdev));
}

/* The user removed a guest memory mapping: drop our pins on it */
static void nvme_mdev_ops_dma_unmap(struct vfio_device *vdev,
				    u64 iova, u64 length)
{
	nvme_mdev_vctrl_viommu_unmap(vdev_to_vctrl(vdev), iova, length);
}

/* Helper function for bar/pci config read/write access */
static ssize_t nvme_mdev_access(struct nvme_mdev_vctrl *vctrl,
				char *buf, size_t count,
				loff_t pos, bool is_write)
{
	int index = OFFSET_TO_REGION(pos);
	int ret = -EINVAL;
	unsigned int offset;

	if (index >= VFIO_PCI_NUM_REGIONS || !vctrl->regions[index].rw)
		goto out;

	offset = pos - REGION_TO_OFFSET(index);
	if (offset + count > vctrl->regions[index].size)
		goto out;

	ret = vctrl->regions[index].rw(vctrl, offset, buf, count, is_write);
out:
	return ret;
}

/* Called when read() is done on the device */
static ssize_t nvme_mdev_ops_read(struct vfio_device *vdev, char __user *buf,
				  size_t count, loff_t *ppos)
{
	unsigned int done = 0;
	int ret;
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);

	while (count) {
		size_t filled;

		if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, false);
			if (ret <= 0)
				goto read_err;

			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;
			filled = sizeof(val);
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, false);
			if (ret <= 0)
				goto read_err;
			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;
			filled = sizeof(val);
		} else {
			u8 val;

			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, false);
			if (ret <= 0)
				goto read_err;
			if (copy_to_user(buf, &val, sizeof(val)))
				goto read_err;
			filled = sizeof(val);
		}

		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}
	return done;
read_err:
	return -EFAULT;
}

/* Called when write() is done on the device */
static ssize_t nvme_mdev_ops_write(struct vfio_device *vdev,
				   const char __user *buf,
				   size_t count, loff_t *ppos)
{
	unsigned int done = 0;
	int ret;
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);

	while (count) {
		size_t filled;

		if (count >= 4 && !(*ppos % 4)) {
			u32 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;
			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, true);
			if (ret <= 0)
				goto write_err;
			filled = sizeof(val);
		} else if (count >= 2 && !(*ppos % 2)) {
			u16 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;

			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, true);
			if (ret <= 0)
				goto write_err;
			filled = sizeof(val);
		} else {
			u8 val;

			if (copy_from_user(&val, buf, sizeof(val)))
				goto write_err;
			ret = nvme_mdev_access(vctrl, (char *)&val,
					       sizeof(val), *ppos, true);
			if (ret <= 0)
				goto write_err;
			filled = sizeof(val);
		}
		count -= filled;
		done += filled;
		*ppos += filled;
		buf += filled;
	}
	return done;
write_err:
	return -EFAULT;
}

/*Helper for IRQ number VFIO query */
static int nvme_mdev_irq_counts(struct nvme_mdev_vctrl *vctrl,
				unsigned int irq_type)
{
	switch (irq_type) {
	case VFIO_PCI_INTX_IRQ_INDEX:
		return 1;
	case VFIO_PCI_MSIX_IRQ_INDEX:
		return MAX_VIRTUAL_IRQS;
	case VFIO_PCI_REQ_IRQ_INDEX:
		return 1;
	default:
		return 0;
	}
}

/* VFIO VFIO_IRQ_SET_ACTION_TRIGGER implementation */
static int nvme_mdev_ioctl_set_irqs_trigger(struct nvme_mdev_vctrl *vctrl,
					    u32 flags,
					    unsigned int irq_type,
					    unsigned int start,
					    unsigned int count,
					    void *data)
{
	u32 data_type = flags & VFIO_IRQ_SET_DATA_TYPE_MASK;
	u8 *bools = NULL;
	unsigned int i;
	int ret = -EINVAL;

	/* Asked to disable the current interrupt mode*/
	if (data_type == VFIO_IRQ_SET_DATA_NONE && count == 0) {
		switch (irq_type) {
		case VFIO_PCI_REQ_IRQ_INDEX:
			nvme_mdev_irqs_set_unplug_trigger(vctrl, -1);
			return 0;
		case VFIO_PCI_INTX_IRQ_INDEX:
			nvme_mdev_irqs_disable(vctrl, NVME_MDEV_IMODE_INTX);
			return 0;
		case VFIO_PCI_MSIX_IRQ_INDEX:
			nvme_mdev_irqs_disable(vctrl, NVME_MDEV_IMODE_MSIX);
			return 0;
		default:
			return -EINVAL;
		}
	}

	if (start + count > nvme_mdev_irq_counts(vctrl, irq_type))
		return -EINVAL;

	switch (data_type) {
	case VFIO_IRQ_SET_DATA_BOOL:
		bools = (u8 *)data;
		fallthrough;
	case VFIO_IRQ_SET_DATA_NONE:
		if (irq_type == VFIO_PCI_REQ_IRQ_INDEX)
			return -EINVAL;

		for (i = 0 ; i < count ; i++) {
			int index = start + i;

			if (!bools || bools[i])
				nvme_mdev_irq_trigger(vctrl, index);
		}
		return 0;

	case VFIO_IRQ_SET_DATA_EVENTFD:
		switch (irq_type) {
		case VFIO_PCI_REQ_IRQ_INDEX:
			return nvme_mdev_irqs_set_unplug_trigger(vctrl,
							*(int32_t *)data);
		case VFIO_PCI_INTX_IRQ_INDEX:
			ret = nvme_mdev_irqs_enable(vctrl,
						    NVME_MDEV_IMODE_INTX);
			break;
		case VFIO_PCI_MSIX_IRQ_INDEX:
			ret = nvme_mdev_irqs_enable(vctrl,
						    NVME_MDEV_IMODE_MSIX);
			break;
		default:
			return -EINVAL;
		}
		if (ret)
			return ret;

		return nvme_mdev_irqs_set_triggers(vctrl, start,
						   count, (int32_t *)data);
	default:
		return -EINVAL;
	}
}

/* VFIO_DEVICE_GET_INFO ioctl implementation */
static int nvme_mdev_ioctl_get_info(struct nvme_mdev_vctrl *vctrl,
				    void __user *arg)
{
	struct vfio_device_info info;
	unsigned int minsz = offsetofend(struct vfio_device_info, num_irqs);

	if (copy_from_user(&info, (void __user *)arg, minsz))
		return -EFAULT;
	if (info.argsz < minsz)
		return -EINVAL;

	info.flags = VFIO_DEVICE_FLAGS_PCI | VFIO_DEVICE_FLAGS_RESET;
	info.num_regions = VFIO_PCI_NUM_REGIONS;
	info.num_irqs = VFIO_PCI_NUM_IRQS;

	if (copy_to_user(arg, &info, minsz))
		return -EFAULT;
	return 0;
}

/* VFIO_DEVICE_GET_REGION_INFO ioctl implementation*/
static int nvme_mdev_ioctl_get_reg_info(struct nvme_mdev_vctrl *vctrl,
					void __user *arg)
{
	struct nvme_mdev_io_region *region;
	struct mdev_nvme_vfio_region_info *info;
	unsigned long minsz, outsz, maxsz;
	int ret = 0;

	minsz = offsetofend(struct vfio_region_info, offset);
	maxsz = sizeof(struct mdev_nvme_vfio_region_info) +
				sizeof(struct vfio_region_sparse_mmap_area);

	info = kzalloc(maxsz, GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (copy_from_user(info, arg, minsz)) {
		ret = -EFAULT;
		goto out;
	}

	outsz = info->base.argsz;
	if (outsz < minsz || outsz > maxsz) {
		ret = -EINVAL;
		goto out;
	}

	if (info->base.index >= VFIO_PCI_NUM_REGIONS) {
		ret = -EINVAL;
		goto out;
	}

	region = &vctrl->regions[info->base.index];
	info->base.offset = REGION_TO_OFFSET(info->base.index);
	info->base.argsz = maxsz;
	info->base.size = region->size;

	info->base.flags = VFIO_REGION_INFO_FLAG_READ |
				VFIO_REGION_INFO_FLAG_WRITE;

	if (region->mmap_ops) {
		info->base.flags |= (VFIO_REGION_INFO_FLAG_MMAP |
						VFIO_REGION_INFO_FLAG_CAPS);

		info->base.cap_offset =
			offsetof(struct mdev_nvme_vfio_region_info, mmap_cap);

		info->mmap_cap.header.id = VFIO_REGION_INFO_CAP_SPARSE_MMAP;
		info->mmap_cap.header.version = 1;
		info->mmap_cap.header.next = 0;
		info->mmap_cap.nr_areas = 1;
		info->mmap_cap.areas[0].offset = region->mmap_area_start;
		info->mmap_cap.areas[0].size = region->mmap_area_size;
	}

	if (copy_to_user(arg, info, outsz))
		ret = -EFAULT;
out:
	kfree(info);
	return ret;
}

/* VFIO_DEVICE_GET_IRQ_INFO ioctl implementation */
static int nvme_mdev_ioctl_get_irq_info(struct nvme_mdev_vctrl *vctrl,
					void __user *arg)
{
	struct vfio_irq_info info;
	unsigned int minsz = offsetofend(struct vfio_irq_info, count);

	if (copy_from_user(&info, arg, minsz))
		return -EFAULT;
	if (info.argsz < minsz)
		return -EINVAL;

	info.count = nvme_mdev_irq_counts(vctrl, info.index);
	info.flags = VFIO_IRQ_INFO_EVENTFD;

	if (info.index == VFIO_PCI_INTX_IRQ_INDEX)
		info.flags |= VFIO_IRQ_INFO_MASKABLE | VFIO_IRQ_INFO_AUTOMASKED;

	if (copy_to_user(arg, &info, minsz))
		return -EFAULT;
	return 0;
}

/* VFIO VFIO_DEVICE_SET_IRQS ioctl implementation */
static int nvme_mdev_ioctl_set_irqs(struct nvme_mdev_vctrl *vctrl,
				    void __user *arg)
{
	int ret, irqcount;
	struct vfio_irq_set hdr;
	u8 *data = NULL;
	size_t data_size = 0;
	unsigned long minsz = offsetofend(struct vfio_irq_set, count);

	if (copy_from_user(&hdr, arg, minsz))
		return -EFAULT;

	irqcount = nvme_mdev_irq_counts(vctrl, hdr.index);
	ret = vfio_set_irqs_validate_and_prepare(&hdr,
						 irqcount,
						 VFIO_PCI_NUM_IRQS,
						 &data_size);
	if (ret)
		return ret;

	if (data_size) {
		data = memdup_user((arg + minsz), data_size);
		if (IS_ERR(data))
			return PTR_ERR(data);
	}

	ret = -ENOTTY;
	switch (hdr.index) {
	case VFIO_PCI_INTX_IRQ_INDEX:
	case VFIO_PCI_MSIX_IRQ_INDEX:
	case VFIO_PCI_REQ_IRQ_INDEX:
		switch (hdr.flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
		case VFIO_IRQ_SET_ACTION_MASK:
		case VFIO_IRQ_SET_ACTION_UNMASK:
			// pretend to support this (even with eventfd)
			ret = hdr.index == VFIO_PCI_INTX_IRQ_INDEX ?
					0 : -EINVAL;
			break;
		case VFIO_IRQ_SET_ACTION_TRIGGER:
			ret = nvme_mdev_ioctl_set_irqs_trigger(vctrl, hdr.flags,
							       hdr.index,
							       hdr.start,
							       hdr.count,
							       data);
			break;
		}
		break;
	}

	kfree(data);
	return ret;
}

/* ioctl() implementation */
static long nvme_mdev_ops_ioctl(struct vfio_device *vdev, unsigned int cmd,
				unsigned long arg)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
		return nvme_mdev_ioctl_get_info(vctrl, (void __user *)arg);
	case VFIO_DEVICE_GET_REGION_INFO:
		return nvme_mdev_ioctl_get_reg_info(vctrl, (void __user *)arg);
	case VFIO_DEVICE_GET_IRQ_INFO:
		return nvme_mdev_ioctl_get_irq_info(vctrl, (void __user *)arg);
	case VFIO_DEVICE_SET_IRQS:
		return nvme_mdev_ioctl_set_irqs(vctrl, (void __user *)arg);
	case VFIO_DEVICE_RESET:
		nvme_mdev_vctrl_reset(vctrl);
		return 0;
	default:
		return -ENOTTY;
	}
}

/* mmap() implementation (doorbell area) */
static int nvme_mdev_ops_mmap(struct vfio_device *vdev,
			      struct vm_area_struct *vma)
{
	struct nvme_mdev_vctrl *vctrl = vdev_to_vctrl(vdev);
	int index = OFFSET_TO_REGION((u64)vma->vm_pgoff << PAGE_SHIFT);
	unsigned long size, start;

	if (index >= VFIO_PCI_NUM_REGIONS || !vctrl->regions[index].mmap_ops)
		return -EINVAL;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;

	size = vma->vm_end - vma->vm_start;
	start = vma->vm_pgoff << PAGE_SHIFT;

	if (start < vctrl->regions[index].mmap_area_start)
		return -EINVAL;
	if (size > vctrl->regions[index].mmap_area_size)
		return -EINVAL;

	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	vma->vm_ops = vctrl->regions[index].mmap_ops;
	vma->vm_private_data = vctrl;
	return 0;
}

/* Request removal of the device*/
static void nvme_mdev_ops_request(struct vfio_device *vdev, unsigned int count)
{
	nvme_mdev_irq_raise_unplug_event(vdev_to_vctrl(vdev), count);
}

/*
 * Add a namespace backed by a host block device, e.g.
 * "echo /dev/nvme0n1p5 > namespaces/add_namespace"
 */
static ssize_t add_namespace_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);
	char *path;
	int ret;

	if (!vctrl)
		return -ENODEV;

	path = kstrdup(buf, GFP_KERNEL);
	if (!path)
		return -ENOMEM;

	ret = nvme_mdev_vns_open(vctrl, strim(path));
	kfree(path);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(add_namespace);

/* Remove a user namespace */
static ssize_t remove_namespace_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long user_nsid;
	int ret;
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;

	ret = kstrtoul(buf, 10, &user_nsid);
	if (ret)
		return ret;

	ret =  nvme_mdev_vns_destroy(vctrl, user_nsid);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(remove_namespace);

/* Show list of user namespaces */
static ssize_t namespaces_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;
	return nvme_mdev_vns_print_description(vctrl, buf, PAGE_SIZE - 1);
}
static DEVICE_ATTR_RO(namespaces);

/* change the cpu binding of the IO threads*/
static ssize_t shadow_doorbell_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	bool val;
	int ret;
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	ret = nvme_mdev_vctrl_set_shadow_doorbell_supported(vctrl, val);
	if (ret)
		return ret;
	return count;
}

/* change the cpu binding of the IO threads*/
static ssize_t shadow_doorbell_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;

	return sysfs_emit(buf, "%d\n", vctrl->mmio.shadow_db_supported ? 1 : 0);
}
static DEVICE_ATTR_RW(shadow_doorbell);

static struct attribute *nvme_mdev_dev_ns_atttributes[] = {
	&dev_attr_add_namespace.attr,
	&dev_attr_remove_namespace.attr,
	&dev_attr_namespaces.attr,
	NULL
};

/* idle time (msec) before the I/O thread stops polling, 0: poll forever */
static ssize_t poll_timeout_ms_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);
	unsigned int val;
	int ret;

	if (!vctrl)
		return -ENODEV;
	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (val > NVME_MDEV_POLL_TIMEOUT_MAX_MS)
		return -EINVAL;
	/* taken into account when the I/O thread starts polling again */
	WRITE_ONCE(vctrl->poll_timeout_ms, val);
	return count;
}

static ssize_t poll_timeout_ms_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;
	return sysfs_emit(buf, "%u\n", READ_ONCE(vctrl->poll_timeout_ms));
}
static DEVICE_ATTR_RW(poll_timeout_ms);

/* interrupt mode of the I/O thread, see vctrl->irq_mode */
static ssize_t interrupt_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);
	bool val;
	int ret;

	if (!vctrl)
		return -ENODEV;
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	/* (not with I/O in flight: polled bios need the polling) */
	mutex_lock(&vctrl->lock);
	if (vctrl->inuse)
		ret = -EBUSY;
	else
		WRITE_ONCE(vctrl->irq_mode, val);
	mutex_unlock(&vctrl->lock);
	return ret ? ret : count;
}

static ssize_t interrupt_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nvme_mdev_vctrl *vctrl = dev_to_vctrl(dev);

	if (!vctrl)
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", READ_ONCE(vctrl->irq_mode));
}
static DEVICE_ATTR_RW(interrupt_mode);

static struct attribute *nvme_mdev_dev_settings_atttributes[] = {
	&dev_attr_shadow_doorbell.attr,
	&dev_attr_poll_timeout_ms.attr,
	&dev_attr_interrupt_mode.attr,
	NULL
};

static const struct attribute_group nvme_mdev_ns_attr_group = {
	.name = "namespaces",
	.attrs = nvme_mdev_dev_ns_atttributes,
};

static const struct attribute_group nvme_mdev_setting_attr_group = {
	.name = "settings",
	.attrs = nvme_mdev_dev_settings_atttributes,
};

static const struct attribute_group *nvme_mdev_dev_attributte_groups[] = {
	&nvme_mdev_ns_attr_group,
	&nvme_mdev_setting_attr_group,
	NULL,
};

static const struct vfio_device_ops nvme_mdev_vfio_ops = {
	.name			= "nvme-mdev",
	.init			= nvme_mdev_ops_init,
	.release		= nvme_mdev_ops_release,
	.open_device		= nvme_mdev_ops_open,
	.close_device		= nvme_mdev_ops_close,
	.read			= nvme_mdev_ops_read,
	.write			= nvme_mdev_ops_write,
	.ioctl			= nvme_mdev_ops_ioctl,
	.mmap			= nvme_mdev_ops_mmap,
	.request		= nvme_mdev_ops_request,
	.dma_unmap		= nvme_mdev_ops_dma_unmap,
	.bind_iommufd		= vfio_iommufd_emulated_bind,
	.unbind_iommufd		= vfio_iommufd_emulated_unbind,
	.attach_ioas		= vfio_iommufd_emulated_attach_ioas,
	.detach_ioas		= vfio_iommufd_emulated_detach_ioas,
};

struct mdev_driver nvme_mdev_driver = {
	.device_api		= VFIO_DEVICE_API_PCI_STRING,
	.probe			= nvme_mdev_probe,
	.remove			= nvme_mdev_remove,
	.max_instances		= NVME_MDEV_MAX_INSTANCES,
	.show_description	= nvme_mdev_type_show_description,
	.driver = {
		.name		= "nvme_mdev",
		.owner		= THIS_MODULE,
		.mod_name	= KBUILD_MODNAME,
		.dev_groups	= nvme_mdev_dev_attributte_groups,
	},
};
