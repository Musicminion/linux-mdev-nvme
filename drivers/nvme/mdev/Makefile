
obj-$(CONFIG_NVME_MDEV_VFIO) 	+=	nvme-mdev.o

nvme-mdev-y += instance.o host.o io.o irq.o nvmet.o \
	       udata.o viommu.o vsq.o vcq.o vctrl.o mmio.o pci.o

# for the tracepoints (trace.h)
ccflags-y += -I$(src)
