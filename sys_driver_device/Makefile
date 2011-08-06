obj-m :=phone.o
CURRENT_PATH := $(shell pwd)
LINUX_KERNEL_PATH := /home/pete/T4/kernel_imx
all:
	make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) modules
clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions Module.symvers .Makefile.swp modules.order
