obj-m :=globalmem.o
CURRENT_PATH := $(shell pwd)
LINUX_KERNEL_PATH :=/home/pete/kernel/linux-2.6.26/

all:
	make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) modules
clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions Module.symvers .Makefile.swp
