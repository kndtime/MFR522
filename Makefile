MODULE_NAME=mfrc522_spi_driver
TARGET=arm
PWD:=$(shell pwd)
MODULE_UPLOAD = $(MODULE_NAME).ko

obj-m := $(MODULE_NAME).o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) ARCH=$(TARGET) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

modules_install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
	$(DEPMOD)
