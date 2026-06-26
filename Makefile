# SPDX-License-Identifier: GPL-2.0-only
# Out-of-tree build support for mt7601u with AP mode

ifneq ($(KERNELRELEASE),)
# Called from kernel build system
obj-m	+= mt7601u_ap.o

mt7601u_ap-objs	= \
	usb.o init.o main.o mcu.o trace.o dma.o core.o eeprom.o phy.o \
	mac.o util.o debugfs.o tx.o beacon.o

CFLAGS_trace.o := -I$(src)
ccflags-y := -DCONFIG_MT7601U=m

else
# Called directly
KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

endif
