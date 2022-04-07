// SPDX-License-Identifier: GPL-2.0

obj-y += usb2jtag_v1.o
ifeq ($(CONFIG_MTK_ENG_BUILD),y)
CFLAGS_usb2jtag_v1.o := -DDEBUG
endif
ifeq ($(CONFIG_MACH_MT6761),y)
obj-y += mt6765/
else
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif