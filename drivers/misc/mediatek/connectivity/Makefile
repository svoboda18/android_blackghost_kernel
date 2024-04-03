#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# Connectivity combo driver
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
	subdir-ccflags-y += -I$(srctree)/
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
	subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
	subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat
	subdir-ccflags-y += -I$(srctree)/drivers/mmc/core
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)
	subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/

	# Do Nothing, move to standalone repo
	MODULE_NAME := connadp
	obj-y += $(MODULE_NAME).o
	$(MODULE_NAME)-objs += common/connectivity_build_in_adapter.o
	$(MODULE_NAME)-objs += common/wmt_build_in_adapter.o

	subdir-ccflags-y += -D CFG_ENABLE_KEYWORD_EXCEPTION_MECHANISM
	subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
	subdir-ccflags-y += -D MTK_WCN_BUILT_IN_DRIVER

	obj-y += common/

	ifeq ($(CONFIG_WLAN_DRV_BUILD_IN),y)
		MTK_PLATFORM_ID := $(patsubst CONSYS_%,%,$(subst ",,$(CONFIG_MTK_COMBO_CHIP)))

		# Do build-in for xxx.c checking
		obj-y += wlan/adaptor/
		obj-y += wlan/core/gen2/
	endif

	obj-y += bt/mt66xx/legacy/
	obj-y += gps/

	# For FM built-in mode
	FM_CHIP_ID :=$(patsubst CONSYS_%,%,$(subst ",,$(CONFIG_MTK_COMBO_CHIP)))
	FM_CHIP := soc
	export CFG_BUILD_CONNAC2=false
	export CFG_FM_CHIP_ID=$(FM_CHIP_ID)
	export CFG_FM_CHIP=$(FM_CHIP)

	obj-y += fmradio/

# Otherwise we were called directly from the command line;
# invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
