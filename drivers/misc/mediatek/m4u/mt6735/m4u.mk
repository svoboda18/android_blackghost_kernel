// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

ifeq ($(CONFIG_MACH_MT6735),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/mt6735/
endif
ifeq ($(CONFIG_MACH_MT6735M),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/mt6735m/
endif
ifeq ($(CONFIG_MACH_MT6753),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/mt6753/
endif
