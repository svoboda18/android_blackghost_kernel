// SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/$(MTK_PLATFORM)

obj-$(CONFIG_MTK_ACCDET) := accdet_drv.o

obj-$(CONFIG_MTK_ACCDET) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
