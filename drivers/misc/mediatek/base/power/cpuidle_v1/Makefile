// SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v1/

obj-y += mt_cpuidle.o

ifneq ($(CONFIG_ARM64), y)
obj-y += cpu_dormant-v7.o
else
obj-y += cpu_dormant.o
endif

ifeq ($(CONFIG_MICROTRUST_TEE_SUPPORT),y)
include $(srctree)/drivers/tee/teei/Makefile.include
endif
ifeq ($(CONFIG_MICROTRUST_TEE_LITE_SUPPORT),y)
include $(srctree)/drivers/tee/teei/Makefile.include
endif
ifeq ($(CONFIG_TRUSTKERNEL_TEE_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/tee/tkcore/include
endif
