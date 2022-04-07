# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_MICROTRUST_TEE_SUPPORT),y)
include $(srctree)/drivers/tee/teei/Makefile.include
endif
ifeq ($(CONFIG_MICROTRUST_TEE_LITE_SUPPORT),y)
include $(srctree)/drivers/tee/teei/Makefile.include
endif
ifeq ($(CONFIG_TRUSTKERNEL_TEE_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/tee/tkcore/include
endif

ccflags-y  += -I$(srctree)/drivers/misc/mediatek/thermal/fakeHeader/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/base/power/spm_v1
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpuidle_v1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/


obj-y += mt_pm_init.o
obj-y += mt_spm_mtcmos.o
obj-y += mt_clkmgr_common.o
obj-y += mt_clkmgr.o
obj-y += mt_clkbuf_ctl.o
obj-y += mt_golden_setting.o
obj-y += mt_sleep.o
obj-y += mt_dcm.o
obj-y += mt_gpufreq.o
obj-y += mt_cpufreq.o
obj-y += mt_ptp.o
obj-y += mt_dcm.o
obj-y += mt-smp.o
obj-y += mt-headsmp.o
obj-y += hotplug.o
obj-y += mt_hotplug.o
obj-y += mt_hotplug_strategy_main.o
obj-y += mt_hotplug_strategy_core.o
obj-y += mt_hotplug_strategy_procfs.o
obj-y += mt_hotplug_strategy_cpu.o
obj-y += mt_hotplug_strategy_api.o
obj-y += mt_hotplug_strategy_algo.o
