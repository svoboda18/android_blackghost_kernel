# SPDX-License-Identifier: GPL-2.0


ccflags-y  += -I$(srctree)/drivers/misc/mediatek/thermal/fakeHeader/
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpuidle_v1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/



obj-y += mt_pm_init.o
obj-y += mt_spm_mtcmos.o
obj-y += mt_pbm.o
obj-y += mt_sleep.o
obj-y += mt_clkbuf_ctl.o
obj-y += mt_dcm.o

obj-$(CONFIG_MACH_MT6735) += mt_vcore_dvfs_1.o
obj-$(CONFIG_MACH_MT6735) += mt_spm_vcore_dvfs_1.o
obj-$(CONFIG_MACH_MT6735M) += mt_vcore_dvfs_2.o
obj-$(CONFIG_MACH_MT6735M) += mt_spm_vcore_dvfs_2.o
obj-$(CONFIG_MACH_MT6753) += mt_vcore_dvfs_3.o
obj-$(CONFIG_MACH_MT6753) += mt_spm_vcore_dvfs_3.o
obj-y += mt_static_power.o
obj-y += mt_cpufreq.o
obj-y += mt_cpu_ss.o
obj-y += mt_ptp.o
obj-y += mt_gpufreq.o


ifeq ($(strip $(CONFIG_MTK_CLKMGR)),y)
obj-y += mt_clkmgr_common_legacy.o
obj-$(CONFIG_MACH_MT6735) += mt_clkmgr_legacy.o
obj-$(CONFIG_MACH_MT6735M) += mt_clkmgr2.o
obj-$(CONFIG_MACH_MT6753) += mt_clkmgr3.o
else
obj-y += mt_clkmgr_common.o
obj-$(CONFIG_MACH_MT6735) += mt_clkmgr.o
ifeq ($(strip $(CONFIG_DEBUG_FS)),y)
obj-$(CONFIG_MACH_MT6735) += mt_clkdbg.o
endif
endif

ifneq ($(CONFIG_ARM64), y)

obj-y += mt-smp.o
obj-y += hotplug.o
obj-y += mt-headsmp.o
obj-y += mt_hotplug.o

else

obj-$(CONFIG_MTK_PSCI)  += mt_psci.o

endif

obj-y += mt_hotplug_strategy_main.o
obj-y += mt_hotplug_strategy_core.o
obj-y += mt_hotplug_strategy_procfs.o
obj-y += mt_hotplug_strategy_cpu.o
obj-y += mt_hotplug_strategy_api.o
obj-y += mt_hotplug_strategy_algo.o

