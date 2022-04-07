// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/notifier.h>

#include <linux/platform_device.h>
#include <mach/mt_lbc.h>

/*--------------DEFAULT SETTING-------------------*/

#define TARGET_CORE (4)
#define TARGET_FREQ (1092000)

/*-----------------------------------------------*/

static int last_enable;

int perfmgr_get_target_core(void)
{
	return TARGET_CORE;
}

int perfmgr_get_target_freq(void)
{
	return TARGET_FREQ;
}

void perfmgr_boost(int enable, int core, int freq)
{
	struct ppm_limit_data core_to_set, freq_to_set;

	if (enable == last_enable)
		return;

	if (enable) {
		core_to_set.min = core;
		core_to_set.max = -1;
		freq_to_set.min = freq;
		freq_to_set.max = -1;
	} else {
		core_to_set.min = -1;
		core_to_set.max = -1;
		freq_to_set.min = -1;
		freq_to_set.max = -1;
	}

	last_enable = enable;

	update_userlimit_cpu_core(PPM_KIR_PERF_KERN, 1, &core_to_set);
	update_userlimit_cpu_freq(PPM_KIR_PERF_KERN, 1, &freq_to_set);
}

