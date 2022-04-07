// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __MT_LBC_H__
#define __MT_LBC_H__

#define PPM_KIR_PERF 0
#define PPM_KIR_PERF_KERN 1
#define PPM_KIR_FBC 2
#define PPM_KIR_WIFI 3
#define PPM_MAX_KIR 4


struct ppm_limit_data {
	int min;
	int max;
};

int update_userlimit_cpu_freq(int kicker, int num_cluster, struct ppm_limit_data *freq_limit);
int update_userlimit_cpu_core(int kicker, int num_cluster, struct ppm_limit_data *core_limit);

#endif /* __MT_LBC_H__ */
