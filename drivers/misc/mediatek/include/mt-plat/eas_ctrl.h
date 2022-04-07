/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef _EAS_CTRL_H
#define _EAS_CTRL_H


enum {
	CGROUP_ROOT = 0,
	CGROUP_FG,
	CGROUP_BG,
	CGROUP_TA,
	CGROUP_RT,
	NR_CGROUP
};

enum {
	EAS_KIR_PERF = 0,
	EAS_KIR_BOOT,
	EAS_KIR_TOUCH,
	EAS_MAX_KIR
};

extern int boost_write_for_perf_idx(int group_idx, int boost_value);

extern int update_eas_boost_value(int kicker, int cgroup_idx, int value);

#endif /* _EAS_CTRL_H */
