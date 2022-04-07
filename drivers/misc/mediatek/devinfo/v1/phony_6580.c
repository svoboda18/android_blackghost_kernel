// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/types.h>

enum idle_lock_spm_id {
	    IDLE_SPM_LOCK_VCORE_DVFS = 0,
};
void __weak idle_lock_spm(enum idle_lock_spm_id id) {}
void __weak idle_unlock_spm(enum idle_lock_spm_id id) {}
void __weak enable_dpidle_by_bit(int id) {}
void __weak disable_dpidle_by_bit(int id) {}
void __weak enable_soidle_by_bit(int id) {}
void __weak disable_soidle_by_bit(int id) {}

