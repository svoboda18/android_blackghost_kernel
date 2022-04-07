// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __MT_CHARGING_SEL_INTF_H
#define __MT_CHARGING_SEL_INTF_H

#include <mt-plat/charging.h>

extern int is_bq24157_exist(void);
extern int is_fan5405_exist(void);
extern signed int fan5405_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data);
extern signed int bq24157_chr_control_interface(CHARGING_CTRL_CMD cmd, void *data);

struct charger_candidate_table {
	char *name;
	int (*exist_fun)(void);
	signed int (*chr_ctrl_intf)(CHARGING_CTRL_CMD cmd, void *data);
};

struct charger_candidate_table charger_candidate_func[] = {
	{"bq24157", is_bq24157_exist, bq24157_chr_control_interface},
	{"fan5405", is_fan5405_exist, fan5405_chr_control_interface}
};

#endif /* __MT_CHARGING_SEL_INTF_H */
