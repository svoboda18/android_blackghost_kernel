// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __CCCI_SUPPORT_H__
#define __CCCI_SUPPORT_H__

#include "ccci_modem.h"
#include "ccci_debug.h"

struct ccci_setting {
	int sim_mode;
	int slot1_mode;		/* 0:CDMA 1:GSM 2:WCDMA 3:TDCDMA */
	int slot2_mode;		/* 0:CDMA 1:GSM 2:WCDMA 3:TDCDMA */
};

void ccci_reload_md_type(struct ccci_modem *md, int type);
int ccci_get_sim_switch_mode(void);
int ccci_store_sim_switch_mode(struct ccci_modem *md, int simmode);
struct ccci_setting *ccci_get_common_setting(int md_id);

#endif				/* __CCCI_SUPPORT_H__ */
