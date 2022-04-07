// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SMI_DEBUG_H__
#define __SMI_DEBUG_H__

#include "smi_configuration.h"
#define SMI_DBG_LARB_SELECT(smi_dbg_larb, n) ((smi_dbg_larb) & (1<<n))

void smi_dumpCommonDebugMsg(int gce_buffer);
void smi_dumpLarbDebugMsg(unsigned int larb_indx, int gce_buffer);
void smi_dumpDebugMsg(void);
void smi_dump_larb_m4u_register(unsigned int larb_indx);
int smi_debug_bus_hanging_detect(unsigned short larbs, int show_dump,
	int gce_buffer, int enable_m4u_reg_dump);
#endif /* __SMI_DEBUG_H__ */
