/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

extern struct LCM_DRIVER hct_otm1285a_dsi_vdo_hd_boe;
extern struct LCM_DRIVER hct_ili9881_dsi_vdo_hd_cpt;
extern struct LCM_DRIVER hct_hx8394f_dsi_vdo_hd_cmi;
extern struct LCM_DRIVER hct_otm1282_dsi_vdo_hd_auo;
extern struct LCM_DRIVER hct_rm68200_dsi_vdo_hd_cpt;
extern struct LCM_DRIVER hct_nt35521s_dsi_vdo_hd_boe_50_xld;
extern struct LCM_DRIVER hct_hx8394d_dsi_vdo_hd_cmi;

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
