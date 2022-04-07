// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

/******************************************************************************
*
 *
 * Filename:
 * ---------
 *   AudDrv_Common_func.h
 *
 * Project:
 * --------
 *   MT6583 FPGA LDVT Audio Driver
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 *   George
 *
 *---------------------------------------------------------------------------
---
 *
 *

*******************************************************************************/

#ifndef AUDIO_COMMON_FUNC_H
#define AUDIO_COMMON_FUNC_H

bool get_voice_bt_status(void);
bool get_voice_status(void);
bool get_voice_md2_bt_status(void);
bool get_voice_md2_status(void);
void Auddrv_Read_Efuse_HPOffset(void);
bool get_internalmd_status(void);

/* for AUDIO_DL2_ISR_COPY_SUPPORT */
void mtk_dl2_copy_l(void);
void mtk_dl2_copy2buffer(const void *addr, uint32_t size);

#endif
