// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Shane Chien <shane.chien@mediatek.com>
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 *  mt_sco_afe_connection.c
 *
 * Project:
 * --------
 *   MT6583  Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/
#ifndef _MT_SOC_AFE_CONNECTION_H_
#define _MT_SOC_AFE_CONNECTION_H_

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"

/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E
 *****************************************************************************/
bool SetConnectionState(unsigned int ConnectionState, unsigned int Input, unsigned int Output);

#endif
