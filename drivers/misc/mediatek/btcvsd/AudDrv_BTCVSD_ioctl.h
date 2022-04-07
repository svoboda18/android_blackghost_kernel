// SPDX-License-Identifier: GPL-2.0
//
// Mediatek BT SCO CVSD/MSBC Driver
//
// Copyright (c) 2015 MediaTek Inc.
// Author: Tina Tsai <tina.tsai@mediatek.com>

#ifndef _AUDDRV_BTCVSD_IOCTL_MSG_H
#define _AUDDRV_BTCVSD_IOCTL_MSG_H

/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/

/*below is control message*/
#define AUD_DRV_BTCVSD_IOC_MAGIC 'C'

#define ALLOCATE_FREE_BTCVSD_BUF _IOWR(AUD_DRV_BTCVSD_IOC_MAGIC, 0xE0, unsigned int)
#define SET_BTCVSD_STATE         _IOWR(AUD_DRV_BTCVSD_IOC_MAGIC, 0xE1, unsigned int)
#define GET_BTCVSD_STATE         _IOWR(AUD_DRV_BTCVSD_IOC_MAGIC, 0xE2, unsigned int)
#define GET_BTCVSD_RX_TIME_BUFFER_INFO _IOWR(AUD_DRV_BTCVSD_IOC_MAGIC, 0xE3, unsigned long long)
#define GET_BTCVSD_TX_TIME_BUFFER_INFO _IOWR(AUD_DRV_BTCVSD_IOC_MAGIC, 0xE4, unsigned long long)


#endif

