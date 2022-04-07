// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/
/*
 * Definitions for TS3A225E Audio Switch chip.
 */
#ifndef __TS3A225E_H__
#define __TS3A225E_H__

int ts3a225e_read_byte(unsigned char cmd, unsigned char *returnData);
int ts3a225e_write_byte(unsigned char cmd, unsigned char writeData);

#endif
