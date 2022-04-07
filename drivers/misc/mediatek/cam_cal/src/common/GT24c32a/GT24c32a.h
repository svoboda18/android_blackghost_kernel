//SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */
#ifndef __GT24C32A_H
#define __GT24C32A_H
#include <linux/i2c.h>


/*extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);*/
extern int iBurstReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
/*extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);*/
extern int iReadReg(u16 a_u2Addr , u8 *a_puBuff , u16 i2cId);

unsigned int gt24c32a_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size);


#endif /* __EEPROM_H */

