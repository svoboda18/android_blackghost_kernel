//SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */
#ifndef __BRCB032GWZ_3_H
#define __BRCB032GWZ_3_H
#include <linux/i2c.h>

unsigned int brcb032gwz_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size);


#endif /* __CAM_CAL_H */

