/*
 *
 * svbFocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
 * Copyright (c) 2020, svoboda18
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_SVB_FOCALTECH_COMMON_H__
#define __LINUX_SVB_FOCALTECH_COMMON_H__

#include "svbfocaltech_config.h"

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_VERSION      "svbFocaltech V1.0 2021"
#define FTS_DRIVER_NAME         "svbfts_ts"

#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)((x >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)((x >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)((x >> 24) & 0xFF)
#define FLAGBIT(x)              (0x00000001 << (x))
#define FLAGBITS(x, y)          ((0xFFFFFFFF >> (32 - (y) - 1)) << (x))

#define FLAG_ICSERIALS_LEN      8
#define FLAG_HID_BIT            10
#define FLAG_IDC_BIT            11

#define I2C_BUFFER_LENGTH_MAXINUM           256
#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
#define FTS_GESTURE_INIT_PIXLE_X            TPD_RES_X
#define FTS_GESTURE_INIT_PIXLE_Y            TPD_RES_Y
#define FTS_GESTURE_CUT_PIXLE_X             10
#define FTS_GESTURE_CUT_PIXLE_Y             5
#define FTS_GESTURE_TIME_FRAME              60
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_READ_ID                     0x90
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_FLOW_WORK_CNT               0x91
#define FTS_REG_WORKMODE                    0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      0x40
#define FTS_REG_WORKMODE_WORK_VALUE         0x00
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP_VALUE      0x03
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4

#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/

#if FTS_DEBUG_EN
#define FTS_DEBUG_LEVEL 2

#define FTS_DEBUG(fmt, args...) printk("[svbFTS][%s] "fmt"\n", __func__, ##args)
#define FTS_FUNC_ENTER() printk("[svbFTS][%s]: Enter\n", __func__)
#define FTS_FUNC_EXIT()  printk("[svbFTS][%s]: Exit(%d)\n", __func__, __LINE__)

#else
#define FTS_DEBUG(fmt, args...)
#define FTS_FUNC_ENTER()
#define FTS_FUNC_EXIT()
#endif

#define FTS_INFO(fmt, args...) printk(KERN_INFO "[svbFTS][INFO][%s] "fmt"\n", __func__, ##args)
#define FTS_ERROR(fmt, args...) printk(KERN_ERR "[svbFTS][ERROR][%s] "fmt"\n", __func__, ##args)

#endif /* __LINUX_SVB_FOCALTECH_COMMON_H__ */
