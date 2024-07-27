/*
 *
 * svbFocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
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
 
#ifndef _LINUX_SVB_FOCLATECH_CONFIG_H_
#define _LINUX_SVB_FOCLATECH_CONFIG_H_

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define FTS_DEBUG_EN 0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define FTS_MT_PROTOCOL_B_EN   1

/*
 * Report Pressure in multitouch
 * 1:enable(default),0:disable
 */
#define FTS_REPORT_PRESSURE_EN 1

/*
 * Gesture function enable
 * default: disable
 */
#define FTS_GENERIC_GESTURE_EN 1

/*
 * Set gesture on if enable
 */
#if FTS_GENERIC_GESTURE_EN
#define FTS_GESTURE_EN 1
#endif

/*
 * TPD power enable
 * enable it when TP requires regulator power enable
 * default: enable
 */
#define FTS_POWER_SOURCE_CUST_EN 1

#endif /* _LINUX_SVB_FOCLATECH_CONFIG_H_ */
