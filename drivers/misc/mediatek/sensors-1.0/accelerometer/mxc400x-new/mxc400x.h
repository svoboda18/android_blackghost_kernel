/*Transsion Top Secret*/
/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Aaron Peng
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for MXC400X accelorometer sensor chip.
 */
#ifndef __MXC400X_H__
#define __MXC400X_H__


#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define GSENSOR_IOCTL_GET_DELAY             _IOR(GSENSOR, 0x10, int)
#define GSENSOR_IOCTL_GET_STATUS            _IOR(GSENSOR, 0x11, int)
#define GSENSOR_IOCTL_GET_DATA              _IOR(GSENSOR, 0x12, int[3])
#define GSENSOR_IOCTL_SET_DATA              _IOW(GSENSOR, 0x13, int[3])
#define GSENSOR_IOCTL_GET_TEMP              _IOR(GSENSOR, 0x14, int)
#define GSENSOR_IOCTL_GET_DANT              _IOR(GSENSOR, 0x15, int[4])
#define GSENSOR_IOCTL_READ_REG              _IOR(GSENSOR, 0x19, int)
#define GSENSOR_IOCTL_GET_POWER_STATUS      _IOR(GSENSOR, 0x20, int)
#define GSENSOR_IOCTL_GET_LAYOUT            _IOR(GSENSOR, 0x21, int)
#define GSENSOR_IOCTL_WRITE_REG             _IOW(GSENSOR, 0x1A, int)
#define GSENSOR_IOCTL_GET_OPEN_STATUS       _IOR(GSENSOR, 0x22, int)

#ifdef CONFIG_COMPAT
#define COMPAT_GSENSOR_IOCTL_GET_DELAY             _IOR(GSENSOR, 0x10, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_STATUS            _IOR(GSENSOR, 0x11, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_DATA              _IOR(GSENSOR, 0x12, compat_int_t[3])
#define COMPAT_GSENSOR_IOCTL_SET_DATA              _IOW(GSENSOR, 0x13, compat_int_t[3])
#define COMPAT_GSENSOR_IOCTL_GET_TEMP              _IOR(GSENSOR, 0x14, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_DANT              _IOR(GSENSOR, 0x15, compat_int_t[4])
#define COMPAT_GSENSOR_IOCTL_READ_REG              _IOR(GSENSOR, 0x19, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_POWER_STATUS      _IOR(GSENSOR, 0x20, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_LAYOUT            _IOR(GSENSOR, 0x21, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_WRITE_REG             _IOW(GSENSOR, 0x1A, compat_int_t)
#define COMPAT_GSENSOR_IOCTL_GET_OPEN_STATUS       _IOR(GSENSOR, 0x22, compat_int_t)
#endif
/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define MXC400X_DEV_NAME		"MXC400X"

#define MXC400X_I2C_ADDR		0x15
#define MXC400X_ID_1			0x02
#define MXC400X_ID_2			0x03

/* MXC400X register address */
#define MXC400X_REG_X			0x03
#define MXC400X_REG_Y			0x05
#define MXC400X_REG_Z			0x07
#define MXC400X_REG_TEMP		0x09
#define MXC400X_REG_CTRL		0x0D
#define MXC400X_REG_ID			0x0E


/*para setting*/
#define MXC400X_AWAKE			0x40	/* power on */
#define MXC400X_SLEEP			0x01	/* power donw */

#define MXC400X_BW_50HZ			0x00
#define MXC400X_RANGE_2G		0x00
#define MXC400X_RANGE_4G		0x20
#define MXC400X_RANGE_8G		0x40


/*ERR code*/
#define MXC400X_SUCCESS			0
#define MXC400X_ERR_I2C			-1
#define MXC400X_ERR_STATUS		-3
#define MXC400X_ERR_SETUP_FAILURE	-4
#define MXC400X_ERR_GETGSENSORDATA	-5
#define MXC400X_ERR_IDENTIFICATION	-6

#define MXC400X_BUFSIZE			256
#define MXC400X_STABLE_DELAY	10


#endif /* __MXC400X_H__ */

