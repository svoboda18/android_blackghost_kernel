 //SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   gc6153serialyuv_Sensor.c
 *
 * Project:
 * --------
 *   Maui_sw
 *
 * Description:
 * ------------
 *   Image sensor driver function_11A&11B
 *
 * Author:
 * -------
 *   Mormo_Hui
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$1.0.0
 *
 * $Modtime:$2012/11/15
 *
 * $Log:$
 * 2012/11/15 Fristly Released By Mormo.
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "GC6153_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc6153serialyuv_Sensor.h"

#define LOG_1 LOG_INF("GC6153 serial\n")
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
extern int iReadRegI2CWithCamIndex(u32, u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2CWithCamIndex(u32, u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);


#define GC6153_I2C_WRITE_ID 0x80


//#define IMAGE_NORMAL
//#define IMAGE_H_MIRROR
#define IMAGE_V_MIRROR
//#define IMAGE_HV_MIRROR

#ifdef IMAGE_NORMAL
#define MIRROR 0x7c
#define AD_NUM 0x05
#define COL_SWITCH 0x18
#endif

#ifdef IMAGE_H_MIRROR
#define MIRROR 0x69
#define AD_NUM 0x04
#define COL_SWITCH 0x19
#endif

#ifdef IMAGE_V_MIRROR
#define MIRROR 0x7e
#define AD_NUM 0x05
#define COL_SWITCH 0x18
#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 0x6b
#define AD_NUM 0x04
#define COL_SWITCH 0x19
#endif


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	int ret = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };
	ret = iReadRegI2CWithCamIndex(2,pu_send_cmd, 1, (u8*)&get_byte, 1, GC6153_I2C_WRITE_ID);
	if (ret < 0)
		return ret;
	else
		return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };
	iWriteRegI2CWithCamIndex(2,pu_send_cmd, 2, GC6153_I2C_WRITE_ID);
}


#define GC6153_SERIAL_SET_PAGE0 write_cmos_sensor(0xfe , 0x00)
#define GC6153_SERIAL_SET_PAGE1 write_cmos_sensor(0xfe , 0x01)
#define GC6153_SERIAL_SET_PAGE2 write_cmos_sensor(0xfe , 0x02)

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 start
//add sensor clk config to enable sensor
    printk("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */

    sensor_info->SensorMasterClockSwitch = 0;
    sensor_info->SensorDrivingCurrent = ISP_DRIVING_4MA;
    sensor_info->SensorClockFreq = 24;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 end
    return ERROR_NONE;
}
static kal_uint32 get_resolution(
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
    return 0;
}
static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    return 0;
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 retry = 2;
    do {
        *sensor_id = (read_cmos_sensor(0xf0)<<8) | read_cmos_sensor(0xf1);
        if (*sensor_id == GC6153_SENSOR_ID) {
            printk("i2c write id: 0x%x, sensor id: 0x%x\n",
                GC6153_I2C_WRITE_ID, *sensor_id);
            return ERROR_NONE;
        }
        printk("Read sensor id fail, id: 0x%x\n",
            GC6153_I2C_WRITE_ID);
        retry--;
    } while (retry > 0);

	if (*sensor_id != GC6153_SENSOR_ID) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   read_Y_average
*
* DESCRIPTION
*   This function read the registers of Y_average
* RETURNS
*   Y_average or ErrorCode
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_int32 read_Y_average(void)
{
    GC6153_SERIAL_SET_PAGE0;
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 start
    return (kal_int16)read_cmos_sensor(0x93);
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 end
}
/*************************************************************************
* FUNCTION
*   sensor_init
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*   IMPORTANT NOTICE:
*     the output format should be YUV422, order: YUYV
*     data output should be at pclk falling edge
*     VSYNC should be low active
*     HSYNC should be hight active
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void sensor_init(void)
	{

		/*SYS*/
	write_cmos_sensor(0xfe, 0xa0);
	write_cmos_sensor(0xfe, 0xa0);
	write_cmos_sensor(0xfe, 0xa0);
	write_cmos_sensor(0xfa, 0x11);
	write_cmos_sensor(0xfc, 0x00);
	write_cmos_sensor(0xf6, 0x00);
	write_cmos_sensor(0xfc, 0x12);

	/*ANALOG & CISCTL*/
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x01, 0x40);
	write_cmos_sensor(0x02, 0x12);
	write_cmos_sensor(0x0d, 0x40);
	write_cmos_sensor(0x14, MIRROR);
	write_cmos_sensor(0x16, AD_NUM);
	write_cmos_sensor(0x17, COL_SWITCH);
	write_cmos_sensor(0x1c, 0x31);
	write_cmos_sensor(0x1d, 0xb9);
	write_cmos_sensor(0x1f, 0x1a);
	write_cmos_sensor(0x73, 0x20);
	write_cmos_sensor(0x74, 0x71);
	write_cmos_sensor(0x77, 0x22);
	write_cmos_sensor(0x7a, 0x08);
	write_cmos_sensor(0x11, 0x18);
	write_cmos_sensor(0x13, 0x48);
	write_cmos_sensor(0x12, 0xc8);
	write_cmos_sensor(0x70, 0xc8);
	write_cmos_sensor(0x7b, 0x18);
	write_cmos_sensor(0x7d, 0x30);
	write_cmos_sensor(0x7e, 0x02);

	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x10);
	write_cmos_sensor(0xfe, 0x00);

	write_cmos_sensor(0x49, 0x61);
	write_cmos_sensor(0x4a, 0x40);
	write_cmos_sensor(0x4b, 0x58);

	/*ISP*/
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x39, 0x02);
	write_cmos_sensor(0x3a, 0x80);
	write_cmos_sensor(0x20, 0x7e);
	write_cmos_sensor(0x26, 0x87);

	/*BLK*/
	write_cmos_sensor(0x33, 0x10);
	write_cmos_sensor(0x37, 0x06);
	write_cmos_sensor(0x2a, 0x21);

	/*GAIN*/
	write_cmos_sensor(0x3f, 0x16);

	/*DNDD*/
	write_cmos_sensor(0x52, 0xa6);
	write_cmos_sensor(0x53, 0x81);
	write_cmos_sensor(0x54, 0x43);
	write_cmos_sensor(0x56, 0x78);
	write_cmos_sensor(0x57, 0xaa);
	write_cmos_sensor(0x58, 0xff);

	/*ASDE*/
	write_cmos_sensor(0x5b, 0x60);
	write_cmos_sensor(0x5c, 0x50);
	write_cmos_sensor(0xab, 0x2a);
	write_cmos_sensor(0xac, 0xb5);

	/*INTPEE*/
	write_cmos_sensor(0x5e, 0x06);
	write_cmos_sensor(0x5f, 0x06);
	write_cmos_sensor(0x60, 0x44);
	write_cmos_sensor(0x61, 0xff);
	write_cmos_sensor(0x62, 0x69);
	write_cmos_sensor(0x63, 0x13);

	/*CC*/
	write_cmos_sensor(0x65, 0x13);
	write_cmos_sensor(0x66, 0x26);
	write_cmos_sensor(0x67, 0x07);
	write_cmos_sensor(0x68, 0xf5);
	write_cmos_sensor(0x69, 0xea);
	write_cmos_sensor(0x6a, 0x21);
	write_cmos_sensor(0x6b, 0x21);
	write_cmos_sensor(0x6c, 0xe4);
	write_cmos_sensor(0x6d, 0xfb);

	/*YCP*/
	write_cmos_sensor(0x81, 0x3b);
	write_cmos_sensor(0x82, 0x3b);
	write_cmos_sensor(0x83, 0x4b);
	write_cmos_sensor(0x84, 0x90);
	write_cmos_sensor(0x86, 0xf0);
	write_cmos_sensor(0x87, 0x1d);
	write_cmos_sensor(0x88, 0x16);
	write_cmos_sensor(0x8d, 0x74);
	write_cmos_sensor(0x8e, 0x25);

	/*AEC*/
	write_cmos_sensor(0x90, 0x36);
	write_cmos_sensor(0x92, 0x43);
	write_cmos_sensor(0x9d, 0x32);
	write_cmos_sensor(0x9e, 0x81);
	write_cmos_sensor(0x9f, 0xf4);
	write_cmos_sensor(0xa0, 0xa0);
	write_cmos_sensor(0xa1, 0x04);
	write_cmos_sensor(0xa3, 0x2d);
	write_cmos_sensor(0xa4, 0x01);

	/*AWB*/
	write_cmos_sensor(0xb0, 0xc2);
	write_cmos_sensor(0xb1, 0x1e);
	write_cmos_sensor(0xb2, 0x10);
	write_cmos_sensor(0xb3, 0x20);
	write_cmos_sensor(0xb4, 0x2d);
	write_cmos_sensor(0xb5, 0x1b);
	write_cmos_sensor(0xb6, 0x2e);
	write_cmos_sensor(0xb8, 0x13);
	write_cmos_sensor(0xba, 0x60);
	write_cmos_sensor(0xbb, 0x62);
	write_cmos_sensor(0xbd, 0x78);
	write_cmos_sensor(0xbe, 0x55);
	write_cmos_sensor(0xbf, 0xa0);
	write_cmos_sensor(0xc4, 0xe7);
	write_cmos_sensor(0xc5, 0x15);
	write_cmos_sensor(0xc6, 0x16);
	write_cmos_sensor(0xc7, 0xeb);
	write_cmos_sensor(0xc8, 0xe4);
	write_cmos_sensor(0xc9, 0x16);
	write_cmos_sensor(0xca, 0x16);
	write_cmos_sensor(0xcb, 0xe9);
	write_cmos_sensor(0x22, 0xf8);

	/*SPI*/
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x01, 0x01);
	write_cmos_sensor(0x02, 0x02);
	write_cmos_sensor(0x03, 0x20);
	write_cmos_sensor(0x04, 0x20);
	write_cmos_sensor(0x0a, 0x00);
	write_cmos_sensor(0x13, 0x10);
	write_cmos_sensor(0x24, 0x00);
	write_cmos_sensor(0x28, 0x03);
	write_cmos_sensor(0xfe, 0x00);

	/*OUTPUT*/
	write_cmos_sensor(0xf2, 0x00);
	write_cmos_sensor(0xfe, 0x00);

	}


/*************************************************************************
* FUNCTION
*   open
*
* DESCRIPTION
*   This function read sensor id and init sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   Error code
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    int y_ave = 0;
    kal_uint32 SensorId = 0;
    get_imgsensor_id(&SensorId);

    if (GC6153_SENSOR_ID == SensorId)
    {
        printk("GC6153_SERIAL_SENSOR_ID success");
        sensor_init(); /* apply the sensor initial setting */
        write_cmos_sensor(0xfe,0x00);
        y_ave = read_cmos_sensor(0x93);
        printk("gc6153y_ave=%d\n",y_ave);
    }else{
        printk("GC6153_SERIAL_SENSOR_ID fail(0x%x)",SensorId);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   close
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   Error code
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    return 0;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	INT32 *feature_return_para_32 = (INT32 *) feature_para;
	switch (feature_id) {
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(feature_return_para_32);
        break;

    case SENSOR_FEATURE_GET_Y_AVERAGE:
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 start
    case SENSOR_FEATURE_GET_AE_STATUS:
//ITD:modify KBQJHLYW-1834 by quan.chang 190301 end
        *feature_return_para_32 = read_Y_average();
        break;

	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 GC6153_SERIAL_YUV_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}