#ifndef _DUMMY_CAMERA_H_
#define _DUMMY_CAMERA_H_

#include "kd_imgsensor_define.h"

/*************custom define start**************/
//#define CONFIG_DCAM_CAM2_BF3905	1
#define  CONFIG_DCAM_CAM2_SP0A08	1
//#define  CONFIG_DCAM_CAM2_GC0310	0
//don't support compat sensor, now
//dummy camera2:
#ifdef CONFIG_DCAM_CAM2_GC0310
#define GC0310_MIPI_YUV_MAIN2_I2C_ADDR				0x21
#define GC0310_MIPI_YUV_MAIN2_SENSOR_ID_ADDR		0xf0 //0xfe
#define GC0310_MIPI_YUV_MAIN2_SENSOR_ID				0xa3//10   0xa3-->gc0310   0x23-->gc032a
#define GC0310_MIPI_YUV_MAIN2_SENSOR_LIGHT_ADDR			0Xef 	//read the light reg
#define GC0310_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_ADDR			0xfe 	//write cmd to switch page addr
#define GC0310_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_NUM			0x00 	//write cmd to switch page num
#define GC0310_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_LIGHT_ADDR			0xfe 	//write cmd to switch page light addr
#define GC0310_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_LIGHT_NUM			0x00 	//write cmd to switch page light num
#endif

#ifdef CONFIG_DCAM_CAM2_GC032A
#define GC032A_MIPI_YUV_MAIN2_I2C_ADDR				0x21
#define GC032A_MIPI_YUV_MAIN2_SENSOR_ID_ADDR		0xf0 //0xfe
#define GC032A_MIPI_YUV_MAIN2_SENSOR_ID				0x03//10   0xa3-->gc0310   0x23-->gc032a
#define GC032A_MIPI_YUV_MAIN2_SENSOR_LIGHT_ADDR			0Xef 	//read the light reg
#define GC032A_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_ADDR			0xfe 	//write cmd to switch page addr
#define GC032A_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_NUM			0x00 	//write cmd to switch page num
#define GC032A_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_LIGHT_ADDR			0xfe 	//write cmd to switch page light addr
#define GC032A_MIPI_YUV_MAIN2_SENSOR_SWITCH_PAGE_LIGHT_NUM			0x00 	//write cmd to switch page light num
#endif

#ifdef CONFIG_DCAM_CAM2_SP0A08
#define SP0A08_MIPI_RAW_MAIN2_I2C_ADDR				0x3D	
#define SP0A08MAIN2MIPI_SENSOR_ID_ADDR				0x02	//read the ID reg
#define SP0A08MAIN2MIPI_SENSOR_ID					0xA2
#define SP0A08MAIN2MIPI_SENSOR_LIGHT_ADDR			0Xf2 	//read the light reg
#define SP0A08MAIN2MIPI_SENSOR_SWITCH_PAGE_ADDR			0xfd 	//write cmd to switch page addr
#define SP0A08MAIN2MIPI_SENSOR_SWITCH_PAGE_NUM			0x00 	//write cmd to switch page num
#define SP0A08MAIN2MIPI_SENSOR_SWITCH_PAGE_LIGHT_ADDR			0xfd 	//write cmd to switch page light addr
#define SP0A08MAIN2MIPI_SENSOR_SWITCH_PAGE_LIGHT_NUM			0x00 	//write cmd to switch page light num
#endif

#ifdef CONFIG_DCAM_CAM2_GC030A
#define GC030A_MIPI_RAW_MAIN2_I2C_ADDR				0x21
#define GC030AMIPI_SENSOR_ID_MAIN2					0x030A
#define SENSOR_DRVNAME_GC030A_MIPI_RAW_MAIN2    		"gc030a_mipi_raw_main2"
#endif

#ifdef CONFIG_DCAM_CAM2_BF3905
#define BF3905_MIPI_RAW_MAIN2_I2C_ADDR				0x6E
#define BF3905_MIPI_RAW_SENSOR_ID_MAIN2				0x39
#define BF3905_MIPI_RAW_SENSOR_ID_MAIN2_ADDR		0xFC
#define BF3905_MIPI_RAW_LIGHT_ADDR    		        0x88
#endif


/*************custom define end**************/
#endif
