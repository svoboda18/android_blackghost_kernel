 //SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   image_sensor_GC6153_SERIAL.h
 *
 * Project:
 * --------
 *   MAUI_sw
 *
 * Description:
 * ------------
 *   CMOS sensor header file
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
 * $Modtime:$2011/08/25
 *
 * $Log:$
 * 2011/08/25 Firstly Released By Mormo.
 *
 * Initial revision.
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef _IMAGE_SENSOR_GC6153_SERIAL_H
#define _IMAGE_SENSOR_GC6153_SERIAL_H


/* This used for debug phase use only, to speed up the initial setting modify, 
    NOTICE: It need to be marked when MP release version, or else it will effect the enter
    camera time and waste some ram. */
//#define GC6153_SERIAL_LOAD_FROM_T_FLASH

/* SENSOR MASTER CLOCK */
#define GC6153_SERIAL_MCLK                          24000000

// TODO: modify sensor dependent macro value here

/* SENSOR I2C WRITE ID */
#define GC6153_SERIAL_WRITE_ID_0                    (0x80)
#define GC6153_SERIAL_WRITE_ID_1                    (0xFF)
#define GC6153_SERIAL_WRITE_ID_2                    (0xFF)
#define GC6153_SERIAL_WRITE_ID_3                    (0xFF)

/* SENSOR I2C ADDR/DATA WIDTH */
#define GC6153_SERIAL_I2C_ADDR_BITS                 CAMERA_SCCB_8BIT /* CAMERA_SCCB_8BIT / CAMERA_SCCB_16BIT */
#define GC6153_SERIAL_I2C_DATA_BITS                 CAMERA_SCCB_8BIT /* CAMERA_SCCB_8BIT / CAMERA_SCCB_16BIT */

/* SENSOR PREVIEW SIZE (5M:1296x972 or 640x480, 3M: 1024x768 or 640x480, 2M: 800x600, 1.3M: 640x512, VGA: 640x480, CIF: 352x288) */
#define GC6153_SERIAL_IMAGE_SENSOR_PV_WIDTH         (240)
#define GC6153_SERIAL_IMAGE_SENSOR_PV_HEIGHT        (320)
/* SENSOR CAPTURE SIZE (5M: 2592x1944, 3M: 2048x1536, 2M: 1600x1200, 1.3M: 1280x1024, VGA: 640x480, CIF: 352x288) */
#define GC6153_SERIAL_IMAGE_SENSOR_FULL_WIDTH       (240)
#define GC6153_SERIAL_IMAGE_SENSOR_FULL_HEIGHT      (320)

/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define GC6153_SERIAL_PV_PERIOD_PIXEL_NUMS          (240)
#define GC6153_SERIAL_PV_PERIOD_LINE_NUMS           (320)
#define GC6153_SERIAL_FULL_PERIOD_PIXEL_NUMS        (240)
#define GC6153_SERIAL_FULL_PERIOD_LINE_NUMS         (320)

/* SENSOR LINELENGTH&FRAMELENGTH LIMITATION */
#define GC6153_SERIAL_MAX_PV_LINELENGTH             (4095 + 694) /* register limitation */
#define GC6153_SERIAL_MAX_PV_FRAMELENGTH            (4095 + 488) /* register limitation */
#define GC6153_SERIAL_MAX_CAP_LINELENGTH            (GC6153_SERIAL_MAX_PV_LINELENGTH) /* register limitation */
#define GC6153_SERIAL_MAX_CAP_FRAMELENGTH           (GC6153_SERIAL_MAX_PV_FRAMELENGTH) /* register limitation */

/* SENSOR SHUTTER MARGIN */
#define GC6153_SERIAL_SHUTTER_MARGIN                (1)

/* SENSOR DELAY FRAME */
#define GC6153_SERIAL_FIRST_PREVIEW_DELAY_FRAME     (3)
#define GC6153_SERIAL_PREVIEW_DELAY_FRAME           (3)
#define GC6153_SERIAL_CAPTURE_DELAY_FRAME           (2)

/* SENSOR PREVIEW/CAPTURE INTERNAL CLOCK */
#if GC6153_SERIAL_MCLK == 24000000
    #if (defined(DRV_ISP_6276_SERIES) || defined(DRV_ISP_MT6236_HW_SUPPORT))
    /* pclk limitation is 96MHz, preview pclk = min(pclk@30fps,96MHz), capture pclk = min(pclk@15fps,96MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           12000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          12000000
    #elif (defined(DRV_ISP_MT6268_HW_SUPPORT))
    /* pclk limitation is 52MHz, preview pclk = min(pclk@30fps,52MHz), capture pclk = min(pclk@15fps,52MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           12000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          12000000
    #else
    /* pclk limitation is 48MHz, preview pclk = min(pclk@30fps,48MHz), capture pclk = min(pclk@15fps,48MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           12000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          12000000
    #endif
#else
    #if (defined(DRV_ISP_6276_SERIES) || defined(DRV_ISP_MT6236_HW_SUPPORT))
    /* pclk limitation is 96MHz, preview pclk = min(pclk@30fps,96MHz), capture pclk = min(pclk@15fps,96MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           13000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          13000000
    #elif (defined(DRV_ISP_MT6268_HW_SUPPORT))
    /* pclk limitation is 52MHz, preview pclk = min(pclk@30fps,52MHz), capture pclk = min(pclk@15fps,52MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           13000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          13000000
    #else
    /* pclk limitation is 48MHz, preview pclk = min(pclk@30fps,48MHz), capture pclk = min(pclk@15fps,48MHz) */
    #define GC6153_SERIAL_PV_INTERNAL_CLK           13000000
    #define GC6153_SERIAL_CAP_INTERNAL_CLK          13000000
    #endif
#endif

/* ===============================================================================
   ========================= No Changing The Macro Below =========================
   ===============================================================================
*/

/* CONFIG THE ISP GRAB START X & START Y, CONFIG THE ISP GRAB WIDTH & HEIGHT */
#define GC6153_SERIAL_PV_GRAB_START_X               (0)
#define GC6153_SERIAL_PV_GRAB_START_Y               (1)
#define GC6153_SERIAL_FULL_GRAB_START_X             (0)
#define GC6153_SERIAL_FULL_GRAB_START_Y             (1)
#define GC6153_SERIAL_PV_GRAB_WIDTH                 (GC6153_SERIAL_IMAGE_SENSOR_PV_WIDTH - 2)
#define GC6153_SERIAL_PV_GRAB_HEIGHT                (GC6153_SERIAL_IMAGE_SENSOR_PV_HEIGHT - 2)
#define GC6153_SERIAL_FULL_GRAB_WIDTH               (GC6153_SERIAL_IMAGE_SENSOR_FULL_WIDTH - 2)
#define GC6153_SERIAL_FULL_GRAB_HEIGHT              (GC6153_SERIAL_IMAGE_SENSOR_FULL_HEIGHT - 2)

/* FLICKER OF FREQUENCY */
#define GC6153_SERIAL_50HZ                          100
#define GC6153_SERIAL_60HZ                          120

/* RESET/POWER DOWN PIN CONTROL */
#define GC6153_SERIAL_SET_RST_LOW                   CamRstPinCtrl(GC6153_SERIALSensor.SensorIdx, 0)
#define GC6153_SERIAL_SET_RST_HIGH                  CamRstPinCtrl(GC6153_SERIALSensor.SensorIdx, 1)
#define GC6153_SERIAL_SET_PDN_LOW                   CamPdnPinCtrl(GC6153_SERIALSensor.SensorIdx, 0)
#define GC6153_SERIAL_SET_PDN_HIGH                  CamPdnPinCtrl(GC6153_SERIALSensor.SensorIdx, 1)

/* HW I2C SPEED */
#define GC6153_SERIAL_HW_I2C_SPEED                  100 /* Kbps */

/* FRAME RATE UNIT */
#define GC6153_SERIAL_FPS(x)                        ((kal_uint32)(10 * (x)))
#endif
