/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"

#if defined(BUILD_LK)
#else

#include <linux/proc_fs.h>   //proc file use 
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1280)
#define LCM_ID                       (0x1283)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE                                    0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
    
    /*
    Note :
    Data ID will depends on the following rule.
    
        count of parameters > 1 => Data ID = 0x39
        count of parameters = 1 => Data ID = 0x15
        count of parameters = 0 => Data ID = 0x05
    Structure Format :
    {DCS command, count of parameters, {parameter list}}
    {REGFLAG_DELAY, milliseconds of time, {}},
    ...
    Setting ending by predefined flag
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    */
   { 0xFF, 0x03, {0x98, 0x81, 0x07}},
    { 0x03, 0x01, {0x20}},
    { 0x04, 0x01, {0x06}},
    { 0x05, 0x01, {0x00}},
    { 0x06, 0x01, {0x01}},
    { 0x07, 0x01, {0x00}},
    { 0x08, 0x01, {0x00}},
    { 0x09, 0x01, {0x00}},
    { 0x0A, 0x01, {0x00}},
    { 0x0B, 0x01, {0x2F}},
    { 0x0C, 0x01, {0x00}},
    { 0x0D, 0x01, {0x00}},
    { 0x0E, 0x01, {0x00}},
    { 0x0F, 0x01, {0x00}},
    { 0x10, 0x01, {0x40}},
    { 0x11, 0x01, {0x02}},
    { 0x12, 0x01, {0x05}},
    { 0x13, 0x01, {0x90}},
    { 0x14, 0x01, {0x02}},
    { 0x15, 0x01, {0x00}},
    { 0x16, 0x01, {0x2F}},
    { 0x17, 0x01, {0x2F}},
    { 0x18, 0x01, {0x00}},
    { 0x19, 0x01, {0x00}},
    { 0x1A, 0x01, {0x00}},
    { 0x1B, 0x01, {0x50}},
    { 0x1C, 0x01, {0x2C}},
    { 0x1D, 0x01, {0x0C}},
    { 0x1E, 0x01, {0x00}},
    { 0x1F, 0x01, {0x87}},
    { 0x20, 0x01, {0x86}},
    { 0x21, 0x01, {0x00}},
    { 0x22, 0x01, {0x00}},
    { 0x23, 0x01, {0xC0}},
    { 0x24, 0x01, {0x30}},
    { 0x25, 0x01, {0x00}},
    { 0x26, 0x01, {0x00}},
    { 0x27, 0x01, {0x03}},
    { 0x30, 0x01, {0x01}},
    { 0x31, 0x01, {0x23}},
    { 0x32, 0x01, {0x45}},
    { 0x33, 0x01, {0x67}},
    { 0x34, 0x01, {0x89}},
    { 0x35, 0x01, {0xAB}},
    { 0x36, 0x01, {0x01}},
    { 0x37, 0x01, {0x23}},
    { 0x38, 0x01, {0x45}},
    { 0x39, 0x01, {0x67}},
    { 0x3A, 0x01, {0x89}},
    { 0x3B, 0x01, {0xAB}},
    { 0x3C, 0x01, {0xCD}},
    { 0x3D, 0x01, {0xEF}},
    { 0x50, 0x01, {0x11}},
    { 0x51, 0x01, {0x06}},
    { 0x52, 0x01, {0x0C}},
    { 0x53, 0x01, {0x0D}},
    { 0x54, 0x01, {0x0E}},
    { 0x55, 0x01, {0x0F}},
    { 0x56, 0x01, {0x02}},
    { 0x57, 0x01, {0x02}},
    { 0x58, 0x01, {0x02}},
    { 0x59, 0x01, {0x02}},
    { 0x5A, 0x01, {0x02}},
    { 0x5B, 0x01, {0x02}},
    { 0x5C, 0x01, {0x02}},
    { 0x5D, 0x01, {0x02}},
    { 0x5E, 0x01, {0x02}},
    { 0x5F, 0x01, {0x02}},
    { 0x60, 0x01, {0x05}},
    { 0x61, 0x01, {0x05}},
    { 0x62, 0x01, {0x05}},
    { 0x63, 0x01, {0x02}},
    { 0x64, 0x01, {0x01}},
    { 0x65, 0x01, {0x00}},
    { 0x66, 0x01, {0x08}},
    { 0x67, 0x01, {0x08}},
    { 0x68, 0x01, {0x0C}},
    { 0x69, 0x01, {0x0D}},
    { 0x6A, 0x01, {0x0E}},
    { 0x6B, 0x01, {0x0F}},
    { 0x6C, 0x01, {0x02}},
    { 0x6D, 0x01, {0x02}},
    { 0x6E, 0x01, {0x02}},
    { 0x6F, 0x01, {0x02}},
    { 0x70, 0x01, {0x02}},
    { 0x71, 0x01, {0x02}},
    { 0x72, 0x01, {0x02}},
    { 0x73, 0x01, {0x02}},
    { 0x74, 0x01, {0x02}},
    { 0x75, 0x01, {0x02}},
    { 0x76, 0x01, {0x05}},
    { 0x77, 0x01, {0x05}},
    { 0x78, 0x01, {0x05}},
    { 0x79, 0x01, {0x02}},
    { 0x7A, 0x01, {0x01}},
    { 0x7B, 0x01, {0x00}},
    { 0x7C, 0x01, {0x06}},
    { 0xFF, 0x03, {0x98, 0x81, 0x08}},
    { 0x76, 0x01, {0xC4}},
    { 0x78, 0x01, {0x02}},
    { 0x74, 0x01, {0x2F}},
    { 0x8E, 0x01, {0x15}},
    { 0x40, 0x01, {0x01}},
    { 0x84, 0x01, {0x81}},
    { 0xE3, 0x01, {0x75}},
    { 0x7D, 0x01, {0xCB}},
    { 0x7E, 0x01, {0x49}},
    { 0x49, 0x01, {0x10}},
    { 0x72, 0x01, {0x25}},
    { 0x32, 0x01, {0x05}},
    { 0x3C, 0x01, {0x05}},
    { 0xA3, 0x01, {0x40}},
    { 0xFF, 0x03, {0x98, 0x81, 0x01}},
    { 0x22, 0x01, {0x0A}},
    { 0x43, 0x01, {0x01}},
    { 0x31, 0x01, {0x00}},
    { 0x53, 0x01, {0x80}},
    { 0x55, 0x01, {0x15}},
    { 0x50, 0x01, {0x96}},
    { 0x51, 0x01, {0x96}},
    { 0xA0, 0x01, {0x06}},
    { 0xA1, 0x01, {0x18}},
    { 0xA2, 0x01, {0x24}},
    { 0xA3, 0x01, {0x0D}},
    { 0xA4, 0x01, {0x13}},
    { 0xA5, 0x01, {0x23}},
    { 0xA6, 0x01, {0x16}},
    { 0xA7, 0x01, {0x1A}},
    { 0xA8, 0x01, {0x80}},
    { 0xA9, 0x01, {0x1B}},
    { 0xAA, 0x01, {0x26}},
    { 0xAB, 0x01, {0x87}},
    { 0xAC, 0x01, {0x15}},
    { 0xAD, 0x01, {0x12}},
    { 0xAE, 0x01, {0x4C}},
    { 0xAF, 0x01, {0x28}},
    { 0xB0, 0x01, {0x30}},
    { 0xB1, 0x01, {0x67}},
    { 0xB2, 0x01, {0x71}},
    { 0xB3, 0x01, {0x33}},
    { 0xC0, 0x01, {0x0A}},
    { 0xC1, 0x01, {0x15}},
    { 0xC2, 0x01, {0x20}},
    { 0xC3, 0x01, {0x13}},
    { 0xC4, 0x01, {0x13}},
    { 0xC5, 0x01, {0x24}},
    { 0xC6, 0x01, {0x1C}},
    { 0xC7, 0x01, {0x1B}},
    { 0xC8, 0x01, {0x82}},
    { 0xC9, 0x01, {0x1C}},
    { 0xCA, 0x01, {0x2A}},
    { 0xCB, 0x01, {0x84}},
    { 0xCC, 0x01, {0x24}},
    { 0xCD, 0x01, {0x28}},
    { 0xCE, 0x01, {0x59}},
    { 0xCF, 0x01, {0x24}},
    { 0xD0, 0x01, {0x25}},
    { 0xD1, 0x01, {0x60}},
    { 0xD2, 0x01, {0x73}},
    { 0xD3, 0x01, {0x3F}},
    { 0xFF, 0x03, {0x98, 0x81, 0x00}},
    { 0x36, 0x01, {0x00}},
    { 0x11, 0x01, {0x00}},
    { 0xFE, 0x78, {0x00}},
    { 0x29, 0x01, {0x00}},
    { 0xFE, 0x14, {0x00}},
    { 0x01, 0x00, {0x00}},
    { 0x11, 0x01, {0x00}},
    { 0xFE, 0x78, {0x00}},
    { 0x29, 0x01, {0x00}},
    { 0xFE, 0x14, {0x00}},
    { 0x01, 0x00, {0x00}},
    { 0x28, 0x01, {0x00}},
    { 0xFE, 0x14, {0x00}},
    { 0x10, 0x01, {0x00}},
    { 0xFE, 0x78, {0x00}},
    { 0x01, 0x00, {0x00}},
    { 0x00, 0x6C, {0x2C, 0xB4, 0xC0, 0xB8, 0x81, 0x3B, 0xC0, 0x04, 0x81, 0x3B, 0xC0, 0x5C, 0x82, 0x3B, 0xC0, 0x18, 0x83, 0x3B, 0xC0, 0xE0, 0x82, 0x3B, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x80, 0x3B, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    { 0x10, 0x01, {0x00}},
    { 0xFE, 0x96, {0x00}},
    { 0x01, 0x00, {0x00}},
    {REGFLAG_DELAY,120,{}},

    {0x29,1,{0x00}},//Display ON 
    {REGFLAG_DELAY,20,{}},  

// Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    //{0x2C, 1, {0x00}},
    //{0x13, 1, {0x00}},
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
    // Display off sequence
    {0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//static int vcom=0x40;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
			/*case 0xd9:
			table[i].para_list[0]=vcom;
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            vcom+=2;
			break;
			*/
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE :
                break;
                
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
    
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

params->type   = LCM_TYPE_DSI;

  params->dsi.LANE_NUM = 3;
  params->dsi.packet_size = 256;
  params->dsi.vertical_backporch = 18;
  params->dsi.vertical_frontporch = 8;
  params->dsi.horizontal_backporch = 92;
  params->dsi.horizontal_frontporch = 76;
  params->dsi.PLL_CLOCK = 284;
  params->dsi.ssc_disable = 1;
  params->width = 720;
  params->type = 2;
  params->dsi.mode = 2;
  params->dsi.data_format.format = 2;
  params->dsi.intermediat_buffer_num = 2;
  params->dsi.PS = 2;
  params->dsi.vertical_sync_active = 2;
  params->dsi.horizontal_active_pixel = 720;
  params->height = 1280;
  params->dsi.vertical_active_line = 1280;
  params->dbi.te_mode = 0;
  params->dbi.te_edge_polarity = 0;
  params->dsi.data_format.color_order = 0;
  params->dsi.data_format.trans_seq = 0;
  params->dsi.data_format.padding = 0;
  params->dsi.horizontal_sync_active = 60;
  params->dsi.horizontal_blanking_pixel = 60;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);   //wqtao. enable
    #ifdef BUILD_LK
        printf("[erick-lk]%s\n", __func__);
    #else
        printk("[erick-k]%s\n", __func__);
    #endif
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    #ifdef BUILD_LK
        printf("[erick-lk]%s\n", __func__);
    #else
        printk("[erick-k]%s\n", __func__);
    #endif
#endif
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];


    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}
#endif

#if 0   //wqtao.        
static void lcm_setbacklight(unsigned int level)
{
    unsigned int default_level = 145;
    unsigned int mapped_level = 0;

    //for LGE backlight IC mapping table
    if(level > 255) 
            level = 255;

    if(level >0) 
            mapped_level = default_level+(level)*(255-default_level)/(255);
    else
            mapped_level=0;

    // Refresh value of backlight level.
    lcm_backlight_level_setting[0].para_list[0] = mapped_level;

    push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
#endif

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    /// please notice: the max return packet size is 1
    /// if you want to change it, you can refer to the following marked code
    /// but read_reg currently only support read no more than 4 bytes....
    /// if you need to read more, please let BinHan knows.
    /*
            unsigned int data_array[16];
            unsigned int max_return_size = 1;
            
            data_array[0]= 0x00003700 | (max_return_size << 16);    
            
            dsi_set_cmdq(&data_array, 1, 1);
    */

    if(read_reg(0x0a) == 0x9c)
    {
        return FALSE;
    }
    else
    {            
        return TRUE;
    }
#endif
}

static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(120);
      push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
      push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
    MDELAY(10);

    return TRUE;
}


static unsigned int lcm_compare_id(void)
{
        int array[4];
        char buffer[5];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0;
        //Do reset here
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(50);      
       
        array[0]=0x00043902;
        array[1]=0x018198FF;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
        array[0]=0x00023700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);
    
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; ///////////////////////0x98
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[1]; ///////////////////////0x81
        read_reg_v2(0x02, buffer,1);
        id_low = buffer[2]; ////////////////////////0x00
       // id = (id_midd &lt;&lt; 8) | id_low;

#ifdef BUILD_LK
    printf("[erick-lk]%s,  9881 id = 0x%08x,0x%08x\n", __func__, id_high,id_midd);
#else
    printk("[erick-k]%s,  9881 id = 0x%08x,0x%08x\n", __func__, id_high,id_midd);
#endif
	if((0x98 == id_high)&&(0x81 == id_midd))
	{
		return 1;
	}
	else
	{
		return 0;
	}
    //return (0x98 == id_high)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_ili9881_dsi_vdo_hd_cpt = 
{
    .name           = "hct_ili9881_dsi_vdo_hd_cpt",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id    = lcm_compare_id,    
#if (LCM_DSI_CMD_MODE)
    //.set_backlight    = lcm_setbacklight,
    //.esd_check   = lcm_esd_check, 
    //.esd_recover   = lcm_esd_recover, 
    .update         = lcm_update,
#endif  //wqtao
};

