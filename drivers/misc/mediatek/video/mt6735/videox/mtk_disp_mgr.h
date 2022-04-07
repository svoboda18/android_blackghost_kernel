// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2018 MediaTek Inc.
*
*/

#ifndef __H_MTK_DISP_MGR__
#define __H_MTK_DISP_MGR__

#include "disp_session.h"
#include "primary_display.h"


typedef enum {
	PREPARE_INPUT_FENCE,
	PREPARE_OUTPUT_FENCE,
	PREPARE_PRESENT_FENCE
} ePREPARE_FENCE_TYPE;

long mtk_disp_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
extern struct mutex disp_trigger_lock;

int disp_create_session(struct disp_session_config *config);
int disp_destroy_session(struct disp_session_config *config);
int disp_get_session_number(void);
char *disp_session_mode_spy(unsigned int session_id);

extern struct mutex session_config_mutex;
extern struct disp_session_input_config *captured_session_input;
extern struct disp_session_input_config *cached_session_input;
extern struct disp_mem_output_config *captured_session_output;
extern struct disp_mem_output_config *cached_session_output;
extern unsigned int ext_session_id;
#if defined(HDMI_MT8193_SUPPORT) || defined(CONFIG_SINGLE_PANEL_OUTPUT)
extern struct SWITCH_MODE_INFO_STRUCT path_info;
#endif

#endif
