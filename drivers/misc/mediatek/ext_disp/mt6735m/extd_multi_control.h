// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/
#ifndef __EXTD_MULTI_CONTROL_H__
#define __EXTD_MULTI_CONTROL_H__

#include "extd_info.h"
#include "disp_session.h"
#ifdef CONFIG_SINGLE_PANEL_OUTPUT
extern bool is_early_suspended;
#endif
void external_display_control_init(void);
int external_display_config_input(struct disp_session_input_config *input, int idx, unsigned int session);
int external_display_trigger(enum EXTD_TRIGGER_MODE trigger, unsigned int session);
int external_display_wait_for_vsync(void *config, unsigned int session);
int external_display_get_info(void *info, unsigned int session);
int external_display_switch_mode(enum DISP_MODE mode, unsigned int *session_created, unsigned int session);
int external_display_path_change_without_cascade(enum DISP_MODE mode, unsigned int session_id,
		unsigned int device_id, unsigned int change_flag);

#endif
