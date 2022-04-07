// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __H_DDP_COLOR_FORMAT__
#define __H_DDP_COLOR_FORMAT__

#include "ddp_info.h"

int fmt_bpp(enum DP_COLOR_ENUM fmt);
int fmt_swap(enum DP_COLOR_ENUM fmt);
int fmt_color_space(enum DP_COLOR_ENUM fmt);
int fmt_is_yuv422(enum DP_COLOR_ENUM fmt);
int fmt_is_yuv420(enum DP_COLOR_ENUM fmt);
int fmt_hw_value(enum DP_COLOR_ENUM fmt);
char *fmt_string(enum DP_COLOR_ENUM fmt);
enum DP_COLOR_ENUM fmt_type(int unique, int swap);

#endif
