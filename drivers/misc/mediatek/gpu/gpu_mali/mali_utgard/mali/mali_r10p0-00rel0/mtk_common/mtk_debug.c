// SPDX-License-Identifier: GPL-2.0

/*

 * Copyright (c) 2019 MediaTek Inc.

 */

#include "mtk_debug.h"

#ifdef MTK_DEBUG

void MTKDebugInit()
{
#ifdef MTK_DEBUG_PROC_PRINT
	MTKPP_Init();
#endif
}

void MTKDebugDeinit()
{
#ifdef MTK_DEBUG_PROC_PRINT
	MTKPP_Deinit();
#endif
}

#endif

