// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_MMP_H__
#define __CMDQ_MMP_H__

#include "cmdq_core.h"
#include "mmprofile.h"
#include "mmprofile_function.h"

struct CMDQ_MMP_Events_t {
	MMP_Event CMDQ;
	MMP_Event CMDQ_IRQ;
	MMP_Event thread_en;
	MMP_Event warning;
	MMP_Event loopBeat;
	MMP_Event autoRelease_add;
	MMP_Event autoRelease_done;
	MMP_Event consume_add;
	MMP_Event consume_done;
	MMP_Event alloc_task;
	MMP_Event wait_task;
	MMP_Event wait_thread;
	MMP_Event MDP_reset;
	MMP_Event thread_suspend;
	MMP_Event thread_resume;
};

void cmdq_mmp_init(void);
struct CMDQ_MMP_Events_t *cmdq_mmp_get_event(void);

extern void MMProfileEnable(int enable);
extern void MMProfileStart(int start);
#endif /* __CMDQ_MMP_H__ */
