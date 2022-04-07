// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_SEC_TRUSTONIC_H__
#define __CMDQ_SEC_TRUSTONIC_H__

#include "mobicore_driver_api.h"

/* context for tee vendor */
struct cmdq_sec_tee_context {
	/* Universally Unique Identifier of secure tl/dr */
	struct mc_uuid_t uuid;
	struct mc_session_handle session;	/* session handle */
	/* true if someone has opened mobicore device
	 * in this prpocess context
	 */
	u32 open_mobicore_by_other;
	u32 wsm_size;
};

#endif	/* __CMDQ_SEC_TRUSTONIC_H__ */
