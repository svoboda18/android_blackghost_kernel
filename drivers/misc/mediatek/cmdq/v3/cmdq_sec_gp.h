// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_SEC_GP_H__
#define __CMDQ_SEC_GP_H__

#include <linux/types.h>
#include "tee_client_api.h"

/* context for tee vendor */
struct cmdq_sec_tee_context {
#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
	/* Universally Unique Identifier of secure tl/dr */
	struct TEEC_UUID uuid;
#else
	TEEC_UUID uuid; /* Universally Unique Identifier of secure tl/dr */
#endif
	struct TEEC_Context gp_context; /* basic context */
	struct TEEC_Session session; /* session handle */
	struct TEEC_SharedMemory shared_mem; /* shared memory */
};

#endif	/* __CMDQ_SEC_GP_H__ */
