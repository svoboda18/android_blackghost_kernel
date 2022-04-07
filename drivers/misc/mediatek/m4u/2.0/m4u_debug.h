/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __M4U_DEBUG_H__
#define __M4U_DEBUG_H__

extern unsigned long gM4U_ProtectVA;
#if 0
extern int __attribute__((weak)) ddp_mem_test(void);
extern int __attribute__((weak)) __ddp_mem_test(unsigned int *pSrc, unsigned int pSrcPa,
			    unsigned int *pDst, unsigned int pDstPa,
			    int need_sync);
#else
#define ddp_mem_test(...)
#define __ddp_mem_test(...)
#endif
#ifdef M4U_TEE_SERVICE_ENABLE
extern int m4u_sec_init(void);
extern int m4u_config_port_tee(struct m4u_port_config_struct *pM4uPort);
#endif
#endif
