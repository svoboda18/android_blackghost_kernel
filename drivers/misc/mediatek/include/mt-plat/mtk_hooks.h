/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef __MTK_HOOKS_H__
#define __MTK_HOOKS_H__

extern int __weak arm_undefinstr_retry(struct pt_regs *regs,
		unsigned int instr);
extern void __weak ioremap_debug_hook_func(phys_addr_t phys_addr,
		size_t size, pgprot_t prot);

#endif /* __MTK_HOOKS_H__ */
