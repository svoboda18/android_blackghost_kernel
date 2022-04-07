// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __MT_INNERCACHE_H
#define __MT_INNERCACHE_H

extern void __inner_flush_dcache_all(void);
extern void __inner_flush_dcache_L1(void);
extern void __inner_flush_dcache_L2(void);
extern void __disable_dcache(void);

#ifdef CONFIG_MTK_CACHE_FLUSH_RANGE_PARALLEL
#define CACHE_FLUSH_BY_SETWAY	1
#define CACHE_FLUSH_BY_MVA	2
#define CACHE_FLUSH_TIMEOUT	1000
#endif

#endif
