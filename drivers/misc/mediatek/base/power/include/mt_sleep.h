/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#ifndef __MT_SLEEP_H__
#define __MT_SLEEP_H__

#if defined(CONFIG_ARCH_MT6755) || defined(CONFIG_ARCH_MT6757) || defined(CONFIG_ARCH_MT6797)

#include "spm_v2/mt_sleep.h"

#elif defined(CONFIG_MACH_MT6735) || defined(CONFIG_MACH_MT6735M) || defined(CONFIG_MACH_MT6753)

#include "../mt6735/mt_sleep.h"

#elif defined(CONFIG_ARCH_MT6570) || defined(CONFIG_MACH_MT6580)

#include "spm_v1/mt_sleep.h"

#elif defined(CONFIG_ARCH_ELBRUS)

#include "spm_v3/mt_sleep.h"

#endif

#endif /* __MT_SLEEP_H__ */

