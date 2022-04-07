// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef _MT_VCORE_DVFS_
#define _MT_VCORE_DVFS_

#if defined(CONFIG_MACH_MT6735)
#include "mt_vcore_dvfs_1.h"
#endif

#if defined(CONFIG_MACH_MT6735M)
#include "mt_vcore_dvfs_2.h"
#endif

#if defined(CONFIG_MACH_MT6753)
#include "mt_vcore_dvfs_3.h"
#endif

#endif

