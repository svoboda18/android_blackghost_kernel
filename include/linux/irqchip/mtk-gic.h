// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#ifndef __MT_GIC_H
#define __MT_GIC_H
#include "mtk-gic-extend.h"
void mt_gic_cpu_init_for_low_power(void);
extern int mt_get_supported_irq_num(void);
#endif
