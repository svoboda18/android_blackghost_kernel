/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

extern struct smp_operations mt_smp_ops;

extern void mt_secondary_startup(void);
extern void mt_gic_secondary_init(void);
extern unsigned int irq_total_secondary_cpus;

