// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

extern void mt_secondary_startup(void);
extern void mt_gic_secondary_init(void);
extern unsigned int irq_total_secondary_cpus;

extern void __init mt_smp_prepare_cpus(unsigned int max_cpus);
extern void mt_smp_secondary_init(unsigned int cpu);
extern int mt_smp_boot_secondary(unsigned int cpu, struct task_struct *idle);
extern int mt_cpu_kill(unsigned int cpu);

