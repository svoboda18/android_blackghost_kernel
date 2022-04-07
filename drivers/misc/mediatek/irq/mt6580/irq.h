// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#ifndef _MT6580_IRQ_H
#define _MT6580_IRQ_H

void set_irq_flags(unsigned int irq, unsigned int flags);
int request_trusty_fiq(struct device *tdev, int irq, fiq_isr_handler handler,
		unsigned long irq_flags, void *arg);

#define IRQF_VALID	(1 << 0)
#define IRQF_PROBE	(1 << 1)
#define IRQF_NOAUTOEN	(1 << 2)

#define cpus_addr(src)	((src).bits)

#endif
