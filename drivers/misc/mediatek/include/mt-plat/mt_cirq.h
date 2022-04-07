/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef __CIRQ_H__
#define __CIRQ_H__

/*
 * Define function prototypes.
 */
void mt_cirq_enable(void);
void mt_cirq_disable(void);
void mt_cirq_clone_gic(void);
void mt_cirq_flush(void);

#endif  /*!__CIRQ_H__ */
