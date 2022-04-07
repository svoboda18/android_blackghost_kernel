/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */

#ifndef __CCCI_TTY_H__
#define __CCCI_TTY_H__

#include <mt_ccci_common.h>

#define  CCCI_TTY_MODEM      0
#define  CCCI_TTY_META       1
#define  CCCI_TTY_IPC         2
#define  CCCI_TTY_ICUSB      3

struct buffer_control_tty_t {
	unsigned read;
	unsigned write;
	unsigned length;
};

struct shared_mem_tty_t {
	struct buffer_control_tty_t rx_control;
	struct buffer_control_tty_t tx_control;
	unsigned char buffer[0];	/*  [RX | TX] */
	/* unsigned char            *tx_buffer; */
};

extern void ccci_reset_buffers(struct shared_mem_tty_t *shared_mem, int size);
extern int  ccci_tty_init(int);
extern void ccci_tty_exit(int);

#endif				/*  __CCCI_TTY_H__ */
