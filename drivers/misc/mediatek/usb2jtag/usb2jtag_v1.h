// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __USB2JTAG_H_
#define __USB2JTAG_H_
unsigned int usb2jtag_mode(void);
struct mtk_usb2jtag_driver *get_mtk_usb2jtag_drv(void);
struct mtk_usb2jtag_driver {
	int	(*usb2jtag_init)(void);
	int	(*usb2jtag_resume)(void);
	int	(*usb2jtag_suspend)(void);
};

extern bool usb_enable_clock(bool enable);
extern struct clk *musb_clk;
extern int usb2jtag_usb_init(void);
#endif
