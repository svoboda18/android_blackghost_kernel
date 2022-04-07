// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/
#ifndef __PORT_NET_H__
#define __PORT_NET_H__
#include "ccmni.h"
extern struct ccmni_dev_ops ccmni_ops;

extern int mbim_start_xmit(struct sk_buff *skb, int ifid);
#endif /*__PORT_NET_H__*/
