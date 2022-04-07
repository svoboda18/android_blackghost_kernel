// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __PORT_CFG_H__
#define __PORT_CFG_H__
#include "port_proxy.h"

/* external: port ops  mapping */
extern struct ccci_port_ops char_port_ops;
extern struct ccci_port_ops net_port_ops;
extern struct ccci_port_ops rpc_port_ops;
extern struct ccci_port_ops sys_port_ops;
extern struct ccci_port_ops poller_port_ops;
extern struct ccci_port_ops ctl_port_ops;
extern struct ccci_port_ops ipc_port_ack_ops;
extern struct ccci_port_ops ipc_kern_port_ops;
#endif /* __PORT_CFG_H__ */
