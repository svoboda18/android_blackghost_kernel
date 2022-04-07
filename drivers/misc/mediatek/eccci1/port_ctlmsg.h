// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __PORT_CTLMSG_H__
#define __PORT_CTLMSG_H__

#include "ccci_core.h"

/****************************************************************************************************************/
/* External API Region called by port ctl object */
/****************************************************************************************************************/
extern int mdee_ctlmsg_handler(struct ccci_port *port, struct sk_buff *skb);
#endif	/* __PORT_CTLMSG_H__ */
