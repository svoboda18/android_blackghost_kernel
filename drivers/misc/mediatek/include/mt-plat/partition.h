/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef _PARTITION_H
#define _PARTITION_H

#include <linux/genhd.h>

extern struct hd_struct *get_part(char *name);
extern void put_part(struct hd_struct *part);


#endif
