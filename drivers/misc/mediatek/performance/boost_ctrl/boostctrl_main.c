// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/proc_fs.h>

#include "boost_ctrl.h"

int init_boostctrl(struct proc_dir_entry *parent)
{
	struct proc_dir_entry *bstctrl_root = NULL;

	pr_debug("__init init_boostctrl\n");


	bstctrl_root = proc_mkdir("boost_ctrl", parent);

	topo_ctrl_init(bstctrl_root);

	return 0;
}
