// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include "tpd.h"

static int __init tpd_probe_init(void)
{
	tpd_device_init();
	return 0;
}

static void __exit tpd_probe_exit(void)
{
	tpd_device_exit();
}
late_initcall(tpd_probe_init);
module_exit(tpd_probe_exit);

MODULE_DESCRIPTION("MediaTek touch panel driver");
MODULE_AUTHOR("Qiangming.xia@mediatek.com>");
MODULE_LICENSE("GPL");
