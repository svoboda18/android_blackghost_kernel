// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __SMI_INFO_UTIL_H__
#define __SMI_INFO_UTIL_H__

#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <mt-plat/mtk_smi.h>

extern struct MTK_SMI_BWC_MM_INFO g_smi_bwc_mm_info;

int smi_set_mm_info_ioctl_wrapper(struct file *pFile, unsigned int cmd,
	unsigned long param);
int smi_get_mm_info_ioctl_wrapper(struct file *pFile, unsigned int cmd,
	unsigned long param);
void smi_bwc_mm_info_set(int property_id, long val1, long val2);
#endif /* __SMI_INFO_UTIL_H__ */
