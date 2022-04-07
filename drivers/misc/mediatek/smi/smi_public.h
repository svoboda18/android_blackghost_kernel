// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2019  MediaTek Inc.
*
*/

#ifndef __SMI_PUBLIC_H__
#define __SMI_PUBLIC_H__

#define SMI_LARB0_REG_INDX 0
#define SMI_LARB1_REG_INDX 1
#define SMI_LARB2_REG_INDX 2
#define SMI_LARB3_REG_INDX 3
#define SMI_LARB4_REG_INDX 4
#define SMI_LARB5_REG_INDX 5
#define SMI_LARB6_REG_INDX 6
#define SMI_LARB7_REG_INDX 7
#define SMI_LARB8_REG_INDX 8
#define SMI_PARAM_BUS_OPTIMIZATION  (0x1FF)

struct smi_bwc_scen_cb {
    struct list_head list;
    char *name;
    void (*smi_bwc_scen_cb_handle)(const unsigned int scen);
};

#if IS_ENABLED(CONFIG_MTK_SMI_EXT)
void __iomem *smi_base_addr_get(const unsigned int reg_indx);
int smi_bus_prepare_enable(const unsigned int reg_indx,
    char *user_name, const bool mtcmos);
int smi_bus_disable_unprepare(const unsigned int reg_indx,
    char *user_name, const bool mtcmos);
void smi_debug_dump_status(const unsigned int reg_indx);
int smi_debug_bus_hang_detect(unsigned int reg_indx, const bool dump,
    const bool gce, const bool m4u);
struct smi_bwc_scen_cb *smi_bwc_scen_cb_register(struct smi_bwc_scen_cb *cb);
#else
#define smi_base_addr_get(reg_indx) ((void)0)
#define smi_bus_prepare_enable(reg_indx, user_name, mtcmos) ((void)0)
#define smi_bus_disable_unprepare(reg_indx, user_name, mtcmos) ((void)0)
#define smi_debug_dump_status(reg_indx) ((void)0)
#define smi_debug_bus_hang_detect(larb_indx, dump, gce, m4u) ((void)0)
#define smi_bwc_scen_cb_register(cb) ((void)0)
#endif

#endif
