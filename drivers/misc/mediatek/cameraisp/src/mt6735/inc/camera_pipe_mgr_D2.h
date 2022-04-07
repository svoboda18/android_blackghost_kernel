/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
*/

/* ----------------------------------------------------------------------------- */
#ifndef CAMERA_PIPE_MGR_H
#define CAMERA_PIPE_MGR_H
/*  */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
extern u32 get_devinfo_with_index(u32 index);
/* ----------------------------------------------------------------------------- */
#define CAM_PIPE_MGR_DEV_NAME       "camera-pipemgr"
#define CAM_PIPE_MGR_MAGIC_NO       'p'
/* ----------------------------------------------------------------------------- */
#define CAM_PIPE_MGR_PIPE_MASK_CAM_IO       ((unsigned long)1 << 0)
#define CAM_PIPE_MGR_PIPE_MASK_POST_PROC    ((unsigned long)1 << 1)
#define CAM_PIPE_MGR_PIPE_MASK_XDP_CAM      ((unsigned long)1 << 2)
/* ----------------------------------------------------------------------------- */
typedef enum {
	CAM_PIPE_MGR_SCEN_SW_NONE,
	CAM_PIPE_MGR_SCEN_SW_CAM_IDLE,
	CAM_PIPE_MGR_SCEN_SW_CAM_PRV,
	CAM_PIPE_MGR_SCEN_SW_CAM_CAP,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_PRV,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_REC,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_VSS,
	CAM_PIPE_MGR_SCEN_SW_ZSD,
	CAM_PIPE_MGR_SCEN_SW_N3D,
} CAM_PIPE_MGR_SCEN_SW_ENUM;
/*  */
typedef enum {
	CAM_PIPE_MGR_SCEN_HW_NONE,
	CAM_PIPE_MGR_SCEN_HW_IC,
	CAM_PIPE_MGR_SCEN_HW_VR,
	CAM_PIPE_MGR_SCEN_HW_ZSD,
	CAM_PIPE_MGR_SCEN_HW_IP,
	CAM_PIPE_MGR_SCEN_HW_N3D,
	CAM_PIPE_MGR_SCEN_HW_VSS
} CAM_PIPE_MGR_SCEN_HW_ENUM;
/*  */
typedef enum {
	CAM_PIPE_MGR_DEV_CAM,
	CAM_PIPE_MGR_DEV_ATV,
	CAM_PIPE_MGR_DEV_VT
} CAM_PIPE_MGR_DEV_ENUM;
/*  */
typedef struct {
	unsigned int PipeMask;
	unsigned int Timeout;
} CAM_PIPE_MGR_LOCK_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} CAM_PIPE_MGR_UNLOCK_STRUCT;
/*  */
typedef struct {
	CAM_PIPE_MGR_SCEN_SW_ENUM ScenSw;
	CAM_PIPE_MGR_SCEN_HW_ENUM ScenHw;
	CAM_PIPE_MGR_DEV_ENUM Dev;
} CAM_PIPE_MGR_MODE_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} CAM_PIPE_MGR_ENABLE_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} CAM_PIPE_MGR_DISABLE_STRUCT;
/* ----------------------------------------------------------------------------- */
typedef enum {
	CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_HIGH,
	CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_LOW
} CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM;
/* ----------------------------------------------------------------------------- */
typedef enum {
	CAM_PIPE_MGR_CMD_LOCK,
	CAM_PIPE_MGR_CMD_UNLOCK,
	CAM_PIPE_MGR_CMD_DUMP,
	CAM_PIPE_MGR_CMD_SET_MODE,
	CAM_PIPE_MGR_CMD_GET_MODE,
	CAM_PIPE_MGR_CMD_ENABLE_PIPE,
	CAM_PIPE_MGR_CMD_DISABLE_PIPE,
	CAM_PIPE_MGR_CMD_VENC_PLL_CTRL
} CAM_PIPE_MGR_CMD_ENUM;


#ifdef CONFIG_COMPAT
typedef enum {
	compat_CAM_PIPE_MGR_SCEN_SW_NONE,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_IDLE,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_PRV,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_CAP,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_PRV,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_REC,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_VSS,
	compat_CAM_PIPE_MGR_SCEN_SW_ZSD,
	compat_CAM_PIPE_MGR_SCEN_SW_N3D,
} compat_CAM_PIPE_MGR_SCEN_SW_ENUM;
/*  */
typedef enum {
	compat_CAM_PIPE_MGR_SCEN_HW_NONE,
	compat_CAM_PIPE_MGR_SCEN_HW_IC,
	compat_CAM_PIPE_MGR_SCEN_HW_VR,
	compat_CAM_PIPE_MGR_SCEN_HW_ZSD,
	compat_CAM_PIPE_MGR_SCEN_HW_IP,
	compat_CAM_PIPE_MGR_SCEN_HW_N3D,
	compat_CAM_PIPE_MGR_SCEN_HW_VSS
} compat_CAM_PIPE_MGR_SCEN_HW_ENUM;
/*  */
typedef enum {
	compat_CAM_PIPE_MGR_DEV_CAM,
	compat_CAM_PIPE_MGR_DEV_ATV,
	compat_CAM_PIPE_MGR_DEV_VT
} compat_CAM_PIPE_MGR_DEV_ENUM;
/*  */
typedef struct {
	unsigned int PipeMask;
	unsigned int Timeout;
} compat_CAM_PIPE_MGR_LOCK_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} compat_CAM_PIPE_MGR_UNLOCK_STRUCT;
/*  */
typedef struct {
	compat_CAM_PIPE_MGR_SCEN_SW_ENUM ScenSw;
	compat_CAM_PIPE_MGR_SCEN_HW_ENUM ScenHw;
	compat_CAM_PIPE_MGR_DEV_ENUM Dev;
} compat_CAM_PIPE_MGR_MODE_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} compat_CAM_PIPE_MGR_ENABLE_STRUCT;
/*  */
typedef struct {
	unsigned int PipeMask;
} compat_CAM_PIPE_MGR_DISABLE_STRUCT;
/* ----------------------------------------------------------------------------- */
typedef enum {
	compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_HIGH,
	compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_LOW
} compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM;
/* ----------------------------------------------------------------------------- */
typedef enum {
	compat_CAM_PIPE_MGR_CMD_LOCK,
	compat_CAM_PIPE_MGR_CMD_UNLOCK,
	compat_CAM_PIPE_MGR_CMD_DUMP,
	compat_CAM_PIPE_MGR_CMD_SET_MODE,
	compat_CAM_PIPE_MGR_CMD_GET_MODE,
	compat_CAM_PIPE_MGR_CMD_ENABLE_PIPE,
	compat_CAM_PIPE_MGR_CMD_DISABLE_PIPE,
	compat_CAM_PIPE_MGR_CMD_VENC_PLL_CTRL
} compat_CAM_PIPE_MGR_CMD_ENUM;

#endif

/* ----------------------------------------------------------------------------- */
#define CAM_PIPE_MGR_LOCK           _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_LOCK,          CAM_PIPE_MGR_LOCK_STRUCT)
#define CAM_PIPE_MGR_UNLOCK         _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_UNLOCK,        CAM_PIPE_MGR_UNLOCK_STRUCT)
#define CAM_PIPE_MGR_DUMP           _IO(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_DUMP)
#define CAM_PIPE_MGR_SET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_SET_MODE,      CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_GET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_GET_MODE,      CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_ENABLE_PIPE    _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_ENABLE_PIPE,   CAM_PIPE_MGR_ENABLE_STRUCT)
#define CAM_PIPE_MGR_DISABLE_PIPE   _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_DISABLE_PIPE,  CAM_PIPE_MGR_DISABLE_STRUCT)
#define CAM_PIPE_MGR_VENCPLL_CTRL   _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_VENC_PLL_CTRL,  CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM)

#ifdef CONFIG_COMPAT
#define COMPAT_CAM_PIPE_MGR_LOCK           _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_LOCK,          compat_CAM_PIPE_MGR_LOCK_STRUCT)
#define COMPAT_CAM_PIPE_MGR_UNLOCK         _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_UNLOCK,        compat_CAM_PIPE_MGR_UNLOCK_STRUCT)
#define COMPAT_CAM_PIPE_MGR_SET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_SET_MODE,      compat_CAM_PIPE_MGR_MODE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_GET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_GET_MODE,      compat_CAM_PIPE_MGR_MODE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_ENABLE_PIPE    _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_ENABLE_PIPE,   compat_CAM_PIPE_MGR_ENABLE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_DISABLE_PIPE   _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_DISABLE_PIPE,  compat_CAM_PIPE_MGR_DISABLE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_VENCPLL_CTRL   _IOW(CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_VENC_PLL_CTRL,  compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM)
#endif

/* ----------------------------------------------------------------------------- */
#endif
/* ----------------------------------------------------------------------------- */
