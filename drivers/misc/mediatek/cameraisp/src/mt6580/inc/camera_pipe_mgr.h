/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

/* --------------------------------------------- */
#ifndef CAMERA_PIPE_MGR_H
#define CAMERA_PIPE_MGR_H
/*  */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
/* --------------------------------------------- */
#define CAM_PIPE_MGR_DEV_NAME       "camera-pipemgr"
#define CAM_PIPE_MGR_MAGIC_NO       'p'
/* --------------------------------------------- */
#define CAM_PIPE_MGR_PIPE_MASK_CAM_IO       (1UL << 0)
#define CAM_PIPE_MGR_PIPE_MASK_POST_PROC    (1UL << 1)
#define CAM_PIPE_MGR_PIPE_MASK_XDP_CAM      (1UL << 2)
/* --------------------------------------------- */
enum CAM_PIPE_MGR_SCEN_SW_ENUM {
	CAM_PIPE_MGR_SCEN_SW_NONE,
	CAM_PIPE_MGR_SCEN_SW_CAM_IDLE,
	CAM_PIPE_MGR_SCEN_SW_CAM_PRV,
	CAM_PIPE_MGR_SCEN_SW_CAM_CAP,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_PRV,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_REC,
	CAM_PIPE_MGR_SCEN_SW_VIDEO_VSS,
	CAM_PIPE_MGR_SCEN_SW_ZSD,
	CAM_PIPE_MGR_SCEN_SW_N3D,
};
/*  */
enum CAM_PIPE_MGR_SCEN_HW_ENUM {
	CAM_PIPE_MGR_SCEN_HW_NONE,
	CAM_PIPE_MGR_SCEN_HW_IC,
	CAM_PIPE_MGR_SCEN_HW_VR,
	CAM_PIPE_MGR_SCEN_HW_ZSD,
	CAM_PIPE_MGR_SCEN_HW_IP,
	CAM_PIPE_MGR_SCEN_HW_N3D,
	CAM_PIPE_MGR_SCEN_HW_VSS
};
/*  */
enum CAM_PIPE_MGR_DEV_ENUM {
	CAM_PIPE_MGR_DEV_CAM,
	CAM_PIPE_MGR_DEV_ATV,
	CAM_PIPE_MGR_DEV_VT
};
/*  */
struct CAM_PIPE_MGR_LOCK_STRUCT {
	unsigned int PipeMask;
	unsigned int Timeout;
};
/*  */
struct CAM_PIPE_MGR_UNLOCK_STRUCT {
	unsigned int PipeMask;
};
/*  */
struct CAM_PIPE_MGR_MODE_STRUCT {
	enum CAM_PIPE_MGR_SCEN_SW_ENUM ScenSw;
	enum CAM_PIPE_MGR_SCEN_HW_ENUM ScenHw;
	enum CAM_PIPE_MGR_DEV_ENUM Dev;
};
/*  */
struct CAM_PIPE_MGR_ENABLE_STRUCT {
	unsigned int PipeMask;
};
/*  */
struct CAM_PIPE_MGR_DISABLE_STRUCT {
	unsigned int PipeMask;
};
/* --------------------------------------------- */
enum CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM {
	CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_HIGH,
	CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_LOW
};
/* --------------------------------------------- */
enum CAM_PIPE_MGR_CMD_ENUM {
	CAM_PIPE_MGR_CMD_LOCK,
	CAM_PIPE_MGR_CMD_UNLOCK,
	CAM_PIPE_MGR_CMD_DUMP,
	CAM_PIPE_MGR_CMD_SET_MODE,
	CAM_PIPE_MGR_CMD_GET_MODE,
	CAM_PIPE_MGR_CMD_ENABLE_PIPE,
	CAM_PIPE_MGR_CMD_DISABLE_PIPE,
	CAM_PIPE_MGR_CMD_VENC_PLL_CTRL
};
#ifdef CONFIG_COMPAT
enum compat_CAM_PIPE_MGR_SCEN_SW_ENUM {
	compat_CAM_PIPE_MGR_SCEN_SW_NONE,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_IDLE,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_PRV,
	compat_CAM_PIPE_MGR_SCEN_SW_CAM_CAP,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_PRV,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_REC,
	compat_CAM_PIPE_MGR_SCEN_SW_VIDEO_VSS,
	compat_CAM_PIPE_MGR_SCEN_SW_ZSD,
	compat_CAM_PIPE_MGR_SCEN_SW_N3D,
};
/*  */
enum compat_CAM_PIPE_MGR_SCEN_HW_ENUM {
	compat_CAM_PIPE_MGR_SCEN_HW_NONE,
	compat_CAM_PIPE_MGR_SCEN_HW_IC,
	compat_CAM_PIPE_MGR_SCEN_HW_VR,
	compat_CAM_PIPE_MGR_SCEN_HW_ZSD,
	compat_CAM_PIPE_MGR_SCEN_HW_IP,
	compat_CAM_PIPE_MGR_SCEN_HW_N3D,
	compat_CAM_PIPE_MGR_SCEN_HW_VSS
};
/*  */
enum compat_CAM_PIPE_MGR_DEV_ENUM {
	compat_CAM_PIPE_MGR_DEV_CAM,
	compat_CAM_PIPE_MGR_DEV_ATV,
	compat_CAM_PIPE_MGR_DEV_VT
};
/*  */
struct compat_CAM_PIPE_MGR_LOCK_STRUCT {
	unsigned int PipeMask;
	unsigned int Timeout;
};
/*  */
struct compat_CAM_PIPE_MGR_UNLOCK_STRUCT {
	unsigned int PipeMask;
};
/*  */
struct compat_CAM_PIPE_MGR_MODE_STRUCT {
	enum compat_CAM_PIPE_MGR_SCEN_SW_ENUM ScenSw;
	enum compat_CAM_PIPE_MGR_SCEN_HW_ENUM ScenHw;
	enum compat_CAM_PIPE_MGR_DEV_ENUM Dev;
};
/*  */
struct compat_CAM_PIPE_MGR_ENABLE_STRUCT {
	unsigned int PipeMask;
};
/*  */
struct compat_CAM_PIPE_MGR_DISABLE_STRUCT {
	unsigned int PipeMask;
};
/* --------------------------------------------- */
enum compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM {
	compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_HIGH,
	compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_SET_LOW
};
/* --------------------------------------------- */
enum compat_CAM_PIPE_MGR_CMD_ENUM {
	compat_CAM_PIPE_MGR_CMD_LOCK,
	compat_CAM_PIPE_MGR_CMD_UNLOCK,
	compat_CAM_PIPE_MGR_CMD_DUMP,
	compat_CAM_PIPE_MGR_CMD_SET_MODE,
	compat_CAM_PIPE_MGR_CMD_GET_MODE,
	compat_CAM_PIPE_MGR_CMD_ENABLE_PIPE,
	compat_CAM_PIPE_MGR_CMD_DISABLE_PIPE,
	compat_CAM_PIPE_MGR_CMD_VENC_PLL_CTRL
};

#endif
/* --------------------------------------------- */
#define CAM_PIPE_MGR_LOCK           _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_LOCK, struct CAM_PIPE_MGR_LOCK_STRUCT)
#define CAM_PIPE_MGR_UNLOCK         _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_UNLOCK, struct CAM_PIPE_MGR_UNLOCK_STRUCT)
#define CAM_PIPE_MGR_DUMP           _IO(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_DUMP)
#define CAM_PIPE_MGR_SET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_SET_MODE, struct CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_GET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_GET_MODE, struct CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_ENABLE_PIPE    _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_ENABLE_PIPE, struct CAM_PIPE_MGR_ENABLE_STRUCT)
#define CAM_PIPE_MGR_DISABLE_PIPE   _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_DISABLE_PIPE, struct CAM_PIPE_MGR_DISABLE_STRUCT)
#define CAM_PIPE_MGR_VENCPLL_CTRL   _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_VENC_PLL_CTRL, enum CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM)

#ifdef CONFIG_COMPAT
#define COMPAT_CAM_PIPE_MGR_LOCK           _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_LOCK, struct compat_CAM_PIPE_MGR_LOCK_STRUCT)
#define COMPAT_CAM_PIPE_MGR_UNLOCK         _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_UNLOCK, struct compat_CAM_PIPE_MGR_UNLOCK_STRUCT)
#define COMPAT_CAM_PIPE_MGR_SET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_SET_MODE, struct compat_CAM_PIPE_MGR_MODE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_GET_MODE       _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_GET_MODE, struct compat_CAM_PIPE_MGR_MODE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_ENABLE_PIPE    _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_ENABLE_PIPE, struct compat_CAM_PIPE_MGR_ENABLE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_DISABLE_PIPE   _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_DISABLE_PIPE, struct compat_CAM_PIPE_MGR_DISABLE_STRUCT)
#define COMPAT_CAM_PIPE_MGR_VENCPLL_CTRL   _IOW(CAM_PIPE_MGR_MAGIC_NO, \
CAM_PIPE_MGR_CMD_VENC_PLL_CTRL, enum compat_CAM_PIPE_MGR_CMD_VECNPLL_CTRL_ENUM)
#endif
/* --------------------------------------------- */
#endif
/* --------------------------------------------- */
