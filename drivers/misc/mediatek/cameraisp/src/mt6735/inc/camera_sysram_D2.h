/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
*/

#ifndef CAMERA_SYSRAM_H
#define CAMERA_SYSRAM_H
/* ----------------------------------------------------------------------------- */
#define SYSRAM_DEV_NAME     "camera-sysram"
#define SYSRAM_MAGIC_NO     'p'
/* ----------------------------------------------------------------------------- */

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


typedef enum {
	SYSRAM_USER_VIDO,
	SYSRAM_USER_GDMA,
	SYSRAM_USER_SW_FD,
	SYSRAM_USER_AMOUNT,
	SYSRAM_USER_NONE
} SYSRAM_USER_ENUM;
/*  */
typedef struct {
	unsigned int Alignment;
	unsigned int Size;
	SYSRAM_USER_ENUM User;
	unsigned int Addr;	/* In/Out : address */
	unsigned int TimeoutMS;	/* In : millisecond */
} SYSRAM_ALLOC_STRUCT;
/*  */
typedef enum {
	SYSRAM_CMD_ALLOC,
	SYSRAM_CMD_FREE,
	SYSRAM_CMD_DUMP
} SYSRAM_CMD_ENUM;


#ifdef CONFIG_COMPAT
typedef enum {
	compat_SYSRAM_USER_VIDO,
	compat_SYSRAM_USER_GDMA,
	compat_SYSRAM_USER_SW_FD,
	compat_SYSRAM_USER_AMOUNT,
	compat_SYSRAM_USER_NONE
} compat_SYSRAM_USER_ENUM;
/*  */
typedef struct {
	unsigned int Alignment;
	unsigned int Size;
	compat_SYSRAM_USER_ENUM User;
	unsigned int Addr;	/* In/Out : address */
	unsigned int TimeoutMS;	/* In : millisecond */
} compat_SYSRAM_ALLOC_STRUCT;
#endif


/* ----------------------------------------------------------------------------- */
#define SYSRAM_ALLOC    _IOWR(SYSRAM_MAGIC_NO,    SYSRAM_CMD_ALLOC,   SYSRAM_ALLOC_STRUCT)
#define SYSRAM_FREE     _IOWR(SYSRAM_MAGIC_NO,    SYSRAM_CMD_FREE,    SYSRAM_USER_ENUM)
#define SYSRAM_DUMP     _IO(SYSRAM_MAGIC_NO,    SYSRAM_CMD_DUMP)

#ifdef CONFIG_COMPAT
#define COMPAT_SYSRAM_ALLOC    _IOWR(SYSRAM_MAGIC_NO,    SYSRAM_CMD_ALLOC,   compat_SYSRAM_ALLOC_STRUCT)
#define COMPAT_SYSRAM_FREE     _IOWR(SYSRAM_MAGIC_NO,    SYSRAM_CMD_FREE,    compat_SYSRAM_USER_ENUM)
#endif
/* ----------------------------------------------------------------------------- */
#endif
