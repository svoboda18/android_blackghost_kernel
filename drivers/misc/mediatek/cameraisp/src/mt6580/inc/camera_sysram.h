/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#ifndef CAMERA_SYSRAM_H
#define CAMERA_SYSRAM_H
/* ---------------------------------------------- */
#define SYSRAM_DEV_NAME     "camera-sysram"
#define SYSRAM_MAGIC_NO     'p'
/* ---------------------------------------------- */

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


enum SYSRAM_USER_ENUM {
	SYSRAM_USER_VIDO,
	SYSRAM_USER_GDMA,
	SYSRAM_USER_SW_FD,
	SYSRAM_USER_AMOUNT,
	SYSRAM_USER_NONE
};
/*  */
struct SYSRAM_ALLOC_STRUCT {
	unsigned int Alignment;
	unsigned int Size;
	enum SYSRAM_USER_ENUM User;
	unsigned int Addr;	/* In/Out : address */
	unsigned int TimeoutMS;	/* In : millisecond */
};
/*  */
enum SYSRAM_CMD_ENUM {
	SYSRAM_CMD_ALLOC,
	SYSRAM_CMD_FREE,
	SYSRAM_CMD_DUMP
};


#ifdef CONFIG_COMPAT
enum compat_SYSRAM_USER_ENUM {
	compat_SYSRAM_USER_VIDO,
	compat_SYSRAM_USER_GDMA,
	compat_SYSRAM_USER_SW_FD,
	compat_SYSRAM_USER_AMOUNT,
	compat_SYSRAM_USER_NONE
};
/*  */
struct compat_SYSRAM_ALLOC_STRUCT {
	unsigned int Alignment;
	unsigned int Size;
	enum compat_SYSRAM_USER_ENUM User;
	unsigned int Addr;	/* In/Out : address */
	unsigned int TimeoutMS;	/* In : millisecond */
};
#endif


/* ---------------------------------------------- */
#define SYSRAM_ALLOC    _IOWR(SYSRAM_MAGIC_NO, \
SYSRAM_CMD_ALLOC,   struct SYSRAM_ALLOC_STRUCT)
#define SYSRAM_FREE     _IOWR(SYSRAM_MAGIC_NO, \
SYSRAM_CMD_FREE,    enum SYSRAM_USER_ENUM)
#define SYSRAM_DUMP     _IO(SYSRAM_MAGIC_NO, \
SYSRAM_CMD_DUMP)

#ifdef CONFIG_COMPAT
#define COMPAT_SYSRAM_ALLOC    _IOWR(SYSRAM_MAGIC_NO, \
SYSRAM_CMD_ALLOC,   struct compat_SYSRAM_ALLOC_STRUCT)
#define COMPAT_SYSRAM_FREE     _IOWR(SYSRAM_MAGIC_NO, \
SYSRAM_CMD_FREE,    enum compat_SYSRAM_USER_ENUM)
#endif
/* ---------------------------------------------- */
#endif
