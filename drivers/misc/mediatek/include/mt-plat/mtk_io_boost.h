/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef __MTK_IO_BOOST_H
#define __MTK_IO_BOOST_H

extern int mtk_io_boost_add_tid(int tid);
extern int mtk_io_boost_test_and_add_tid(int tid, bool *done);

#endif

