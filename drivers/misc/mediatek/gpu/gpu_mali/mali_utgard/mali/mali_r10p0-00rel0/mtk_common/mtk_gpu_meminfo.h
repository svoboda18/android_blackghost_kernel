/*  SPDX-License-Identifier: GPL-2.0 */  

/*

 * Copyright (c) 2019 MediaTek Inc.

 */

#ifdef ENABLE_MTK_MEMINFO
#define MTK_MEMINFO_SIZE 128

typedef struct {
	int pid;
	int used_pages;
} mtk_gpu_meminfo_type;

void mtk_gpu_meminfo_init(void);
void mtk_gpu_meminfo_remove(void);
void mtk_gpu_meminfo_reset(void);
void mtk_gpu_meminfo_set(ssize_t index, int pid, int used_pages);

bool mtk_dump_mali_memory_usage(void);
unsigned int mtk_get_mali_memory_usage(void);
#endif /* ENABLE_MTK_MEMINFO */
