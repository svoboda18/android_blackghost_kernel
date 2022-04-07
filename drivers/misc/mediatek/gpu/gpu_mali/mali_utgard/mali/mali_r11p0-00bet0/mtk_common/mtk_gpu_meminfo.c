// SPDX-License-Identifier: GPL-2.0

/*

 * Copyright (c) 2019 MediaTek Inc.

 */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include "mtk_gpu_meminfo.h"
#include "mali_osk.h"
#include "mali_osk_list.h"
#include "mali_session.h"
#include "mali_ukk.h"

#ifdef ENABLE_MTK_MEMINFO
extern unsigned int (*mtk_get_gpu_memory_usage_fp)(void);
extern bool (*mtk_dump_gpu_memory_usage_fp)(void);

int g_mtk_gpu_total_memory_usage_in_pages_debugfs;
static mtk_gpu_meminfo_type g_mtk_gpu_meminfo[MTK_MEMINFO_SIZE];


void mtk_gpu_meminfo_init(void)
{
	mtk_dump_gpu_memory_usage_fp = mtk_dump_mali_memory_usage;
	mtk_get_gpu_memory_usage_fp = mtk_get_mali_memory_usage;
}

void mtk_gpu_meminfo_remove(void)
{
	mtk_dump_gpu_memory_usage_fp = NULL;
	mtk_get_gpu_memory_usage_fp = NULL;
}

void mtk_gpu_meminfo_reset(void)
{
	int i = 0;
	for (i = 0; i < MTK_MEMINFO_SIZE; i++) {
		g_mtk_gpu_meminfo[i].pid = 0;
		g_mtk_gpu_meminfo[i].used_pages = 0;
	}
}

void mtk_gpu_meminfo_set(ssize_t index, int pid, int used_pages)
{
	if (index < MTK_MEMINFO_SIZE) {
		g_mtk_gpu_meminfo[index].pid = pid;
		g_mtk_gpu_meminfo[index].used_pages = used_pages;
	}
}

bool mtk_dump_mali_memory_usage(void)
{
	int i = 0;

	pr_warn(KERN_DEBUG "%10s\t%16s\n", "PID", "Memory by Page");
	pr_warn(KERN_DEBUG "============================\n");

	for (i = 0; (i < MTK_MEMINFO_SIZE) && (g_mtk_gpu_meminfo[i].pid != 0); i++) {
		pr_warn(KERN_DEBUG "%10d\t%16d\n", g_mtk_gpu_meminfo[i].pid, g_mtk_gpu_meminfo[i].used_pages);
	}

	pr_warn(KERN_DEBUG "============================\n");
	pr_warn(KERN_DEBUG "%10s\t%16u\n", "Total", g_mtk_gpu_total_memory_usage_in_pages_debugfs);
	pr_warn(KERN_DEBUG "============================\n");
	return true;
}

unsigned int mtk_get_mali_memory_usage(void)
{
	return (g_mtk_gpu_total_memory_usage_in_pages_debugfs*4096);
}
#endif

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *mali_pentry;

// For per-process memory usage query
typedef struct _MTK_MEM_PROC_REC
{
	pid_t					    pid;
	int		                    i32Bytes;
    struct _MTK_MEM_PROC_REC    *psNext;
	struct _MTK_MEM_PROC_REC    **ppsThis;
} MTK_MEM_PROC_REC;

void mtrack_mali_session_memory_tracking(_mali_osk_print_ctx *print_ctx)
{
	struct mali_session_data *session, *tmp;
	u32 mali_mem_usage;
	u32 total_mali_mem_size;
#ifdef MALI_MEM_SWAP_TRACKING
	u32 swap_pool_size;
	u32 swap_unlock_size;
#endif
#ifdef ENABLE_MTK_MEMINFO
    u32 mtk_kbase_gpu_meminfo_index = 0;
#endif /* ENABLE_MTK_MEMINFO */

	MALI_DEBUG_ASSERT_POINTER(print_ctx);
	mali_session_lock();

#ifdef MALI_MEM_SWAP_TRACKING
	_mali_osk_ctxprintf(print_ctx, "  %-25s  %-10s  %-10s  %-15s  %-15s  %-10s  %-10s %-10s \n"\
		   "=================================================================================================================================\n",
		   "Name (:bytes)", "pid", "mali_mem", "max_mali_mem",
		   "external_mem", "ump_mem", "dma_mem", "swap_mem");
#else
	_mali_osk_ctxprintf(print_ctx, "  %-25s  %-10s  %-10s  %-15s  %-15s  %-10s  %-10s \n"\
		   "========================================================================================================================\n",
		   "Name (:bytes)", "pid", "mali_mem", "max_mali_mem",
		   "external_mem", "ump_mem", "dma_mem");
#endif

	MALI_SESSION_FOREACH(session, tmp, link) {
#ifdef MALI_MEM_SWAP_TRACKING
		_mali_osk_ctxprintf(print_ctx, "  %-25s  %-10u  %-10u  %-15u  %-15u  %-10u  %-10u  %-10u\n",
				    session->comm, session->pid,
				    (atomic_read(&session->mali_mem_allocated_pages)) * _MALI_OSK_MALI_PAGE_SIZE,
				    (unsigned int)session->max_mali_mem_allocated_size,
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_EXTERNAL])) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_UMP])) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_DMA_BUF])) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_SWAP])) * _MALI_OSK_MALI_PAGE_SIZE)
				   );
#else
		_mali_osk_ctxprintf(print_ctx, "  %-25s  %-10u  %-10u  %-15u  %-15u  %-10u  %-10u  \n",
				    session->comm, session->pid,
				    (unsigned int)((atomic_read(&session->mali_mem_allocated_pages)) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)session->max_mali_mem_allocated_size,
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_EXTERNAL])) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_UMP])) * _MALI_OSK_MALI_PAGE_SIZE),
				    (unsigned int)((atomic_read(&session->mali_mem_array[MALI_MEM_DMA_BUF])) * _MALI_OSK_MALI_PAGE_SIZE)
				   );
#endif
		mtk_gpu_meminfo_set(mtk_kbase_gpu_meminfo_index, session->pid, (atomic_read(&session->mali_mem_allocated_pages)));
		mtk_kbase_gpu_meminfo_index++;
	}
	mali_session_unlock();
	mali_mem_usage  = _mali_ukk_report_memory_usage();

	//  Memory by Page
    g_mtk_gpu_total_memory_usage_in_pages_debugfs = mali_mem_usage/4096;

	total_mali_mem_size = _mali_ukk_report_total_memory_size();
	_mali_osk_ctxprintf(print_ctx, "Mali mem usage: %u\nMali mem limit: %u\n", mali_mem_usage, total_mali_mem_size);

#ifdef MALI_MEM_SWAP_TRACKING
	mali_mem_swap_tracking(&swap_pool_size, &swap_unlock_size);
	_mali_osk_ctxprintf(print_ctx, "Mali swap mem pool : %u\nMali swap mem unlock: %u\n", swap_pool_size, swap_unlock_size);
#endif
}


static int gpu_memory_usage_show(struct seq_file *m, void *v)
{
	int i = 0;

	// clean the meminfo
	for (i = 0; i < MTK_MEMINFO_SIZE; i++) {
		g_mtk_gpu_meminfo[i].pid = 0;
		g_mtk_gpu_meminfo[i].used_pages = 0;
	}

	// get the meminfo
    mtrack_mali_session_memory_tracking(m);
	seq_printf(m, "============================\n");
	seq_printf(m, "%10s\t%16s\n", "PID", "Memory by Page");
	seq_printf(m, "============================\n");

	// print meminfo
	for (i = 0; (i < MTK_MEMINFO_SIZE) && (g_mtk_gpu_meminfo[i].pid != 0); i++) {
    	seq_printf(m,  "%10d\t%16d\n", g_mtk_gpu_meminfo[i].pid, g_mtk_gpu_meminfo[i].used_pages);
	}

    seq_printf(m,  "============================\n");
    seq_printf(m,  "%10s\t%16u\n", "Total", g_mtk_gpu_total_memory_usage_in_pages_debugfs);
    seq_printf(m,  "============================\n");

    return 0;
}

static int gpu_memory_usage_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpu_memory_usage_show, NULL);
}

static const struct file_operations gpu_memory_usage_procfs_open = {
	.open	 = gpu_memory_usage_open,
	.read    = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};

void proc_mali_register(void)
{
	mali_pentry = proc_mkdir("mali", NULL);

	if (!mali_pentry)
		return;

	proc_create("memory_usage", 0, mali_pentry, &gpu_memory_usage_procfs_open);

}

void proc_mali_unregister(void)
{
	if (!mali_pentry)
		return;

	remove_proc_entry("memory_usage", mali_pentry);
	mali_pentry = NULL;
}
#else
#define proc_mali_register() do {} while (0)
#define proc_mali_unregister() do {} while (0)
#endif /* CONFIG_PROC_FS */
