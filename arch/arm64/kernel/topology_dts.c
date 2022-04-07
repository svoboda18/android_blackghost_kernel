// SPDX-License-Identifier:GPL-2.0
/*
 * Copyright (C) 2017 MediaTek Inc.
 */

/* TODO: implement function */
int arch_get_nr_clusters(void)
{
	return 2;
}

/* TODO: implement function */
void sched_arch_get_cluster_cpus(struct cpumask *cpus, int cluster_id)
{
}

/* TODO: implement function */
unsigned long arch_get_max_cpu_capacity(int cpu)
{
	return 0;
}

/* TODO: implement function */
unsigned long arch_get_cur_cpu_capacity(int cpu)
{
	return 0;
}

/* TODO: implement function */
int arch_get_cluster_id(unsigned int cpu)
{
	return 0;
}
