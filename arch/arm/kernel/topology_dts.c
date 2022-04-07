// SPDX-License-Identifier:GPL-2.0
/*
 * Copyright (C) 2017 MediaTek Inc.
 */
int arch_get_nr_clusters(void)
{
	int __arch_nr_clusters = -1;
	int max_id = 0;
	unsigned int cpu;

	/* assume socket id is monotonic increasing without gap. */
	for_each_possible_cpu(cpu) {
		struct cputopo_arm *cpu_topo = &cpu_topology[cpu];
		if (cpu_topo->socket_id > max_id)
			max_id = cpu_topo->socket_id;
	}
	__arch_nr_clusters = max_id + 1;
	return __arch_nr_clusters;
}

int arch_is_multi_cluster(void)
{
	return (arch_get_nr_clusters() > 1);
}

void sched_arch_get_cluster_cpus(struct cpumask *cpus, int socket_id)
{
	unsigned int cpu;

	cpumask_clear(cpus);
	for_each_possible_cpu(cpu) {
		struct cputopo_arm *cpu_topo = &cpu_topology[cpu];

		if (cpu_topo->socket_id == socket_id)
			cpumask_set_cpu(cpu, cpus);
	}
}

unsigned long arch_get_max_cpu_capacity(int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

unsigned long arch_get_cur_cpu_capacity(int cpu)
{
	unsigned long scale_freq;

	scale_freq  = arch_scale_freq_capacity(cpu);
	if (!scale_freq)
		scale_freq = SCHED_CAPACITY_SCALE;

	return (per_cpu(cpu_scale, cpu) * scale_freq / SCHED_CAPACITY_SCALE);
}

int arch_get_cluster_id(unsigned int cpu)
{
	struct cputopo_arm *cpu_topo = &cpu_topology[cpu];

	return cpu_topo->socket_id < 0 ? 0 : cpu_topo->socket_id;
}

int arch_is_smp(void)
{
	int cap = 0, max_cap = 0;
	int cpu;

	for_each_possible_cpu(cpu) {
		cap = arch_get_max_cpu_capacity(cpu);
		if (max_cap == 0)
			max_cap = cap;

		if (max_cap != cap)
			return 0;
	}
	return 1;
}
