/*
 *  linux/arch/arm/mach-axxia/platsmp.c
 *
 *  Copyright (C) 2012 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>

#include <mach/axxia-gic.h>

/*
 * Control for which core is the next to come out of the secondary
 * boot "holding pen".
 */
volatile int __cpuinitdata pen_release = -1;

extern void axxia_secondary_startup(void);

#define APB2_SER3_PHY_ADDR      0x002010030000ULL
#define APB2_SER3_ADDR_SIZE   0x10000

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_RAW_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	/*
	 * If this isn't the first physical core in a secondary cluster
	 * then run the standard GIC secondary init routine. Otherwise,
	 * run the Axxia secondary cluster init routine.
	 */
	if (cpu_logical_map(cpu) % 4)
		axxia_gic_secondary_init();
	else
		axxia_gic_secondary_cluster_init();

	/*
	 * Let the primary processor know we're out of the
	 * pen, then head off into the C entry point.
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	_raw_spin_lock(&boot_lock);
	_raw_spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	int phys_cpu, cluster;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one.
	 */
	_raw_spin_lock(&boot_lock);

	/*
	 * In the Axxia, the bootloader does not put the secondary cores
	 * into a wait-for-event (wfe) or wait-for-interrupt (wfi) state
	 * because of the multi-cluster design (i.e., there's no way for
	 * the primary core in cluster 0 to send an event or interrupt
	 * to secondary cores in the other clusters).
	 *
	 * Instead, the secondary cores are immediately put into a loop
	 * that polls the "pen_release" global and MPIDR register. The two
	 * are compared and if they match, a secondary core then executes
	 * the Axxia secondary startup code.
	 *
	 * Here we convert the "cpu" variable to be compatible with the
	 * ARM MPIDR register format (CLUSTERID and CPUID):
	 *
	 * Bits:   |11 10 9 8|7 6 5 4 3 2|1 0
	 *         | CLUSTER | Reserved  |CPU
	 */
	phys_cpu = cpu_logical_map(cpu);
	cluster = (phys_cpu / 4) << 8;
	phys_cpu = cluster + (phys_cpu % 4);

	/* Release the specified core */
	write_pen_release(phys_cpu);

#ifdef CONFIG_HOTPLUG_CPU
	/* Send a wakeup IPI to get the idled cpu out of WFI state */
	axxia_gic_raise_softirq(cpumask_of(cpu), 1);
#endif

	/* Wait for so long, then give up if nothing happens ... */
#ifdef CONFIG_ARCH_AXXIA_SIM
	timeout = jiffies + (1 * HZ);
#else
	timeout = jiffies + (10 * HZ);
#endif

	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * Now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish.
	 */
	_raw_spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	int ncores = 0;
	struct device_node *np;
	u32 cpu_num;

	for_each_node_by_name(np, "cpu") {
		if (++ncores > nr_cpu_ids) {
			pr_warn("SMP: More cores (%u) in DTB than max (%u)\n",
				ncores, nr_cpu_ids);
			break;
		}
		if (!of_property_read_u32(np, "reg", &cpu_num)) {
			if (cpu_num >= 0 && cpu_num < 16)
				set_cpu_possible(cpu_num, true);
			else
				pr_warn("SMP: Invalid cpu number (%u)\n",
					 cpu_num);
		}
	}

	set_smp_cross_call(axxia_gic_raise_softirq);
}

void __init
platform_smp_prepare_cpus(unsigned int max_cpus)
{
#ifdef CONFIG_ARCH_AXXIA_SIM
	int i;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * This is the entry point of the routine that the secondary
	 * cores will execute once they are released from their
	 * "holding pen".
	 */
	*(u32 *)phys_to_virt(0x10000020) =
		virt_to_phys(axxia_secondary_startup);
#else
	int i;
	void __iomem *apb2_ser3_base;
	unsigned long resetVal;
	int phys_cpu, cpu_count=0;
	struct device_node *np;
	unsigned long release_addr[NR_CPUS] = {0};
	unsigned long release;

	if (of_find_compatible_node(NULL, NULL, "lsi,axm5516")) {
		for_each_node_by_name(np, "cpu") {
			if (of_property_read_u32(np, "reg", &phys_cpu))
				continue;

			if (0 == phys_cpu)
				continue;

			if (of_property_read_u32(np, "cpu-release-addr",
						 &release))
				continue;

			release_addr[phys_cpu] = release;
			printk(KERN_ERR
			       "%s:%d - set address for %d to 0x%08x\n",
			       __FILE__, __LINE__,
			       phys_cpu, release_addr[phys_cpu]);
		}

		/*
		 * Initialise the present map, which describes the set of CPUs
		 * actually populated at the present time.
		 */

		apb2_ser3_base = ioremap(APB2_SER3_PHY_ADDR, APB2_SER3_ADDR_SIZE);

		for (i = 0; i < NR_CPUS; i++) {	
			/* check if this is a possible CPU and
			   it is within max_cpus range */
			if ((cpu_possible(i)) &&
			    (cpu_count < max_cpus) &&
			    (0 != release_addr[i])) {
				resetVal = readl(apb2_ser3_base + 0x1010);
				phys_cpu = cpu_logical_map(i);
				set_cpu_present(cpu_count, true);
				if (phys_cpu != 0) {
					writel(0xab, apb2_ser3_base+0x1000);
					resetVal &= ~(1 << phys_cpu);
					writel(resetVal, apb2_ser3_base+0x1010);
					udelay(1000);
				}
				cpu_count++;
			}
		}

		iounmap(apb2_ser3_base);

		/*
		 * This is the entry point of the routine that the secondary
		 * cores will execute once they are released from their
		 * "holding pen".
		 */
		*(u32 *)phys_to_virt(release) =
			virt_to_phys(axxia_secondary_startup);
		smp_wmb();
		__cpuc_flush_dcache_area((void *)phys_to_virt(release),
					 sizeof(u32));
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5516-sim")) {
		for (i = 0; i < max_cpus; i++)
                	set_cpu_present(i, true);

		/*
		 * This is the entry point of the routine that the secondary
		 * cores will execute once they are released from their
		 * "holding pen".
		 */
		*(u32 *)phys_to_virt(0x10000020) =
			virt_to_phys(axxia_secondary_startup);
		smp_wmb();
		__cpuc_flush_dcache_area((void *)phys_to_virt(0x10000020),
					 sizeof(u32));
	}

	return;
#endif
}
