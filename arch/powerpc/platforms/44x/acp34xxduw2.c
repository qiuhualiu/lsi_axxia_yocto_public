/*
 * PPC476 board specific routines
 *
 * - Add DUW2 support
 *
 * Copyright 2009 Torez Smith, IBM Corporation.
 *
 * Based on earlier code:
 *    Matt Porter <mporter@kernel.crashing.org>
 *    Copyright 2002-2005 MontaVista Software Inc.
 *
 *    Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>
 *    Copyright (c) 2003-2005 Zultys Technologies
 *
 *    Rewritten and ported to the merged powerpc tree:
 *    Copyright 2007 David Gibson <dwg@au1.ibm.com>, IBM Corporation.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/rtc.h>
#include <mm/mmu_decl.h>
#include <linux/pci.h>
#include <linux/rio_ids.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>

#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/time.h>
#include <asm/uic.h>
#include <asm/ppc4xx.h>
#include <asm/mpic.h>
#include <asm/mmu.h>
#include <asm/pci.h>
#include <asm/lsi_ppc4xx_pcie.h>
#include <asm/acp_pcie.h>
#include <asm/acp_rio_hotplug.h>
#include <asm/rio.h>

static __initdata struct of_device_id acp3400duw2_of_bus[] = {
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,plb6", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "dus,duxi-fpga", },
	{ .compatible = "dus,cbf4-fpga" },
	{ .compatible = "acp,rapidio-delta", },
	{},
};

static int __init acp3400duw2_device_probe(void)
{
	of_platform_bus_probe(NULL, acp3400duw2_of_bus, NULL);

	return 0;
}
machine_device_initcall(acp3400duw2, acp3400duw2_device_probe);

/* We can have either UICs or MPICs */
static void __init acp3400duw2_init_irq(void)
{
	struct device_node *np;

	/* Find top level interrupt controller */
	for_each_node_with_property(np, "interrupt-controller") {
		if (of_get_property(np, "interrupts", NULL) == NULL)
			break;
	}
	if (np == NULL)
		panic("Can't find top level interrupt controller");

	/* Check type and do appropriate initialization */
	if (of_device_is_compatible(np, "chrp,open-pic")) {
		/* The MPIC driver will get everything it needs from the
		 * device-tree, just pass 0 to all arguments
		 */
		struct mpic *mpic =
			mpic_alloc(np, 0, 0, 0, 0, " MPIC     ");
		BUG_ON(mpic == NULL);
		mpic_init(mpic);
		ppc_md.get_irq = mpic_get_irq;
	} else
		panic("Unrecognized top level interrupt controller");
}

#ifdef CONFIG_SMP
static void __cpuinit smp_acp3400duw2_setup_cpu(int cpu)
{
	mpic_setup_this_cpu();
}

static int __cpuinit smp_acp3400duw2_kick_cpu(int cpu)
{
	struct device_node *cpunode = of_get_cpu_node(cpu, NULL);
	const u64 *spin_table_addr_prop;
	u32 *spin_table;
	extern void start_secondary_47x(void);

	BUG_ON(cpunode == NULL);

	/* Assume spin table. We could test for the enable-method in
	 * the device-tree but currently there's little point as it's
	 * our only supported method
	 */
	spin_table_addr_prop =
		of_get_property(cpunode, "cpu-release-addr", NULL);

	if (spin_table_addr_prop == NULL) {
		pr_err("CPU%d: Can't start, macpx1ing cpu-release-addr !\n",
		       cpu);
		return -1;
	}

	/* Assume it's mapped as part of the linear mapping. This is a bit
	 * fishy but will work fine for now
	 */
	spin_table = (u32 *)__va(*spin_table_addr_prop);
	pr_debug("CPU%d: Spin table mapped at %p\n", cpu, spin_table);

	spin_table[3] = cpu;
	smp_wmb();
	spin_table[1] = __pa(start_secondary_47x);
	mb();
	return 0;
}

static struct smp_ops_t acpx1_smp_ops = {
	.probe		= smp_mpic_probe,
	.message_pass	= smp_mpic_message_pass,
	.setup_cpu	= smp_acp3400duw2_setup_cpu,
	.kick_cpu	= smp_acp3400duw2_kick_cpu,
	.give_timebase	= smp_generic_give_timebase,
	.take_timebase	= smp_generic_take_timebase,
};

static void __init acp3400duw2_smp_init(void)
{
	if (mmu_has_feature(MMU_FTR_TYPE_47x))
		smp_ops = &acpx1_smp_ops;
}

#else /* CONFIG_SMP */
static void __init acp3400duw2_smp_init(void) { }
#endif /* CONFIG_SMP */

static void __init acp3400duw2_setup_arch(void)
{
	acp3400duw2_smp_init();
}

static void __devinit acp3400duw2_pcibios_after_init(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "lsi,plb-pciex") {
		struct pci_controller *hose = pci_find_hose_for_OF_device(np);

		if (hose) {
			if (hose->bus) {
				if (lsi_ppc4xx_pci_sysfs_init(hose))
					goto err;
				if (acp_pci_sysfs_init(hose))
					goto err;
			}
		}
	}

	return;

err:
	pr_info("create sysfs failed\n");
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init acp3400duw2_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "acp34xxduw2"))
		return 0;

	return 1;
}

extern int __init acp3400_clk_init(void);

static void __init acp3400duw2_init(void)
{
	unsigned long val;

	/* TMP FIX: disable System watchdog */
	val = mfdcr(0xe03);
	if (val & 0x08000000) {
		pr_info("disabling SystemWD (misc_ctrl:0x%lx)\n", val);
		val &= ~0x08000000;
		mtdcr(0xe03, val);
	}

	acp3400_clk_init();
}

/*
 * Early initialization.
 */
static void __init acp3400duw2_init_early(void)
{
	udbg_arm_amba_init_uart();

	/* workaround for having VP engines memory in a middle of DRAM */
	__allow_ioremap_reserved = 1;
}

/*
 * the default powerpc phys_mem_access_prot function will
 * mark any memory mapping above the top of linux managed
 * DRAM as uncached. For ACP we (probably) want any sysmem
 * mapping to be cached. For now we define our own
 * phys_mem_access_prot() method that just uses the default
 * memmory attributes for any sysmem address.
 */
pgprot_t acp3400duw2_phys_mem_access_prot(struct file *file, unsigned long pfn,
					  unsigned long size, pgprot_t vma_prot)
{
	if (pfn >= 0x01000000) {
		/* address is above maximum possible sysmem size */
		vma_prot = pgprot_noncached(vma_prot);
	}

	return vma_prot;
}

#define SPRN_MCSR_CLR	0x33c

int acp3400duw2_mcheck_exception(struct pt_regs *regs)
{
	u32 esr;
	u32 mcsr;

	esr = mfspr(SPRN_ESR);
	if (esr & ESR_MCI) {
		printk(KERN_ERR	"Instruction Synchronous Machine Check "
				"exception\n");
		mtspr(SPRN_ESR, esr & ~ESR_MCI);
		return 0;
	}

	if (acp_rio_mcheck_exception(regs))
		goto silent_out;

	mcsr = mfspr(SPRN_MCSR);

	printk(KERN_ERR "Machine check in kernel mode.\n");
	printk(KERN_ERR "Caused by (from MCSR=%x): ", mcsr);

	if (mcsr & MCSR_TLBP)
		printk(KERN_ERR "UTLB Parity Error\n");
	if (mcsr & MCSR_ICP) {
		flush_instruction_cache();
		printk(KERN_ERR "I-Cache Parity Error\n");
	}
	if (mcsr & MCSR_DCSP)
		printk(KERN_ERR "D-Cache Search Parity Error\n");
	if (mcsr & PPC47x_MCSR_GPR)
		printk(KERN_ERR "GPR Parity Error\n");
	if (mcsr & PPC47x_MCSR_FPR)
		printk(KERN_ERR "FPR Parity Error\n");
	if (mcsr & PPC47x_MCSR_IPR)
		printk(KERN_ERR "Machine Check exception is imprecise\n");
	if (mcsr & PPC47x_MCSR_L2)
		printk(KERN_ERR "Error or system error reported through "
				"the L2 cache\n");
	if (mcsr & PPC47x_MCSR_DCR)
		printk(KERN_ERR "DCR timeout\n");

	/* Clear MCSR */
	mtspr(SPRN_MCSR_CLR, mcsr);

	return 0;
silent_out:
	return mfspr(SPRN_MCSR) == 0;
}

define_machine(acp3400duw2) {
	.name			= "ACP3400DUW2",
	.probe			= acp3400duw2_probe,
	.init                   = acp3400duw2_init,
	.init_early             = acp3400duw2_init_early,
	.progress		= udbg_progress,
	.init_IRQ		= acp3400duw2_init_irq,
	.setup_arch		= acp3400duw2_setup_arch,
	.pcibios_after_init     = acp3400duw2_pcibios_after_init,
	.restart		= ppc4xx_reset_system,
	.calibrate_decr		= generic_calibrate_decr,
	.phys_mem_access_prot   = acp3400duw2_phys_mem_access_prot,
	.machine_check_exception = acp3400duw2_mcheck_exception,
};
