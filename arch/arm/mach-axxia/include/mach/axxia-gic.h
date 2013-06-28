/*
 *  arch/arm/mach-axxia/axxia-gic.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __AXXIA_GIC_H
#define __AXXIA_GIC_H

void axxia_gic_handle_irq(struct pt_regs *regs);
void axxia_gic_raise_softirq(const struct cpumask *mask, unsigned int irq);
void axxia_gic_secondary_cluster_init(void);
void axxia_gic_secondary_init(void);

#endif
