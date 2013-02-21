/*
 * drivers/lsi/acp/wrappers.c
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <asm/irq.h>
#include <asm/io.h>

/*
  ==============================================================================
  ==============================================================================
  MDIO Access
  ==============================================================================
  ==============================================================================
*/

#ifndef CONFIG_ACPISS

#undef BZ33327_WA
#define BZ33327_WA

static unsigned long mdio_base;
DEFINE_SPINLOCK(mdio_lock);

#define MDIO_CONTROL_RD_DATA ((void *)(mdio_base + 0x0))
#define MDIO_STATUS_RD_DATA  ((void *)(mdio_base + 0x4))
#define MDIO_CLK_OFFSET      ((void *)(mdio_base + 0x8))
#define MDIO_CLK_PERIOD      ((void *)(mdio_base + 0xc))

/*
  ------------------------------------------------------------------------------
  acp_mdio_read
*/

int
acp_mdio_read(unsigned long address,
	      unsigned long offset,
	      unsigned short *value)
{
	unsigned long command = 0;
	unsigned long status;
	unsigned long flags;

	spin_lock_irqsave(&mdio_lock, flags);
#if defined(BZ33327_WA)
	/* Set the mdio_busy (status) bit. */
	status = in_le32(MDIO_STATUS_RD_DATA);
	status |= 0x40000000;
	out_le32(MDIO_STATUS_RD_DATA, status);
#endif /* BZ33327_WA */

	/* Write the command.*/
	command |= 0x10000000;	/* op_code: read */
	command |= (address & 0x1f) << 16; /* port_addr (target device) */
	command |= (offset & 0x1f) << 21; /* device_addr (target register) */
	out_le32(MDIO_CONTROL_RD_DATA, command);

#if defined(BZ33327_WA)
	/* Wait for the mdio_busy (status) bit to clear. */
	do {
		status = in_le32(MDIO_STATUS_RD_DATA);
	} while (0 != (status & 0x40000000));

	/* Wait for the mdio_busy (control) bit to clear. */
	do {
		command = in_le32(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

	*value = (unsigned short)(command & 0xffff);
#endif /* BZ33327_WA */
	spin_unlock_irqrestore(&mdio_lock, flags);

	return 0;
}
EXPORT_SYMBOL(acp_mdio_read);

/*
  ------------------------------------------------------------------------------
  acp_mdio_write
*/

int
acp_mdio_write(unsigned long address,
	       unsigned long offset,
	       unsigned short value)
{
	unsigned long command = 0;
	unsigned long status;
	unsigned long flags;

	spin_lock_irqsave(&mdio_lock, flags);

	/* Wait for mdio_busy (control) to be clear. */
	do {
		command = in_le32(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

#if defined(BZ33327_WA)
	/* Set the mdio_busy (status) bit. */
	status = in_le32(MDIO_STATUS_RD_DATA);
	status |= 0x40000000;
	out_le32(MDIO_STATUS_RD_DATA, status);
#endif /* BZ33327_WA */

	/* Write the command. */
	command = 0x08000000;	/* op_code: write */
	command |= (address & 0x1f) << 16; /* port_addr (target device) */
	command |= (offset & 0x1f) << 21; /* device_addr (target register) */
	command |= (value & 0xffff); /* value */
	out_le32(MDIO_CONTROL_RD_DATA, command);

#if defined(BZ33327_WA)
	/* Wait for the mdio_busy (status) bit to clear. */
	do {
		status = in_le32(MDIO_STATUS_RD_DATA);
	} while (0 != (status & 0x40000000));
#endif /* BZ33327_WA */

	/* Wait for the mdio_busy (control) bit to clear. */
	do {
		command = in_le32(MDIO_CONTROL_RD_DATA);
	} while (0 != (command & 0x80000000));

	spin_unlock_irqrestore(&mdio_lock, flags);

	return 0;
}
EXPORT_SYMBOL(acp_mdio_write);

/*
  ------------------------------------------------------------------------------
  acp_mdio_initialize
*/

static int
acp_mdio_initialize(void)
{
	out_le32(MDIO_CLK_OFFSET, 0x10);
	out_le32(MDIO_CLK_PERIOD, 0x2c);

	return 0;
}

#endif /* CONFIG_ACPISS */

/*
  ==============================================================================
  ==============================================================================
  Interrupts
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  acp_irq_create_mapping
*/

unsigned int
acp_irq_create_mapping(struct irq_domain *host, irq_hw_number_t hwirq)
{
	return irq_create_mapping(host, hwirq);
}
EXPORT_SYMBOL(acp_irq_create_mapping);

/*
  ==============================================================================
  ==============================================================================
  Spin Locks
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  acp_spin_lock_init
*/

void
acp_spin_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}
EXPORT_SYMBOL(acp_spin_lock_init);

/*
  ------------------------------------------------------------------------------
  acp_spin_lock
*/

void
acp_spin_lock(spinlock_t *lock)
{
	spin_lock(lock);
}
EXPORT_SYMBOL(acp_spin_lock);

/*
  ------------------------------------------------------------------------------
  acp_spin_unlock
*/

void
acp_spin_unlock(spinlock_t *lock)
{
	spin_unlock(lock);
}
EXPORT_SYMBOL(acp_spin_unlock);

/*
  ------------------------------------------------------------------------------
  acp_spin_lock_bh
*/

void
acp_spin_lock_bh(spinlock_t *lock)
{
	spin_lock_bh(lock);
}
EXPORT_SYMBOL(acp_spin_lock_bh);

/*
  ------------------------------------------------------------------------------
  acp_spin_unlock_bh
*/

void
acp_spin_unlock_bh(spinlock_t *lock)
{
	spin_unlock_bh(lock);
}
EXPORT_SYMBOL(acp_spin_unlock_bh);

/*
  ------------------------------------------------------------------------------
  acp_spin_lock_irqsave
*/

void
acp_spin_lock_irqsave(spinlock_t *lock, unsigned long flags)
{
	spin_lock_irqsave(lock, flags);
}
EXPORT_SYMBOL(acp_spin_lock_irqsave);

/*
  ------------------------------------------------------------------------------
  acp_spin_unlock_irqrestore
*/

void
acp_spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
{
	spin_unlock_irqrestore(lock, flags);
}
EXPORT_SYMBOL(acp_spin_unlock_irqrestore);

/*
  ------------------------------------------------------------------------------
  acp_wrappers_init
*/

int __init
acp_wrappers_init(void)
{
	int rc = 0;

	printk(KERN_INFO "Initializing ACP Wrappers.\n");
#ifndef CONFIG_ACPISS
	mdio_base = (unsigned long) ioremap(0x002000409000ULL, 0x1000);
	rc = acp_mdio_initialize();
#endif

	if (0 != rc)
		printk(KERN_ERR "MDIO Initiailzation Failed!\n");

	return 0;
}
module_init(acp_wrappers_init);

MODULE_AUTHOR("LSI Corporation");
MODULE_DESCRIPTION("Timing Test");
MODULE_LICENSE("GPL");
