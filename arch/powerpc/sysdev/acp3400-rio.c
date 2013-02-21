/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* #define DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/dmapool.h>
#include "acp3400-rio.h"

#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/uaccess.h>
#include <asm/acp3400-rio.h>

static DEFINE_SPINLOCK(rio_io_lock);

/**
 * NOTE:
 *
 * sRIO Bridge in ACP3400 is what it is...
 *
 * - Paged access to configuration registers makes
 *   local config read a non-atomic operation.
 *
 * - Big and Little Endian mode registers
 *   Big Endian:
 *   0x0000-0xFFFC   - RapidIO Standard Registers
 *   0x10000-0x1FFFC - Endpoint Controller Specific Registers
 *   Little Endian
 *   0x20000-0x3FFFC - Peripheral Bus Bridge Specific Registers
 *
 * To avoid an extra spin-lock layer in __acp_local_config_read
 * and __acp_local_config_write, all internal driver accesses
 * local config register through the generic rio driver API.
 *
 * Accesses trough the generic driver: __rio_local_write_config_32(),
 * __rio_local_read_config_32(), rio_mport_write_config_32() and
 * rio_mport_read_config_32() all uses spin_lock_irqsave().
 *
 */

/**
 * __acp_local_config_read - Generate a ACP3400 local config space read
 * @priv: Master port private data
 * @offset: Offset into configuration space
 * @data: Value to be read into
 *
 * Generates a ACP3400 local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int __acp_local_config_read(struct rio_priv *priv,
				   u32 offset, u32 *data)
{
	u32 page_sel;

	/* Set correct page to operate on */
	page_sel = (offset & 0x00fff800) << 5;
	out_le32(priv->regs_win_fixed + RAB_APB_CSR_BASE, page_sel);

	__asm__ __volatile__(PPC_MSYNC);

	if (offset < 0x20000) {
		/*
		 * RapidIO Standard Registers (0x0000-0xFFFC)
		 * Endpoint Controller Specific Registers (0x1_0000-0x1_FFFC)
		 */
		*data = in_be32(priv->regs_win_paged + (offset & 0x7ff));
	} else if ((offset >= 0x20000) && (offset < 0x40000)) {
		/*
		 * Peripheral Bus Bridge Specific Registers
		 * (0x2_0000-0x3_FFFC)
		 */
		*data = in_le32(priv->regs_win_paged + (offset & 0x7ff));
	} else {
		dev_err(priv->dev,
			"RIO: Reading config register not specified for AXXIA (0x%8.8x)\n",
			offset);
	}
	return 0;
}

/**
 * acp_local_config_write - Generate a ACP3400 local config space write
 * @priv: Master port private data
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a ACP3400 local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int __acp_local_config_write(struct rio_priv *priv,
				    u32 offset, u32 data)
{
	u32 page_sel;

	/* Set correct page to operate on */
	page_sel = (offset & 0x00fff800) << 5;
	out_le32(priv->regs_win_fixed + RAB_APB_CSR_BASE, page_sel);

	__asm__ __volatile__(PPC_MSYNC);

	if (offset < 0x20000) {
		/*
		 * RapidIO Standard Registers (0x0000-0xFFFC)
		 * Endpoint Controller Specific Registers (0x1_0000-0x1_FFFC)
		 */
		out_be32(priv->regs_win_paged + (offset & 0x7ff), data);
	} else if ((offset >= 0x20000) && (offset < 0x40000)) {
		/*
		 * Peripheral Bus Bridge Specific Registers (0x2_0000-0x3_FFFC)
		 */
		out_le32(priv->regs_win_paged + (offset & 0x7ff), data);
	} else {
		dev_err(priv->dev, "RIO: Trying to write to config register not specified for AXIA (0x%8.8x)\n",
			offset);
	}
	__asm__ __volatile__(PPC_MSYNC);

	return 0;
}

/**
 * acp_local_config_read - Generate a ACP3400 local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a ACP3400 local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int acp_local_config_read(struct rio_mport *mport,
				 int index, u32 offset, int len, u32 *data)
{
	struct rio_priv *priv = mport->priv;

	return __acp_local_config_read(priv, offset, data);
}

/**
 * acp_local_config_write - Generate a ACP3400 local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a ACP3400 local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int acp_local_config_write(struct rio_mport *mport,
				  int index, u32 offset, int len, u32 data)
{
	struct rio_priv *priv = mport->priv;

	return __acp_local_config_write(priv, offset, data);
}

#define __acp_read_rio_config(x, addr, err, op)			\
	__asm__ __volatile__(					\
		PPC_MSYNC "\n"					\
		"0:	"op" %1,0(%2)\n"			\
		"1:     sync\n"					\
		"2:     nop\n"					\
		"3:\n"						\
		".section .fixup,\"ax\"\n"			\
		"4:	li %1,-1\n"				\
		"	li %0,%3\n"				\
		"	b 3b\n"					\
		".previous\n"					\
		".section __ex_table,\"a\"\n"			\
		PPC_LONG_ALIGN "\n"				\
		PPC_LONG "0b,4b\n"				\
		PPC_LONG "1b,4b\n"				\
		PPC_LONG "2b,4b\n"				\
		".previous"					\
		: "=r" (err), "=r" (x)				\
		: "b" (addr), "i" (-EFAULT), "0" (err))


int acp_rio_mcheck_exception(struct pt_regs *regs)
{
	const struct exception_table_entry *entry;
	u32 mcsr = mfspr(SPRN_MCSR);

	if (mcsr & (PPC47x_MCSR_IPR | PPC47x_MCSR_L2)) {
		entry = search_exception_tables(regs->nip);
		if (entry) {
			pr_debug("(%s): Recoverable exception %lx\n",
				 __func__, regs->nip);
			regs->msr |= MSR_RI;
			regs->nip = entry->fixup;
			mcsr &= ~(PPC47x_MCSR_IPR | PPC47x_MCSR_L2);
			if (mcsr == MCSR_MCS)
				mcsr &= ~MCSR_MCS;
			mtspr(SPRN_MCSR, mcsr);
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(acp_rio_mcheck_exception);

/**
 * acp_rio_config_read - Generate a ACP3400 read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a ACP3400 read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */

static int acp_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			       u8 hopcount, u32 offset, int len, u32 *val)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb = &priv->outb_atmu[priv->maint_win_id];
	u8 *addr;
	u32 rval = 0;
	u32 rbar = 0, ctrl;
	int rc = 0;

	/* 16MB maintenance windows possible */
	/* allow only aligned access to maintenance registers */
	if (offset > (0x1000000 - len) || !IS_ALIGNED(offset, len))
		return -EINVAL;

	__acp_local_config_read(priv,
				RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				&ctrl);

	if (ctrl & 0x6) { /* Not maintanance */
		dev_err(priv->dev, "(%s): Window is not setup for Maintanance operations. 0x%8.8x\n",
			__func__, ctrl);
		return -EINVAL;
	}

	rbar |= HOP_COUNT(hopcount);
	__acp_local_config_write(priv,
				 RAB_APIO_AMAP_RBAR(priv->maint_win_id),
				 rbar);

	ctrl &= ~(0xffff0000); /* Target id clear */
	ctrl |= TARGID(destid); /* Target id set */
	__acp_local_config_write(priv,
				 RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				 ctrl);

	addr = (u8 *) aoutb->win +
		(offset & (CONFIG_RIO_MAINT_WIN_SIZE - 1));

	switch (len) {

	case 1:
		__acp_read_rio_config(rval, addr, rc, "lbz");
		break;
	case 2:
		__acp_read_rio_config(rval, addr, rc, "lhz");
		break;
	case 4:
		__acp_read_rio_config(rval, addr, rc, "lwz");
		break;
	default:
		rc = -EINVAL;
	}
	if (rc) {
		dev_dbg(priv->dev, "acp_rio_config_read: Error when reading\n");
		*val = 0;
	} else
		*val = rval;

	return rc;
}

/**
 * acp_rio_config_write - Generate a ACP3400 write maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an ACP3400 write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int acp_rio_config_write(struct rio_mport *mport, int index, u16 destid,
				u8 hopcount, u32 offset, int len, u32 val)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb = &priv->outb_atmu[priv->maint_win_id];
	u8 *data;
	u32 rbar = 0, ctrl;

	/* 16MB maintenance windows possible */
	/* allow only aligned access to maintenance registers */
	if (offset > (0x1000000 - len) || !IS_ALIGNED(offset, len))
		return -EINVAL;

	__acp_local_config_read(priv,
				RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				&ctrl);

	if (ctrl & 0x6) { /* Not maintanance */
		dev_err(priv->dev,
			"(%s): Window is not setup for Maintanance operations.\n",
			__func__);
		return -EINVAL;
	}

	rbar |= HOP_COUNT(hopcount);
	__acp_local_config_write(priv,
				 RAB_APIO_AMAP_RBAR(priv->maint_win_id),
				 rbar);

	ctrl &= ~(0xffff0000); /* Target id clear */
	ctrl |= TARGID(destid); /* Target id set */
	__acp_local_config_write(priv,
				 RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				 ctrl);

	data = (u8 *) aoutb->win +
		(offset & (CONFIG_RIO_MAINT_WIN_SIZE - 1));
	switch (len) {
	case 1:
		out_8((u8 *) data, val);
		break;
	case 2:
		out_be16((u16 *) data, val);
		break;
	case 4:
		out_be32((u32 *) data, val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline int __flags2rio_tr_type(u32 mflags, u32 *trans_type)
{
	*trans_type = 0;
	/* Set type of window */
	if ((mflags == 0) || (mflags & RIO_NWRITE_R))
		*trans_type = TTYPE(NRD_NWR_R); /* nread and nwrite_r */
	else if (mflags & RIO_MAINT_WRITE)
		*trans_type = TTYPE(MRD_MWR); /* mread and mwrite */
	else if (mflags & RIO_NWRITE)
		*trans_type = TTYPE(NRD_NWR); /* nread and nwrite */
	else if (mflags & RIO_SWRITE)
		*trans_type = TTYPE(NRD_SWR); /* nread and swrite */
	else
		return -EINVAL;
	return 0;
}

/**
 * acp_rio_map_outb_mem -- Mapping outbound memory.
 * @mport:  RapidIO master port
 * @win:    Outbound ATMU window for this access
 *          - obtained by calling acp_rio_req_outb_region.
 * @destid: Destination ID of transaction
 * @addr:   RapidIO space start address.
 * @res:    Mapping region phys and virt start address
 *
 * Return: 0 -- Success.
 *
 */
static int acp_rio_map_outb_mem(struct rio_mport *mport, u32 win,
				u16 destid, u32 addr, u32 mflags,
				struct rio_map_addr *res)
{
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb;
	u32 rbar = 0, ctrl, trans_type;
	unsigned long flags;
	int rc;

	rc = __flags2rio_tr_type(mflags, &trans_type);
	if (rc < 0) {
		dev_err(priv->dev, "(%s) invalid transaction flags %x\n",
			__func__, mflags);
		return rc;
	}

	spin_lock_irqsave(&rio_io_lock, flags);

	aoutb = &priv->outb_atmu[win];
	if (unlikely(win >= RIO_OUTB_ATMU_WINDOWS ||
		     !(aoutb->in_use && aoutb->riores))) {
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev, "(%s) faulty ATMU window (%d, %d, %8.8x)\n",
			__func__, win, aoutb->in_use, (u32) aoutb->riores);
		return -EINVAL;
	}
	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &ctrl);

	if ((ctrl & 0x6) != trans_type) {
		ctrl &= ~0x6;
		ctrl |= trans_type;
	}
	if (ctrl & 0x6) { /* RIO adress set - Not maintanance */
		rbar |= RIO_ADDR_BASE(addr);
		__rio_local_write_config_32(mport,
					    RAB_APIO_AMAP_RBAR(win),
					    rbar);
	}
	ctrl &= ~(0xffff0000); /* Target id clear */
	ctrl |= TARGID(destid); /* Target id set */
	ctrl |= ENABLE_AMBA; /* Enable window */
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_CTRL(win), ctrl);

	res->phys = aoutb->riores->start + RIO_ADDR_OFFSET(addr);
	res->va = aoutb->win + RIO_ADDR_OFFSET(addr);

	spin_unlock_irqrestore(&rio_io_lock, flags);

	return 0;
}

/**
 * acp_rio_req_outb_region -- Request outbound region in the
 * RapidIO bus address space.
 * @mport:  RapidIO master port
 * @size:   The mapping region size.
 * @name:   Resource name
 * @flags:  Flags for mapping. 0 for using default flags.
 * @id:     Allocated outbound ATMU window id
 *
 * Return: 0 -- Success.
 *
 * This function will reserve a memory region that may
 * be used to create mappings from local iomem to rio space.
 */
static int acp_rio_req_outb_region(struct rio_mport *mport,
				   resource_size_t size,
				   const char *name,
				   u32 mflags, u32 *id)
{
	u32 win, reg, win_size = 0, trans_type = 0, wabar = 0;
	struct rio_priv *priv = mport->priv;
	struct atmu_outb *aoutb;
	int rc = 0;
	void __iomem *iowin;
	struct resource *riores;
	unsigned long flags;

	if (!(is_power_of_2(size))) {
		dev_err(priv->dev, "(%s) size is not power of 2 (%llu)\n",
			__func__, size);
		return -EFAULT;
	}
	rc = __flags2rio_tr_type(mflags, &trans_type);
	if (rc < 0) {
		dev_err(priv->dev, "(%s) invalid transaction flags %x\n",
			__func__, mflags);
		return rc;
	}

	spin_lock_irqsave(&rio_io_lock, flags);

	for (win = 0; win < RIO_OUTB_ATMU_WINDOWS; win++) {
		if (!(priv->outb_atmu[win].in_use))
			break;
	}

	if (win == RIO_OUTB_ATMU_WINDOWS) {
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev,
			"(%s) out of ATMU windows to use\n",
			__func__);
		return -ENOMEM;
	}
	aoutb = &priv->outb_atmu[win];
	aoutb->in_use = 1;
	aoutb->win = NULL;
	aoutb->riores = NULL;

	riores = kzalloc(sizeof(struct resource), GFP_ATOMIC);
	if (!riores) {
		aoutb->in_use = 0;
		spin_unlock_irqrestore(&rio_io_lock, flags);
		dev_err(priv->dev,
			"(%s) failed to allocate resources\n",
			__func__);
		return -ENOMEM;
	}

	spin_unlock_irqrestore(&rio_io_lock, flags);

	riores->name = name;
	riores->flags = IORESOURCE_MEM;
	if (allocate_resource(&mport->iores, riores,
			      size, mport->iores.start,
			      mport->iores.end, 0x400, NULL, NULL)) {
		/* Allign on 1kB boundry */
		rc = -ENOMEM;
		goto out_err_resource;
	}

	iowin = ioremap(riores->start, size);
	if (!iowin) {
		rc = -ENOMEM;
		goto out_err_ioremap;
	}

	/* Set base adress for window on PIO side */
	wabar = AXI_BASE_HIGH(riores->start);
	wabar |= AXI_BASE(riores->start);
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_ABAR(win), wabar);

	/* Set size of window */
	win_size |= WIN_SIZE((u32)size);
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_SIZE(win), win_size);
	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &reg);
	reg &= ~0x6;
	reg |= trans_type;
	__rio_local_write_config_32(mport, RAB_APIO_AMAP_CTRL(win), reg);

	spin_lock_irqsave(&rio_io_lock, flags);
	aoutb->win = iowin;
	aoutb->riores = riores;
	spin_unlock_irqrestore(&rio_io_lock, flags);

	*id = win;
	return 0;

out_err_ioremap:
	dev_err(priv->dev, "(%s) ioremap IO-mem failed\n",
		__func__);
	if (release_resource(riores))
		dev_err(priv->dev, "(%s) clean-up resource failed\n", __func__);
out_err_resource:
	dev_err(priv->dev, "(%s) alloc IO-mem for %s failed\n",
		__func__, name);
	kfree(riores);

	spin_lock_irqsave(&rio_io_lock, flags);
	aoutb->in_use = 0;
	spin_unlock_irqrestore(&rio_io_lock, flags);
	return rc;
}

/**
 * acp_rio_release_outb_region -- Unreserve outbound memory region.
 * @mport: RapidIO master port
 * @win:   Allocated outbound ATMU window id
 *
 * Disables and frees the memory resource of an outbound memory region
 */
static void acp_rio_release_outb_region(struct rio_mport *mport,
					u32 win)
{
	struct rio_priv *priv = mport->priv;
	u32 ctrl;
	unsigned long flags;

	if (unlikely(win >= RIO_OUTB_ATMU_WINDOWS))
		return;

	spin_lock_irqsave(&rio_io_lock, flags);

	__rio_local_read_config_32(mport, RAB_APIO_AMAP_CTRL(win), &ctrl);
	if (likely(priv->outb_atmu[win].in_use)) {
		struct atmu_outb *aoutb = &priv->outb_atmu[win];
		struct resource *riores = aoutb->riores;
		void __iomem *iowin = aoutb->win;

		__rio_local_write_config_32(mport,
					    RAB_APIO_AMAP_CTRL(win),
					    ctrl & ~ENABLE_AMBA);
		aoutb->riores = NULL;
		aoutb->win = NULL;

		spin_unlock_irqrestore(&rio_io_lock, flags);

		iounmap(iowin);
		if (release_resource(riores))
			dev_err(priv->dev, "(%s) clean-up resource failed\n",
				__func__);
		kfree(riores);

		spin_lock_irqsave(&rio_io_lock, flags);
		aoutb->in_use = 0;
	}

	spin_unlock_irqrestore(&rio_io_lock, flags);
}


/**
 * rio_set_mport_disc_mode - Set master port discovery/eumeration mode
 *
 * @mport: Master port
 *
 */
void acp3400_rio_set_mport_disc_mode(struct rio_mport *mport)
{
	u32 result;

	if (mport->enum_host) {
		__rio_local_write_config_32(mport, RIO_GCCSR,
					    RIO_PORT_GEN_HOST |
					    RIO_PORT_GEN_MASTER |
					    RIO_PORT_GEN_DISCOVERED);
	} else {
		__rio_local_write_config_32(mport, RIO_GCCSR, 0x00000000);
		__rio_local_write_config_32(mport, RIO_DID_CSR,
					    RIO_SET_DID(mport->sys_size,
					    RIO_ANY_DESTID(mport->sys_size)));
	}
	__rio_local_read_config_32(mport, RIO_GCCSR, &result);

	/* Set to receive any dist ID for serial RapidIO controller. */
	if (mport->phy_type == RIO_PHY_SERIAL)
		__rio_local_write_config_32(mport, EPC_PNPTAACR, 0x00000000);

#ifdef CONFIG_RAPIDIO_HOTPLUG
	if (CONFIG_RAPIDIO_SECOND_DEST_ID != DESTID_INVALID) {
		struct rio_priv *priv = mport->priv;

		result = EPC_PNADIDCSR_ADE;
		result |= EPC_PNADIDCSR_ADID(CONFIG_RAPIDIO_SECOND_DEST_ID);
		__rio_local_write_config_32(mport, EPC_PNADIDCSR, result);
		dev_dbg(priv->dev, "Second destination id set to 0x%X for main port\n",
			 CONFIG_RAPIDIO_SECOND_DEST_ID);
	}
#endif

}

/**
 * rio_init_port_data - HW Setup of master port
 *
 * @mport: Master port
 *
 */
static void rio_init_port_data(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ccsr, data;

#if defined(CONFIG_ACP_RIO_16B_ID)
	__rio_local_read_config_32(mport, RAB_SRDS_CTRL0, &data);
	__rio_local_write_config_32(mport, RAB_SRDS_CTRL0,
				    data | RAB_SRDS_CTRL0_16B_ID);
#endif
	/* Probe the master port phy type */
	__rio_local_read_config_32(mport, RIO_CCSR, &ccsr);
	mport->phy_type = (ccsr & 1) ? RIO_PHY_SERIAL : RIO_PHY_PARALLEL;
	dev_dbg(priv->dev, "RapidIO PHY type: %s\n",
		 (mport->phy_type == RIO_PHY_PARALLEL) ? "parallel" :
		 ((mport->phy_type == RIO_PHY_SERIAL) ? "serial" :
		  "unknown"));

	__rio_local_read_config_32(mport, RIO_PEF_CAR, &data);
	mport->sys_size = (data & RIO_PEF_CTLS) >> 4;
	dev_dbg(priv->dev, "RapidIO Common Transport System size: %d\n",
		mport->sys_size ? 65536 : 256);
}

/**
 * acp_rio_info - Log Port HW setup
 *
 * @dev: RIO device
 * @ccsr: Port N Error and Command Status register
 *
 */
static void acp_rio_info(struct device *dev, u32 ccsr)
{
	const char *str;
	if (ccsr & 1) {
		/* Serial phy */
		switch (ccsr >> 30) {
		case 0:
			str = "1";
			break;
		case 1:
			str = "4";
			break;
		default:
			str = "Unknown";
			break;
		}
		dev_dbg(dev, "Hardware port width: %s\n", str);

		switch ((ccsr >> 27) & 7) {
		case 0:
			str = "Single-lane 0";
			break;
		case 1:
			str = "Single-lane 2";
			break;
		case 2:
			str = "Four-lane";
			break;
		default:
			str = "Unknown";
			break;
		}
		dev_dbg(dev, "Training connection status: %s\n", str);
	} else {
		/* Parallel phy */
		if (!(ccsr & 0x80000000))
			dev_dbg(dev, "Output port operating in 8-bit mode\n");
		if (!(ccsr & 0x08000000))
			dev_dbg(dev, "Input port operating in 8-bit mode\n");
	}
}

/**
 * rio_start_port - Check the master port
 * @mport: Master port to be checked
 *
 * Check the type of the master port and if it is not ready try to
 * restart the connection.
 */
static int rio_start_port(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ccsr, escsr;

	/* Probe the master port phy type */
	__rio_local_read_config_32(mport, RIO_CCSR, &ccsr);
	__rio_local_read_config_32(mport, RIO_ESCSR, &escsr);

	if (escsr & RIO_ESCSR_PU) {

		dev_err(priv->dev,
			"Port is not ready/restart ordered. Try to restart connection...\n");
		/* Disable ports */
		ccsr |= RIO_CCSR_PD;
		__rio_local_write_config_32(mport, RIO_CCSR, ccsr);
		switch (mport->phy_type) {
		case RIO_PHY_SERIAL:
			/* Set 1x lane */
			ccsr &= ~RIO_CCSR_PWO;
			ccsr |= RIO_CCSR_FORCE_LANE0;
			__rio_local_write_config_32(mport, RIO_CCSR, ccsr);
			break;
		case RIO_PHY_PARALLEL:
			break;
		}
		/* Enable ports */
		ccsr &= ~RIO_CCSR_PD;
		__rio_local_write_config_32(mport, RIO_CCSR, ccsr);
		msleep(100);
		__rio_local_read_config_32(mport, RIO_ESCSR, &escsr);
		acp_rio_info(priv->dev, ccsr);
		if (escsr & RIO_ESCSR_PU) {
			dev_dbg(priv->dev, "Port restart failed.\n");
			return -ENOLINK;
		} else {
			dev_dbg(priv->dev, "Port restart success!\n");
			return 0;
		}
	}
	dev_dbg(priv->dev, "Port Is ready\n");
	return 0;
}

/**
 * rio_rab_ctrl_setup - Bridge Control HW setup
 *
 * @mport: Master port
 *
 * Response Prio = request prio +1. 2) No AXI byte swap
 * Internal (RIO Mem) DME desc access
 * Priority based MSG arbitration
 * RIO & AMBA PIO Enable
 */
static void rio_rab_ctrl_setup(struct rio_mport *mport)
{
	u32 rab_ctrl;

	__rio_local_write_config_32(mport, AXI_TIMEOUT, 0x1000);
	__rio_local_write_config_32(mport, DME_TIMEOUT, 0xC0001000);

	rab_ctrl = 0;
	rab_ctrl |= (1 << 12);
	rab_ctrl |= (2 << 6);
	rab_ctrl |= 3;
	__rio_local_write_config_32(mport, RAB_CTRL, rab_ctrl);
}

/**
 * rio_rab_pio_enable - Setup Peripheral Bus bridge,
 *                      RapidIO <-> Peripheral bus, HW.
 *
 * @mport: Master port
 *
 * Enable AXI PIO + outbound nwrite/nread/maintenance
 * Enable RIO PIO (enable rx maint port-write packet)
 */
static void rio_rab_pio_enable(struct rio_mport *mport)
{
	__rio_local_write_config_32(mport, RAB_APIO_CTRL,
				    RAB_APIO_MAINT_MAP_EN |
				    RAB_APIO_MEM_MAP_EN |
				    RAB_APIO_PIO_EN);
	__rio_local_write_config_32(mport, RAB_RPIO_CTRL, RAB_RPIO_PIO_EN);
}

/**
 * rio_static_win_init -- Setup static ATMU window for maintenance
 *                        access and enable doorbells
 *
 * @mport: Master port
 *
 * Returns:
 * 0        - At success
 * -EFAULT  - Requested utbound region can not be claimed
 */
int acp3400_rio_static_win_init(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ctrl;

	/* Enable inbound doorbell */
	__rio_local_write_config_32(mport, RAB_IB_DB_CSR, IB_DB_CSR_EN);

	/* Configure maintenance transaction window */
	if ((acp_rio_req_outb_region(mport, CONFIG_RIO_MAINT_WIN_SIZE,
				     "rio_maint_win", RIO_MAINT_WRITE,
				     &priv->maint_win_id)) < 0)
		goto err;

	__rio_local_read_config_32(mport,
				   RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				   &ctrl);
	/* Enable window */
	ctrl |= ENABLE_AMBA;
	__rio_local_write_config_32(mport,
				    RAB_APIO_AMAP_CTRL(priv->maint_win_id),
				    ctrl);

	return 0;
err:
	return -EFAULT;
}

/**
 * acp3400_rio_static_win_release -- Release static ATMU maintenance window
 *                                   disable doorbells
 *
 * @mport: Master port
 *
 */
void acp3400_rio_static_win_release(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	u32 ibdb;

	/* Disable inbound doorbell */
	__rio_local_read_config_32(mport, RAB_IB_DB_CSR, &ibdb);
	ibdb &= ~IB_DB_CSR_EN;
	__rio_local_write_config_32(mport, RAB_IB_DB_CSR, ibdb);

	/* Release maintenance transaction window */
	acp_rio_release_outb_region(mport, priv->maint_win_id);
}

/**
 * rio_parse_dtb - Parse RapidIO platform entry
 *
 * @dev: RIO platform device
 * @law_start: Local Access Window start address from DTB
 * @law_size: Local Access Window size from DTB
 * @regs: RapidIO registers from DTB
 * @irq: RapidIO IRQ mapping from DTB
 *
 * Returns:
 * -EFAULT          At failure
 * 0                Success
 */
static int rio_parse_dtb(struct platform_device *dev,
			 u64 *law_start, u64 *law_size,
			 struct resource *regs,
			 int *irq)
{
	const u32 *dt_range, *cell;
	int rlen, rc;
	int paw, aw, sw;

	if (!dev->dev.of_node) {
		dev_err(&dev->dev, "Device OF-Node is NULL");
		return -EFAULT;
	}

	rc = of_address_to_resource(dev->dev.of_node, 0, regs);
	if (rc) {
		dev_err(&dev->dev, "Can't get %s property 'reg'\n",
			dev->dev.of_node->full_name);
		return -EFAULT;
	}
	dev_dbg(&dev->dev,
		"Of-device full name %s\n",
		 dev->dev.of_node->full_name);
	dev_dbg(&dev->dev, "Regs: %pR\n", regs);

	dt_range = of_get_property(dev->dev.of_node, "ranges", &rlen);
	if (!dt_range) {
		dev_err(&dev->dev, "Can't get %s property 'ranges'\n",
			dev->dev.of_node->full_name);
		return -EFAULT;
	}

	/* Get node address wide */
	cell = of_get_property(dev->dev.of_node, "#address-cells", NULL);
	if (cell)
		aw = *cell;
	else
		aw = of_n_addr_cells(dev->dev.of_node);
	/* Get node size wide */
	cell = of_get_property(dev->dev.of_node, "#size-cells", NULL);
	if (cell)
		sw = *cell;
	else
		sw = of_n_size_cells(dev->dev.of_node);
	/* Get parent address wide wide */
	paw = of_n_addr_cells(dev->dev.of_node);

	*law_start = of_read_number(dt_range + aw, paw);
	*law_size = of_read_number(dt_range + aw + paw, sw);

	dev_dbg(&dev->dev, "LAW start 0x%016llx, size 0x%016llx.\n",
		*law_start, *law_size);

	*irq = irq_of_parse_and_map(dev->dev.of_node, 0);
	dev_dbg(&dev->dev, "irq: %d\n", *irq);

	return 0;
}

/**
 * rio_ops_setup - Alloc and initiate the RIO ops struct
 *
 * Returns:
 * ERR_PTR(-ENOMEM)      At failure
 * struct rio_ops *ptr   to initialized ops data at Success
 */
static struct rio_ops *rio_ops_setup(void)
{
	struct rio_ops *ops = kzalloc(sizeof(*ops), GFP_KERNEL);

	if (!ops)
		return ERR_PTR(-ENOMEM);

	ops->lcread = acp_local_config_read;
	ops->lcwrite = acp_local_config_write;
	ops->cread = acp_rio_config_read;
	ops->cwrite = acp_rio_config_write;
	ops->dsend = acp3400_rio_doorbell_send;
	ops->pwenable = acp3400_rio_pw_enable;
	ops->open_outb_mbox = acp3400_open_outb_mbox;
	ops->open_inb_mbox = acp3400_open_inb_mbox;
	ops->close_outb_mbox = acp3400_close_outb_mbox;
	ops->close_inb_mbox = acp3400_close_inb_mbox;
	ops->add_outb_message = acp3400_add_outb_message;
	ops->add_inb_buffer = acp3400_add_inb_buffer;
	ops->get_inb_message = acp3400_get_inb_message;
#ifdef CONFIG_RAPIDIO_HOTPLUG
	ops->hotswap = acp3400_rio_hotswap;
	ops->port_notify_cb = acp3400_rio_port_notify_cb;
	ops->port_op_state = acp3400_rio_port_op_state;
#endif
	return ops;
}

/**
 * rio_mport_dtb_setup - Alloc and initialize the master port data
 *                       structure with data retrevied from DTB
 *
 * @dev: RIO platform device
 * @law_start: Local Access Window start address from DTB
 * @law_size: Local Access Window size from DTB
 * @ops: RIO ops data structure
 *
 * Init mport data structure
 * Request RIO iomem resources
 * Register doorbell and mbox resources with generic RIO driver

 * Returns:
 * ERR_PTR(-ENOMEM)        At failure
 * struct rio_mport *ptr   to initialized mport data at Success
 */
static int rio_port_index = 0;
static struct rio_mem_ops acp_mem_ops = {
	.req_outb = acp_rio_req_outb_region,
	.map_outb = acp_rio_map_outb_mem,
	.release_outb = acp_rio_release_outb_region,
};

static struct rio_mport *rio_mport_dtb_setup(struct platform_device *dev,
					     u64 law_start, u64 law_size,
					     struct rio_ops *ops)
{
	struct rio_mport *mport = kzalloc(sizeof(*mport), GFP_KERNEL);

	if (!mport)
		return ERR_PTR(-ENOMEM);

	mport->index = rio_port_index++; /* DTB maybe??? */

	INIT_LIST_HEAD(&mport->dbells);
	mport->iores.start = law_start;
	mport->iores.end = law_start + law_size - 1;
	mport->iores.flags = IORESOURCE_MEM;
	mport->iores.name = "rio_io_win";
	mport->iores.parent = NULL;
	mport->iores.child = NULL;
	mport->iores.sibling = NULL;

	if (request_resource(&iomem_resource, &mport->iores) < 0) {
		dev_err(&dev->dev,
			"RIO: Error requesting master port region 0x%016llx-0x%016llx\n",
			(u64)mport->iores.start, (u64)mport->iores.end);
		kfree(mport);
		return ERR_PTR(-ENOMEM);
	}
	rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0, 8);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0, 3);
	sprintf(mport->name, "RIO%d mport", mport->index);

	mport->ops = ops;
	mport->mops = &acp_mem_ops;
	mport->phys_efptr = 0x100; /* define maybe */

	return mport;
}

/**
 * rio_priv_dtb_setup - Alloc and initialize the master port private data
 *                      structure with data retrevied from DTB
 *
 * @dev: RIO platform device
 * @regs: RapidIO registers from DTB
 * @mport: master port
 *
 * Init master port private data structure
 *
 * Returns:
 * ERR_PTR(-ENOMEM)        At failure
 * struct rio_priv *ptr    to initialized priv data at Success
 */

static struct rio_priv *rio_priv_dtb_setup(struct platform_device *dev,
					   struct resource *regs,
					   struct rio_mport *mport,
					   int irq)
{
	struct rio_priv *priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	int i, rc;

	if (!priv)
		return ERR_PTR(-ENOMEM);

	memset(&priv->acpres[ACP_HW_DESC_RESOURCE], 0, sizeof(struct resource));
	priv->acpres[ACP_HW_DESC_RESOURCE].start = 0;
	priv->acpres[ACP_HW_DESC_RESOURCE].end = 1024;
	priv->acpres[ACP_HW_DESC_RESOURCE].flags = ACP_RESOURCE_HW_DESC;

	/* mport port driver handle */
	mport->priv = priv;
	/* Interrupt handling */
	priv->irq_line = irq;
	acp3400_rio_port_irq_init(mport);
	/* dev ptr for debug printouts */
	priv->dev = &dev->dev;
	/* init ATMU data structures */
	for (i = 0; i < RIO_OUTB_ATMU_WINDOWS; i++) {
		priv->outb_atmu[i].in_use = 0;
		priv->outb_atmu[i].riores = NULL;
	}
	/* setup local access */
	priv->regs_win_fixed = ioremap(regs->start, 0x800); /* define maybe? */
	if (!priv->regs_win_fixed) {
		rc = -ENOMEM;
		goto err_fixed;
	}
	priv->regs_win_paged = ioremap(regs->start + 0x800, 0x800);
	if (!priv->regs_win_paged) {
		rc = -ENOMEM;
		goto err_paged;
	}
	return priv;

err_paged:
	iounmap(priv->regs_win_fixed);
err_fixed:
	kfree(priv);
	return ERR_PTR(rc);
}
/**
 * acp3400_rio_start_port - Start master port
 *
 * @mport: Master port
 *
 * Check the type of the master port and if it is not ready try to
 * restart the connection.
 * In hotplug mode we don't really care about connection state
 * elsewise we give up if the port is not up.
 *
 * Setup HW for basic memap access support:
 * enable AXI bridge, maintenance window and doorbells.
 */
int acp3400_rio_start_port(struct rio_mport *mport)
{
	int rc;
	struct rio_priv *priv = mport->priv;

	/*
	 * Set port lin request ack timout 1.5 - 3 s
	 * Set port response timeout 1.5 - 3 s
	 */
	__rio_local_write_config_32(mport, RIO_PLTOCCSR, 0x7fffff);
	__rio_local_write_config_32(mport, RIO_PRTOCCSR, 0x7fffff);

	/* Check port traning state:
	 */

	rc = rio_start_port(mport);
	if (rc < 0) {
#ifdef CONFIG_RAPIDIO_HOTPLUG
		dev_warn(priv->dev, "Link is down - will continue anyway\n");
#else
		dev_err(priv->dev, "Link is down - SRIO Init failed\n");
		return rc;
#endif
	}

	/* Enable memory mapped access
	 */
	rio_rab_ctrl_setup(mport); /* init */

	rio_rab_pio_enable(mport);

	/* Setup maintenance window
	 * enable doorbells
	 */
	rc = acp3400_rio_static_win_init(mport);

	return rc;
}

/**
 * acp_rio_setup - Setup ACP3400 RapidIO interface
 * @dev: platform_device pointer
 *
 * Initializes ACP3400 RapidIO hardware interface, configures
 * master port with system-specific info, and registers the
 * master port with the RapidIO subsystem.
 *
 * Init sequence is divided into two phases
 * 1:
 *    All one-time initialization: e.g. driver software structures,
 *    work queues, tasklets, sync resources etc. are allocated and
 *    and initialized. At this stage No HW access is possible, to avoid
 *    race conditions, all HW accesses to local configuration space must
 *    be handled through the generic RIO driver access functions and
 *    these may not be used prior to init of master port data structure.
 * 2:
 *    Setup and try to start RapidIO master port controller HW
 *    If the driver is built with hotplug support, the setup routine
 *    does not require that the link is up to complete successfully,
 *    the port may be restarted at any point later in time. Without
 *    hotplug the setup function will fail if link tranining sequence
 *    doesn't complete successfully.
 *
 * Returns:
 * <0           Failure
 * 0            Success
 */
static int acp_rio_setup(struct platform_device *dev)
{
	struct rio_ops *ops;
	struct rio_mport *mport;
	struct rio_priv *priv;
	int rc = -EFAULT;
	struct resource regs;
	u64 law_start, law_size;
	int irq;

	/* Get address boundaries from DTB */
	if (rio_parse_dtb(dev, &law_start, &law_size, &regs, &irq))
		return -EFAULT;

	/* Alloc and Initialize driver SW data structure */
	ops = rio_ops_setup();
	if (IS_ERR(ops)) {
		rc = PTR_ERR(ops);
		goto err_ops;
	}
	mport = rio_mport_dtb_setup(dev, law_start, law_size, ops);
	if (IS_ERR(mport)) {
		rc = PTR_ERR(mport);
		goto err_port;
	}
	priv = rio_priv_dtb_setup(dev, &regs, mport, irq);
	if (IS_ERR(priv)) {
		rc = PTR_ERR(priv);
		goto err_priv;
	}
	/* !!! HW access to local config space starts here !!! */

	/* Get and set master port data:
	 */
	rio_init_port_data(mport);

	/* Start port and enable basic memmap access
	 */
	rc = acp3400_rio_start_port(mport);
	if (rc < 0)
		goto err_maint;

	/* Hookup IRQ handlers
	 */
	if (acp3400_rio_port_irq_enable(mport))
		goto err_irq;

#ifdef CONFIG_ACP_RIO_STAT
	dev_set_drvdata(&dev->dev, mport);
	acp3400_rio_init_sysfs(dev);
#endif
	/* Register port with core driver
	 */
	if (rio_register_mport(mport)) {
		dev_err(&dev->dev, "register mport failed\n");
		goto err_mport;
	}
	if (mport->host_deviceid >= 0)
		mport->enum_host = 1;
	else
		mport->enum_host = 0;

	acp3400_rio_set_mport_disc_mode(mport);

	return 0;

err_mport:
	acp3400_rio_port_irq_disable(mport);
#ifdef CONFIG_ACP_RIO_STAT
	acp3400_rio_release_sysfs(dev);
#endif
err_irq:
	acp3400_rio_static_win_release(mport);
err_maint:
	iounmap(priv->regs_win_fixed);
	iounmap(priv->regs_win_paged);
	kfree(priv);
err_priv:
	kfree(mport);
err_port:
	kfree(ops);
err_ops:
	irq_dispose_mapping(irq);
	return rc;
}

/*
  The probe function for RapidIO peer-to-peer network.
*/
static int __devinit acp_of_rio_rpn_probe(struct platform_device *dev)
{
	printk(KERN_INFO "Setting up RapidIO peer-to-peer network %s\n",
	       dev->dev.of_node->full_name);

	return acp_rio_setup(dev);
};

static const struct of_device_id acp_of_rio_rpn_ids[] = {
	{
		.compatible = "acp,rapidio-delta",
	},
	{},
};

static struct platform_driver acp_of_rio_rpn_driver = {
	.driver = {
		.name = "acp-of-rio",
		.owner = THIS_MODULE,
		.of_match_table = acp_of_rio_rpn_ids,
	},
	.probe = acp_of_rio_rpn_probe,
};

static __init int acp_of_rio_rpn_init(void)
{
	printk(KERN_INFO "Register RapidIO platform driver\n");
	return platform_driver_register(&acp_of_rio_rpn_driver);
}

subsys_initcall_sync(acp_of_rio_rpn_init);
