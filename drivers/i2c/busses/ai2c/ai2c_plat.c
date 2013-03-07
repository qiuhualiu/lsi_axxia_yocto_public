/*
 *  Copyright (C) 2013 LSI Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*! @file ai2c_plat.c

    @brief Linux driver implementation of I2C using the ACP I2C
	   features upon an LSI development board (San Antonio,
	   Mission, El Paso, ...)

    @details Command line module parameters (with defaults) include,
		int ai2c_trace_level = (AI2C_MSG_INFO | AI2C_MSG_ERROR);
		int ai2c_chip_ver    = -1;
			//Optional: Needed to figure out memory map, etc.
			//Can verify against value from 0xa.0x10.0x2c
			//Values; 0=X1_REL1
			//	1=X1_REL2+
			//	7=X7_REL1+

    @details Several items contained in the 'i2c' section of the '.dts'
	     are used to configure this module including the addresses of
	     the memory partition, IRQ number, number of DMEs to use (when
	     we want to override the inferences based on the chipType), etc.
*/

/*
#define EXTENDED_GSDBG_INFO
#define AI2C_EXTERNAL_BUILD

#define CONFIG_LSI_UBOOTENV
#define CONFIG_I2C
#define AI2C_CHIP_VER=<verNum>
*/

#include "ai2c_plat_pvt.h"
#include "regs/ai2c_cfg_node_reg_defines.h"
#include "regs/ai2c_cfg_node_regs.h"
#include "asm/lsi/acp_ncr.h"

/*****************************************************************************
 * Local State
 *****************************************************************************/
/*
 * This block of code defines the memory addresses for each h/w block
 * that is accessible as a direct bus i/o operation.
 *
 * IMPORTANT: ALL BUS GROUPINGS MUST BE MAINTAINED
 */
/* Note: This table now contains the length of the block for wrap checking
 *       and such (when implemented).  Note that each table is arranged in
 *       order of the "AI2C_DEV_PAGE_xxx" enumeration.
 */
static struct ai2c_dev_page ai2c_dev_page_34xx[AI2C_DEV_PAGE_END_MARKER] = {
	/* Begin: I2C_0 */
	{
		AI2C_DEV_PAGE_I2C_0, "AXXIA_I2C", 0x02000403000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_acp3400_cfg
	},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPIO_0, NULL, 0x02000400000ULL, /*aka APB1/GPIO0*/
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_RESET_CTRL, NULL, 0x02000003800ULL,
		AI2C_DEV_SIZE_1KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_TIMER, NULL, 0x02000408000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPREG, NULL, 0x0200040C000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	/* End: I2C_0 */
};

static struct ai2c_dev_page ai2c_dev_page_25xx[AI2C_DEV_PAGE_END_MARKER] = {
	/* Begin: I2C_0, I2C_1 */
	{
		AI2C_DEV_PAGE_I2C_0, "AXXIA_I2C0", 0x02000427000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_acp3400_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_1, "AXXIA_I2C1", 0x02000428000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_acp3400_cfg},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPIO_0, NULL, 0x02000420000ULL,/*aka APB1/GPIO0*/
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_RESET_CTRL, NULL, 0x02000005c00ULL,
		AI2C_DEV_SIZE_1KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_TIMER, NULL, 0x02000429000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPREG, NULL, 0x02000400000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	/* End: I2C_0, I2C_1 */
};

static struct ai2c_dev_page ai2c_dev_page_55xx[AI2C_DEV_PAGE_END_MARKER] = {
	/* Begin: I2C_0, I2C_1, I2C_2, I2C_3 (SMB) */
	{
		AI2C_DEV_PAGE_I2C_0, "AXXIA_I2C0", 0x02010084000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_1, "AXXIA_I2C1", 0x02010085000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_2, "AXXIA_I2C2", 0x02010086000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_3, "AXXIA_SMB", 0x02010087000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_GPIO_0, NULL, 0x02010092000ULL, /*aka APB1/GPIO1*/
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_RESET_CTRL, NULL, 0x02022060000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_TIMER, NULL, 0x02010091000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPREG, NULL, 0x02010094000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	/* End: I2C_1 */
};

static struct ai2c_dev_page ai2c_dev_page_35xx[AI2C_DEV_PAGE_END_MARKER] = {
	/* Begin: I2C_0 */
	{
		AI2C_DEV_PAGE_I2C_0, "AXXIA_I2C0", 0x02000426000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_1, "AXXIA_I2C1", 0x02000427000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_I2C_2, "AXXIA_I2C2", 0x02000428000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_I2CBUS, &ai2c_axm5500_cfg
	},
	{
		AI2C_DEV_PAGE_END_MARKER, NULL, 0x00000000000ULL, 0, 0,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPIO_0, NULL, 0x02000420000ULL, /*aka APB1/GPIO0*/
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_RESET_CTRL, NULL, 0x02000003800ULL,
		AI2C_DEV_SIZE_1KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_TIMER, NULL, 0x02000429000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	{
		AI2C_DEV_PAGE_GPREG, NULL, 0x02000400000ULL,
		AI2C_DEV_SIZE_4KB, AI2C_DEV_ACCESS_LITTLE_ENDIAN,
		AI2C_PAGE_FLAGS_NONE, NULL
	},
	/* End: I2C_0 */
};

static struct ai2c_dev_page_entry ai2c_dev_page_index[] = {
	{ AI2C_CHIP_ACP34xx, "ACP34xx", FALSE, ai2c_dev_page_34xx, 2, },
	{ AI2C_CHIP_ACP32xx, "ACP32xx", FALSE, ai2c_dev_page_34xx, 2, },
	{ AI2C_CHIP_ACP25xx, "ACP25xx", FALSE, ai2c_dev_page_25xx, 2, },
	{ AI2C_CHIP_ACP55xx, "AXM55xx", TRUE, ai2c_dev_page_55xx, 4, },
	{ AI2C_CHIP_ACP35xx, "AXM35xx", TRUE, ai2c_dev_page_35xx, 3, },
};

static int ai2c_dev_page_index_cnt = sizeof(ai2c_dev_page_index) /
				     sizeof(struct ai2c_dev_page_entry);

	/* Region Map
	 *   Note: Must be same number of entries (and in same order) in the
	 *	 arrays of type 'ai2c_dev_page_entry_t' above.  Note that
	 *	 each table is arranged in order of the "AI2C_DEV_PAGE_xxx"
	 *	 enumeration.
	 */

static struct ai2c_access_map ai2cDummyRegionMap[] = AI2C_DUMMY_REGION_MAP_INIT;

static struct ai2c_region_io ai2c_region_io_map[] = {
	/* 323.0 */
	{
		AI2C_REGION_I2C_0, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_I2C_0,
	},
	/* 332.0 */
	{
		AI2C_REGION_I2C_1, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_I2C_1,
	},
	/* 332.0 */
	{
		AI2C_REGION_I2C_2, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_I2C_2,
	},
	/* 348.0 */
	{
		AI2C_REGION_I2C_3, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_I2C_3,
	},
	/* 320.0 */
	{
		AI2C_REGION_GPIO_0, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_GPIO_0,
	},
	/* 398.0 */
	{
		AI2C_REGION_RESET_CTRL, ai2cDummyRegionMap,
		__ai2c_dev_dcr_read, __ai2c_dev_dcr_write,
		AI2C_DEV_PAGE_RESET_CTRL,
	},
	/* 326.0 */
	{
		AI2C_REGION_TIMER, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_TIMER,
	},
	/* 329.0 */
	{
		AI2C_REGION_GPREG, ai2cDummyRegionMap,
		__ai2c_dev_direct_read, __ai2c_dev_direct_write,
		AI2C_DEV_PAGE_GPREG,
	},
};

static const u32 ai2c_region_pages_max =
	sizeof(ai2c_region_io_map) / sizeof(struct ai2c_region_io);


/*****************************************************************************
 * Miscellaneous Utility functions
 *****************************************************************************/

u32 ai2c_page_to_region(
	struct ai2c_priv	  *priv,
	u32	 pageId)
{
	int i;
	for (i = 0; i < ai2c_region_pages_max; i++)
		if (pageId == ai2c_region_io_map[i].pageId)
			return ai2c_region_io_map[i].regionId;
	return AI2C_REGION_NULL;
}

struct ai2c_region_io *ai2c_region_lookup(
	struct ai2c_priv	  *priv,
	u32      regionId)
{
	int i;
	for (i = 0; i < ai2c_region_pages_max; i++)
		if (regionId == ai2c_region_io_map[i].regionId)
			return &ai2c_region_io_map[i];
	return NULL;
}

/*****************************************************************************
 * Read/Write ACP Memory Spaces
 *****************************************************************************/

/*
 * ai2c_dev_direct_read
 *
 *   Perform 32-bit AI2C device I/O to non-ConfigRing region.
 */
int ai2c_dev_direct_read(
	struct ai2c_priv        *priv,
	struct ai2c_region_io   *region,
	u64	offset,
	u32      *buffer,
	u32	count,
	u32	flags,
	u32	cmdType,
	u32	xferWidth)
{
	int           st = 0;
	u32       endianness;
	unsigned long       busAddr;
	u32       i;

	AI2C_MSG(AI2C_MSG_ENTRY,
		"direct_read enter: %x.%x.%llx %d\n",
		AI2C_NODE_ID(region->regionId),
		AI2C_TARGET_ID(region->regionId),
		(unsigned long long) offset, count);

	if (priv->pageAddr[region->pageId] == 0) {
		st = -EBADSLT;
		goto cleanup;
	}

	busAddr = AI2C_DEV_BUS_ADDR(priv, region->pageId, offset);
	endianness = AI2C_DEV_PAGE_ENDIANNESS(region->pageId);

	switch (xferWidth) {
	case 4:
		for (i = 0; i < count; i++, busAddr += 4, offset += 4) {
			buffer[i] = AI2C_BUS_READ32(busAddr, endianness);
			AI2C_MSG(AI2C_MSG_IOR,
				"direct_read: region=%x offset = %llx busAddr=%lx v=%x\n",
				region->regionId, offset, busAddr, buffer[i]);
		}
		break;
	case 2:
		{
			u16 *p16 = (u16 *) buffer;
			for (i = 0; i < count; i++, busAddr += 2)
				p16[i] = AI2C_BUS_READ16(busAddr, endianness);
		}
		break;
	case 1:
		{
			u8 *p8 = (u8 *) buffer;
			for (i = 0; i < count; i++, busAddr += 1)
				p8[i] = AI2C_BUS_READ8(busAddr);
		}
		break;
	default:
		st = -EACCES;
		break;
	}

cleanup:
	AI2C_MSG(AI2C_MSG_EXIT,
		"direct_read exit: st=%d %x.%x.%llx=0x%08x\n",
		st, AI2C_NODE_ID(region->regionId),
		AI2C_TARGET_ID(region->regionId), (unsigned long long) offset,
		buffer[0]);
	return (int) st;
}

/*
 * ai2c_dev_direct_write
 *
 *   Perform 32-bit AI2C device I/O to non-ConfigRing region.
 */
int ai2c_dev_direct_write(
	struct ai2c_priv        *priv,
	struct ai2c_region_io   *region,
	u64	offset,
	u32      *buffer,
	u32	count,
	u32	flags,
	u32	cmdType,
	u32	xferWidth)
{
	int           st = 0;
	u32       endianness;
	unsigned long       busAddr;
	u32       i;

	AI2C_MSG(AI2C_MSG_ENTRY,
		"direct_write enter: %x.%x.%llx 0x%08x (%d)\n",
		AI2C_NODE_ID(region->regionId),
		AI2C_TARGET_ID(region->regionId),
		(unsigned long long) offset,
		buffer[0], count);

	if (priv->pageAddr[region->pageId] == 0) {
		st = -EBADSLT;
		goto cleanup;
	}

	busAddr = AI2C_DEV_BUS_ADDR(priv, region->pageId, offset);
	endianness = AI2C_DEV_PAGE_ENDIANNESS(region->pageId);

	switch (xferWidth) {
	case 4:
		for (i = 0; i < count; i++, busAddr += 4, offset += 4) {
			AI2C_BUS_WRITE32(busAddr, buffer[i], endianness);
			AI2C_MSG(AI2C_MSG_IOW,
				"direct_write: region=%x offset=%llx busAddr=%lx v=%x\n",
				region->regionId, offset, busAddr, buffer[i]);
		}
		break;

	case 2:
		{
			u16 *buf16 = (u16 *) buffer;
			for (i = 0; i < count; i++, busAddr += 2) {
				AI2C_BUS_WRITE16(busAddr, buf16[i], endianness);
				AI2C_MSG(AI2C_MSG_IOW,
					"direct_write: region=%x offset=%llx busAddr=%lx v=%x\n",
					region->regionId,
					offset, busAddr, buf16[i]);
			}
		}
		break;
	case 1:
		{
			u8 *buf8 = (u8 *) buffer;
			for (i = 0; i < count; i++, busAddr++) {
				AI2C_BUS_WRITE8(busAddr, buf8[i]);
				AI2C_MSG(AI2C_MSG_IOW,
					"direct_write: region=%x offset=%llx busAddr=%lx v=%x\n",
					region->regionId,
					offset, busAddr, buf8[i]);
			}
		}
		break;
	default:
		st = -EACCES;
		break;
	}

cleanup:
	AI2C_MSG(AI2C_MSG_EXIT, "direct_write exit st=%d\n", st);
	return (int) st;
}

/*
 * ai2c_dev_read32
 *
 */
int ai2c_dev_read32(
	struct ai2c_priv         *priv,
	u32     regionId,
	u64        offset,
	u32       *buffer)
{
	int	ai2cStatus = 0;
	struct ai2c_region_io *region = ai2c_region_lookup(priv, regionId);
	unsigned long lflags = 0;

	AI2C_SPINLOCK_INTERRUPT_DISABLE(&priv->regLock, lflags);

	AI2C_MSG(AI2C_MSG_ENTRY,
		"dev_read32 enter: %x.%x.%llx %d\n",
		AI2C_NODE_ID(regionId), AI2C_TARGET_ID(regionId),
		(unsigned long long) offset, 1);

	if (region) {
		ai2cStatus =
			AI2C_EDEV_BUS_BLOCK_READ32(priv,
				region->pageId, offset, 1, buffer);

	} else {

#ifdef CONFIG_LSI_UBOOTENV
		ai2cStatus = ncr_read(regionId, (u32) offset,
			1 * sizeof(u32), buffer);
#else
		ai2cStatus = -ENOSYS;
#endif
	}

	AI2C_SPINLOCK_INTERRUPT_ENABLE(&priv->regLock, lflags);

	return ai2cStatus;
}

/*
 * ai2c_dev_write32
 *
 */
int ai2c_dev_write32(
	struct ai2c_priv         *priv,
	u32     regionId,
	u64        offset,
	u32        buffer)
{
	int ai2cStatus = 0;
	struct ai2c_region_io    *region = ai2c_region_lookup(priv, regionId);
	unsigned long lflags = 0;

	AI2C_SPINLOCK_INTERRUPT_DISABLE(&priv->regLock, lflags);

	AI2C_MSG(AI2C_MSG_ENTRY,
		"dev_write32 enter: %x.%x.%llx 0x%08x (%d)\n",
		AI2C_NODE_ID(regionId), AI2C_TARGET_ID(regionId),
		(unsigned long long) offset, (unsigned int)&buffer, 1);

	if (region) {
		ai2cStatus =
			AI2C_EDEV_BUS_BLOCK_WRITE32(priv,
				region->pageId, offset, 1,
			&buffer);

	} else {

#ifdef CONFIG_LSI_UBOOTENV
	ai2cStatus = ncr_write(regionId, (u32) offset,
		1 * sizeof(u32), &buffer);
#else
	ai2cStatus = -ENOSYS;
#endif
	}

	AI2C_SPINLOCK_INTERRUPT_ENABLE(&priv->regLock, lflags);

	return ai2cStatus;
}

/*
 * ai2c_dev_dcr_read
 *
 *   Perform 32-bit AI2C device I/O to non-Config Ring region.
 */
int ai2c_dev_dcr_read(
	struct ai2c_priv      *priv,
	struct ai2c_region_io *region,
	u64	  offset,
	u32    *buffer,
	u32	  count,
	u32	  flags,
	u32	  cmdType,
	u32	  xferWidth)
{
	return -ENOSYS;
}

/*
 * ai2c_dev_dcr_write
 *
 *   Perform 32-bit AI2C device I/O from non-Config Ring region.
 */
int ai2c_dev_dcr_write(
	struct ai2c_priv         *priv,
	struct ai2c_region_io    *region,
	u64	offset,
	u32       *buffer,
	u32	count,
	u32	flags,
	u32	cmdType,
	u32	xferWidth)
{
	return -ENOSYS;
}


/*****************************************************************************
 * Basic configuration Fill-in
 *****************************************************************************/

static int ai2c_getChipType(struct ai2c_priv *priv)
{
	int                  ai2cStatus = AI2C_ST_SUCCESS;
#ifdef CONFIG_LSI_UBOOTENV
	ai2c_bool_t                has_ECID = TRUE;
	u32	       rev_reg;
	u32	       pt_reg;
	ai2c_cfg_node_node_cfg_r_t  node_cfg;
	ai2c_cfg_node_node_info_0_r_t node_info;

	/*
	 * Determine device revision
	 */

	/* Read the NCA local config node to see if we are an ASIC or FPGA */
	AI2C_CALL(ai2c_dev_read32(priv, AI2C_REGION_NCA_CFG,
		AI2C_CFG_NODE_NODE_CFG,
		(u32 *) &node_cfg));
	AI2C_CALL(ai2c_dev_read32(priv, AI2C_REGION_NCA_CFG,
		AI2C_CFG_NODE_NODE_INFO_0,
		(u32 *) &node_info));

	if (node_cfg.fpga) {
		priv->hw_rev.isFpga = 1;
		/* v1 FPGA doesn't have the ECID block */
		if (node_info.module_revision == 0)
			has_ECID = FALSE;

	} else
		priv->hw_rev.isAsic = 1;

	if (node_info.module_revision == AI2C_CHIP_ACP25xx ||
	    node_info.module_revision == AI2C_CHIP_ACP55xx)
		has_ECID = FALSE;

	/* Get the device chipType/Version from the ECID fuse block */
	if (has_ECID) {

		AI2C_CALL(ai2c_dev_read32(priv,
			AI2C_REGION_ID(AI2C_NODE_X1_ECID, 0x10),
			0x2c, (u32 *) &rev_reg));

		AI2C_CALL(ai2c_dev_read32(priv,
			AI2C_REGION_ID(AI2C_NODE_X1_ECID, 0x10),
			0x20, &pt_reg));

		priv->hw_rev.chipType = (rev_reg & 0x0000001f);
		priv->hw_rev.chipVersion = (rev_reg & 0x000007e0) >> 5;
		priv->hw_rev.cpuDisable = (rev_reg & 0x00007800) >> 11;
		priv->hw_rev.sppDisable = (rev_reg & 0x00008000) >> 15;

		priv->hw_rev.packageType = (pt_reg & 0xf0000000) >> 28;
	} else {
		/* if we don't have an ECID just use the NCA module version */
		priv->hw_rev.chipType = node_info.module_revision;
		priv->hw_rev.chipVersion = 0;
		priv->hw_rev.packageType = 0;
		priv->hw_rev.cpuDisable = 0;
		priv->hw_rev.sppDisable = 0;
	}

	/* fixup chipType for ACP344x variants */
	switch (priv->hw_rev.chipType) {
	case 3:
	case 4:
		priv->hw_rev.chipType = AI2C_CHIP_ACP34xx;
		break;
	case 5:
		priv->hw_rev.chipType = AI2C_CHIP_ACP34xx;
		break;
	default:
		break;
	}
#endif

	/* Environment variable override */
	if (ai2c_chip_ver != -1) {
		priv->hw_rev.chipType    = ai2c_chip_ver;
		priv->hw_rev.chipVersion = 0;
	}
#ifdef AI2C_CHIP_VER
	else {
		priv->hw_rev.chipType    = AI2C_CHIP_VER;
		priv->hw_rev.chipVersion = 0;
	}
#endif

	AI2C_LOG(AI2C_MSG_INFO, "AI2C %d.%d.%d %s\n",
		priv->hw_rev.chipType, priv->hw_rev.chipVersion,
		priv->hw_rev.packageType,
		(priv->hw_rev.isFpga) ? "FPGA" : "ASIC");

AI2C_RETURN_LABEL
	return ai2cStatus;
}

int ai2c_memSetup(struct ai2c_priv **outPriv)
{
	int           ai2cStatus = AI2C_ST_SUCCESS;
	struct ai2c_priv        *priv = NULL;
	int                 i;

	/* Now for the private memory for this module. */
	priv = ai2c_malloc(sizeof(struct ai2c_priv));
	if (!priv) {
		AI2C_LOG(AI2C_MSG_ERROR,
			"Could not allocate AI2C private memory root!\n");
		ai2cStatus = -ENOMEM;
		goto ai2c_return;
	}
	memset(priv, 0, sizeof(struct ai2c_priv));

	/* Check chipType/chipVersion fields of 0xa.0x10.0x2c, first */
	ai2cStatus = ai2c_getChipType(priv);
	if (ai2cStatus != AI2C_ST_SUCCESS)
		goto ai2c_return;

	/* What is the page mapping scheme for this platform? */
	for (i = 0; i < ai2c_dev_page_index_cnt; i++) {

		if (ai2c_dev_page_index[i].chipType == priv->hw_rev.chipType) {
			priv->pages = ai2c_dev_page_index[i].pages;
			priv->busCfg = &ai2c_dev_page_index[i];
			priv->numActiveBusses =
				ai2c_dev_page_index[i].numActiveBusses;
		}
	}
	if (priv->pages == NULL) {
		AI2C_LOG(AI2C_MSG_ERROR,
			"Can't select memory cfg for chip ver %d\n",
			priv->hw_rev.chipType);
		ai2cStatus = -ENXIO;
		goto ai2c_return;
	}

	/*
	* Program Address Map driver tables
	*   Set up the base offsets for SRIO RAB and GRIO access via paged
	*   PLB access.
	*/
	priv->pageAddr =
		ai2c_malloc(AI2C_DEV_PAGE_END_MARKER * sizeof(u32));
	if (priv->pageAddr == NULL) {
		AI2C_LOG(AI2C_MSG_ERROR,
			"Could not allocate AI2C pageAddr memory!\n");
		ai2cStatus = -ENOMEM;
		goto ai2c_return;
	}
	memset(priv->pageAddr, 0,
		AI2C_DEV_PAGE_END_MARKER * sizeof(u32));
	for (i = 0; i < AI2C_DEV_PAGE_END_MARKER; i++) {

		if (priv->pages[i].pageId != AI2C_DEV_PAGE_END_MARKER) {
			priv->pageAddr[i] =
				(u32) ioremap(priv->pages[i].busAddr,
			priv->pages[i].size);
			if (priv->pageAddr[i] == 0) {
				AI2C_LOG(AI2C_MSG_ERROR,
					"Could not ioremap AI2C pageAddr memory %d!\n",
					i);
				ai2cStatus = -ENOMEM;
				goto ai2c_return;

			} else {

				AI2C_LOG(AI2C_MSG_DEBUG,
					"Map page %d (%08x) / %llx for %x => %x\n",
					priv->pages[i].pageId,
					ai2c_page_to_region(priv,
					priv->pages[i].pageId),
					(unsigned long long)
						priv->pages[i].busAddr,
					priv->pages[i].size,
					priv->pageAddr[i]);
			}
		}
	}

	AI2C_SPINLOCK_INIT(&priv->regLock);
	AI2C_SPINLOCK_INIT(&priv->ioLock);

AI2C_RETURN_LABEL

	if (ai2cStatus != AI2C_ST_SUCCESS) {
		if (priv) {
			if (priv->pageAddr) {
				for (i = 0; i < AI2C_DEV_PAGE_END_MARKER; i++)
					if (priv->pageAddr[i] != 0)
						iounmap(
						    (void __iomem *)priv->pageAddr[i]);
				ai2c_free(priv->pageAddr);
			}
			ai2c_free(priv);
		}
		(*outPriv) = NULL;
	} else
		(*outPriv) = priv;

	return ai2cStatus;
}

int ai2c_memDestroy(struct ai2c_priv *inPriv)
{
	int	    ai2cStatus = AI2C_ST_SUCCESS;
	int             i;

	if (inPriv) {
		if (inPriv->pageAddr) {
			for (i = 0; i < AI2C_DEV_PAGE_END_MARKER; i++)
				if (inPriv->pageAddr[i] != 0)
					iounmap((void __iomem *)inPriv->pageAddr[i]);

			ai2c_free(inPriv->pageAddr);
		}

		ai2c_free(inPriv);
	}

	return ai2cStatus;
}
