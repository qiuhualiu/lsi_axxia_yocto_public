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

#ifndef __AI2C_DEV_PVT_H__
#define __AI2C_DEV_PVT_H__

#include "ai2c_sal.h"


/* BEGIN: Important forward type references */

struct ai2c_region_io;
struct ai2c_priv;

/* END:   Important forward type references */

#include "regs/ai2c_regions.h"
#include "ai2c_dev.h"

/* --- Linux References --- */
#ifndef AI2C_MOD_NAME
#define AI2C_MOD_NAME        "ai2c"
#endif


/* --- Internal Types & Definitions --- */

#define AI2C_DEV_ACCESS_NONE            (0x00)
#define AI2C_DEV_ACCESS_READ            (0x01)
#define AI2C_DEV_ACCESS_WRITE           (0x02)
#define AI2C_DEV_ACCESS_RW              (0x03)
#define AI2C_DEV_ACCESS_BIG_ENDIAN      (0x04)
#define AI2C_DEV_ACCESS_LITTLE_ENDIAN   (0x08)


#define AI2C_DEV_SIZE_1KB                (1024*1)
#define AI2C_DEV_SIZE_4KB                (1024*4)
#define AI2C_DEV_SIZE_128KB              (1024*128)
#define AI2C_DEV_SIZE_256KB              (1024*256)
#define AI2C_DEV_SIZE_2MB                (1024*1024*2)
#define AI2C_DEV_SIZE_16MB               (1024*1024*16)
#define AI2C_DEV_SIZE_128MB              (1024*1024*128)
#define AI2C_DEV_SIZE_1GB                (1024*1024*1024)
#define AI2C_DEV_SIZE_NO_SIZE            (0)


/* read/write fn prototypes for region map function pointers */

typedef int (*_ai2c_dev_read_fn_t) (
		   struct ai2c_priv          *priv,
		   struct ai2c_region_io     *region,
		   u64	 offset,
		   u32	*buffer,
		   u32	 count,
		   u32	 flags,
		   u32	 cmdType,
		   u32	 xferWidth);

typedef int (*_ai2c_dev_write_fn_t) (
		   struct ai2c_priv          *priv,
		   struct ai2c_region_io     *region,
		   u64	 offset,
		   u32	*buffer,
		   u32	 count,
		   u32	 flags,
		   u32	 cmdType,
		   u32	 xferWidth);

/*
 * Structure definition(s) for the region map.
 * See above for typedef ai2c_region_io_t.
 */
struct ai2c_region_io {
	u32 regionId;
	struct ai2c_access_map     *accessMap;
	_ai2c_dev_read_fn_t    readFn;
	_ai2c_dev_write_fn_t   writeFn;
	u32          pageId;
};

/*
 * Sometimes it would be better to define a range of similar regions
 * with a single entry in the region map, especially, for regions
 * that are logical or virtual entities that involve interpretation,
 * calculated addresses based upon the regionId, or some other
 * transformation.  The alternate region map such definitions.
 */
struct ai2c_region_iorng {
	u32 startRegionId;
	u32 endRegionId;
	struct ai2c_access_map     *accessMap;
	_ai2c_dev_read_fn_t    readFn;
	_ai2c_dev_write_fn_t   writeFn;
	u32 pageId;
};


/*
 * Basic i/o methods
 */

#ifdef DEBUG_EDEV_IO
#define AI2C_WRITE_LOG(ctx, dev, pageId, offset, value) \
	AI2C_MSG(AI2C_MSG_DEBUG_IO, \
	    "%s: pageId=0x%x offset=0x%x addr=0x%x value=0x%02x\n", \
	    ctx, pageId, offset, AI2C_DEV_BUS_ADDR(dev, pageId, offset), value)
#else
#define AI2C_WRITE_LOG(ctx, dev, pageId, offset, value)
#endif

#define AI2C_DEV_BUS_READ8(dev, pageId, offset) \
	AI2C_BUS_READ8(AI2C_DEV_BUS_ADDR(dev, pageId, offset),\
		AI2C_DEV_PAGE_ENDIANNESS(pageId))

#define AI2C_DEV_BUS_READ16(dev, pageId, offset) \
	AI2C_BUS_READ16(AI2C_DEV_BUS_ADDR(dev, pageId, offset),\
		AI2C_DEV_PAGE_ENDIANNESS(pageId))

#define AI2C_DEV_BUS_READ32(dev, pageId, offset) \
	AI2C_BUS_READ32(AI2C_DEV_BUS_ADDR(dev, pageId, offset),\
		AI2C_DEV_PAGE_ENDIANNESS(pageId))

#define AI2C_DEV_BUS_WRITE8(dev, pageId, offset, value) \
	do { \
		AI2C_WRITE_LOG("edev_bus_write8", dev, pageId, offset, value); \
		AI2C_BUS_WRITE8( \
		AI2C_DEV_BUS_ADDR(dev, pageId, offset), value); \
		if (AI2C_DEV_PAGE_FLAGS(pageId) == AI2C_IO_SYNC) { \
			volatile u32 ___val___; \
			___val___ = AI2C_BUS_READ32(AI2C_DEV_BUS_ADDR(dev, \
			AI2C_DEV_PAGE_PCIE0_PEI, AI2C_PEI_CONFIG), \
			AI2C_DEV_ACCESS_LITTLE_ENDIAN); \
		} \
	} while (0);

#define AI2C_DEV_BUS_WRITE16(dev, pageId, offset, value) \
	do { \
		AI2C_WRITE_LOG("edev_bus_write16", \
			dev, pageId, offset, value); \
		AI2C_BUS_WRITE16( \
			AI2C_DEV_BUS_ADDR(dev, pageId, offset), value); \
		if (AI2C_DEV_PAGE_FLAGS(pageId) == AI2C_IO_SYNC) { \
			volatile u32 ___val___; \
			___val___ = AI2C_BUS_READ32(AI2C_DEV_BUS_ADDR(dev, \
			AI2C_DEV_PAGE_PCIE0_PEI, AI2C_PEI_CONFIG), \
			AI2C_DEV_ACCESS_LITTLE_ENDIAN); \
		} \
	} while (0);


#endif /* __AI2C_DEV_PVT_H__ */
