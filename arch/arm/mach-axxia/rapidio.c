/*
 * linux/arch/arm/mach-axxia/rapidio.c
 *
 * Helper module for board specific RAPIDIO bus registration
 *
 * Copyright (C) 2013 LSI Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <../../../drivers/misc/lsi-ncr.h>

#include "rapidio.h"

/*****************************************************************************
* Local Definitions & State
*****************************************************************************/

#define	MSLEEP_DELAY_2_5_G	500

struct localSpeed_s {
	u32	offset;
	u32	value;
};

static const struct localSpeed_s serdes_2_5_g[] = {
	{ 480, 0x00000100 },
	{ 484, 0xc1400228 },
	{ 480, 0x10000068 },
	{ 484, 0xc1400200 },
	{ 480, 0x00001000 },
	{ 484, 0xc1400204 },
	{ 480, 0x0000000F },
	{ 484, 0xc140022c },
	{ 480, 0x77FF77FF },
	{ 484, 0xc1400208 },
	{ 480, 0x06126527 },
	{ 484, 0xc1400230 },
	{ 480, 0x33333333 },
	{ 484, 0xc1400244 },
	{ 480, 0x10000008 },
	{ 484, 0xc1400200 },
};

static const u32 serdes_2_5_g_count = sizeof(serdes_2_5_g)/
					sizeof(struct localSpeed_s);


/**
 * axxia_rapidio_init - Perform initialization to support use of
 *                      RapidIO busses
 *
 * Returns 0 on success or an error code.
 */
int __init
axxia_rapidio_init(
	void)
{
	int			count = 0;
	int			i;
	struct device_node	*np = NULL;

	/* How many of these devices will be needed? */
	for_each_compatible_node(np, NULL, "axxia,rapidio-delta") {
		u32    pval;
		if (!of_property_read_u32(np, "enabled", &pval))
			count++;
	}
	np = NULL;
	for_each_compatible_node(np, NULL, "acp,rapidio-delta") {
		u32    pval;
		if (!of_property_read_u32(np, "enabled", &pval))
			count++;
	}

	if (count == 0)
		return 0;

	/* Select speed 2.5Gbps, and update SERDES phy */
	for (i=0; i < serdes_2_5_g_count; i++) {
		msleep_interruptible(MSLEEP_DELAY_2_5_G);
		ncr_write(NCP_REGION_ID(0x0153,0),
			serdes_2_5_g[i].offset,
			4,
			&serdes_2_5_g[i].value);
	}

	return 0;
}
