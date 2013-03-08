/*
 * linux/arch/arm/mach-axxia/i2c.c
 *
 * Helper module for board specific I2C bus registration
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
#include <linux/i2c.h>
#include <linux/i2c-axxia.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/irqs.h>

#include "i2c.h"

/*****************************************************************************
* Local Definitions & State
*****************************************************************************/

static const char name[] = "axxia_ai2c";

#define I2C_RESOURCE_BUILDER(base, irq)			\
	{						\
		.start	= (base),			\
		.end	= (base) + AXXIA_I2C_SIZE,	\
		.flags	= IORESOURCE_MEM,		\
	},						\
	{						\
		.start	= (irq),			\
		.flags	= IORESOURCE_IRQ,		\
	},

        /* Filler values for now; real values will be filled in
         * based on chip type / chip version / etc. */
static struct resource i2c_resources[][2] = {
	{ I2C_RESOURCE_BUILDER(0, 0) },
};

#define I2C_DEV_BUILDER(bus_id, res, data)		\
	{						\
		.id	= (bus_id),			\
		.name	= name,				\
		.num_resources	= ARRAY_SIZE(res),	\
		.resource	= (res),		\
		.dev		= {			\
			.platform_data	= (data),	\
		},					\
	}

static struct axxia_i2c_bus_platform_data i2c_pdata[ARCH_AXXIA_MAX_I2C_BUSSES];
static struct platform_device axxia_i2c_devices[ARCH_AXXIA_MAX_I2C_BUSSES] =
{
    I2C_DEV_BUILDER(ARCH_AXXIA_MAX_I2C_BUS_NR, i2c_resources[0], &i2c_pdata[0]),
};


static inline
int
axxia_add_i2c_bus(
    int         ndx,
    int         bus_id)
{
    struct platform_device              *pdev;
    struct axxia_i2c_bus_platform_data  *pdata;
    struct resource                     *res;

    pdev = &axxia_i2c_devices[ndx];
    res = pdev->resource;
    res[0].start = AXXIA1_I2C_BASE;
    res[0].end = res[0].start + AXXIA_I2C_SIZE;
    res[1].start = 0;           /* I2C interrup handle? */
    pdata = &i2c_pdata[ndx];

    pdata->rev = AXXIA_I2C_IP_VERSION_2;
    pdata->flags = 0;

    return platform_device_register(pdev);
}


/**
 * axxia_register_i2c_busses - register I2C busses with device descriptors
 *
 * Returns 0 on success or an error code.
 */
int __init
axxia_register_i2c_busses(
    void)
{
    int         i;
    int         err;

    for (i=0; i < ARCH_AXXIA_MAX_I2C_BUSSES; i++)
    {
        err = axxia_add_i2c_bus(i, i+ARCH_AXXIA_MAX_I2C_BUS_NR);
        if (err)
            return err;
    }

    return 0;
}
