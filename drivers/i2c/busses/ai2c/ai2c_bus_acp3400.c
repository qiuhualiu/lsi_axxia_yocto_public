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

/* #define EXTRA_DEBUG */

#include "ai2c_plat_pvt.h"
#include "ai2c_dev_clock_ext.h"
#include "regs/ai2c_i2c_regs.h"
#include "regs/ai2c_axi_timer_regs.h"


/*****************************************************************************
* Local Macros & Constants                                                   *
*****************************************************************************/

#define AI2C_I2C_ADDR(offset)            (0x00000000+(offset))


/*****************************************************************************
* Local Type Definitions                                                     *
*****************************************************************************/

struct localTimer {
	u32    chipType;
	u32    i2cRegionId;
	u32    timerRegionId;
	u32       baseOffset;
	u32       timerLoadOffset;
	u32       timerLoadValue;
	u32       timerControlOffset;
	u32       timerControlValue;
};

/*****************************************************************************
* Forward Function Declarations                                              *
*****************************************************************************/

/*****************************************************************************
* Local State                                                                *
*****************************************************************************/

static u32   protoConfig;

/*****************************************************************************
* Functions: Initialization & Base Configuration                             *
*****************************************************************************/

/*
 * Title:       ai2c_bus_init_acp3400
 * Description: This function will initialize the timer(s) and other
 *              features used by I2C.  This is a one time initialization
 *              and will called by the generic initialization sequence.
 * Inputs:
 *   @param[in] priv: handle of device to access
 *   @param[in] regionId: Reference to specific bus within device
 * Returns: completion status
 */
static int ai2c_bus_init_acp3400(
	struct ai2c_priv        *priv,
	u32    regionId)
{
	return -ENOSYS;
}


/*
 * Title: ai2c_bus_block_read8_acp3400
 * Description:
 *   Read num bytes from the offset and store it in buffer.
 *
 * Inputs:
 *   @param[in] dev      Device handle
 *   @param[in] regionId Bus reference handle
 *   @param[in] *adap    Ptr to I2C adapter
 *   @param[in] *msg     Ptr to next I2C message to process
 *   @param[in] stop     Op flag: append 'stop' to this msg
 *
 * Returns: completion status
 */
static int ai2c_bus_block_read8_acp3400(
	struct ai2c_priv        *priv,
	u32    regionId,
	struct i2c_adapter *adap,
	struct i2c_msg     *msg,
	int                 stop)
{
	return -ENOSYS;
}


/*
 * Title:       ai2c_bus_block_write8_acp3400
 * Description: This function will read count bytes from the buffer
 *              and will store at the offset location in the device.
 * Inputs:
 *   @param[in] dev      Device handle
 *   @param[in] regionId Bus reference handle
 *   @param[in] *adap    Ptr to I2C adapter
 *   @param[in] *msg     Ptr to next I2C message to process
 *   @param[in] stop     Op flag: append 'stop' to this msg
 * Returns: completion status
 *   actCount: Actual number of bytes written from the ones that were
 *             provided.
 */
static int ai2c_bus_block_write8_acp3400(
	struct ai2c_priv        *priv,
	u32    regionId,
	struct i2c_adapter *adap,
	struct i2c_msg     *msg,
	int                 stop)
{
	return -ENOSYS;
}


/*****************************************************************************
* More Exported State                                                        *
*****************************************************************************/

struct ai2c_i2c_access ai2c_acp3400_cfg = {
	0,
	/* maxXfrSize */ AI2C_I2CPROT_MAX_XFR_SIZE,
	/* deviceLen */ 0,
	/* i.e. unbounded */

	ai2c_bus_init_acp3400,
	ai2c_bus_block_write8_acp3400,
	ai2c_bus_block_read8_acp3400,

	(void *) &protoConfig,
};
