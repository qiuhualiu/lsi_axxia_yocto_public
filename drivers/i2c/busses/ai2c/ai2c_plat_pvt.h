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

/*! @file       ai2c_plat_pvt.h
 *  @brief      Constants, structs, and APIs used to communicate with the
 *              direct ACP I2C Hardware Layer registers
 */

#ifndef AI2C_PLAT_PVT_H
#define AI2C_PLAT_PVT_H

#include "ai2c_plat.h"
#include "ai2c_bus.h"

/*****************************************************************************
* Externally Visible Function Prototypes				     *
*****************************************************************************/

/*! @fn u32 ai2c_page_to_region(struct ai2c_priv *priv,
 *                                           u32 pageId);
 *  @brief Map a memory page handle to a regionId handle.
    @param[in] inPriv Created device state structure
    @param[in] inPageId Original page id to be mapped
    @Returns mapped value
 */
extern u32 ai2c_page_to_region(struct ai2c_priv *priv, u32 pageId);

/*! @fn u32 *ai2c_region_lookup(struct ai2c_priv *priv,
 *                                           u32 regionId);
 *  @brief Map a memory region handle to a region description structure.
    @param[in] inPriv Created device state structure
    @param[in] inRegionId Original region id to be mapped
    @Returns mapped value
 */
extern struct ai2c_region_io *ai2c_region_lookup(
	struct ai2c_priv *priv,
	u32 regionId);

/*! @fn int ai2c_memSetup(struct ai2c_priv **outPriv);
    @brief This is a one time initialization for the I2C protocol
	   layers to be called by the device initialization step.
    @param[out] outPriv Created device state structure
    @Returns success/failure status of the operation
*/
extern int ai2c_memSetup(struct ai2c_priv **outPriv);

/*! @fn int ai2c_memDestroy(struct ai2c_priv  *inPriv);
    @brief This function will release resources acquired for the specified
	   I2C device driver.
    @param[in] inPriv Created device state structure
    @Returns success/failure status of the operation
*/
extern int ai2c_memDestroy(struct ai2c_priv *inPriv);

#endif   /* defined(AI2C_PLAT_PVT_H) */
