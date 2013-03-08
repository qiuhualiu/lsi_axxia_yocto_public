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

/*! @file      AI2C_dev.h
 *  @brief     Exported Device-level APIs.
 *  @addtogroup _device_APIs Device Level APIs
 *  Device-level APIs work with flat memory in each device, specifying
 *  addresses or ranges of addresses.  The object-level APIs call these
 *  device-level APIs, and the host program can also call the device-level
 *  APIs directly.
 *
 *  @{
 */

#ifndef __AI2C_DEV_H__
#define __AI2C_DEV_H__

/*
 * AI2C_dev.h
 *
 * This is the external header file for the AI2C device driver.
 * This provides the function prototypes and associated definitions
 * needed for the user application to access the device APIs.
 *
 */

/*
 * Includes
 */

#include "ai2c_types.h"


/* --- Maximum version string length --- */
#define AI2C_DEV_MAX_VERSION_LENGTH  (41)


/* --- NCA Config Ring Commands --- */
#define AI2C_NCA_CMD_CRBR            (0x00000004)
#define AI2C_NCA_CMD_CRBW            (0x00000005)
#define AI2C_NCA_CMD_CRSW            (0x00000006)
#define AI2C_NCA_CMD_CRBF            (0x00000007)
#define AI2C_NCA_CMD_CRRMW           (0x00000008)
#define AI2C_NCA_CMD_CRBBW           (0x00000009)
#define AI2C_NCA_CMD_CRBSW           (0x0000000A)
#define AI2C_NCA_CMD_CRBBF           (0x0000000B)
#define AI2C_NCA_CMD_SMBR            (0x0000000C)
#define AI2C_NCA_CMD_SMBW            (0x0000000D)
#define AI2C_NCA_CMD_CSMBR           (0x0000000E)
#define AI2C_NCA_CMD_CSMBW           (0x0000000F)

#define AI2C_NCA_NUM_IO_CMDS         12

#define AI2C_CFG_CMD_BUF_SIZE    (sizeof(ai2c_coherent_sysmem_io_t))


/* --- AI2C Trace Level Definitions --- */
#define AI2C_MSG_NONE                (0x00000000)
#define AI2C_MSG_INFO                (0x00000001)
#define AI2C_MSG_ERROR               (0x00000002)
#define AI2C_MSG_ENTRY               (0x00000004)
#define AI2C_MSG_EXIT                (0x00000008)
#define AI2C_MSG_CALL                (AI2C_MSG_ENTRY | AI2C_MSG_EXIT)
#define AI2C_MSG_IOR                 (0x00000010)
#define AI2C_MSG_IOW                 (0x00000020)
#define AI2C_MSG_IORW                (AI2C_MSG_IOR | AI2C_MSG_IOW)
#define AI2C_MSG_MEM                 (0x00000040)
#define AI2C_MSG_MDIO                (0x00000080)
#define AI2C_MSG_DEBUG_IO            (0x20000000)
#define AI2C_MSG_DEBUG               (0x40000000)
#define AI2C_MSG_INTR                (0x80000000)
#define AI2C_MSG_ALL                 (0xFFFFFFFF)


/* --- Device Target Access Map --- */
struct ai2c_access_map {
	u64    begin;
	u64    end;
	s32     word_size_in_bytes;
	s32     access_size_in_words;
};


#define AI2C_DUMMY_REGION_MAP_INIT                              \
	{                                                       \
	{ 0x00000000, 0x03000000, AI2C_DEV_ACCESS_RW   },     \
	{          0,          0, AI2C_DEV_ACCESS_NONE }      \
	}

#endif /* __AI2C_DEV_H__ */
