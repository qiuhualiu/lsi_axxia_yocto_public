/*
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
#ifndef __ASM__ARCH_AXXIA_I2C_H
#define __ASM__ARCH_AXXIA_I2C_H


/*
 * Minimum number of I2C busses to expect for an AXXIA platform.
 * Runtime checks will determine the actual number of busses that
 * are present depending upon chip type / chip version / etc.
 */
#define ARCH_AXXIA_MAX_I2C_BUSSES       1
#define ARCH_AXXIA_MAX_I2C_BUS_NR       2

/*
 * Maximum byte size of I2C bus name string including null terminator
 */
#define MAX_AXXIA_I2C_HWMOD_NAME_LEN    16


#define AXXIA_I2C_SIZE                  0x0000003f
#define AXXIA1_I2C_BASE                 0x00000000


extern int axxia_register_i2c_busses(void);

#endif /* __ASM__ARCH_AXXIA_I2C_H */
