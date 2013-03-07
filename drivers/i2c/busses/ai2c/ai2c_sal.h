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

/*! @file      ai2c_sal.h
    @brief     OS Specific definitions are located here.
*/

#ifndef __AI2C_SAL_H__
#define __AI2C_SAL_H__

#include "ai2c_sal_types.h"
#include "ai2c_dev.h"
#include "ai2c_sal_linux.h"

/* should not be in sal */
#ifndef MIN
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)	((a) > (b) ? (a) : (b))
#endif

extern void *ai2c_malloc(size_t size);
extern void *ai2c_realloc(void *ptr, size_t size);
extern void *ai2c_calloc(size_t no, size_t size);
extern void  ai2c_free(void *ptr);

#endif /* __AI2C_SAL_H__ */
