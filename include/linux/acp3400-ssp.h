/*
 * ACP3400 SSP driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef ACP3400_SSP_H_
#define ACP3400_SSP_H_

/* SSP flags */
#define ACP3400_SSP_CLK_6_25MHZ	0x00000000
#define ACP3400_SSP_CLK_12_5MHZ	0x00000001
#define ACP3400_SSP_CLK_25MHZ	0x00000002
#define ACP3400_SSP_CLK_50MHZ	0x00000003

#define ACP3400_SSP_CS(_cs)	((_cs) << 24)
#define ACP3400_SSP_CS_UND	0x00000004

#define ACP3400_SSP_LSB_FIRST	0x00000008

int acp3400_ssp_write(void *data, unsigned int len, unsigned int flags);

#endif /* ACP3400_SSP_H_ */
