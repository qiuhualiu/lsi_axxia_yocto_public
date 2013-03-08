#ifndef _IDT_GEN2_H
#define _IDT_GEN2_H

/*
 * IDT Generation 2 support
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define IDT_PORT_N_ACK_STS_CSR(n)	(0x148 + (n)*0x20)
#define IDT_PORT_N_CTL(n)		(0x15c + (n)*0x20)

#define LOCAL_RTE_CONF_DESTID_SEL	0x010070
#define LOCAL_RTE_CONF_DESTID_SEL_PSEL	0x0000001f

#define IDT_PORT_N_ERR_RATE_EN(n)	(0x1044 + (n)*0x40)
#define IDT_PORT_N_ERR_RATE(n)		(0x1068 + (n)*0x40)
#define IDT_PORT_N_ERR_RATE_THRESHOLD(n)(0x106C + (n)*0x40)

#define IDT_LT_ERR_REPORT_EN	0x03100c

#define IDT_LANE_N_STATUS(n)		(0x002010 + (n)*0x20)

#define IDT_PORT_ERR_REPORT_EN(n)	(0x031044 + (n)*0x40)
#define IDT_PORT_ERR_REPORT_EN_BC	0x03ff04

#define IDT_PORT_ISERR_REPORT_EN(n)	(0x03104C + (n)*0x40)
#define IDT_PORT_ISERR_REPORT_EN_BC	0x03ff0c
#define IDT_PORT_INIT_TX_ACQUIRED	0x00000020

#define IDT_LANE_ERR_REPORT_EN(n)	(0x038010 + (n)*0x100)
#define IDT_LANE_ERR_REPORT_EN_BC	0x03ff10

#define IDT_PLL_CTRL_1(n)	(0xff0000 + (n)*0x10)
#define IDT_PLL_CTRL_1_DIV_SEL_BIT	(0x1)

#define IDT_DEV_CTRL_1		0xf2000c
#define IDT_DEV_CTRL_1_GENPW		0x02000000
#define IDT_DEV_CTRL_1_PRSTBEH		0x00000001

#define IDT_CFGBLK_ERR_CAPTURE_EN	0x020008
#define IDT_CFGBLK_ERR_REPORT		0xf20014
#define IDT_CFGBLK_ERR_REPORT_GENPW		0x00000002

#define IDT_AUX_PORT_ERR_CAP_EN	0x020000
#define IDT_AUX_ERR_REPORT_EN	0xf20018
#define IDT_AUX_PORT_ERR_LOG_I2C	0x00000002
#define IDT_AUX_PORT_ERR_LOG_JTAG	0x00000001

#define	IDT_ISLTL_ADDRESS_CAP	0x021014

#define IDT_RIO_DOMAIN		0xf20020
#define IDT_RIO_DOMAIN_MASK		0x000000ff

#define IDT_PW_INFO_CSR		0xf20024

#define IDT_SOFT_RESET		0xf20040
#define IDT_SOFT_RESET_REQ		0x00030097

#define IDT_I2C_MCTRL		0xf20050
#define IDT_I2C_MCTRL_GENPW		0x04000000

#define IDT_JTAG_CTRL		0xf2005c
#define IDT_JTAG_CTRL_GENPW		0x00000002

#define IDT_QUAD_CONFIG		0xf20200

#define IDT_DEV_RESET_CTRL	0xf20300

#define IDT_LANE_CTRL(n)	(0xff8000 + (n)*0x100)
#define IDT_LANE_CTRL_BC	0xffff00
#define IDT_LANE_CTRL_GENPW		0x00200000
#define IDT_LANE_DFE_1_BC	0xffff18
#define IDT_LANE_DFE_2_BC	0xffff1c

#define IDT_PORT_OPS(n)		(0xf40004 + (n)*0x100)
#define IDT_PORT_OPS_GENPW		0x08000000
#define IDT_PORT_OPS_PL_ELOG		0x00000040
#define IDT_PORT_OPS_LL_ELOG		0x00000020
#define IDT_PORT_OPS_LT_ELOG		0x00000010
#define IDT_PORT_OPS_BC		0xf4ff04

#define IDT_PORT_ISERR_DET(n)	(0xf40008 + (n)*0x100)

#define IDT_ERR_CAP		0xfd0000
#define IDT_ERR_CAP_LOG_OVERWR		0x00000004

#define IDT_ERR_RD		0xfd0004

#define IDT_DEFAULT_ROUTE	0xde
#define IDT_NO_ROUTE		0xdf

#define IDT_NUM_QUADS		(4)  /* Valid for cps 1848 */
#define IDT_MAX_NUM_LANES	(48) /* Valid for cps 1848 */
#define IDT_MAX_PORT_WIDTH	(0x4)

extern int idtg2_set_enum_boundry(struct rio_mport *mport, u16 destid,
				  u8 hopcount, u8 port, int enable);
extern int idt2_set_port_speed(struct rio_mport *mport, u16 destid,
			       u8 hopcount, u8 port, int speed /* Mb/s */);

#endif
