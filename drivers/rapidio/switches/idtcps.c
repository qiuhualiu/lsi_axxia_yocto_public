/*
 * IDT CPS RapidIO switches support
 *
 * Copyright 2009-2010 Integrated Device Technology, Inc.
 * Alexandre Bounine <alexandre.bounine@idt.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>
#include "../rio.h"

#define CPS_DEFAULT_ROUTE	0xde
#define CPS_NO_ROUTE		0xdf

#define IDTCPS_RIO_DOMAIN 0xf20020

static int
idtcps_route_add_entry(struct rio_mport *mport, u16 destid, u8 hopcount,
		       u16 table, u16 route_destid, u8 route_port)
{
	u32 result;
	int rc = 0;

	if (route_port == RIO_INVALID_ROUTE)
		route_port = CPS_DEFAULT_ROUTE;

	if (table == RIO_GLOBAL_TABLE) {
		rc = rio_mport_write_config_32(mport, destid, hopcount,
					RIO_STD_RTE_CONF_DESTID_SEL_CSR,
					route_destid);
		if (rc < 0)
			goto done;

		rc = rio_mport_read_config_32(mport, destid, hopcount,
						RIO_STD_RTE_CONF_PORT_SEL_CSR,
						&result);
		if (rc < 0)
			goto done;

		result = (0xffffff00 & result) | (u32)route_port;
		rc = rio_mport_write_config_32(mport, destid, hopcount,
					       RIO_STD_RTE_CONF_PORT_SEL_CSR,
					       result);
	}
done:
	return rc;
}

static int
idtcps_route_get_entry(struct rio_mport *mport, u16 destid, u8 hopcount,
		       u16 table, u16 route_destid, u8 *route_port)
{
	u32 result;

	if (table == RIO_GLOBAL_TABLE) {
		rio_mport_write_config_32(mport, destid, hopcount,
				RIO_STD_RTE_CONF_DESTID_SEL_CSR, route_destid);

		rio_mport_read_config_32(mport, destid, hopcount,
				RIO_STD_RTE_CONF_PORT_SEL_CSR, &result);

		if (CPS_DEFAULT_ROUTE == (u8)result ||
		    CPS_NO_ROUTE == (u8)result)
			*route_port = RIO_INVALID_ROUTE;
		else
			*route_port = (u8)result;
	}

	return 0;
}

static int
idtcps_route_clr_table(struct rio_mport *mport, u16 destid, u8 hopcount,
		       u16 table)
{
	u32 i;
	int rc = 0;
	for (i = 0; i <= RIO_ANY_DESTID(mport->sys_size); i++) {
		rc = idtcps_route_add_entry(mport, destid, hopcount,
					    table, i, RIO_INVALID_ROUTE);
		if (rc)
			break;
	}
	return rc;
}

static int
idtcps_set_domain(struct rio_mport *mport, u16 destid, u8 hopcount,
		       u8 sw_domain)
{
	/*
	 * Switch domain configuration operates only at global level
	 */
	rio_mport_write_config_32(mport, destid, hopcount,
				  IDTCPS_RIO_DOMAIN, (u32)sw_domain);
	return 0;
}

static int
idtcps_get_domain(struct rio_mport *mport, u16 destid, u8 hopcount,
		       u8 *sw_domain)
{
	u32 regval;

	/*
	 * Switch domain configuration operates only at global level
	 */
	rio_mport_read_config_32(mport, destid, hopcount,
				IDTCPS_RIO_DOMAIN, &regval);

	*sw_domain = (u8)(regval & 0xff);

	return 0;
}

static int idtcps_switch_init(struct rio_dev *rdev, int do_enum)
{
	pr_debug("RIO: %s for %s\n", __func__, rio_name(rdev));
	rdev->rswitch->add_entry = idtcps_route_add_entry;
	rdev->rswitch->get_entry = idtcps_route_get_entry;
	rdev->rswitch->clr_table = idtcps_route_clr_table;
	rdev->rswitch->set_domain = idtcps_set_domain;
	rdev->rswitch->get_domain = idtcps_get_domain;
	rdev->rswitch->em_init = NULL;
	rdev->rswitch->em_handle = NULL;

	if (do_enum) {
		/* set TVAL = ~50us */
		rio_write_config_32(rdev,
			rdev->phys_efptr + RIO_PORT_LINKTO_CTL_CSR, 0x8e << 8);
		/* Ensure that default routing is disabled on startup */
		rio_write_config_32(rdev,
				    RIO_STD_RTE_DEFAULT_PORT, CPS_NO_ROUTE);
	}

	return 0;
}

DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDTCPS6Q, idtcps_switch_init);
DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDTCPS8, idtcps_switch_init);
DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDTCPS10Q, idtcps_switch_init);
DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDTCPS12, idtcps_switch_init);
DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDTCPS16, idtcps_switch_init);
DECLARE_RIO_SWITCH_INIT(RIO_VID_IDT, RIO_DID_IDT70K200, idtcps_switch_init);
