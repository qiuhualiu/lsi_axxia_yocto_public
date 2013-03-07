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

/*! @file ai2c_mod.c

    @brief Linux driver implementation of I2C using the ACP I2C
	   features upon an LSI development board (San Antonio,
	   Mission, El Paso, ...)

    @details Command line module parameters (with defaults) include,
		int ai2c_is_enabled  = -1;
			//Override: DTB option enabled
			//Values: 0=not, 1=is, -1=check DTB
		int ai2c_trace_level = (AI2C_MSG_INFO | AI2C_MSG_ERROR);
		int ai2c_chip_ver    = -1;
			//Optional: Needed to figure out memory map, etc.
			//Can verify against value from 0xa.0x10.0x2c
			//Values; 0=X1_REL1
			//	1=X1_REL2+
			//	7=X7_REL1+

    @details Several items contained in the 'i2c' section of the '.dts'
	     are used to configure this module including the addresses of
	     the memory partition, IRQ number, number of DMEs to use (when
	     we want to override the inferences based on the chipType), etc.
*/

/*
#define DATA_STREAM_DEBUG
#define AI2C_EXTERNAL_BUILD

#define CONFIG_LSI_UBOOTENV
#define CONFIG_I2C
*/

#include "ai2c_plat_pvt.h"
#include "regs/ai2c_cfg_node_reg_defines.h"
#include "regs/ai2c_cfg_node_regs.h"
#include "asm/lsi/acp_ncr.h"

/******************************************************************************
* --- Linux Module Interface                                              --- *
******************************************************************************/
#define AI2C_PARM_INT	          int
#define AI2C_PARM_UINT	          uint
#define AI2C_MODULE_PARM(v, t)    module_param(v, t, (S_IRUSR|S_IRGRP|S_IROTH|\
						      S_IWUSR|S_IWGRP))
#define AI2C_MODULE_INIT(fn)      module_init(fn)
#define AI2C_MODULE_EXIT(fn)      module_exit(fn)
#define AI2C_MODULE_VERSION(v)    MODULE_VERSION(v)

/*****************************
* --- Module Parameters  --- *
*****************************/
int AI2C_MSG_TRACE_LEVEL;
int ai2c_chip_ver;		/* Opt: Needed to figure out memory map, etc.
				 * Can verify against value from 0xa.0x10.0x2c
				 * Values; 0=X1_REL1
				 *	 1=X1_REL2+
				 *	 9=X7_REL1+
				 */

MODULE_LICENSE("GPL");

MODULE_DESCRIPTION("LSI ACP I2C Platform Driver");
MODULE_AUTHOR("LSI");
AI2C_MODULE_VERSION("1.0.0");

AI2C_MODULE_PARM(AI2C_MSG_TRACE_LEVEL, AI2C_PARM_INT);
AI2C_MODULE_PARM(ai2c_chip_ver, AI2C_PARM_INT);


/***************************************
* --- Character Driver Definitions --- *
***************************************/


/*****************************************************************************
 * Local State
 *****************************************************************************/

static struct ai2c_priv *ai2cState;

struct local_state {
	struct i2c_adapter	  adapter;
	struct i2c_client	   *client;
};

static  struct local_state *ai2cModState;

/*****************************************************************************
 * I2C Algorithm
 *****************************************************************************/

static int ai2c_master_xfer(
	struct i2c_adapter  *adap,
	struct i2c_msg       msgs[],
	int		  num)
{
	u32  regionId = (u32) i2c_get_adapdata(adap);
	struct ai2c_priv	 *priv = ai2cState;
	int		  err = 0;
	int		  i;

	AI2C_LOG(AI2C_MSG_ENTRY, ">>>Enter ai2c_master_xfer\n");

	for (i = 0; (i < num) && (err == 0); i++) {

		int stop = (i == (num - 1)) ? 1 : 0;

		if (msgs[i].flags & I2C_M_RD) {

#ifdef DATA_STREAM_DEBUG
			int j;
			char buf[80];
			strcat(buf, "mstRead:");
#endif /* DATA_STREAM_DEBUG */

			err = priv->pages[i].api->rdFn(priv, regionId,
			adap, &msgs[i], stop);

#ifdef DATA_STREAM_DEBUG
			for (j = 0; j < msgs[i].len; j++) {

				char hb[4];
				sprintf(hb, " %02x", msgs[i].buf[j]);
				strcat(buf, hb);
			}
			printk(KERN_INFO "%s\n", buf);
#endif /* DATA_STREAM_DEBUG */

		} else {

#ifdef DATA_STREAM_DEBUG
			int j;
			char buf[80];
			strcat(buf, "mstWrite:");
			for (j = 0; j < msgs[i].len; j++) {
				char hb[4];
				sprintf(hb, " %02x", msgs[i].buf[j]);
				strcat(buf, hb);
			}
			printk(KERN_INFO "%s\n", buf);
#endif /* DATA_STREAM_DEBUG */

			err = priv->pages[i].api->wrFn(priv, regionId,
				adap, &msgs[i], stop);
		}
	}

AI2C_RETURN_LABEL
	AI2C_LOG(AI2C_MSG_EXIT, ">>>Exit ai2c_master_xfer %d\n", err);
	return err;
}


static u32 ai2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}


static const struct i2c_algorithm ai2c_algorithm = {
	.master_xfer	= ai2c_master_xfer,
	.functionality      = ai2c_func,
};


/*****************************************************************************
 * Device Probe/Setup
 *****************************************************************************/

#ifdef AI2C_PLATFORM_BUILD
static int __devinit ai2c_probe(struct platform_device *pdev)
#else
static int __devinit ai2c_init(void)
#endif
{
	int  ai2cStatus = AI2C_ST_SUCCESS;
	struct ai2c_priv	 *priv = NULL;
	int			  i;

	/* Initialization of externals, initial state */
	AI2C_MSG_TRACE_LEVEL = (AI2C_MSG_INFO | AI2C_MSG_ERROR);
	ai2c_chip_ver = -1;
	ai2cState = NULL;

	AI2C_LOG(AI2C_MSG_ENTRY, ">>>Enter ai2c_probe/init\n");

	AI2C_CALL(ai2c_memSetup(&priv));
	ai2cState = priv;

	/* Hook up bus driver(s) and devices to tree */
	ai2cModState =
		ai2c_malloc(priv->numActiveBusses * sizeof(struct local_state));
	if (!ai2cModState) {
		ai2cStatus = -ENOMEM;
		goto exit_release;
	}
	memset(ai2cModState, 0,
		priv->numActiveBusses * sizeof(struct local_state));

	for (i = 0; i < priv->numActiveBusses; i++) {
		u32 rid;

		if (priv->pages[i].busName == NULL)
			continue;

		rid = ai2c_page_to_region(priv, priv->pages[i].pageId);
		i2c_set_adapdata(&ai2cModState[i].adapter, (void *)rid);

		snprintf(ai2cModState[i].adapter.name,
			sizeof(ai2cModState[i].adapter.name),
			"%s", ai2cState->pages[i].busName);
		ai2cModState[i].adapter.algo = &ai2c_algorithm;
		ai2cModState[i].adapter.owner = THIS_MODULE;
		ai2cModState[i].adapter.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
		ai2cModState[i].adapter.retries = 3;
		/* Retry up to 3 times on lost
		 * arbitration */
#ifdef AI2C_PLATFORM_BUILD
		ai2cModState[i].adapter.dev.parent = &pdev->dev;
#else
		ai2cModState[i].adapter.dev.parent  = NULL;
#endif
		ai2cModState[i].adapter.dev.of_node = NULL;

		/* Add I2C adapter to I2C tree */
		ai2cStatus = i2c_add_adapter(&ai2cModState[i].adapter);
		if (ai2cStatus) {
			printk(KERN_ERR "Failed to add I2C adapter [%d]\n", i);
			goto exit_release;
		}

		/* Any detailed bus-specific initialization */
		ai2cStatus = priv->pages[i].api->initFn(priv, rid);
		if (ai2cStatus)
			goto exit_release;
	}

#ifdef AI2C_PLATFORM_BUILD
	platform_set_drvdata(pdev, priv);
#endif

	AI2C_LOG(AI2C_MSG_EXIT, ">>>Exit ai2c_probe/init %d\n", 0);
	return 0;

AI2C_RETURN_LABEL
	if (ai2cStatus != -ENOMEM)
		ai2cStatus = -ENOSYS;

exit_release:
	ai2c_memDestroy(priv);

	AI2C_LOG(AI2C_MSG_EXIT, ">>>Exit ai2c_probe/init %d\n", ai2cStatus);
	return ai2cStatus;
}

#ifdef AI2C_PLATFORM_BUILD
static int __devexit ai2c_remove(struct platform_device *dev)
#else
static void __devexit ai2c_exit(void)
#endif
{
	int	 i;

	AI2C_LOG(AI2C_MSG_ENTRY, ">>>Enter ai2c_remove/exit\n");

	if (ai2cState != NULL) {

		if (ai2cModState != NULL) {

			for (i = 0; i < ai2cState->numActiveBusses; i++) {

				if (ai2cModState[i].client)
					i2c_unregister_device(
						ai2cModState[i].client);
				i2c_del_adapter(&ai2cModState[i].adapter);
			}
			ai2c_free(ai2cModState);
		}

		ai2c_memDestroy(ai2cState);
	}

#ifdef AI2C_PLATFORM_BUILD
	platform_set_drvdata(dev, NULL);
#endif

	AI2C_LOG(AI2C_MSG_EXIT, ">>>Exit ai2c_remove/exit %d\n", 0);

#ifdef AI2C_PLATFORM_BUILD
	return 0;
#endif
}

/* ------------------------------------------------------------------------- */

#ifdef AI2C_PLATFORM_BUILD

#define ai2c_suspend	NULL
#define ai2c_resume	NULL

static struct platform_driver ai2c_driver = {
	.driver = {
		.name   = "axxia_ai2c",	 /* Must match with platform-specific
					 * code! */
		.owner  = THIS_MODULE,
	},
	.probe      = ai2c_probe,
	.remove     = __devexit_p(ai2c_remove),
	.suspend    = ai2c_suspend,
	.resume     = ai2c_resume,
};

static int __init ai2c_init(void)
{
	AI2C_LOG(AI2C_MSG_ENTRY, ">>>Enter ai2c_init\n");
	return platform_driver_register(&ai2c_driver);
}

static void __exit ai2c_exit(void)
{
	AI2C_LOG(AI2C_MSG_ENTRY, ">>>Enter ai2c_exit\n");
	platform_driver_unregister(&ai2c_driver);
}

#endif  /* AI2C_PLATFORM_BUILD */

module_init(ai2c_init);
module_exit(ai2c_exit);

