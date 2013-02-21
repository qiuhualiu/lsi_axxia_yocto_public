/*
 * ACP3400 SSP driver
 *
 * Based on DU-TS SSP driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>


#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/acp3400-ssp.h>

#define DRV_NAME	"acp3400-ssp"

/*
 * SSP register values
 */
/* CR0 */
#define SSP_CR0_DSS_8BIT	0x7
#define SSP_CR0_FRF_SPI		(0 << 4)
/* (200MHz / (data_rate * SSP_CPSR)) - 1 */
#define SSP_CR0_SCR_50MHZ	(1 << 8)
#define SSP_CR0_SCR_25MHZ	(3 << 8)
#define SSP_CR0_SCR_12_5MHZ	(7 << 8)
#define SSP_CR0_SCR_6_25MHZ	(15 << 8)

/* CR1 */
#define SSP_CR1_LBM		(1 << 0)
#define SSP_CR1_SSE_EN		(1 << 1)
#define SSP_CR1_MS_MASTER	(0 << 2)
#define SSP_CR1_MS_SLAVE	(1 << 2)
/* SR */
#define SSP_SR_TFE		(1 << 0)
#define SSP_SR_TNF		(1 << 1)
#define SSP_SR_RNE		(1 << 2)
#define SSP_SR_RFF		(1 << 3)
#define SSP_SR_BSY		(1 << 4)
/* CPSR */
#define SSP_CPSR		2

struct acp3400_ssp_regs {
	unsigned int cr0;	/* 0x00 */
	unsigned int cr1;	/* 0x04*/
	unsigned int dr;	/* 0x08 */
	unsigned int sr;	/* 0x0c */
	unsigned int cpsr;	/* 0x10 */
	unsigned int imsc;
	unsigned int ris;
	unsigned int mis;
	unsigned int icr;
	unsigned int dmacr;
	unsigned int reserved[2];
	unsigned int csr;
};

struct acp3400_ssp {
	struct device *dev;
	struct acp3400_ssp_regs __iomem *ssp_regs;
	struct mutex ssp_lock;
};

static struct acp3400_ssp *ssp;

static inline unsigned char swap8(unsigned char c)
{
	int i;
	unsigned char result = 0;
	for (i = 0; i < 8; ++i) {
		result = result << 1;
		result |= (c & 1);
		c = c >> 1;
	}
	return result;
}

/*
 * acp3400_ssp_write - send data
 *
 * Only Motorola SPI frame format and 8-bit data size is supported
 */
int acp3400_ssp_write(void *data, unsigned int len, unsigned int flags)
{
	unsigned int val;
	unsigned char *p = data;
	unsigned long tmo_jiffies;
	int ret = 0, do_swap = 0;

	if (!ssp)
		return -ENODEV;
	if (!len)
		return 0;
	if (!data)
		return -EINVAL;

	if (flags & ACP3400_SSP_LSB_FIRST)
		do_swap = 1;

	mutex_lock(&ssp->ssp_lock);

	/* CS setup */
	if (flags & ACP3400_SSP_CS_UND)
		val = 0x1f;
	else
		val = flags >> 24;
	ssp->ssp_regs->csr = cpu_to_le32(val);

	/* CPSR setup */
	ssp->ssp_regs->cpsr = cpu_to_le32(SSP_CPSR);
	wmb();

	/* CR0 setup */
	val = SSP_CR0_FRF_SPI | SSP_CR0_DSS_8BIT;
	switch (flags & 0x03) {
	case ACP3400_SSP_CLK_50MHZ:
		val |= SSP_CR0_SCR_50MHZ;
		break;
	case ACP3400_SSP_CLK_25MHZ:
		val |= SSP_CR0_SCR_25MHZ;
		break;
	case ACP3400_SSP_CLK_12_5MHZ:
		val |= SSP_CR0_SCR_12_5MHZ;
		break;
	case ACP3400_SSP_CLK_6_25MHZ:
	default:
		val |= SSP_CR0_SCR_6_25MHZ;
		break;
	}
	ssp->ssp_regs->cr0 = cpu_to_le32(val);
	wmb();

	/* enable SSP port in master mode */
	ssp->ssp_regs->cr1 = cpu_to_le32(SSP_CR1_SSE_EN | SSP_CR1_MS_MASTER);
	mb();

	/* transfer data */
	tmo_jiffies = jiffies + msecs_to_jiffies(10000); /* 10s timeout */
	while (len--) {
		ssp->ssp_regs->dr = do_swap ?
			cpu_to_le32((unsigned int) swap8(*p)) :
			cpu_to_le32((unsigned int) *p);

		while (!(le32_to_cpu(ssp->ssp_regs->sr) & SSP_SR_TNF) &&
		       time_before(jiffies, tmo_jiffies)) udelay(1);

		if (time_after(jiffies, tmo_jiffies)) {
			ret = -EIO;
			goto timeout;
		}

		p++;
	}

	/* wait for transfer to be finished */
	tmo_jiffies = jiffies + msecs_to_jiffies(1000); /* 1s timeout */
	while (le32_to_cpu(ssp->ssp_regs->sr) & SSP_SR_BSY &&
	       time_before(jiffies, tmo_jiffies))
		udelay(1);

	if (time_after(jiffies, tmo_jiffies)) {
		ret = -EIO;
		goto timeout;
	}

timeout:
	/* deselect and disable */
	ssp->ssp_regs->csr = cpu_to_le32(0x1f);
	ssp->ssp_regs->cr1 = cpu_to_le32(0x0);
	mb();

	mutex_unlock(&ssp->ssp_lock);

	return ret;
}
EXPORT_SYMBOL(acp3400_ssp_write);

static int __init acp3400_ssp_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "acp,acp3400-ssp");
	if (!np)
		return -ENODEV;

	ssp = kzalloc(sizeof(*ssp), GFP_KERNEL);
	if (!ssp)
		return -ENOMEM;

	ssp->ssp_regs = of_iomap(np, 0);
	if (!ssp->ssp_regs) {
		pr_err("%s: failed to map I/O\n", np->full_name);
		goto err;
	}

	mutex_init(&ssp->ssp_lock);

	/* reset SSP */
	ssp->ssp_regs->csr = cpu_to_le32(0x1f);
	ssp->ssp_regs->cr1 = cpu_to_le32(0x00);
	ssp->ssp_regs->cr0 = cpu_to_le32(0x00);
	ssp->ssp_regs->imsc = cpu_to_le32(0x00);
	ssp->ssp_regs->dmacr = cpu_to_le32(0x00);

	pr_info("%s: ready\n", np->full_name);

	return 0;
err:
	if (ssp)
		kfree(ssp);
	ssp = NULL;
	return -ENODEV;
}
arch_initcall(acp3400_ssp_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ACP3400 SSP driver");
MODULE_AUTHOR("Andrey Panteleev <andrey.xx.panteleev@ericsson.com>");
