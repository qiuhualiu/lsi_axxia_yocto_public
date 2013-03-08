/*
 * ACP3400 GPIO driver
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#define DRV_NAME	"acp3400-gpio"

#define ACP3400_GPIO0		0
#define ACP3400_GPIO1		1
#define ACP3400_GPION		2

/* total number of pins */
#define ACP3400_NGPIO		12

struct acp3400_gpio_regs {
	unsigned int gpiodata[0x100];
	unsigned int gpiodir;
	unsigned int gpiois;
	unsigned int gpioibe;
	unsigned int gpioiev;
	unsigned int gpioie;
	unsigned int gpioris;
	unsigned int gpiomis;
	unsigned int gpioic;
	unsigned int gpioafsel;
};

struct acp3400_gpio {
	struct device *dev;
	struct acp3400_gpio_regs __iomem *gpio_regs[ACP3400_GPION];
	struct gpio_chip gpio_chip;
	spinlock_t gpio_lock;

	int irq_top;
	struct irq_domain *irq;
	int irq_base;
	spinlock_t irq_lock;
};

static struct acp3400_gpio_pin {
	int block;
	int addr;
	int mask;
} acp3400_gpio_pin_map[ACP3400_NGPIO] = {
	{ACP3400_GPIO0, 1 << 0, 1 << 24}, /* 0 */
	{ACP3400_GPIO0, 1 << 1, 1 << 25}, /* 1 */
	{ACP3400_GPIO0, 1 << 2, 1 << 26}, /* 2 */
	{ACP3400_GPIO0, 1 << 3, 1 << 27}, /* 3 */
	{ACP3400_GPIO0, 1 << 5, 1 << 29}, /* 5 */
	{ACP3400_GPIO0, 1 << 6, 1 << 30}, /* 6 */
	{ACP3400_GPIO0, 1 << 7, 1 << 31}, /* 7 */
	{ACP3400_GPIO1, 1 << 1, 1 << 25}, /* 9 */
	{ACP3400_GPIO1, 1 << 2, 1 << 26}, /* 10 */
	{ACP3400_GPIO1, 1 << 4, 1 << 28}, /* 12 */
	{ACP3400_GPIO1, 1 << 6, 1 << 30}, /* 14 */
	{ACP3400_GPIO1, 1 << 7, 1 << 31}, /* 15 */
};

static void acp3400_gpio_irq_mask(struct irq_data *d)
{
	struct acp3400_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	struct acp3400_gpio_pin *pin =
		&acp3400_gpio_pin_map[irqd_to_hwirq(d)];

	spin_lock_irqsave(&chip->irq_lock, flags);
	chip->gpio_regs[pin->block]->gpioie &= ~pin->mask;
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static void acp3400_gpio_irq_unmask(struct irq_data *d)
{
	struct acp3400_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	struct acp3400_gpio_pin *pin =
		&acp3400_gpio_pin_map[irqd_to_hwirq(d)];

	spin_lock_irqsave(&chip->irq_lock, flags);
	chip->gpio_regs[pin->block]->gpioie |= pin->mask;
	spin_unlock_irqrestore(&chip->irq_lock, flags);
}

static void acp3400_gpio_irq_ack(struct irq_data *d)
{
	struct acp3400_gpio *chip = irq_data_get_irq_chip_data(d);
	struct acp3400_gpio_pin *pin =
		&acp3400_gpio_pin_map[irqd_to_hwirq(d)];

	chip->gpio_regs[pin->block]->gpioic = pin->mask;
}

static int acp3400_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct acp3400_gpio *chip = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	struct acp3400_gpio_pin *pin =
		&acp3400_gpio_pin_map[irqd_to_hwirq(d)];

	spin_lock_irqsave(&chip->irq_lock, flags);

	switch (type) {
	case IRQ_TYPE_EDGE_FALLING:
		chip->gpio_regs[pin->block]->gpiois &= ~pin->mask;
		chip->gpio_regs[pin->block]->gpioiev &= ~pin->mask;
		chip->gpio_regs[pin->block]->gpioibe &= ~pin->mask;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		chip->gpio_regs[pin->block]->gpiois &= ~pin->mask;
		chip->gpio_regs[pin->block]->gpioibe |= pin->mask;
		break;
	case IRQ_TYPE_EDGE_RISING:
		chip->gpio_regs[pin->block]->gpiois &= ~pin->mask;
		chip->gpio_regs[pin->block]->gpioiev |= pin->mask;
		chip->gpio_regs[pin->block]->gpioibe &= ~pin->mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		chip->gpio_regs[pin->block]->gpiois |= pin->mask;
		chip->gpio_regs[pin->block]->gpioiev &= ~pin->mask;
		break;
	case IRQ_TYPE_NONE: /* default */
	case IRQ_TYPE_LEVEL_HIGH:
		chip->gpio_regs[pin->block]->gpiois |= pin->mask;
		chip->gpio_regs[pin->block]->gpioiev |= pin->mask;
		break;
	default:
		spin_unlock_irqrestore(&chip->irq_lock, flags);
		return -EINVAL;
	}

	mb();
	spin_unlock_irqrestore(&chip->irq_lock, flags);
	return 0;
}

static struct irq_chip acp3400_gpio_irq_chip = {
	.name			= "acp3400",
	.irq_mask		= acp3400_gpio_irq_mask,
	.irq_unmask		= acp3400_gpio_irq_unmask,
	.irq_set_type		= acp3400_gpio_irq_set_type,
	.irq_ack		= acp3400_gpio_irq_ack,
};

static void acp3400_gpio_irq_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct acp3400_gpio *chip = irq_desc_get_handler_data(desc);
	struct irq_chip *ichip = irq_desc_get_chip(desc);
	struct irq_data *idata = irq_desc_get_irq_data(desc);
	unsigned int i, cascade_irq;
	struct acp3400_gpio_pin *pin;
	unsigned int mis[ACP3400_GPION];

	raw_spin_lock(&desc->lock);

	if (unlikely(irqd_irq_inprogress(idata)))
		goto unlock;

	irqd_set_chained_irq_inprogress(idata);

	mis[ACP3400_GPIO0] = chip->gpio_regs[ACP3400_GPIO0]->gpiomis;
	mis[ACP3400_GPIO1] = chip->gpio_regs[ACP3400_GPIO1]->gpiomis;

	/* FIXME: add mask2pin map to avoid loop */
	for (i = 0; i < ACP3400_NGPIO; ++i) {
		pin = &acp3400_gpio_pin_map[i];
		if (mis[pin->block] & pin->mask) {
			cascade_irq = irq_linear_revmap(chip->irq, i);
			if (cascade_irq != NO_IRQ)
				generic_handle_irq(cascade_irq);
		}
	}

	irqd_clr_chained_irq_inprogress(idata);

	if (ichip->irq_eoi)
		ichip->irq_eoi(idata);

unlock:
	raw_spin_unlock(&desc->lock);
}

static int acp3400_gpio_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	irq_set_chip_data(virq, h->host_data);
	irq_set_chip_and_handler(virq, &acp3400_gpio_irq_chip,
			handle_level_irq);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static int acp3400_gpio_irq_xlate(struct irq_domain *h, struct device_node *ct,
				  const u32 *intspec, unsigned int intsize,
				  irq_hw_number_t *out_hwirq,
				  unsigned int *out_flags)
{
	*out_hwirq = intspec[0];
	*out_flags = intspec[1];

	return 0;
}

static struct irq_domain_ops acp3400_gpio_irq_ops = {
	.map	= acp3400_gpio_irq_map,
	.xlate	= acp3400_gpio_irq_xlate,
};

static int acp3400_gpio_irq_setup(struct device_node *np,
		struct acp3400_gpio *chip)
{
	/* always mask and clear interrupts */
	chip->gpio_regs[ACP3400_GPIO0]->gpioie = 0x00;
	chip->gpio_regs[ACP3400_GPIO0]->gpioic = 0xffffffff;
	chip->gpio_regs[ACP3400_GPIO1]->gpioie = 0x00;
	chip->gpio_regs[ACP3400_GPIO1]->gpioic = 0xffffffff;

	chip->irq_top = irq_of_parse_and_map(np, 0);
	if (chip->irq_top == NO_IRQ)
		return 0;

	spin_lock_init(&chip->irq_lock);

	chip->irq = irq_domain_add_linear(np, ACP3400_NGPIO,
			&acp3400_gpio_irq_ops, chip);
	if (!chip->irq)
		return 0;

	chip->irq->host_data = chip;
	irq_set_handler_data(chip->irq_top, chip);
	irq_set_chained_handler(chip->irq_top, acp3400_gpio_irq_cascade);

	return 0;
}

static int acp3400_gpio_direction_in(struct gpio_chip *gc, unsigned off)
{
	struct acp3400_gpio *chip;
	struct acp3400_gpio_pin *pin;
	unsigned long flags;

	chip = container_of(gc, struct acp3400_gpio, gpio_chip);
	pin = &acp3400_gpio_pin_map[off];

	spin_lock_irqsave(&chip->gpio_lock, flags);
	chip->gpio_regs[pin->block]->gpiodir &= ~pin->mask;
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int acp3400_gpio_direction_out(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct acp3400_gpio *chip;
	struct acp3400_gpio_pin *pin;
	unsigned long flags;

	chip = container_of(gc, struct acp3400_gpio, gpio_chip);
	pin = &acp3400_gpio_pin_map[off];

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/*
	 * configure direction first; opposite sequence is
	 * ignored by the GPIO block, which is sad
	 */
	chip->gpio_regs[pin->block]->gpiodir |= pin->mask;

	if (val)
		chip->gpio_regs[pin->block]->gpiodata[pin->addr] = ~0;
	else
		chip->gpio_regs[pin->block]->gpiodata[pin->addr] = 0;

	spin_unlock_irqrestore(&chip->gpio_lock, flags);
	return 0;
}

static int acp3400_gpio_get(struct gpio_chip *gc, unsigned off)
{
	struct acp3400_gpio *chip;
	struct acp3400_gpio_pin *pin;

	chip = container_of(gc, struct acp3400_gpio, gpio_chip);
	pin = &acp3400_gpio_pin_map[off];

	return chip->gpio_regs[pin->block]->gpiodata[pin->addr] ? 1 : 0;
}

static void acp3400_gpio_set(struct gpio_chip *gc, unsigned off,
		int val)
{
	struct acp3400_gpio *chip;
	struct acp3400_gpio_pin *pin;

	chip = container_of(gc, struct acp3400_gpio, gpio_chip);
	pin = &acp3400_gpio_pin_map[off];

	if (val)
		chip->gpio_regs[pin->block]->gpiodata[pin->addr] = ~0;
	else
		chip->gpio_regs[pin->block]->gpiodata[pin->addr] = 0;
}

static int acp3400_gpio_to_irq(struct gpio_chip *gc, unsigned off)
{
	struct acp3400_gpio *chip;

	chip = container_of(gc, struct acp3400_gpio, gpio_chip);

	if (!chip->irq)
		return -ENXIO;

	return irq_create_mapping(chip->irq, off);
}

static struct gpio_chip acp3400_gpio_chip = {
	.label			= "acp3400",
	.owner			= THIS_MODULE,
	.direction_input	= acp3400_gpio_direction_in,
	.get			= acp3400_gpio_get,
	.direction_output	= acp3400_gpio_direction_out,
	.set			= acp3400_gpio_set,
	.to_irq			= acp3400_gpio_to_irq,
};

static int __init acp3400_gpio_init(void)
{
	struct device_node *np;
	for_each_compatible_node(np, NULL, "acp,acp3400-gpio") {
		struct acp3400_gpio *chip;
		const __be32 *val;
		int size, ret;

		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (chip == NULL)
			return -ENOMEM;

		chip->gpio_regs[ACP3400_GPIO0] = of_iomap(np, 0);
		chip->gpio_regs[ACP3400_GPIO1] = of_iomap(np, 1);
		if (!chip->gpio_regs[ACP3400_GPIO0] ||
		    !chip->gpio_regs[ACP3400_GPIO1]) {
			pr_err("%s: failed to map I/O\n", np->full_name);
			goto err;
		}

		spin_lock_init(&chip->gpio_lock);

		/* setup gpio chip */
		chip->gpio_chip = acp3400_gpio_chip;
		chip->gpio_chip.ngpio = ACP3400_NGPIO;

		chip->gpio_chip.base = -1;
		val = of_get_property(np, "gpio-base", &size);
		if (val && size == sizeof(*val))
			chip->gpio_chip.base = be32_to_cpup(val);

		/* setup interrupts */
		ret = acp3400_gpio_irq_setup(np, chip);
		if (ret)
			goto err;

		pr_info("%s: base:%d ngpio:%u irq:%d\n", np->full_name,
				chip->gpio_chip.base, chip->gpio_chip.ngpio,
				chip->irq_top);

		ret = gpiochip_add(&chip->gpio_chip);
		if (ret < 0) {
			pr_err("%s: could not register gpio chip, %d\n",
					np->full_name, ret);
			goto err;
		}
		continue;
err:
		if (chip->gpio_regs[ACP3400_GPIO0])
			iounmap(chip->gpio_regs[ACP3400_GPIO0]);
		if (chip->gpio_regs[ACP3400_GPIO1])
			iounmap(chip->gpio_regs[ACP3400_GPIO1]);
		kfree(chip);
	}
	return 0;
}
arch_initcall(acp3400_gpio_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ACP3400 GPIO driver");
MODULE_AUTHOR("Andrey Panteleev <andrey.xx.panteleev@ericsson.com>");
