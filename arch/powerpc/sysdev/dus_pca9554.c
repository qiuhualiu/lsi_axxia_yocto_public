/*
 *  dus_pca9554.c - PCA9554 8 bit I/O port
 *
 *  Copied and stripped from drivers/gpio/pca953x.c
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define PCA9554_INPUT          0
#define PCA9554_OUTPUT         1
#define PCA9554_INVERT         2
#define PCA9554_DIRECTION      3

#define PCA9554_GPIOS	       0x00FF
#define PCA9554_INT	       0x0100

static const struct i2c_device_id pca9554_id[] = {
	{ "pca9554", 8, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9554_id);

struct pca9554_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;
	struct mutex i2c_lock;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
};

static int pca9554_write_reg(struct pca9554_chip *chip, int reg, uint16_t val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else
		ret = i2c_smbus_write_word_data(chip->client, reg << 1, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pca9554_read_reg(struct pca9554_chip *chip, int reg, uint16_t *val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);
	else
		ret = i2c_smbus_read_word_data(chip->client, reg << 1);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t)ret;
	return 0;
}

static int pca9554_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pca9554_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9554_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction | (1u << off);
	ret = pca9554_write_reg(chip, PCA9554_DIRECTION, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca9554_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pca9554_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9554_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca9554_write_reg(chip, PCA9554_OUTPUT, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = pca9554_write_reg(chip, PCA9554_DIRECTION, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca9554_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca9554_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9554_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	ret = pca9554_read_reg(chip, PCA9554_INPUT, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void pca9554_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca9554_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca9554_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca9554_write_reg(chip, PCA9554_OUTPUT, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void pca9554_setup_gpio(struct pca9554_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca9554_gpio_direction_input;
	gc->direction_output = pca9554_gpio_direction_output;
	gc->get = pca9554_gpio_get_value;
	gc->set = pca9554_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
}

static int __devinit pca9554_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca9554_chip *chip;
	int ret, size;
	const __be32 *val;
	unsigned int invert;

	chip = kzalloc(sizeof(struct pca9554_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;

	chip->gpio_start = -1;
	val = of_get_property(client->dev.of_node, "gpio-base", &size);
	if (val && size == sizeof(*val))
		chip->gpio_start = be32_to_cpup(val);

	invert = 0x00;
	val = of_get_property(client->dev.of_node, "polarity", &size);
	if (val && size == sizeof(*val))
		invert = be32_to_cpup(val);

	mutex_init(&chip->i2c_lock);

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca9554_setup_gpio(chip, id->driver_data & PCA9554_GPIOS);

	ret = pca9554_read_reg(chip, PCA9554_OUTPUT, &chip->reg_output);
	if (ret)
		goto out_failed;

	ret = pca9554_read_reg(chip, PCA9554_DIRECTION, &chip->reg_direction);
	if (ret)
		goto out_failed;

	/* set platform specific polarity inversion */
	ret = pca9554_write_reg(chip, PCA9554_INVERT, invert);
	if (ret)
		goto out_failed;

	pr_info("%s: base:%d ngpio:%d polarity:%u\n",
			client->dev.of_node->full_name,
			chip->gpio_chip.base,
			chip->gpio_chip.ngpio, invert);

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed;

	i2c_set_clientdata(client, chip);
	return 0;

out_failed:
	kfree(chip);
	return ret;
}

static int pca9554_remove(struct i2c_client *client)
{
	struct pca9554_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip);
	return 0;
}

static struct i2c_driver pca9554_driver = {
	.driver = {
		.name	= "pca9554",
	},
	.probe		= pca9554_probe,
	.remove		= pca9554_remove,
	.id_table	= pca9554_id,
};

static int __init dus_pca9554_init(void)
{
	return i2c_add_driver(&pca9554_driver);
}
arch_initcall(dus_pca9554_init);

MODULE_AUTHOR("Andrey Panteleev <andrey.xx.panteleev@ericsson.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA9554");
MODULE_LICENSE("GPL");
