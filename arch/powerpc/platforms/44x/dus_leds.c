#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/hrtimer.h>

#define DUS_LED_FAULT	35
#define DUS_LED_OPER	36
#define DUS_LED_MAINT	37
#define DUS_LED_INFO	38

static struct gpio_led gpm5_leds[] = {
	{
		.name = "gpm5-gpio:green:operation",
		.default_trigger = "default-on",
		.gpio = DUS_LED_OPER,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name = "gpm5-gpio:yellow:information",
		.default_trigger = "default-on",
		.gpio = DUS_LED_INFO,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_KEEP,
	},
	{
		.name = "gpm5-gpio:red:fault",
		.default_trigger = "none",
		.gpio = DUS_LED_FAULT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	{
		.name = "gpm5-gpio:blue:maint",
		.default_trigger = "none",
		.gpio = DUS_LED_MAINT,
		.active_low = 0,
		.retain_state_suspended = 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data gpm5_led_data = {
	.num_leds       = ARRAY_SIZE(gpm5_leds),
	.leds           = gpm5_leds,
};

static struct platform_device gpm5_gpio_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &gpm5_led_data,
	},
};

static __init int add_dus_leds_device(void)
{
	return platform_device_register(&gpm5_gpio_leds);
}
postcore_initcall(add_dus_leds_device);

