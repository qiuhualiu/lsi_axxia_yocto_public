/*
 * udbg for ARM AMBA-type serial ports
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 * Original patch created on arch/powerpc/kernel/udbg_16550.c by
 * john.jacques@lsi.com
 */


#include <linux/types.h>
#include <asm/io.h>
#include <asm/udbg.h>

#if defined(CONFIG_PPC_EARLY_DEBUG_AXXIA)

#define AXXIA_EARLY_DBG_TIMERADDR  0x00008040
#define AXXIA_EARLY_DBG_UARTADDR   0x00004000

static void __iomem *timer_base = (void __iomem *)(AXXIA_EARLY_DEBUG_VIRTADDR |\
						   AXXIA_EARLY_DBG_TIMERADDR);
static void __iomem *uart_base = (void __iomem *)(AXXIA_EARLY_DEBUG_VIRTADDR | \
						  AXXIA_EARLY_DBG_UARTADDR);

#define UART_DR	   0x00
#define UART_FR	   0x18
#define UART_IBRD  0x24
#define UART_FBRD  0x28
#define UART_IFLS  0x34
#define UART_IMSC  0x38
#define UART_ECR   0x04

#define FR_RXFE 0x10
#define FR_TXFF 0x20

static int __init early_parse_comport(char *p)
{
	unsigned long base;

	if (!p || !(*p))
		return 0;

	base = simple_strtoul(p, 0, 16);
	uart_base = (void __iomem *)(AXXIA_EARLY_DEBUG_VIRTADDR | base);
	return 0;
}
early_param("uart_addr", early_parse_comport);

static int __init early_parse_timer(char *p)
{
	unsigned long base;

	if (!p || !(*p))
		return 0;

	base = simple_strtoul(p, 0, 16);
	timer_base = (void __iomem *)(AXXIA_EARLY_DEBUG_VIRTADDR | base);
	return 0;
}
early_param("timer_addr", early_parse_timer);

static void
acp_putc(char c)
{
	while (0 != (in_le32(uart_base + UART_FR) & FR_TXFF))
		;

	if ('\n' == c) {
		out_le32(uart_base + UART_DR, '\r');
		while (0 != (in_le32(uart_base + UART_FR) & FR_TXFF))
			;
	}

	out_le32(uart_base + UART_DR, c);

	return;
}

static int
acp_getc(void)
{
	while (0 != (in_le32(uart_base + UART_FR) & FR_RXFE))
		;
	return in_le32(uart_base + UART_DR);
}

void __init
udbg_init_arm_amba(void)
{
	udbg_putc = acp_putc;
	udbg_getc = acp_getc;

	return;
}
#endif

void
udbg_arm_amba_init_uart(void)
{
#if defined(CONFIG_PPC_EARLY_DEBUG_AXXIA)
	out_le32(uart_base + UART_IFLS, 0);
	out_le32(uart_base + UART_IMSC, 0x700);
	out_le32(uart_base + UART_ECR, 0);
#endif
	return;
}


