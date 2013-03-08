/*
 * arch/powerpc/boot/acp.c
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#include "io.h"
#include "ops.h"

#if 1
#define writeapb(offset, value) out_le32((offset), (value))
#define readapb(offset) in_le32((offset))
#else
static inline void
writeapb(unsigned *address, int value)
{
	__asm__ __volatile__("stwbrx %1,0,%2"
			     : "=m" (*address)
			     : "r" (value), "r" (address));
}
static inline unsigned
readapb(const unsigned *address)
{
	unsigned value;

	__asm__ __volatile__("lwbrx %0,0,%1; twi 0,%0,0"
			     : "=r" (value)
			     : "r" (address), "m" (*address));

	return value;
}
#endif

/*
  ======================================================================
  ======================================================================
  Clocks and Divisors...
  ======================================================================
  ======================================================================
*/

static void *timer_base;

#define TIMER_LOAD		       0x00
#define TIMER_VALUE		       0x04
#define TIMER_CONTROL		       0x08
#define TIMER_CONTROL_ENABLE	       0x80
#define TIMER_CONTROL_MODE	       0x40
#define TIMER_CONTROL_INTERRUPT_ENABLE 0x20
#define TIMER_CONTROL_OUTPUT_MODE      0x10
#define TIMER_CONTROL_PRESCALER	       0x0c
#define TIMER_CONTROL_SIZE	       0x02
#define TIMER_CONTROL_ONE_SHOT	       0x01
#define TIMER_INTCLR		       0x0C
#define TIMER_RIS		       0x10
#define TIMER_MIS		       0x14
#define TIMER_BGLOAD		       0x18

/*
  ======================================================================
*/

struct clock_stuff {

	unsigned char ibrd;
	unsigned char fbrd;

};

/*
  ----------------------------------------------------------------------
  get_clock_stuff
*/

static int
get_clock_stuff(int baud_rate, struct clock_stuff *clock_stuff)
{

	unsigned long speed_;
	unsigned long divisor_;
	unsigned long ibrd_, fbrd_;

	speed_ = 6500000;

	/*
	  The UART clock is derived from the ARM core clock using the second
	  timer (timer 1).  Each time timer 1 crosses zero, the UART clock
	  gets toggled.	 The timer load value acts as a divisor.

	  Since the IBDR (integer part of the baud rate divisor) is a 16 bit
	  quatity, find the minimum load value that will let the IBDR/FBDR
	  result in the desired baud rate.
	*/

	divisor_ = 1;

	do {
		ibrd_ = (speed_ / ++divisor_) / (16 * baud_rate);
	} while (0xff < ibrd_);

	/*
	  The following forumla is from the ARM document (ARM DDI 0183E).

	  Baud Rate Divisor = (Uart Clock / (16 * Baud Rate))

	  Baud Rate Divisor is then split into integral and fractional
	  parts.  The IBRD value is simply the itegral part.	The FBRD is
	  calculated as follows.

	  FBRD = fractional part of the Baud Rate Divisor * 64 + 0.5

	  The fractional part of the Baud Rate Divisor can be represented as
	  follows.

	  (Uart Clock % (16 * baud_rate)) / (16 * baud_rate)

	  As long as the division isn't done till the end.  So, the above *
	  64 + 0.5 is the FBRD.	 Also note that x/y + 1/2 = (2x+y)/2y.	This
	  leads to

	  ((Uart Clock % (16 * baud_rate)) * 64 * 2 + (16 * baud_rate))
	  ---------------------------------------------------------------------
	  2 * (16 * baud_rate)
	*/

	fbrd_ = (speed_ / divisor_) % (16 * baud_rate);
	fbrd_ *= 128;
	fbrd_ += (16 * baud_rate);
	fbrd_ /= (2 * (16 * baud_rate));

	--divisor_;
	clock_stuff->ibrd = ibrd_;
	clock_stuff->fbrd = fbrd_;

	/*
	 * Set up the clock.
	 */

	writeapb(timer_base + TIMER_CONTROL, 0);
	writeapb(timer_base + TIMER_LOAD, divisor_);
	writeapb(timer_base + TIMER_CONTROL,
		 (TIMER_CONTROL_ENABLE | TIMER_CONTROL_MODE));

	return 0;

}

/*
  ======================================================================
  ======================================================================
  serial
  ======================================================================
  ======================================================================
*/

static void *uart0_base;

#define UART_DR	   0x00
#define UART_FR	   0x18
#define UART_IBRD  0x24
#define UART_FBRD  0x28
#define UART_LCR_H 0x2c
#define UART_CR	   0x30
#define UART_IFLS  0x34
#define UART_IMSC  0x38
#define UART_ECR   0x04

#define FR_RXFE 0x10
#define FR_TXFF 0x20

static int
acp_serial_open(void)
{
	struct clock_stuff clock_stuff;

	get_clock_stuff(9600, &clock_stuff);
	writeapb(uart0_base + UART_IBRD, clock_stuff.ibrd);
	writeapb(uart0_base + UART_FBRD, clock_stuff.fbrd);
	writeapb(uart0_base + UART_LCR_H, 0x70);
	writeapb(uart0_base + UART_CR, 0x301);
	writeapb(uart0_base + UART_IFLS, 0);
	writeapb(uart0_base + UART_IMSC, 0x700);
	writeapb(uart0_base + UART_ECR, 0);

	return 0;
}

static void
acp_serial_putc(unsigned char c)
{
	while (0 != (readapb(uart0_base + UART_FR) & FR_TXFF))
		;

	if ('\n' == c) {
		writeapb(uart0_base + UART_DR, '\r');
		while (0 != (readapb(uart0_base + UART_FR) & FR_TXFF))
			;
	}

	writeapb(uart0_base + UART_DR, c);

	return;
}

static unsigned char
acp_serial_getc(void)
{
	while (0 != (readapb(uart0_base + UART_FR) & FR_RXFE))
		;
	return readapb(uart0_base + UART_DR);
}

static unsigned char
acp_serial_tstc(void)
{
	return (0 != (readapb(uart0_base + UART_FR) & FR_RXFE));
}

int
acp_console_init(void *devp, struct serial_console_data *scdp)
{
#if 0
	unsigned long reg_offset;

	if (dt_get_virtual_reg(devp, (void **) &uart0_base, 1) < 1)
		return -1;

	if (sizeof(reg_offset) ==
	    getprop(devp, "reg-offset", &reg_offset, sizeof(reg_offset)))
		uart0_base += reg_offset;

	if (sizeof(reg_shift) !=
	    getprop(devp, "reg-shift", &reg_shift, sizeof(reg_shift)))
		reg_shift = 0;
#else
	uart0_base = (void *) 0xf0004000;
	/*uart0_base = (void *) 0xf0024000;*/
	timer_base = (void *) 0xf0008040;
	/*timer_base = (void *) 0xf0029040;*/
#endif
	scdp->open = acp_serial_open;
	scdp->putc = acp_serial_putc;
	scdp->getc = acp_serial_getc;
	scdp->tstc = acp_serial_tstc;
	scdp->close = NULL;

	return 0;
}
