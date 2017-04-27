/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2012-2013 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 */
/*
 * Pins handled by hw peripheral:
 * Tx  <-> PB1
 * Rx  <-> PB0
 * Input pins handled manually via interrupts:
 * DCD <-> PA2
 * DSR <-> PA3
 * RI  <-> PA4
 * CTS <-> PA5 (UNUSED)
 * Output pins handled via commands from the host:
 * DTR <-> PA6
 * RTS <-> PA7
 */

#include "usb_to_serial_cdcacm.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/nvic.h>

static void uart_ctl_line_setup(void)
{
	uint32_t inpins, outpins;
/*
	inpins = PIN_DCD | PIN_DSR | PIN_RI | PIN_CTS;
	outpins = PIN_DTR | PIN_RTS;

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, outpins);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, inpins);
*/}

void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	/* Finally enable the USART. */
	usart_enable(USART1);
}


uint8_t uart_get_ctl_line_state(void)
{
/*	return gpio_read(GPIOA, PIN_RI | PIN_DSR | PIN_DCD);*/
}

void uart_set_ctl_line_state(uint8_t dtr, uint8_t rts)
{
	uint8_t val = 0;
/*
	val |= dtr ? PIN_DTR : 0;
	val |= rts ? PIN_RTS : 0;

	gpio_write(GPIOA, PIN_DTR | PIN_RTS, val);
*/}

void uart1_isr(void)
{
	static uint8_t data;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO13);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1) & 0xff;
		glue_data_received_cb(&data, 1);
	}
}
