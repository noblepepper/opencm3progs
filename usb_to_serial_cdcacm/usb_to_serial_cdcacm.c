/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
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
 * Flashes the Red, Green and Blue diodes on the board, in order.
 *
 * RED controlled by PF1
 * Green controlled by PF3
 * Blue controlled by PF2
 */

#include "usb_to_serial_cdcacm.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/cm3/scb.h>

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOC);
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
}
/*
 * GPIO setup:
 */

static void gpio_setup(void)
{
	gpio_set(GPIOC, GPIO13);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void glue_data_received_cb(uint8_t * buf, uint16_t len)
{
	cdcacm_send_data(buf, len);
}

void glue_set_line_state_cb(uint8_t dtr, uint8_t rts)
{
	uart_set_ctl_line_state(dtr, rts);
}

int glue_set_line_coding_cb(uint32_t baud, uint8_t databits,
			    enum usb_cdc_line_coding_bParityType cdc_parity,
			    enum usb_cdc_line_coding_bCharFormat cdc_stopbits)
{
	/*enum uart_parity parity;
	uint8_t uart_stopbits;

	if (databits < 5 || databits > 8)
		return 0;

	switch (cdc_parity) {
	case USB_CDC_NO_PARITY:
		parity = UART_PARITY_NONE;
		break;
	case USB_CDC_ODD_PARITY:
		parity = UART_PARITY_ODD;
		break;
	case USB_CDC_EVEN_PARITY:
		parity = UART_PARITY_EVEN;
		break;
	default:
		return 0;
	}

	switch (cdc_stopbits) {
	case USB_CDC_1_STOP_BITS:
		uart_stopbits = 1;
		break;
	case USB_CDC_2_STOP_BITS:
		uart_stopbits = 2;
		break;
	default:
		return 0;
	}*/

	/* Disable the UART while we mess with its settings */
	/*uart_disable(UART1);*/
	/* Set communication parameters */
/*	uart_set_baudrate(UART1, baud);
	uart_set_databits(UART1, databits);
	uart_set_parity(UART1, parity);
	uart_set_stopbits(UART1, uart_stopbits);*/
	/* Back to work. */
/*	uart_enable(UART1);*/

	return 1;
}
/**
 * \brief UART Send Data Word with Blocking
 *
 * Blocks until the transmit data FIFO can accept the next data word for
 * transmission.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 */
void uart_send_blocking(uint32_t uart, uint16_t data)
{
	while ((USART_SR(USART1) & USART_SR_TXE));
	usart_send(uart, data);
}

/**
 * \brief UART Read a Received Data Word with Blocking.
 *
 * Wait until a data word has been received then return the word.
 *
 * @param[in] uart UART block register address base @ref uart_reg_base
 * @return data from the Rx FIFO.
 */
uint16_t uart_recv_blocking(uint32_t uart)
{
	uart_wait_recv_ready(uart);
	return uart_recv(uart);
}

void glue_send_data_cb(uint8_t * buf, uint16_t len)
{
	int i;

	for (i = 0; i < len; i++) {
		uart_send_blocking(USART1, buf[i]);
	}
}

static void mainloop(void)
{
	uint8_t linestate, cdcacmstate;
	static uint8_t oldlinestate = 0;

	/* See if the state of control lines has changed */
	linestate = uart_get_ctl_line_state();
	if (oldlinestate != linestate) {
		/* Inform host of state change */
		cdcacmstate = 0;
		if (linestate & PIN_RI)
			cdcacmstate |= CDCACM_RI;
		if (linestate & PIN_DSR)
			cdcacmstate |= CDCACM_DSR;
		if (linestate & PIN_DCD)
			cdcacmstate |= CDCACM_DCD;

		cdcacm_line_state_changed_cb(cdcacmstate);
	}
	oldlinestate = linestate;
}

int main(void)
{
	clock_setup();
	gpio_setup();

	cdcacm_init();
	usart_setup();

	while (1)
		mainloop();

	return 0;
}
