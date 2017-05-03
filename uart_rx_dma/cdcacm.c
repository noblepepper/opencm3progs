/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dma.h>

/*
#define BAUDRATE 460800
#define BAUDRATE 230400
#define BAUDRATE 115200
*/

#define BAUDRATE 230400

#define BUFSIZE 256


struct uartbuf {
	uint16_t start, end, high;
	uint8_t data[BUFSIZE];
};

struct uartbuf uart1RxBuf;
char tx1buf[64];
char rx1buf[64];

static int bufFull(struct uartbuf *buf)
{
	return (((buf->end + 1) % BUFSIZE) == buf->start);
}

static int bufEmpty(struct uartbuf *buf)
{
	return (buf->start == buf->end);
}

static int bufAdd(struct uartbuf *buf, uint8_t data)
{
	uint16_t current = 0;
	if (bufFull(buf))
		return 0;
	else {
		buf->data[buf->end] = data;
		buf->end = ((buf->end + 1) % BUFSIZE);
		current = buf->start <= buf->end ? buf->end -buf->start : BUFSIZE - buf->start + buf->end +1;
		buf->high = current > buf->high ? current : buf->high;
	}
	return 1;
}

static int bufRemove(struct uartbuf * buf, uint8_t *data)
{
	if (bufEmpty(buf))
		return 0;
	else {
		*data = buf->data[buf->start];
		buf->start = ((buf->start+ 1) % BUFSIZE);
	}
	return 1;
}

static void dma_read(char *data, int size)
{
	/*dma_enable_channel(DMA1, DMA_CHANNEL5);*/
	/*
	 * Using channel 5 for USART1_RX
	 */
        usart_enable_rx_dma(USART1);
	dma_channel_reset(DMA1, DMA_CHANNEL5);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL5, size);
	dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_VERY_HIGH);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
	dma_enable_channel(DMA1, DMA_CHANNEL5);
	
	
	/*dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);*/
}
/*
int rxOverflow = 0;
void usart1_isr(void)
{
	static uint8_t data;*/
	/* Check if we were called because of RXNE. */
/*	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

*/		/* Indicate that we got data. */
/*		gpio_toggle(GPIOC, GPIO13);
		dma_read(&uart1RxBuf, 1);
*/		/* Retrieve the data from the peripheral. */
/*		data = usart_recv(USART1) & 0xff;
		if (!bufAdd(&uart1RxBuf, data))
			rxOverflow = 1;*/
/*	}
*/
	/* Check if we were called because of TXE. */
/*	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
*/		/* Put data into the transmit register. */
/*		if (bufRemove(&uart1TxBuf, &data)){
			usart_send(USART1, data);
		} else {
*/		/* Disable the TXE interrupt as we don't need it anymore. */
/*		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		}
	}
}*/

static void dma_write(char *data, int size)
{
	/*
	 * Using channel 4 for USART1_TX
	 */
	dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
	dma_enable_channel(DMA1, DMA_CHANNEL4);
}


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	/* Enable GPIOC clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	// Setup heartbeat timer (AHB = 72mhz) (freq = 1000Hz)

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA1);
}

static void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, BAUDRATE);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	/* Finally enable the USART. */
	usart_enable(USART1);

/* Set dma interrupt priorities */
	nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
	nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
	
/* Setup dma channel 4 for tx */
	dma_channel_reset(DMA1, DMA_CHANNEL4);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL4); // (BIT 6)
	dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
        usart_enable_tx_dma(USART1);
/* Setup dma channel 5 for rx */
	dma_channel_reset(DMA1, DMA_CHANNEL5);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)rx1buf);
	dma_set_number_of_data(DMA1, DMA_CHANNEL5, 1);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
	dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_VERY_HIGH);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL5);
        usart_enable_rx_dma(USART1);
}

void usart1_isr(void)
{
	static uint8_t data;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO13);
/*		dma_read(&uart1RxBuf, 1);*/
		/* Retrieve the data from the peripheral. */
/*		data = usart_recv(USART1) & 0xff;
		if (!bufAdd(&uart1RxBuf, data))
			rxOverflow = 1;*/
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
		/* Put data into the transmit register. */
	/*	if (bufRemove(&uart1TxBuf, &data)){
			usart_send(USART1, data);
		} else {*/
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
/*		}*/
	}
}

volatile int transfered = 0;

void dma1_channel4_isr(void)
{/* unit 1 interrrupt and channel4 complete */
	if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
	/* interrupt clear channel 4 */	
		DMA1_IFCR |= DMA_IFCR_CTCIF4;
		dma_disable_channel(DMA1, DMA_CHANNEL4);
		transfered = 1;
	}
}

volatile int received = 0;

void dma1_channel5_isr(void)
{	
	if ((DMA1_ISR &DMA_ISR_TCIF5) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF5;

		received = 1;
	}
	dma_disable_channel(DMA1, DMA_CHANNEL5);
}

static void gpio_setup(void)
{
	gpio_set(GPIOC, GPIO13);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void)
{
	int i;
	uint8_t data;
	char test[80] = "abcdefghijklmnopqrstuvwxyz\n";

	clock_setup();
	gpio_setup();
	usart_setup();
	gpio_set(GPIOC, GPIO13);
	nvic_set_priority(NVIC_USART1_IRQ, 0);
	nvic_enable_irq(NVIC_USART1_IRQ);
	for (i = 0; i < 0x800000; i++)
		__asm__("nop");
	gpio_clear(GPIOC, GPIO13);
	while (1){
		__asm__("nop");
	}
}
