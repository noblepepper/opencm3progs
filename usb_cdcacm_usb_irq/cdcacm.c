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

/*
#define BAUDRATE 230400
#define BAUDRATE 115200
*/

#define BAUDRATE 115200

#define BUFSIZE 256
struct uartbuf {
	uint16_t start, end, high;
	uint8_t data[BUFSIZE];
};

struct uartbuf uart1TxBuf, uart1RxBuf, uart2TxBuf, uart2RxBuf;

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

static	usbd_device *usbd_dev;

int rxOverflow = 0;
void usart1_isr(void)
{
	static uint8_t data;
	char BufCount[79];
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOC, GPIO13);
		/* Send UART1 buffer status on UART2 */
		for (int i = 0; BufCount[i]; i++)
			bufAdd(&uart2TxBuf, BufCount[i]);
		sprintf(BufCount, "Receiving Data - transmit: %i receive: %i\n\r", uart1TxBuf.end - uart1TxBuf.start, uart1RxBuf.end - uart1RxBuf.start);
	/*	USART_CR1(USART2) |= USART_CR1_TXEIE;*/
		

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1) & 0xff;
		if (!bufAdd(&uart1RxBuf, data))
			rxOverflow = 1;
		/*usbd_ep_write_packet(usbd_dev, 0x82, &data, 1);*/
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
	/*	gpio_toggle(GPIOC, GPIO13);*/
		/* Send UART1 buffer status on UART2 */
		for (int i = 0; BufCount[i]; i++)
			bufAdd(&uart2TxBuf, BufCount[i]);
		sprintf(BufCount, "Transmitting data -transmit: %i receive: %i\n\r", uart1TxBuf.end - uart1TxBuf.start, uart1RxBuf.end - uart1RxBuf.start);
		
		/* Put data into the transmit register. */
		if (bufRemove(&uart1TxBuf, &data)){
			usart_send(USART1, data);
		} else {
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		}
	}
//	USART_CR1(USART2) |= USART_CR1_TXEIE;
}

void usart2_isr(void)
{
	static uint8_t data;

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
	/*	gpio_toggle(GPIOC, GPIO13);*/

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART2) & 0xff;
	/*	usbd_ep_write_packet(usbd_dev, 0x82, &data, 1);*/
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
	/*	gpio_toggle(GPIOC, GPIO13);*/

		/* Put data into the transmit register. */
		if (bufRemove(&uart2TxBuf, &data)){
			usart_send(USART2, data);
		} else {
		/* Disable the TXE interrupt as we don't need it anymore. */
		usart_disable_tx_interrupt(USART2);
/*		USART_CR1(USART2) &= ~USART_CR1_TXEIE;*/
		}
	}
}
/* Interrupts */

static void usb_int_relay(void) {
  /* Need to pass a parameter... otherwise just alias it directly. */
  usbd_poll(usbd_dev);
}

void usb_wakeup_isr(void)
__attribute__ ((alias ("usb_int_relay")));

void usb_hp_can_tx_isr(void)
__attribute__ ((alias ("usb_int_relay")));

void usb_lp_can_rx0_isr(void)
__attribute__ ((alias ("usb_int_relay")));

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return 1;
		}
/**********HERE IS THE BAUDRATE REQUEST***********************/
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return 0;
		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	char buf[64];
	nvic_disable_irq(NVIC_USART1_IRQ);
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	nvic_enable_irq(NVIC_USART1_IRQ);

	if (len) {
		for (int i = 0; i < len; i++){
			bufAdd(&uart1TxBuf, buf[i]);
		}


/*		usbd_ep_write_packet(usbd_dev, 0x82, buf, len);*/
		buf[len] = 0;
		gpio_toggle(GPIOC, GPIO13);
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
}

static void cdcacm_data_rx_cb_stashed(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	char buf[64];
/*	nvic_disable_irq(NVIC_USART1_IRQ);*/
	int len;
  /* Blocking read. Assume RX user buffer is empty. TODO: consider setting a timeout */
  while (0 == (len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64)));
/*ret = usbd_ep_read_packet(usbd_dev, EP_IN, usbcdc_rxbuf, USBCDC_PKT_SIZE_DAT)));*/
/*	nvic_enable_irq(NVIC_USART1_IRQ);*/

	if (len) {
		for (int i = 0; i < len; i++){
			bufAdd(&uart1TxBuf, buf[i]);
		}
/*		usbd_ep_write_packet(usbd_dev, 0x82, buf, len);*/
		buf[len] = 0;
/*		gpio_toggle(GPIOC, GPIO13);*/
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
}

volatile bool usb_ready = false;

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

/*	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, NULL);*/
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
	if (wValue > 0) {
		usb_ready = true;
	}
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
        systick_set_frequency(1000, 72000000);

	// Set priority of systic
	// NOTE: I do not know how this priority pertains to all the others (It was copied out of BlackMagic code))
	/*SCB_SHPR(11) &= ~(15 << 4);
	SCB_SHPR(11) |= IRQ_PRI_SYSTICK;*/
	
	systick_interrupt_disable();
        systick_counter_disable();
}

void NVIC_Configuration(void) {
	/* Configure UART1 IRQ */
	nvic_set_priority(NVIC_USART1_IRQ, 1);

	/* Configure UART2 IRQ */
	nvic_set_priority(NVIC_USART2_IRQ, 2);

}

static void usart_setup(void)
{
	/* disable interrupts */
	nvic_disable_irq(NVIC_DMA1_CHANNEL4_IRQ);
	nvic_disable_irq(NVIC_DMA1_CHANNEL5_IRQ);
	/*nvic_disable_irq(NVIC_DMA1_USART1_IRQ);*/
	nvic_disable_irq(NVIC_TIM2_IRQ);
	nvic_disable_irq(NVIC_EXTI9_5_IRQ);
	
	/* Set priorities */
	nvic_set_priority(NVIC_USART1_IRQ, 1);
	/* NVIC_Configuration(); */
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);
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
}

static void usart2_setup(void)
{
	/* Enable the USART2 interrupt. */
	nvic_set_priority(NVIC_USART2_IRQ, 2);
	nvic_enable_irq(NVIC_USART2_IRQ);
	/* Setup GPIO pin GPIO_USART2_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	/* Setup GPIO pin GPIO_USART2_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200/*230400*/);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	/* Enable USART2 Receive interrupt. */
	USART_CR1(USART2) |= USART_CR1_RXNEIE;
	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	gpio_set(GPIOC, GPIO13);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void usb_lp_can_rx0_irq(void) {
	usbd_poll(usbd_dev);
}

static void cdcacm_reset(void) {
  usb_ready = false;
}

int main(void)
{
	int i;
	uint8_t data;

/*	usbd_device *usbd_dev;*/
	clock_setup();
	gpio_setup();
	usart_setup();
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, 
				usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
  	usbd_register_reset_callback(usbd_dev, cdcacm_reset);
	/*nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);*/
	for (i = 0; i < 0x800000; i++)
		__asm__("nop");
	gpio_clear(GPIOC, GPIO13);
 	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	while (1){
	/*	usbd_poll(usbd_dev);*/
		nvic_disable_irq(NVIC_USART1_IRQ);
		if (!bufEmpty(&uart1RxBuf)){
			bufRemove(&uart1RxBuf, &data);
  /* Blocking write */
  while (0 == (i = usbd_ep_write_packet(usbd_dev, 0x82, &data, 1)));

/*		while (!usbd_ep_write_packet(usbd_dev, 0x82, &data, 1));*/
		}
		nvic_enable_irq(NVIC_USART1_IRQ);
	}
}
