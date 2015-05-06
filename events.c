/*
 * events.c
 *
 *  Created on: May 1, 2015
 *      Author: bsnga
 */

#include "msp430.h"
#include "events.h"
#include "uart.h"
#include "interrupts.h"
#include "nrf24api.h"
#include "stdint.h"
#include <stdio.h>

volatile uint16_t sys_event = 0;
volatile mode dev_mode = DISCONNECTED;

// Receive event, triggered by IRQ receive event
void spi_rx_event() {
	char* data = 0;
	data = rx_fetch();
	print(data);
	sys_event |= UART_TX_EVENT;
	if (dev_mode != CONNECTED)
		connect_RF();
}

// Transmit event
void spi_tx_event() {
	char data[32] = { 0 };
	static char tx_count = 0;
	sprintf(data, "\n\r:%d", ++tx_count);
	if (!tx_send(data))
		sys_event |= PING_EVENT;
	rx_mode();
}

// Serial UART receive, triggered by UART RX interrupt
void uart_rx_event() {

}

// Serial UART transmit
void uart_tx_event() {
	char data[30] = { 0 };
	static int txcount = 0;
	sprintf(data, "\n\r:%d", ++txcount);
	print(data);
}

// Ping connection
void ping_event() {
#if PTX_DEV
	if (connect()) {
		connect_RF();
	} else {
		disconnect_RF();
	}
	rx_mode();
#else
	disconnect_RF();
#endif
}

void connect_RF() {
	P1OUT &= ~RLED;
	P1OUT |= GLED;
	dev_mode = CONNECTED;
}

// Send end connection
void disconnect_RF() {
	P1OUT &= ~GLED;
	P1OUT |= RLED;
	dev_mode = DISCONNECTED;
}
