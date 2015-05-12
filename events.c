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

// Receive event, triggered by IRQ receive event
void spi_rx_event() {
	recieve_bytes();
	sys_event |= UART_TX_EVENT;
}

// Transmit event
void spi_tx_event() {
	char i = 0;
	static int tx_count = 0;
	sprintf(buffer.buf, "\n\r%d", ++tx_count);
	while (buffer.buf[i]) {
		i++;
	}
	buffer.size = i;
	transmit_bytes();
}

// Serial UART receive, triggered by UART RX interrupt
void uart_rx_event() {

}

// Serial UART transmit
void uart_tx_event() {
	print_x(buffer.buf, buffer.size);
}

// Ping connection
void ping_event() {
	if (1) {
		connect_RF();
	} else {
		disconnect_RF();
	}
}

inline void connect_RF() {
	P1OUT &= ~RLED;
	P1OUT |= GLED;
}

// Display connection lost
inline void disconnect_RF() {
	P1OUT &= ~GLED;
	P1OUT |= RLED;
}
