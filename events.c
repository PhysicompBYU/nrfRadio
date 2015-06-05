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
#include "stdint.h"
#include <stdio.h>
#include "nrfradio.h"

volatile uint16_t sys_event = 0;

// Transmit event
void transmit_event() {
	uint8_t i = 0;
	static int counter = 0;
	char buffer[32];
	sprintf(buffer, "\n\r%d:%d", DEV, ++counter);
	while (buffer[i++]) {
	}
	if (radio_put_chars(!DEV, buffer, i))
		connected = 1;
	else
		connected = 0;
}

void recieve_event() {
	uint8_t data;
	while (radio_cget_char(DEV, &data)) {
		uart_putchar(data);
		connected = 1;
	}
}

void irq_event() {
	msprf24_get_irq_reason();
	if (rf_irq & RF24_IRQ_RX)
		sys_event |= RECIEVE_EVENT;
	if (rf_irq & (RF24_IRQ_TX | RF24_IRQ_TX))
		__no_operation();

}

// Serial UART receive, triggered by UART RX interrupt
void uart_rx_event() {

}

// Serial UART transmit
void uart_tx_event() {
}
