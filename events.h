/*
 * events.h
 *
 *  Created on: May 1, 2015
 *      Author: bsnga
 */

#include "stdint.h"

#ifndef EVENTS_H_
#define EVENTS_H_

//enums
typedef enum {
	CONNECTED, DISCONNECTED
} mode;

//externs
extern volatile uint16_t sys_event;
extern volatile mode dev_mode;

// Event flags, in priority order
#define SPI_TX_EVENT	BIT0
#define SPI_RX_EVENT	BIT1
#define UART_RX_EVENT	BIT2
#define UART_TX_EVENT	BIT3
#define PING_EVENT		BIT4

// prototypes
void spi_rx_event();
void spi_tx_event();
void uart_rx_event();
void uart_tx_event();
void ping_event();
void connect_RF();
void disconnect_RF();

#endif /* EVENTS_H_ */
