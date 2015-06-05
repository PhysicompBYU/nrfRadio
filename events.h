/*
 * events.h
 *
 *  Created on: May 1, 2015
 *      Author: bsnga
 */

#include "stdint.h"

#ifndef EVENTS_H_
#define EVENTS_H_

//externs
extern volatile uint16_t sys_event;

// Event flags, in priority order
#define TRANSMIT_EVENT		BIT0
#define RECIEVE_EVENT	BIT1
#define UART_RX_EVENT	BIT2
#define UART_TX_EVENT	BIT3
#define PING_EVENT		BIT4
#define IRQ_EVENT		BIT5

#define DEV 0

// prototypes
void transmit_event();
void recieve_event();
void irq_event();
void uart_rx_event();
void uart_tx_event();
inline void connect_RF();
inline void disconnect_RF();

#endif /* EVENTS_H_ */
