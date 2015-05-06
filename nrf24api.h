/*
 * nrf24api.h
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#ifndef NRF24API_H_
#define NRF24API_H_

#include "stdint.h"

void radio_init();
int16_t connect();
int16_t tx_send(char* data);
void rx_mode();
void disable_rx_mode();
char* rx_fetch();

// enum
typedef enum {
	TX_MODE,
	RX_MODE
}RF_MODE;

#endif /* NRF24API_H_ */
