/*
 * nrf24api.h
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#ifndef NRF24API_H_
#define NRF24API_H_

#include "stdint.h"

// enum
typedef enum {
	TX_MODE, RX_MODE
} RF_MODE;

//function prototypes
void radio_init();
void rx_mode();
void open_stream(RF_MODE mode);
void recieve_bytes();
void transmit_bytes();

typedef struct {
	uint8_t size;
	uint8_t buf[32];
} BUFFER;

//variables
extern volatile BUFFER buffer;

#endif /* NRF24API_H_ */
