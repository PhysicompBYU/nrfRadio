/*
 * nrf24api.h
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#ifndef NRF24API_H_
#define NRF24API_H_

#include "stdint.h"

// enums, typedefs
typedef enum {
	TX_MODE, RX_MODE
} RF_MODE;

typedef enum {
	INIT, TIMEOUT, CONNECTED, LISTEN
} NRF_STATE;

typedef struct {
	uint8_t size;
	uint8_t buf[32];
} BUFFER;

//function prototypes
void radio_init();
void open_stream(RF_MODE mode);
void recieve_bytes();
void transmit_bytes();
void reset_connected();
uint8_t is_connected();

//variables
extern volatile BUFFER buffer;

#endif /* NRF24API_H_ */
