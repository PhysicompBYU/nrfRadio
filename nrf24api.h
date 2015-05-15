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

// io api:	uint8_t read_byte(IO_THING thing)
// 			write_byte(IO_THING thing, uint8_t data)
typedef struct {
	uint8_t *buffer;
	uint16_t tail;
	uint16_t size;
	uint8_t has_next;
} IO_THING;


//defines
// CURR_CPS must be at least 500, which is 1 interrupt per 2 ms
#define CURR_CPS 500	//count every .5ms -> 2000 counts per sec


//function prototypes
void radio_init();
void open_stream(RF_MODE mode);
uint8_t recieve_bytes();
void transmit_bytes();
inline void reset_connected();
uint8_t is_connected();
void timestep_machine();
void IRQ_step_machine();

//variables
extern volatile uint16_t timestep_count;

#endif /* NRF24API_H_ */
