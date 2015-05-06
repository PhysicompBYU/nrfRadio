/*
 * nrf24api.c
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#include "msp430.h"
#include "nrf24api.h"
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "interrupts.h"
#include "stdint.h"

volatile char buf[32] = { 0 };
volatile unsigned int user;

//private globals
static char addr[5] = { 0 };
uint8_t payload_size = 0;

void transmit_bytes(char* data) {
	uint8_t i = 0;

	while (data[i]) {
		buf[i] = data[i];
		i++;
	}

	w_tx_payload(32, buf);
	msprf24_activate_tx();
}

void transmit_Xbytes(const char* data, char size){

}

char recieve_bytes() {
	msprf24_get_irq_reason();
	if (rf_irq & RF24_IRQ_RX) {
		r_rx_payload(32, buf);
		msprf24_irq_clear(RF24_IRQ_RX);
		return (char*) buf;
	}
	return 0;
}

void open_stream(RF_MODE mode) {

	if (mode == TX_MODE)
		return open_tx_stream();
	else
		return open_rx_stream();
}

void open_tx_stream() {
	msprf24_set_pipe_packetsize(0, 32);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	msprf24_standby();

//	user = msprf24_current_state();
}

void rx_mode() {
	// Receive mode
	if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
		flush_rx();
	}
	msprf24_activate_rx();

}

void open_rx_stream() {
	msprf24_set_pipe_packetsize(0, 32);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	msprf24_standby();
	rx_mode();
}

void radio_init() {
	user = 0xFE;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width = 5;
	rf_speed_power = RF24_SPEED_1MBPS | RF24_POWER_0DBM;
	rf_channel = 120;

// Set our RX address
	addr[0] = 0xDE;
	addr[1] = 0xAD;
	addr[2] = 0xBE;
	addr[3] = 0xEF;
	addr[4] = 0x00;

	msprf24_init();
	w_tx_addr(addr);
	w_rx_addr(0, addr); // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
// needs to listen to the TX addr on pipe#0 to receive them.
}

