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

volatile BUFFER buffer;
volatile unsigned int user;

//private globals
static char addr[5] = { 0 };
uint8_t payload_size = 0;

void transmit_bytes() {
	// size 0 indicates dynamic size; must be specified using
	//transmit_Xbytes(); 32 is max
	if (payload_size > 32)
		return;
	else if (payload_size == 0)
		w_tx_payload(buffer.size, buffer.buf);
	else
		w_tx_payload(payload_size, buffer.buf);

	msprf24_activate_tx();
}

void recieve_bytes() {
	if (payload_size > 0)
		buffer.size = payload_size;
	else
		buffer.size = r_rx_peek_payload_size();

	msprf24_get_irq_reason();
	if (rf_irq & RF24_IRQ_RX) {
		r_rx_payload(buffer.size, buffer.buf);
		msprf24_irq_clear(RF24_IRQ_RX);
		return;
	}
	buffer.size = 0;
	return;
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

void open_stream(RF_MODE mode) {

	if (mode == TX_MODE)
		open_tx_stream();
	else
		open_rx_stream();
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

