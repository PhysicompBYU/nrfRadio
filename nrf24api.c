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
#include "msp430_spi.h"

// private defines
#define CSN_EN nrfCSNportout &= ~nrfCSNpin
#define CSN_DIS nrfCSNportout |= nrfCSNpin
#define CE_EN nrfCEportout |= nrfCEpin
#define CE_DIS nrfCEportout &= ~nrfCEpin

#define MAX_TIMEOUT (CURR_CPS / 2)	//2000 ms
#define DELTA_TIMEOUT (CURR_CPS / 20)
#define TO_INIT (CURR_CPS / 500)
#define CONNECTED_TIMEOUT (CURR_CPS / 6)
#define LISTEN_TIMEOUT (CONNECTED_TIMEOUT * 2)

#define CMD_REQ 0x96
#define CONNECT_REQ 0x55
#define ERROR while(1){P1OUT^=(RLED|GLED);delay(50);}

typedef enum {
	INIT, TIMEOUT, CONNECTED, LISTEN
} NRF_STATE;

static const uint8_t pipe_addr[6][5] = { { 0xDE, 0xAD, 0xBE, 0xEF, 0x00 }, {
		0xFA, 0xDE, 0xDA, 0xCE, 0x51 }, { 0xFA, 0xDE, 0xDA, 0xCE, 0x52 }, {
		0xFA, 0xDE, 0xDA, 0xCE, 0x53 }, { 0xFA, 0xDE, 0xDA, 0xCE, 0x54 }, {
		0xFA, 0xDE, 0xDA, 0xCE, 0x55 } };
static const uint8_t byte_mask[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40,
		0x80 };
//globals
volatile uint16_t timestep_count;
static volatile uint8_t timestep_delay = 0;

NRF_STATE nrf_state;

static PIPE* pipes;

// RX status variables
static uint8_t tx_assigned = 0;
static uint8_t tx_active = 0;
static uint8_t tx_pend = 0;
static uint8_t pend_bytes[6] = { 0 };

// TX status variables
static uint8_t commands[6] = { 0 };

// private functions
void set_state(NRF_STATE new_state);
void set_tx_addr(uint8_t pipe);
void set_rx_addr(uint8_t pipe, const uint8_t *addr);

void timestep_machine() {
	switch (nrf_state) {
	case TIMEOUT:
		if (timestep_delay < MAX_TIMEOUT) {
			timestep_delay += DELTA_TIMEOUT;
		}
		command = CONNECT_REQ;
		transmit_bytes();
		timestep_count = timestep_delay;
	case CONNECTED:
		if (!timestep_count) {
			command = CMD_REQ;
			timestep_count = timestep_delay;
		}
		transmit_bytes();
		break;
	case LISTEN:

		break;
	default:
		break;
	}

	return;
}

void IRQ_step_machine() {
	uint8_t pipe;
	int i;
	msprf24_get_irq_reason();

	switch (nrf_state) {
	case TIMEOUT:
		if (rf_irq & RF24_IRQ_RX) {
			recieve_bytes();
			set_state(TIMEOUT);
		}
		break;
	case CONNECTED:
		if (rf_irq & RF24_IRQ_RX) {
			recieve_bytes();
		}
		break;
	case LISTEN:
		static uint8_t count = 0;
		static uint8_t marker = 0;
		if (rf_irq & RF24_IRQ_RX) {
			pipe = recieve_bytes();

			if (rf_irq & RF24_IRQ_TX) {
				if (pipe) {
					for (pend_bytes[pipe]; pend_bytes[pipe] > 0; //1 less than the size in pend_bytes
							pend_bytes[pipe]--)
						read_byte(tx_buf[pipe]);
				}
				pend_bytes[pipe] = 0;
				commands[pipe] = 0;
				count--;
			} else if (count) {
				count = 0;
				flush_tx();
				memset(pend_bytes, 0, sizeof(pend_bytes));
			}
			for (i = 1; i < 7; i++) {
				uint8_t j =
						marker + (marker + i < 6) ? marker + i : marker + i - 6;
				if (j) {
					if (commands[j] || tx_buf[j]->size) {
						pend_bytes[j] = 1 + transmit_bytes(j);
					}
				} else {
					if (commands[0]) {
						pend_bytes[0] = 1;
						transmit_bytes(0);
					}
				}
				count++;
				if (count >= 3)
					break;
			}
		}
		break;
	default:
		ERROR
		;
		break;
	}
	return;
}

void set_state(NRF_STATE new_state) {
	nrf_state = new_state;
	TAR = 0;
	switch (new_state) {
	case INIT:

		break;
	case TIMEOUT:
		timestep_delay = TO_INIT;
		timestep_count = TO_INIT;
		break;
	case CONNECTED:
		timestep_delay = CONNECTED_TIMEOUT;
		timestep_count = CONNECTED_TIMEOUT;
		break;
	case LISTEN:
		timestep_delay = LISTEN_TIMEOUT;
		timestep_count = LISTEN_TIMEOUT;
		break;
	default:
		ERROR
		;
		break;
	}
	return;
}

void transmit_bytes(uint8_t pipe_dest) {
	switch (nrf_state) {
	case TIMEOUT:
	case CONNECTED:
		uint8_t size = tx_buf[0]->size;
		if (size > 31)
			size = 31;
		write_payload(size);
		msprf24_activate_tx();
		break;
	case LISTEN:
		if (pipe) {
			size = (size < 32) ? size : 31;
		} else
			size = 0;
		write_ack_payload(pipe_dest, size);
		break;
	default:
		ERROR
		;
		break;
	}
}

// Write to TX_FIFO, based on Spirilis' API function w_tx_payload()
void write_payload(uint8_t size) {
	uint16_t i;

	CSN_EN;
	rf_status = spi_transfer(RF24_W_TX_PAYLOAD);
	spi_transfer(commands[0]);
	for (i = 0; i < len; i++)
		spi_transfer(read_byte(tx_buf[0]));
	CSN_DIS;
}
void write_ack_payload(uint8_t pipe, uint8_t len) {
	uint16_t i = 0;
	CSN_EN;

	if (pipe > 5)
		return;
	if (!(rf_feature & RF24_EN_ACK_PAY))  // ACK payloads must be enabled...
		return;

	rf_status = spi_transfer(RF24_W_ACK_PAYLOAD | pipe);
	spi_transfer(commands[pipe]);
	for (i = len - 1; i; i--) {
		spi_transfer(tx_buf[pipe]);
	}
	CSN_DIS;
}

// Recieves packets, loading into buffer.buf.  buffer.size contains
// size of payload, 0 if none recieved succesfully.
uint8_t recieve_bytes() {
	uint8_t pipe;
	while (rf_irq & RF24_IRQ_RX) {
		pipe = read_payload();
		msprf24_irq_clear(RF24_IRQ_RX);
	}
	return pipe;
}

// Read from RX FIFO, based on Spirilis' API function r_rx_payload()
uint8_t read_payload() {
	uint16_t i = 0, j;
	uint8_t pipe;
	uint8_t len = r_rx_peek_payload_size();

	CSN_EN;
	rf_status = spi_transfer(RF24_R_RX_PAYLOAD);
	command = spi_transfer(0xFF);
	pipe = (rf_status & 0x0E) >> 1;

	for (i = len - 1; i; i--)
		write_byte(rx_buf[pipe], spi_transfer(0xFF));

	CSN_DIS;
	// The RX pipe this data belongs to is stored in STATUS
	return pipe;
}

PIPE* open_pipe(uint8_t pipe) {
	PIPE* new_pipe = (PIPE*) malloc(sizeof(PIPE));
	if (!new_pipe) {
		ERROR;
	}

	msprf24_set_pipe_packetsize(pipe, 0);
	msprf24_open_pipe(pipe, 1);  // Open pipe#0 with Enhanced ShockBurst



}

void open_tx_stream() {

	set_tx_addr(0);
	set_rx_addr(0, pipe_addr[0]);

	pipes[0] = open_pipe(0);

	msprf24_standby();

//	user = msprf24_current_state();
}

void open_rx_stream() {

	set_rx_addr(0, pipe_addr[0]);
	msprf24_set_pipe_packetsize(0, 0);
	set_rx_addr(1, pipe_addr[1]);
	msprf24_set_pipe_packetsize(1, 0);
	set_rx_addr(2, pipe_addr[2]);
	msprf24_set_pipe_packetsize(2, 0);
	set_rx_addr(3, pipe_addr[3]);
	msprf24_set_pipe_packetsize(3, 0);
	set_rx_addr(4, pipe_addr[4]);
	msprf24_set_pipe_packetsize(4, 0);
	set_rx_addr(5, pipe_addr[5]);
	msprf24_set_pipe_packetsize(5, 0);

	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	msprf24_standby();
	// Receive mode
	if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
		flush_rx();
	}
	msprf24_activate_rx();
}

void radio_init() {
	nrf_state = INIT;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width = 5;
	rf_speed_power = RF24_SPEED_2MBPS | RF24_POWER_0DBM;
	rf_channel = 120;

	msprf24_init();
	msprf24_enable_feature(RF24_EN_ACK_PAY);

}

uint8_t set_tx_addr(uint8_t pipe) {
	if (pipe > 5)
		return 0;
	const uint8_t *addr = pipe_addr[pipe];
	int i;

	CSN_EN;
	rf_status = spi_transfer(RF24_TX_ADDR | RF24_W_REGISTER);
	for (i = rf_addr_width - 1; i >= 0; i--) {
		spi_transfer(addr[i]);
	}
	CSN_DIS;
	return 1;
}

void set_rx_addr(uint8_t pipe, const uint8_t *addr) {
	int i;

	if (pipe > 5)
		return;  // Only 6 pipes available
	CSN_EN;
	rf_status = spi_transfer((RF24_RX_ADDR_P0 + pipe) | RF24_W_REGISTER);
	if (pipe > 1) { // Pipes 2-5 differ from pipe1's addr only in the LSB.
		spi_transfer(addr[rf_addr_width - 1]);
	} else {
		for (i = rf_addr_width - 1; i >= 0; i--) {
			spi_transfer(addr[i]);
		}
	}
	CSN_DIS;
}
