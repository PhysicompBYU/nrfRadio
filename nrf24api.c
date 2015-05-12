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

NRF_STATE nrf_state;
static uint8_t tx_assigned = 0;
static uint8_t tx_active = 0;
static uint8_t tx_pend = 0;
static uint8_t pipe_set = 0;
static uint8_t tiemstep_delay = 0;

void timestep_machine() {
	switch (nrf_state) {
	case TIMEOUT:
		if(TACCR0 == timestep_delay){
			TACCR0 = HR_DELAY;
		}else{
			timestep_delay += DELTA_TIMEOUT;
			TACCR0 = timestep_delay;
		}
		break;
	case CONNECTED:

		break;
		default;
	}
	buffer.buf[0] = CMD_REQUEST;
	buffer.buf[1] = ~CMD_REQUEST;
	buffer.size = 2;
	transmit_bytes();
	return;
}

void interrstep_machine() {

	return;
}

void set_state(NRF_STATE new_state) {
	nrf_state = new_state;
	TAR = 0;
	switch (new_state) {
	case INIT:
		TACCR0 = 0;
		break;
	case TIMEOUT:
		timestep_delay = TO_INIT;
		TACCR0 = TO_INIT;
		break;
	case CONNECTED:
		TACCR0 = CONNECTED_TIMEOUT;
		break;
	case LISTEN:

		break;
	default:
	}
	return;
}

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

// Recieves packets, loading into buffer.buf.  buffer.size contains
// size of payload, 0 if none recieved succesfully.
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
	} else if (rf_irq & RF24_IRQ_TX) {
	} else if (rf_irq & RF24_IRQ_TXFAILED) {
	}
	msprf24_irq_clear(RF24_IRQ_RX);
	buffer.size = 0;
	return;
}

void open_tx_stream() {
	msprf24_set_pipe_packetsize(0, 0);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	msprf24_standby();

//	user = msprf24_current_state();
}

void open_rx_stream() {
	msprf24_set_pipe_packetsize(0, 0);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	msprf24_standby();
	// Receive mode
	if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
		flush_rx();
	}
	msprf24_activate_rx();
}

void open_stream(RF_MODE mode) {

	if (mode == TX_MODE)
		open_tx_stream();
	else
		open_rx_stream();
}

void radio_init() {
	nrf_state = INIT;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width = 5;
	rf_speed_power = RF24_SPEED_2MBPS | RF24_POWER_0DBM;
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

	//crystal init
	BCSCTL1 |= DIVA0 | DIVA1;	// Set ACLK divider to 8
	BCSCTL3 |= XCAP0 | XCAP1;	// Set crystal capacitors to 12.5 pF
	P2DIR &= ~BIT6;		// Set P2.6/7 to XIN/XOUT
	P2DIR |= BIT7;
	P2SEL |= BIT6 | BIT7;
	P2SEL2 &= ~(BIT6 | BIT7);
	while (BCSCTL3 & LFXT1OF)
		;		// wait for crystal to stabilize

	//initialize timerA
	P1OUT &= ~(GLED | RLED);
	TACCR0 = 0;
	TACTL = TASSEL_1 | ID_2 | MC_1;
	TACCTL0 |= CCIE;
	TACCTL1 |= CCIE;
	TAR = 400;
	TACCR1 = 800;
	TACCR0 = 1024;
}

// Timer A1 interrupt service routine
//
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0A1_ISR(void) {
	TACCTL1 &= ~CCIFG;
	P1OUT ^= RLED + GLED;
	return;
} // end TIMERA0_VECTOR
// Timer A1 interrupt service routine
//
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0A0_ISR(void) {
	P1OUT ^= RLED + GLED;
	return;
} // end TIMERA0_VECTOR

