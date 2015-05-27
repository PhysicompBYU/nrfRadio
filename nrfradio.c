/*
 * nrfradio.c
 *
 *  Created on: May 18, 2015
 *      Author: Broderick Gardner
 *      based heavily on code by Spirilis
 *
 */

#include "nrfradio.h"

/* Configuration parameters used to set-up the RF configuration */
uint8_t rf_crc;
uint8_t rf_addr_width;
uint8_t rf_speed_power;
uint8_t rf_channel;
/* Status variable updated every time SPI I/O is performed */
uint8_t rf_status;
/* IRQ state is stored in here after msprf24_get_irq_reason(), RF24_IRQ_FLAGGED raised during
 * the IRQ port ISR--user application issuing LPMx sleep or polling should watch for this to
 * determine if the wakeup reason was due to nRF24 IRQ.
 */
volatile uint8_t rf_irq;

// private variables
static IO_buffer* rx_pipe_buffers[6];
static IO_buffer* tx_pipe_buffers[6];

// Initialize radio, pass NULL for default values:
// frequency=		speed=
int init_radio(int freq, int speed) {
	const speeds[3] = { RF24_SPEED_250KBPS, RF24_SPEED_1MBPS, RF24_SPEED_2MBPS };

	port_init();

	if (freq)
		rf_channel = freq;
	else
		rf_channel = 120;

	rf_speed_power = speeds[speed] | RF24_POWER_0DBM;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width = 5;

	// Wait 100ms for RF transceiver to initialize.
	uint8_t c = 20;
	for (; c; c--) {
		__delay_cycles(DELAY_CYCLES_5MS);
	}

	// Configure RF transceiver with current value of rf_* configuration variables
	msprf24_irq_clear(RF24_IRQ_MASK);  // Forget any outstanding IRQs
	msprf24_close_pipe_all(); /* Start off with no pipes enabled, let the user open as needed.  This also
	 * clears the DYNPD register.
	 */
	msprf24_set_retransmit_delay(500);  // A default I chose
	msprf24_set_retransmit_count(10);    // A default I chose
	msprf24_set_speed_power();
	msprf24_set_channel();
	msprf24_set_address_width();
	rf_feature = 0x00;  // Initialize this so we're starting from a clean slate
	msprf24_enable_feature(RF24_EN_DPL); // Dynamic payload size capability (set with msprf24_set_pipe_packetsize(x, 0))
	msprf24_enable_feature(RF24_EN_DYN_ACK); // Ability to use w_tx_payload_noack()

	msprf24_powerdown();
	flush_tx();
	flush_rx();

	// the following line enables ack payloads
	//msprf24_enable_feature(RF24_EN_ACK_PAY);
}

// ... open given pipe
int radio_open_pipe(int pipe, pipe_mode mode) {

	uint8_t rxen, enaa;

	if (pipe > 5)
		return;

	rxen = r_reg(RF24_EN_RXADDR);
	enaa = r_reg(RF24_EN_AA);

	if (autoack)
		enaa |= (1 << pipeid);
	else
		enaa &= ~(1 << pipeid);
	rxen |= (1 << pipeid);
	w_reg(RF24_EN_RXADDR, rxen);
	w_reg(RF24_EN_AA, enaa);

	if(pipe_mode == RCV){
		rx_pipe_buffers[pipe];
	}else{

	}
}

// close given pipe
int radio_close_pipe(int pipe) {

}

// Non blocking listen, callback function called upon recieve on given pipe
int radio_listen(int pipe, void (*func)(void)) {

}

// Blocking get_char, returns char sent by given pipe
int radio_get_char(int pipe, char* data) {

}

// Conditional get_char returns 0 if no data available from given pipe
int radio_cget_char(int pipe, char* data) {

}

// Put char on buffer to transmit to given pipe
int radio_put_char(int pipe, char data) {

}

// Flush the given pipe (force send any data in the buffer)
int radio_flush_pipe(int pipe) {

}

/* Clear IRQ flags */
void msprf24_irq_clear(uint8_t irqflag) {
	uint8_t fifostat;

	rf_irq = 0x00; // Clear IRQs; afterward analyze RX FIFO to see if we should re-set RX IRQ flag.
	CSN_EN;
	rf_status = spi_transfer(RF24_STATUS | RF24_W_REGISTER);
	spi_transfer(irqflag);
	CSN_DIS;

	// Per datasheet procedure, check FIFO_STATUS to see if there's more RX FIFO data to process.
	if (irqflag & RF24_IRQ_RX) {
		CSN_EN;
		rf_status = spi_transfer(RF24_FIFO_STATUS | RF24_R_REGISTER);
		fifostat = spi_transfer(RF24_NOP);
		CSN_DIS;
		if (!(fifostat & RF24_RX_EMPTY))
			rf_irq |= RF24_IRQ_RX | RF24_IRQ_FLAGGED; // Signal to user that there is remaining data, even if it's not "new"
	}
}

void msprf24_set_retransmit_delay(uint16_t us) {
	uint8_t c;

	// using 'c' to evaluate current RF speed
	c = rf_speed_power & RF24_SPEED_MASK;
	if (us > 4000)
		us = 4000;
	if (us < 1500 && c == RF24_SPEED_250KBPS)
		us = 1500;
	if (us < 500)
		us = 500;

	// using 'c' to save current value of ARC (auto-retrans-count) since we're not changing that here
	c = r_reg(RF24_SETUP_RETR) & 0x0F;
	us = (us - 250) / 250;
	us <<= 4;
	w_reg(RF24_SETUP_RETR, c | (us & 0xF0));
}

// Power down device, 0.9uA power draw
void msprf24_powerdown() {
	CE_DIS;
	msprf24_set_config(0);  // PWR_UP=0
}

// Enable Standby-I, 26uA power draw
void msprf24_standby() {
	uint8_t state = msprf24_current_state();
	if (state == RF24_STATE_NOTPRESENT || state == RF24_STATE_STANDBY_I)
		return;
	CE_DIS;
	msprf24_set_config(RF24_PWR_UP);  // PWR_UP=1, PRIM_RX=0
	if (state == RF24_STATE_POWERDOWN) { // If we're powering up from deep powerdown...
		//CE_EN;  // This is a workaround for SI24R1 chips, though it seems to screw things up so disabled for now til I can obtain an SI24R1 for testing.
		__delay_cycles(DELAY_CYCLES_5MS); // Then wait 5ms for the crystal oscillator to spin up.
		//CE_DIS;
	}
}

void msprf24_set_retransmit_count(uint8_t count) {
	uint8_t c;

	c = r_reg(RF24_SETUP_RETR) & 0xF0;
	w_reg(RF24_SETUP_RETR, c | (count & 0x0F));
}

uint8_t msprf24_get_last_retransmits() {
	return r_reg(RF24_OBSERVE_TX) & 0x0F;
}

uint8_t msprf24_get_lostpackets() {
	return (r_reg(RF24_OBSERVE_TX) >> 4) & 0x0F;
}

void msprf24_set_address_width() {
	if (rf_addr_width < 3 || rf_addr_width > 5)
		return;
	w_reg(RF24_SETUP_AW, ((rf_addr_width - 2) & 0x03));
}

void msprf24_set_channel() {
	if (rf_channel > 125)
		rf_channel = 0;
	w_reg(RF24_RF_CH, (rf_channel & 0x7F));
}

void msprf24_set_speed_power() {
	if ((rf_speed_power & RF24_SPEED_MASK) == RF24_SPEED_MASK) // Speed setting RF_DR_LOW=1, RF_DR_HIGH=1 is reserved, clamp it to minimum
		rf_speed_power = (rf_speed_power & ~RF24_SPEED_MASK) | RF24_SPEED_MIN;
	w_reg(RF24_RF_SETUP, (rf_speed_power & 0x2F));
}

uint8_t msprf24_set_config(uint8_t cfgval) {
	uint8_t previous_config;

	previous_config = r_reg(RF24_CONFIG);
	w_reg(RF24_CONFIG, (_msprf24_crc_mask() | cfgval) & _msprf24_irq_mask());
	return previous_config;
}

inline void port_init() {
#if nrfIRQport == 1
	P1DIR &= ~nrfIRQpin;  // IRQ line is input
	P1OUT |= nrfIRQpin;// Pull-up resistor enabled
	P1REN |= nrfIRQpin;
	P1IES |= nrfIRQpin;// Trigger on falling-edge
	P1IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P1IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 2
	P2DIR &= ~nrfIRQpin;  // IRQ line is input
	P2OUT |= nrfIRQpin;// Pull-up resistor enabled
	P2REN |= nrfIRQpin;
	P2IES |= nrfIRQpin;// Trigger on falling-edge
	P2IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P2IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 3
	P3DIR &= ~nrfIRQpin;  // IRQ line is input
	P3OUT |= nrfIRQpin;// Pull-up resistor enabled
	P3REN |= nrfIRQpin;
	P3IES |= nrfIRQpin;// Trigger on falling-edge
	P3IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P3IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 4
	P4DIR &= ~nrfIRQpin;  // IRQ line is input
	P4OUT |= nrfIRQpin;// Pull-up resistor enabled
	P4REN |= nrfIRQpin;
	P4IES |= nrfIRQpin;// Trigger on falling-edge
	P4IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P4IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 5
	P5DIR &= ~nrfIRQpin;  // IRQ line is input
	P5OUT |= nrfIRQpin;// Pull-up resistor enabled
	P5REN |= nrfIRQpin;
	P5IES |= nrfIRQpin;// Trigger on falling-edge
	P5IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P5IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 6
	P6DIR &= ~nrfIRQpin;  // IRQ line is input
	P6OUT |= nrfIRQpin;// Pull-up resistor enabled
	P6REN |= nrfIRQpin;
	P6IES |= nrfIRQpin;// Trigger on falling-edge
	P6IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P6IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 7
	P7DIR &= ~nrfIRQpin;  // IRQ line is input
	P7OUT |= nrfIRQpin;// Pull-up resistor enabled
	P7REN |= nrfIRQpin;
	P7IES |= nrfIRQpin;// Trigger on falling-edge
	P7IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P7IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 8
	P8DIR &= ~nrfIRQpin;  // IRQ line is input
	P8OUT |= nrfIRQpin;// Pull-up resistor enabled
	P8REN |= nrfIRQpin;
	P8IES |= nrfIRQpin;// Trigger on falling-edge
	P8IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P8IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 9
	P9DIR &= ~nrfIRQpin;  // IRQ line is input
	P9OUT |= nrfIRQpin;// Pull-up resistor enabled
	P9REN |= nrfIRQpin;
	P9IES |= nrfIRQpin;// Trigger on falling-edge
	P9IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P9IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 10
	P10DIR &= ~nrfIRQpin;  // IRQ line is input
	P10OUT |= nrfIRQpin;// Pull-up resistor enabled
	P10REN |= nrfIRQpin;
	P10IES |= nrfIRQpin;// Trigger on falling-edge
	P10IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
	P10IE |= nrfIRQpin;// Enable IRQ interrupt
#endif

	// Setup CSN/CE ports
#if nrfCSNport == 1
	P1DIR |= nrfCSNpin;
#elif nrfCSNport == 2
	P2DIR |= nrfCSNpin;
#elif nrfCSNport == 3
	P3DIR |= nrfCSNpin;
#elif nrfCSNport == 4
	P4DIR |= nrfCSNpin;
#elif nrfCSNport == 5
	P5DIR |= nrfCSNpin;
#elif nrfCSNport == 6
	P6DIR |= nrfCSNpin;
#elif nrfCSNport == 7
	P7DIR |= nrfCSNpin;
#elif nrfCSNport == 8
	P8DIR |= nrfCSNpin;
#elif nrfCSNport == 9
	P9DIR |= nrfCSNpin;
#elif nrfCSNport == 10
	P10DIR |= nrfCSNpin;
#elif nrfCSNport == J
	PJDIR |= nrfCSNpin;
#endif
	CSN_DIS;

#if nrfCEport == 1
	P1DIR |= nrfCEpin;
#elif nrfCEport == 2
	P2DIR |= nrfCEpin;
#elif nrfCEport == 3
	P3DIR |= nrfCEpin;
#elif nrfCEport == 4
	P4DIR |= nrfCEpin;
#elif nrfCEport == 5
	P5DIR |= nrfCEpin;
#elif nrfCEport == 6
	P6DIR |= nrfCEpin;
#elif nrfCEport == 7
	P7DIR |= nrfCEpin;
#elif nrfCEport == 8
	P8DIR |= nrfCEpin;
#elif nrfCEport == 9
	P9DIR |= nrfCEpin;
#elif nrfCEport == 10
	P10DIR |= nrfCEpin;
#elif nrfCEport == J
	PJDIR |= nrfCEpin;
#endif
	CE_DIS;

	/* Straw-man spi_transfer with no Chip Select lines enabled; this is to workaround errata bug USI5
	 * on the MSP430G2452 and related (see http://www.ti.com/lit/er/slaz072/slaz072.pdf)
	 * Shouldn't hurt anything since we expect no CS lines enabled by the user during this function's execution.
	 */
	spi_transfer(RF24_NOP);
}
