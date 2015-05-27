/*
 * nrfradio.h
 *
 *  Created on: May 18, 2015
 *      Author: Broderick Gardner
 */

#ifndef NRFRADIO_H_
#define NRFRADIO_H_

#include <stdint.h>
#include "nRF24L01.h"

// Typedefs
typedef enum {
	RCV,TX
}pipe_mode;
typedef struct{

} IO_buffer;

// available functions
int init_radio(int freq, int speed);
int radio_listen(int pipe, void (*func)(void));
int radio_get_char(int pipe, char* data);
int radio_cget_char(int pipe, char* data);
int radio_put_char(int pipe, char data);
int radio_flush_pipe(int pipe);
int radio_close_pipe(int pipe);
int radio_open_pipe(int_pipe);

// Variables
/* Configuration variables used to tune RF settings during initialization and for
 * runtime reconfiguration.  You should define all 4 of these before running msprf24_init();
 */
extern uint8_t rf_crc;
extern uint8_t rf_addr_width;
extern uint8_t rf_speed_power;
extern uint8_t rf_channel;
/* Status variable updated every time SPI I/O is performed */
extern uint8_t rf_status;
/* Test this against RF24_IRQ_FLAGGED to see if the nRF24's IRQ was raised; it also
 * holds the last recorded IRQ status from msprf24_irq_get_reason();
 */
extern volatile uint8_t rf_irq;

/* Settings for 16MHz MCLK */
#define DELAY_CYCLES_5MS       80000
#define DELAY_CYCLES_130US     2080
#define DELAY_CYCLES_15US      240

/* IRQ */
#define NRFIRQPORTOUT
#define nrfIRQpin BIT2 // P2.2

/* CSN SPI chip-select */
#define nrfCSNport 2
#define nrfCSNportout P2OUT
#define nrfCSNpin BIT1 // P2.1

/* CE Chip-Enable (used to put RF transceiver on-air for RX or TX) */
#define nrfCEport 2
#define nrfCEportout P2OUT
#define nrfCEpin BIT0 // P2.0

/* RF speed settings -- nRF24L01+ compliant, older nRF24L01 does not have 2Mbps. */
#define RF24_SPEED_250KBPS  0x20
#define RF24_SPEED_1MBPS    0x00
#define RF24_SPEED_2MBPS    0x08
#define RF24_SPEED_MAX      RF24_SPEED_2MBPS
#define RF24_SPEED_MIN      RF24_SPEED_250KBPS
#define RF24_SPEED_MASK     0x28

/* RF transmit power settings */
#define RF24_POWER_7DBM        0x07
    // ^ 7dBm available with SI24R1 Taiwanese knockoff modules
#define RF24_POWER_0DBM        0x06
#define RF24_POWER_MINUS6DBM   0x04
#define RF24_POWER_MINUS12DBM  0x02
#define RF24_POWER_MINUS18DBM  0x00
#define RF24_POWER_MAX         RF24_POWER_0DBM
#define RF24_POWER_MIN         RF24_POWER_MINUS18DBM
#define RF24_POWER_MASK        0x07

/* Available states for the transceiver's state machine */
#define RF24_STATE_NOTPRESENT  0x00
#define RF24_STATE_POWERDOWN   0x01
#define RF24_STATE_STANDBY_I   0x02
#define RF24_STATE_STANDBY_II  0x03
#define RF24_STATE_PTX         0x04
#define RF24_STATE_PRX         0x05
#define RF24_STATE_TEST        0x06

/* IRQ "reasons" that can be tested. */
#define RF24_IRQ_TXFAILED      0x10
#define RF24_IRQ_TX            0x20
#define RF24_IRQ_RX            0x40
#define RF24_IRQ_MASK          0x70
// Bit 7 used to signify that the app should check IRQ status, without
// wasting time in the interrupt vector trying to do so itself.
#define RF24_IRQ_FLAGGED       0x80

/* Queue FIFO states that can be tested. */
#define RF24_QUEUE_TXFULL      RF24_FIFO_FULL
#define RF24_QUEUE_TXEMPTY     RF24_TX_EMPTY
#define RF24_QUEUE_RXFULL      RF24_RX_FULL
#define RF24_QUEUE_RXEMPTY     RF24_RX_EMPTY

#endif /* NRFRADIO_H_ */
