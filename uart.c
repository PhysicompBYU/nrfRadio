/*
 * uart.c
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#include <msp430.h>
#include "uart.h"
#include "events.h"
#include <stdint.h>
#include <string.h>

#if !defined(__MSP430_HAS_USCI__)
#error "This code written for the msp430g2553"
#endif

unsigned long SMCLK_HZ = 8000000;			// SMCLK frequency in Hz

unsigned long baud_rate_20_bits;		// Bit rate divisor
unsigned int count;

uint16_t tail = 0;
uint16_t size = 0;
char txbuffer[32] = { 0 };

void uart_init() {
	memset(txbuffer, 0, sizeof(txbuffer));

	// Configure P1.1 and P1.2 as UART controlled pins
	P1DIR &= ~(BIT1 | BIT2);                  // Revert to default to GPIO input
	P1SEL = BIT1 | BIT2;                            // P1.1=RXD, P1.2=TXD
	P1SEL2 = BIT1 | BIT2;                           // P1.1=RXD, P1.2=TXD
	// Configure USCI UART for BPS (9600)
#if BPS == 57600
	UCA0CTL1 = UCSWRST;
	UCA0CTL0 = UCSPB;
	UCA0BR0 = 8;
	UCA0BR1 = 0;
	UCA0MCTL = 11 << 4 | 0 << 1 | UCOS16;
	UCA0CTL1 = UCSSEL_2;
#else
	baud_rate_20_bits = (SMCLK_HZ + (BPS >> 1)) / BPS;
	UCA0CTL1 = UCSWRST;             // Hold USCI in reset to allow configuration
	UCA0CTL0 = UCSPB; // No parity, LSB first, 8 bits, one stop bit, UART (async)
	UCA0BR1 = (baud_rate_20_bits >> 12) & 0xFF;    // High byte of whole divisor
	UCA0BR0 = (baud_rate_20_bits >> 4) & 0xFF;      // Low byte of whole divisor
	UCA0MCTL = ((baud_rate_20_bits << 4) & 0xF0) | UCOS16; // Fractional divisor, over sampling mode
	UCA0CTL1 = UCSSEL_2; // Use SMCLK for bit rate generator, then release reset

#endif

	IE2 |= UCA0RXIE; // enable rx interrupt (echoing)

}

//------------------------------------------------------------------------------
// Inserts char into UART transmit buffer.  Returns 1 if succesful, returns 0 if
// byte overwritten (buffer full)
int uart_putchar(uint8_t c) {
	uint16_t head = (tail + size) & ~TXBUFSIZE;
	if (++size < TXBUFSIZE) {
		txbuffer[head] = c;
		head = (head + 1) & ~TXBUFSIZE;
		head = 1;
	} else {
		size--;
		txbuffer[head] = c;
		tail = (tail + 1) & ~TXBUFSIZE;
		head = (head + 1) & ~TXBUFSIZE;
		head = 0;
	}
	EN_TXIE;

	return head;
}

//------------------------------------------------------------------------------
uint8_t uart_getchar(void) {
	while (!(IFG2 & UCA0RXIFG))
		;
	IFG2 &= ~UCA0RXIFG;
	return UCA0RXBUF;
}

//------------------------------------------------------------------------------
void print(const char *s) {
	while (*s)
		uart_putchar(*s++);
}

void print_x(uint8_t *s, uint8_t size) {
	while (size--)
		uart_putchar(*s++);
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
	if (size == 0) {
		DEN_TXIE;
		return;
	}
	UCA0TXBUF = txbuffer[tail];
	tail = (tail + 1) & ~TXBUFSIZE;
	if (--size == 0)
		DEN_TXIE;
}

/*  Echo    back    RXed    character,  confirm TX  buffer  is  ready   first   */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
	uart_putchar(UCA0RXBUF);
}

