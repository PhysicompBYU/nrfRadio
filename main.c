/* main.c
 * Pin Mapping (if using USCI_A for SPI):
 *
 * MSP430 .....................	RF24
 * P1.1 MISO <----------------- MISO
 * P1.2 MOSI -----------------> MOSI
 * P1.4 SCLK -----------------> SCK
 * P2.0 CE -------------------> CE
 * P2.1 CSn ------------------> CSN
 * P2.2 IRQ <------------------ IRQ
 *
 * Additional Pins:
 * P1.0 LED0 (output)
 * P1.3 BTN0 (input)
 * P1.6 LED1 (output)
 */
/* Alternate Pin Mapping (if using USCI_B for SPI):
 *
 * MSP430 ................ nRF24L01+
 * P1.5 SCLK ------------> SCK
 * P1.6 MISO <------------ MISO
 * P1.7 MOSI ------------> MOSI
 * P2.0 CE --------------> CE
 * P2.1 CSn -------------> CSN
 * P2.2 IRQ <------------- IRQ
 *
 * Additional Pins:
 * P1.0 LED0 (output)
 * P1.3 BTN0 (input)
 * P1.4 LED1 (output)
 * IMPORTANT TODO: Note that since P1.6 is used for MISO, we have to
 * wire LED1 directly to P1.4.
 */

#include <msp430.h>
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "stdint.h"
#include "interrupts.h"
#include "uart.h"
#include "nrf24api.h"
#include "events.h"

void port1_init();

void main() {

	WDTCTL = WDTHOLD | WDTPW;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_1;  // SMCLK = DCOCLK/2
	// SPI (USCI) uses SMCLK, prefer SMCLK < 10MHz (SPI speed limit for nRF24 = 10MHz)

	port1_init();
	interrupts_WDT_init();
	uart_init();
	radio_init();
//
//#if PTX_DEV
//	open_stream(TX_MODE);
//#else
//	open_stream(RX_MODE);
//#endif

	while (1) {
		// disable interrupts while checking sys_event
		_disable_interrupts();

		// if event pending, enable interrupts
		if (sys_event || rf_irq)
			_enable_interrupt();

		// else enable interrupts and goto sleep
		else {
			__bis_SR_register(LPM1_bits | GIE);
		}

		if (rf_irq & RF24_IRQ_FLAGGED) {
			spi_rx_event();
		} else {

			if (sys_event & SPI_TX_EVENT) {
				sys_event &= ~SPI_TX_EVENT;
				spi_tx_event();
			} else if (sys_event & UART_RX_EVENT) {
				sys_event &= ~UART_RX_EVENT;
				uart_rx_event();
			} else if (sys_event & UART_TX_EVENT) {
				sys_event &= ~UART_TX_EVENT;
				uart_tx_event();
			} else if (sys_event & PING_EVENT) {
				sys_event &= ~PING_EVENT;
				ping_event();
			} else {
				P1OUT &= ~(RLED + GLED);
				while (1) {
					delay(50);
					P1OUT ^= RLED + GLED;
				}

			}
		}

	}
}

void port1_init(void) {

	P1DIR |= RLED + GLED;
	P1OUT &= ~(RLED + GLED);

	P1DIR &= ~(SWTCH0);
// configure P1 switch for interrupt
	P1SEL &= ~(SWTCH0);					// select GPIO
	P1OUT |= (SWTCH0);					// use pull-ups
	P1IES |= (SWTCH0);					// high to low transition
	P1REN |= (SWTCH0);					// Enable pull-ups
//	P1IE |= (SWTCH0);						// P1.0-3 interrupt enabled
//	P1IFG &= ~(SWTCH0);						// P1.0-3 IFG cleared
	return;
} // end port1_init
