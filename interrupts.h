/*
 * interrupts.h
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <stdint.h>

#define myCLOCK	16000000
#define WDT_CLOCK 8000000
#define WDT_INT	500
#define	WDT_CTL	WDT_MDLY_0_5		//
#define	WDT_CPS	(WDT_CLOCK/WDT_INT)	// WD clocks / second count = WDT interrupts / second (500 @16MHz clk)
#define HALF_SECOND (WDT_CPS / 2)

#define DELAY WDT_CPS
#define DATA_DELAY WDT_CPS/100

#define GLED BIT4
#define RLED BIT0
#define SWTCH0 BIT3

extern volatile uint16_t timeout;
extern volatile uint16_t tics;

void interrupts_WDT_init();
uint32_t interrupts_set_WDT_interval(uint32_t interval);
void interrupts_start_WDT_PWM();
void interrupts_stop_WDT_PWM();
void delay(uint16_t time);
void set_timeout();
void reset_timeout();

#define PTX_DEV 0
#define NOTEST 0

#endif /* INTERRUPTS_H_ */
