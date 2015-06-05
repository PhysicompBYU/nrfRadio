/*
 * uart.h
 *
 *  Created on: Apr 30, 2015
 *      Author: bsnga
 */
#include "stdint.h"

#define TERMINAL print
#define TERMINAL1(f,x) sprintf(buffer,f,x);print(buffer);
#define TERMINAL2(f,x) sprintf(buffer,f,x,x);print(buffer);
#define TERMINAL3(f,x,y) sprintf(buffer,f,x,y);print(buffer);

#define XTERMINAL1(f,x) sprintf(buffer,"\n\r%d. ",count++);print(buffer);sprintf(buffer,f,x);print(buffer);
#define XTERMINAL2(f,x) sprintf(buffer,"\n\r%d. ",count++);print(buffer);sprintf(buffer,f,x,x);print(buffer);
#define XTERMINAL3(f,x,y) sprintf(buffer,"\n\r%d. ",count++);print(buffer);sprintf(buffer,f,x,y);print(buffer);

#define EN_TXIE IE2 |= UCA0TXIE
#define DEN_TXIE IE2 &= ~UCA0TXIE

//UART port defines
#define		PORTDIR	P1DIR
#define		PORTOUT	P1OUT

#define BPS 9600
#define TXBUFSIZE 32

//functions
void uart_init();
void find_baud_rate();
void print(const char *s);
void print_x(uint8_t *s, uint8_t size);
int uart_putchar(uint8_t c);
uint8_t uart_getchar(void);

//variables
