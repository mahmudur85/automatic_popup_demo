/*
 * uart.h
 *
 * Created: 4/16/2012 5:23:01 PM
 *  Author: Mahmudur
 */ 


#ifndef UART_H_
#define UART_H_


#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>

/***********USART Config************/

#define uart_BaudValue		51 // F_CPU = 16000000UL and U2X = 51 for 38.4k

#define uart_puts_P(__s)	uart_puts_p(PSTR(__s))//Macro to automatically put a string constant into program memory


/**********************************/
extern void uart_init(uint8_t baud, uint8_t UseRxInterrupt);
extern unsigned char uart_getc();
extern void uart_putc(unsigned char c);
extern void uart_puts(const char *s );//Put string to USART1
extern void uart_puts_p(const char *s );//Put string from program memory to USART1
extern int uart_stream(char c, FILE *stream);// uart1 stream function

#endif /* UART_H_ */

