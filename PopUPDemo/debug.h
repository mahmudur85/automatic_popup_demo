/*
 * debug_port.h
 *
 * Created: 22-Jun-13 10:15:26 PM
 *  Author: Mahmudur
 */ 


#ifndef DEBUG_H_
#define DEBUG_H_

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#define debug_BaudValue		207 // F_CPU = 16000000UL and U2X = 51 for 38.4k
							   // U2X = 207 for 9600

#define debug_puts_P(__s)       debug_puts_p(PSTR(__s))//Macro to automatically put a string constant into program memory

extern void debug_init(uint8_t baud, uint8_t UseRxInterrupt);
extern unsigned char debug_getc(void);
extern void debug_putc(unsigned char c);
extern void debug_puts(const char *s );//Put string to USART0
extern void debug_puts_p(const char *s );//Put string from program memory to USART0
extern int debug_stream(char c, FILE *stream);// uart0 stream function



#endif /* DEBUG_PORT_H_ */