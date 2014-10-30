/*
 * uart.c
 *
 * Created: 4/16/2012 5:22:39 PM
 * Author: Mahmudur
 */
#include <stdio.h> 
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "uart.h"

//**********************************************************************//
// UART1
//**********************************************************************//

//////////////////////////////////////////////////////////////////////////
// Initialize UART1
//////////////////////////////////////////////////////////////////////////
void uart_init(uint8_t baud, uint8_t UseRxInterrupt) //Communication with GPRS module
{
	//high boud rate
	UCSR1A = (1<<U2X1);
    UBRR1H=(unsigned char) (baud >>8);
    UBRR1L=(unsigned char) (baud & 0xFF);
    /* enable tx/rx*/
    UCSR1B = (1<<RXEN1) | (1<<TXEN1);

	/* enable interrupt on rx1 */
	if (UseRxInterrupt == 1)
	{
		UCSR1B |= (1<<RXCIE1);
	}

    //format: 8 bit data, 1stop bit,no parity, asynchronous
    UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);//|(0<<USBS1)|(0<<UPM11)|(0<<UMSEL1)
}


//////////////////////////////////////////////////////////////////////////
// send one character to the rs232
//////////////////////////////////////////////////////////////////////////

void uart_putc(unsigned char c) 
{
    /* wait for empty transmit buffer */
    while (!(UCSR1A & (1<<UDRE1)));
    UDR1=c;
}



//////////////////////////////////////////////////////////////////////////
// receive one character from rs232
//////////////////////////////////////////////////////////////////////////

unsigned char uart_getc() 
{
 /* Loop here until bit RXC1 changes to 1, this
    indicates that we have received data and it
    can be read from the USART0 receive buffer. */

    while (!(UCSR1A & (1<<RXC1)));
	return UDR1;  
}

//////////////////////////////////////////////////////////////////////////
// send string to the rs232
//////////////////////////////////////////////////////////////////////////
void uart_puts(const char *s )
{
    while (*s)
	{
    	uart_putc(*s++);
	}
}


//////////////////////////////////////////////////////////////////////////
// send string from program memory to the rs232
//////////////////////////////////////////////////////////////////////////
void uart_puts_p(const char *progmem_s )
{
    register char c;
    
    while ( (c = pgm_read_byte(progmem_s++)) )
	{
		uart_putc(c);
	}

}


//////////////////////////////////////////////////////////////////////////
// Stream function for uart1
//////////////////////////////////////////////////////////////////////////

int uart_stream(char c, FILE *stream)
{
	//uart1_putc((unsigned char)c);
	if (c == '\n')	
	{
		uart_putc('\r');
	}
	loop_until_bit_is_set(UCSR1A, UDRE1);
    UDR1 = c;
	return 0;
}
