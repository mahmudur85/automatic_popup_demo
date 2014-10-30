/*
 * debug_port.c
 *
 * Created: 22-Jun-13 10:15:58 PM
 *  Author: Mahmudur
 */ 

#include "debug.h"

//**********************************************************************//
// UART0
//**********************************************************************//

//////////////////////////////////////////////////////////////////////////
// Initialize UART0
//////////////////////////////////////////////////////////////////////////
void debug_init(uint8_t baud, uint8_t UseRxInterrupt) //Communication with GSM Module
{
	//high baoud rate
	UCSR0A = (1<<U2X0);
    UBRR0H=(unsigned char) (baud >>8);
    UBRR0L=(unsigned char) (baud & 0xFF);
    /* enable tx/rx */
    UCSR0B =(1<<RXEN0) | (1<<TXEN0);

	/* enable interrupt on rx0 */
	if(UseRxInterrupt == 1)
	{
		UCSR0B |= (1<<RXCIE0);
	}

	//format: 8 bit data, 1stop bit,no parity, asynchronous
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);//|(0<<USBS0)|(0<<UPM01)|(0<<UMSEL0)
}

//////////////////////////////////////////////////////////////////////////
// send one character to the rs232
//////////////////////////////////////////////////////////////////////////

void debug_putc(unsigned char c) 
{
    /* wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0=c;
}


//////////////////////////////////////////////////////////////////////////
// receive one character from rs232
//////////////////////////////////////////////////////////////////////////

unsigned char debug_getc() 
{
 /* Loop here until bit RXC0 changes to 1, this
    indicates that we have received data and it
    can be read from the USART0 receive buffer. */

    while (!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

//////////////////////////////////////////////////////////////////////////
// send string to the rs232
//////////////////////////////////////////////////////////////////////////
void debug_puts(const char *s )
{
    while (*s) 
    {
		debug_putc(*s++);
	}
}


//////////////////////////////////////////////////////////////////////////
// send string from program memory to the rs232
//////////////////////////////////////////////////////////////////////////
void debug_puts_p(const char *progmem_s )
{
    register char c;
    
    while ( (c = pgm_read_byte(progmem_s++)) ) 
	{
    	debug_putc(c);
	}

}


//////////////////////////////////////////////////////////////////////////
// Stream function for uart0
//////////////////////////////////////////////////////////////////////////

int  debug_stream(char c, FILE *stream)
{
	//uart0_putc((unsigned char)c);
	if (c == '\n')	
	{
		debug_putc('\r');
	}
	loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
	return 0;
}
