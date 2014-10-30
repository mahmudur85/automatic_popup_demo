/*
 * rx_buffer.h
 *
 * Created: 25-Jun-13 01:04:29 PM
 *  Author: Mahmudur
 */ 


#ifndef RX_BUFFER_H_
#define RX_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_BUFF_LEN 256

struct rx_buffer{
	unsigned char rx_buff[RX_BUFF_LEN];
	unsigned char rx_char;
	unsigned char rx_flag;
	unsigned int rx_count;	
};

void init_rx_buffer(struct rx_buffer *buffer);

#endif /* RX_BUFFER_H_ */