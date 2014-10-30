/*
 * rx_buffer.c
 *
 * Created: 25-Jun-13 01:04:46 PM
 *  Author: Mahmudur
 */ 

#include "rx_buffer.h"

void init_rx_buffer(struct rx_buffer *buffer)
{
	memset((unsigned char *)buffer->rx_buff,0,RX_BUFF_LEN);
	buffer->rx_char = 0;
	buffer->rx_count = 0;
	buffer->rx_flag = 0;
}
