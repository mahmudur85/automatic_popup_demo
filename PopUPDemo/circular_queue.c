/*
 * circular_queue.c
 *
 * Created: 25-Jun-13 12:24:58 PM
 *  Author: Mahmudur
 */ 

#include "circular_queue.h"

void init_queue(struct circular_queue *queue)
{
	memset((unsigned char *)queue->queue_buff,0,QUEUE_LEN);
	queue->queue_top=0;
	queue->queue_end=0;
	queue->queue_count=0;
}

unsigned char queue_push(struct circular_queue *queue, unsigned char val)
{
	if(queue->queue_count != QUEUE_FULL)
	{
		queue->queue_buff[queue->queue_end]=val;
		queue->queue_count++;
		queue->queue_end++;
		if(queue->queue_end==QUEUE_LEN)
		{
			queue->queue_end=0;
		}
	}
	else
	{
		return QUEUE_PUSH_FAIL;
	}
	return QUEUE_PUSH_SUCCESS;
}

unsigned char queue_pop(struct circular_queue *queue, unsigned char *val)
{
	if(queue->queue_count!= QUEUE_EMPTY)
	{
		(*val)=queue->queue_buff[queue->queue_top];
		queue->queue_count--;
		queue->queue_top++;
		if(queue->queue_top==QUEUE_LEN)
		{
			queue->queue_top=0;
		}
	}
	else
	{
		return QUEUE_POP_FAIL;
	}
	return QUEUE_POP_SUCCESS;
}