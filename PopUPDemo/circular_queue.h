/*
 * circular_queue.h
 *
 * Created: 25-Jun-13 12:25:15 PM
 *  Author: Mahmudur
 */ 


#ifndef CIRCULAR_QUEUE_H_
#define CIRCULAR_QUEUE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUEUE_LEN	256

// errors
#define QUEUE_PUSH_SUCCESS	0b00000000
#define QUEUE_PUSH_FAIL		0b00000001
#define QUEUE_POP_SUCCESS	0b00000000
#define QUEUE_POP_FAIL		0b00000001
#define QUEUE_FULL			QUEUE_LEN
#define QUEUE_EMPTY			0

struct circular_queue
{
	unsigned char queue_buff[QUEUE_LEN];
	int queue_top;
	int queue_end;
	int queue_count;
};

void init_queue(struct circular_queue *queue);
unsigned char queue_push(struct circular_queue *queue, unsigned char val);
unsigned char queue_pop(struct circular_queue *queue, unsigned char *val);

#endif /* CIRCULAR_QUEUE_H_ */