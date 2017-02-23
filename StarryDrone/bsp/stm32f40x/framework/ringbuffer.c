/*
 * File      : ringbuffer.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-02-04     zoujiachi   	the first version
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

ringbuffer* ringbuffer_create(uint16_t size)
{
	ringbuffer* rb = malloc(sizeof(ringbuffer));
	if(rb == NULL){
		printf("ringbuffer_create fail\r\n");
		return NULL;
	}
	
	rb->buff = malloc(size);
	if(rb->buff == NULL){
		printf("ringbuffer_create fail\r\n");
		return NULL;
	}
	
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;
	
	return rb;
}

void ringbuffer_delete(ringbuffer* rb)
{
	free(rb->buff);
	free(rb);
}

uint16_t ringbuffer_getlen(ringbuffer* rb)
{
	uint16_t len;
	
	if(rb->head >= rb->tail)
		len = rb->head - rb->tail;
	else
		len = rb->head + (rb->size - rb->tail);
}

uint8_t ringbuffer_putc(ringbuffer* rb, uint8_t c)
{
	if( (rb->head+1)%rb->size == rb->tail ){
		//ringbuffer is full
		return 0;
	}
	
	rb->buff[rb->head] = c;
	rb->head = (rb->head+1)%rb->size;
	
	return 1;
}

uint8_t ringbuffer_getc(ringbuffer* rb)
{
	uint8_t c;
	
	if(!ringbuffer_getlen(rb))
		return 0;
	
	c = rb->buff[rb->tail];
	rb->tail = (rb->tail+1)%rb->size;
	
	return c;
}

void ringbuffer_flush(ringbuffer* rb)
{
	rb->head = rb->tail = 0;
}

