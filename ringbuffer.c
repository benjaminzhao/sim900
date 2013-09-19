/**********************************************************
 * File      : ring buffer.c
 * added for RT-Thread RTOS
 *
 * create:	benjamin_zhao
 * date:	2011-09-17
 **********************************************************/
#include "ringbuffer.h"
#include <string.h>


/***********************
*init a ring buffer
***********************/
void rb_init(struct rb* rb, rt_uint8_t* pool, rt_uint16_t size)
{
	RT_ASSERT(rb != NULL);
	rb->read_index = 0;
	rb->write_index = 0;
	rb->buffer_ptr = pool;
	rb->buffer_size = size;
}


/***********************
*put a char to ringbuffer
***********************/
rt_bool_t rb_put(struct rb* rb, const rt_uint8_t* ptr, rt_uint16_t length)
{
	rt_size_t size;
	if(rb->read_index > rb->write_index)
		size = rb->read_index - rb->write_index;
	else
		size = rb->buffer_size - rb->write_index + rb->read_index;
	if(size < length)
		return RT_FALSE;
	if(rb->read_index > rb->write_index)
	{
		memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
		rb->write_index += length;
	}	
	else	
	{
		size = rb->buffer_size - rb->write_index;
		if(size > length)
		{
			memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
			rb->write_index += length;
		}
		else
		{
			memcpy(&rb->buffer_ptr[rb->write_index], ptr, size);
			memcpy(&rb->buffer_ptr[0], &ptr[size], length - size);
			rb->write_index = length - size;
		}
	}
	return RT_TRUE;
}



/***********************
*get a char from ringbuffer
***********************/
rt_bool_t rb_get(struct rb* rb, rt_uint8_t* ptr, rt_uint16_t length)
{
	rt_size_t size;
	if(rb->read_index > rb->write_index)
		size = rb->buffer_size - rb->read_index + rb->write_index;
	else
		size = rb->write_index - rb->read_index;
	if(size < length)
		return RT_FALSE;
	if(rb->read_index > rb->write_index)
	{
		size = rb->buffer_size - rb->read_index;
		if(size > length)
		{
			memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
			rb->read_index += length;
		}
		else
		{
			memcpy(ptr, &rb->buffer_ptr[rb->read_index], size);
			memcpy(&ptr[size], &rb->buffer_ptr[0], length - size);
			rb->read_index = length - size;
		}
	}
	else
	{
		memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
		rb->read_index += length;
	}
	return RT_TRUE;
}

