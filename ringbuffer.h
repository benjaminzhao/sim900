/**********************************************************
 * File      : ring buffer.c
 * added for RT-Thread RTOS
 *
 * create:	benjamin_zhao
 * date:	2011-09-17
 **********************************************************/
#ifndef	_RINGBUFFER_H_
#define	_RINGBUFFER_H_

#ifdef	__cplusplus
extern "C" {
#endif
	
#include <rtthread.h>


struct rb
{
    rt_uint8_t* buffer_ptr;
    rt_uint16_t buffer_size;// <16bit
    rt_uint16_t read_index;
    rt_uint16_t write_index;
};

void        rb_init(struct rb* rb, rt_uint8_t* pool, rt_uint16_t size);
rt_bool_t   rb_put(struct rb* rb, const rt_uint8_t* ptr, rt_uint16_t length);
rt_bool_t   rb_get(struct rb* rb, rt_uint8_t* ptr, rt_uint16_t length);



#ifdef	__cplusplus
}
#endif

#endif

