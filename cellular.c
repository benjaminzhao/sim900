/*************************************************************************
* File Name:    cellular.h
* create by:    benjamin_zhao
* Date First Issued :   2013-06-15
* Description       :   This file contains the c code of cellular moduler
*                       flexible for three cellular device
* modified by benjaminzhao
* date: 2013/06/15
* ver: v 0.1
*
**************************************************************************/
#include <stm32f10x.h>
#include "board.h"
#include "rtthread.h"
#include "cellular.h"

#if (CELLULAR_TYPE == 1)
#include "sim900.h"
//#elif (CELLULAR_TYPE == 2)
//#include "mg323.h"
//#elif (CELLULAR_TYPE == 3)
//#include "mc37i.h"
#endif


/****************************************
*   rt_celluar_read                     *
*   port to specified device read func  *
****************************************/
rt_size_t rt_celluar_read(void* buffer)
{
    rt_size_t read_len = 0;
#if (CELLULAR_TYPE == 1)
    read_len = rt_sim900_read(buffer);
#endif
    return read_len;
}

/****************************************
*   rt_cellular_write                   *
*   port to specified device write func *
****************************************/
rt_size_t rt_cellular_write(const void* buffer, rt_size_t size)
{
    rt_size_t   write_len = 0;
#if (CELLULAR_TYPE == 1)
    write_len = rt_sim900_write(buffer, size);
#endif
    return write_len;
}

/****************************************
*   rt_celluar_open                     *
*   port to specified device open func  *
****************************************/
rt_err_t rt_celluar_open(void)
{
    rt_err_t    err;
#if (CELLULAR_TYPE == 1)
    err = rt_sim900_open();
#endif
    return err;
}

/****************************************
*   rt_celluar_close                    *
*   port to specified device close func *
****************************************/
rt_err_t rt_celluar_close(void)
{
    rt_err_t    err;
#if (CELLULAR_TYPE == 1)
    err = rt_sim900_close();
#endif
    return err;
}

/****************************************
*   rt_celluar_control                  *
* port to specified device control func *
****************************************/
rt_err_t rt_celluar_control(rt_uint8_t cmd, void *args)
{
    rt_err_t    err;
    #if (CELLULAR_TYPE == 1)
        err = rt_sim900_control(cmd, args);
    #endif
    return err;
}

//static rt_err_t rt_celluar_waitfor(void)
//{
//    rt_err_t    err;
//    #if (CELLULAR_TYPE == 1)
//        err = rt_sim900_waitforEvent();
//    #endif
//    return err;
//}


/****************************************
*   rt_cellular_init                    *
*   port to specified device init func  *
****************************************/
rt_err_t rt_cellular_init(void)
{
    rt_err_t    err;
#if (CELLULAR_TYPE == 1)
    err = rt_sim900_init();
//#elif (CELLULAR_TYPE == 2)
//    rt_mg323_init();
//#elif (CELLULAR_TYPE == 3)
//    rt_mc37i_device_init();
#endif
    return err;
}



/****************************************
*   rt_hw_cellular_init                 *
* port to specified device hw int func  *
****************************************/
void rt_hw_cellular_init(const char* device_name)
{
#if (CELLULAR_TYPE == 1)
    rt_hw_sim900_init(device_name);
//#elif (CELLULAR_TYPE == 2)
//    rt_mg323_init();
//#elif (CELLULAR_TYPE == 3)
//    rt_mc37i_device_init();
#endif
}

/****************************************
*   rt_cellular_thread_startup          *
*   port to specified service thread    *
****************************************/
void rt_cellular_thread_startup(void)
{
#if (CELLULAR_TYPE == 1)
    rt_sim900_thread_startup();
//#elif (CELLULAR_TYPE == 2)
//    rt_mh323_thread_startup();
//#elif (CELLULAR_TYPE == 3)
//    rt_mc37i_thread_startup();
#endif
}

