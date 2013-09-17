/*************************************************************************
* File Name:    cellular.h
* create by:    benjamin_zhao
* Date First Issued :   2013-06-15
* Description        :  This file contains the head code of cellular moduler
*                       for rt thread system
* modified by benjaminzhao
* date: 2013/06/15
* ver: v 0.1
*
**************************************************************************/

#ifndef CELLULAR_H
#define CELLULAR_H

#ifdef __cplusplus
extern "C" {
#endif

/*define celluar control cmd (0-255)*/
#define DO_TCPUDP_CONN          1
#define DO_TCPUDP_DISCONN       2
#define TCPUDP_CLOSED           3

#define DO_OUTGOING_CALL        11
#define ANSWER_INCOMING_CALL    12
#define DO_HANGUP_CALL          13

#define DO_OUTGOING_SMS         21
#define DO_READ_SMS             22
//#define DO_DELETE_SMS           23


#define DO_SIG_CHECK            31


static rt_err_t rt_celluar_control(rt_uint8_t cmd, void *args);

rt_err_t rt_cellular_init(void);
void rt_hw_cellular_init(const char* device_name);
void rt_cellular_thread_startup(void);

#ifdef __cplusplus
}
#endif

#endif /* CELLULAR_H_INCLUDED */

