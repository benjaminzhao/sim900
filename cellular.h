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

/*define cellular control cmd (0-255)*/
#define READYtoCONN             0
#define TCPUDP_CONN             1
#define TCPUDP_DISCONN          2
#define TCPUDP_CLOSE            3
#define WAITFOR_CONN_CLOSED     4
#define TCPUDP_SET_CONN_TYPE    5
#define TCPUDP_AUTO_CONN        6
#define TCPUDP_REMOTE_IP        7
#define TCPUDP_REMOTE_PORT      8
#define QUERY_AUTO_CONN         9

#define DO_CALL                 11
#define PICKUP_CALL             12
#define HANGUP_CALL             13
#define FINISH_CALL             14
#define WAITFOR_IN_CALL         15

#define SNED_SMS                21
#define READ_SMS                22
#define DELETE_SMS              23

#define DO_SIG_CHECK            31



rt_err_t rt_cellular_control(rt_uint8_t cmd, void *args);
rt_size_t rt_cellular_read(void* buffer);
rt_size_t rt_cellular_write(const void* buffer, rt_size_t size);
rt_err_t rt_cellular_init(void);
void rt_hw_cellular_init(const char* device_name);
void rt_cellular_thread_startup(void);



#ifdef __cplusplus
}
#endif

#endif /* CELLULAR_H_INCLUDED */

