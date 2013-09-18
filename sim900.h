/*************************************************************************
* File Name:	sim900.h
* create by:	benjamin_zhao
* Date First Issued :   2013-05-30
* Description        :  This file contains the head code of sim900.c
*
* modified by benjaminzhao
* date: 2013/05/30
* ver: v 0.1
*
**************************************************************************/
#ifndef __SIM900_H
#define __SIM900_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

/*
 * RT-Thread s Driver
 * 2013-05-30 ben added for armfly board
 * sim900 rtt device
 */
#include "rtthread.h"

//#ifndef __IP_ADDR_H
//struct ip_addr
//{
//    rt_uint32_t addr;
//};
//typedef struct ip_addr ip_addr_t;
//
///** Set an IP address given by the four byte-parts.
//    Little-endian version that prevents the use of htonl. */
//#define IP4_ADDR(ipaddr, a,b,c,d) \
//        (ipaddr)->addr = ((rt_uint32_t)((d) & 0xff) << 24) | \
//                         ((rt_uint32_t)((c) & 0xff) << 16) | \
//                         ((rt_uint32_t)((b) & 0xff) << 8)  | \
//                          (rt_uint32_t)((a) & 0xff)
//#endif

//defineS
//sim900 working status mode
//non transparent uart process for <0x10
//transparent uart process for >=0x10
#define INITIAL             0x00	//init
#define STANDBY             0x01	//sim900 standby for anything, CommLineStatus is free
#define IN_CALL             0x02	//sim900 in call mode, CommLineStauts is CALL
#define IN_SMS              0x03	//sim900 in sms mode,
#define IN_GPRS             0x04	//sim900 in gprs mode,
#define IN_ESC              0x05	//sim900 in esc mode of tcp transparent, it is non transparent
#define IN_TCPUDP_NONTRANSP 0x09	//sim900 in non-transparent tcpip mode
#define IN_TCPUDP_TRANSP    0x10	//sim900 in transparent tcpip mode

//sim900 OPs request event definition
//must < 32
#define GPRS_CONN       (1<<0)  //  gprs dail up
#define TCPUDP_CONN     (1<<1)  //  tcp/ip connection
#define TCPUDP_DISCONN  (1<<2)  //  TCP/UDP dis-connect
#define TCPUDP_CLOSE    (1<<3)  //  TCO/UDP CLOSE
#define TCPUDP_CLOSED   (1<<4)  //  TCP/UDP CLOSED BY REMOTE
#define INCOMING_CALL   (1<<5)  //  switch to call
#define OUTGOING_CALL   (1<<6)  //  switch to call
#define HANGUP_CALL     (1<<7)  //  hang up call
#define INCOMING_SMS    (1<<8)  //  switch to sms
#define OUTGOING_SMS    (1<<9)  //  switch to sms
#define SIG_CHECK       (1<<10) //  signal quality check


//sim900 AT cmd response event definition
//MUST < 32
#define OK          (1<<0)
#define CONNECT     (1<<1)
#define RING        (1<<2)  //  incoming call
#define NO_CARRIER  (1<<3)
#define ERROR       (1<<4)
#define NO_DIALTONE (1<<5)
#define BUSY        (1<<6)  //  call hanged up
#define NO_ANSWER   (1<<7)
#define READY       (1<<8)
#define STATE       (1<<9)
#define CLOSED      (1<<11)
#define CLOSE_OK    (1<<12)
#define CONNECT_OK  (1<<13)
#define CONNECT_FAIL (1<<14)
#define SEND_OK     (1<<15)
#define SEND_FAIL   (1<<16)
#define SHUT_OK     (1<<17)
#define SERVER_OK   (1<<18)
#define ALREADY_CONNECT (1<<19)
#define PROMOT_MARK (1<<20) //">"
#define IPD         (1<<21) //ip data in

#define DATA_ACCEPT (1<<27)
#define DATA_SEND   (1<<28)
#define CME_ERROR   (1<<29)
#define SIG_OK      (1<<30)
#define CALLDONE    (1<<31)

#define recv_ats        1
#define recv_ipdata_len 2
#define recv_ipdata     3





#define SIM900_AT_REPONSE_SIZE  128
#define SIM900_GSM_CONN_TOUT	5
#define SIM900_DATA_TOUT		10
#define SIM900_TCP_CONN_TOUT	20

#define SIM900_AT_RESP_MQ_POOL_SIZE	1024
#define SIM900_AT_RESP_MQ_MSG_SIZE	32

//sim900 virtual device define
typedef struct rt_sim900_device
{
    rt_device_t device;     //phy device


    rt_uint8_t  CommLineStatus; //
    rt_int8_t   signalDB;       //signal quality in DB:-115~-52
    rt_uint8_t	rssi;           //rssi:0~99

    rt_uint8_t  tcpudp_autoconn;//auto connection setting

    rt_uint8_t* CenterPhoneNumber; //outgoing phone number[16]
    rt_uint8_t* IncomingCallNumber;//incoming call number[16]
    rt_uint8_t* conn_type;         //"TCP"/"UDP"[4]

    rt_uint8_t* local_addr;        //local ip[16]
    rt_uint8_t* remote_addr;       //serevr ip[16]
    rt_uint8_t* gw_addr;           //gateway ip[16]
    rt_uint8_t* mask_addr;         //mask[16]

    rt_uint8_t* local_port;        //local port[6]
    rt_uint8_t* remote_port;       //server port[6]
    
    rt_uint8_t* id;                 //id[16]:SIM900 R11.0 etc
    rt_uint8_t* manufacturer;       //device producer[16]:SIMCOM_Ltd etc
    rt_uint8_t* type;               //device type[16]:SIMCOM_SIM900 etc
    rt_uint8_t* Rev;                //device revision[22]:1137B01V01SIM900M32_ST
    rt_uint8_t* sn;                 //serial number[16]:013227000201560 etc
    rt_uint8_t* operator;           //celluar operator[16]:CHINA MOBILE etc


    rt_sem_t    rx_semaphore;       //sem for sim900 get a char
    rt_sem_t    frame_sem;          //sem for ip data pack recieved
    rt_sem_t    tcp_connected_sem;  //sem for tcp connected
    rt_sem_t    calldone_sem;       //sem for call finished
    
    rt_event_t  ATResp_event;   //event of modem AT response
//    rt_event_t  promotion_mark;

    rt_mq_t		AT_resp_MQ;		//msg queue for at response str
    rt_uint8_t*	AT_resp_MQ_poll;//mem pool of the AT_resp_MQ

}rt_sim900_device;

//extern rt_sim900_dev*   sim900_device;//a instance

void rt_sim900_thread_startup(void);

rt_err_t rt_sim900_init(void);
rt_err_t rt_sim900_open(void);
rt_err_t rt_sim900_close(void);
rt_size_t rt_sim900_read(void* buffer);
rt_size_t rt_sim900_write(const void* buffer, rt_size_t size);

rt_err_t rt_sim900_control(rt_uint8_t cmd, void *args);
void rt_hw_sim900_init(const char* device_name);

rt_size_t sim900_readDATA(void* buffer);
rt_size_t sim900_sendDATA(const void* buffer, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif




