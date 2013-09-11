/*************************************************************************
* File Name:	sim900.c
* create by:	benjamin_zhao
* Date First Issued :   2013-05-30
* Description        :  This file contains the op code of sim900 & 900A
*                       basicly AT cmd of sim900
*                       using usart1 on armfly board
*                       sim900 work in non transparent mode
* modified by benjaminzhao
* date: 2013/05/30
* ver: v 0.1
*
**************************************************************************/
//std lib
#include    <string.h>
#include    <stdlib.h>
#include    <stdio.h>

//modem driver
#include    "sim900.h"
#include    "ringbuffer.h"

//rt thread lib
#include    <rtthread.h>

//sim900 modem debug info define
#define SIM900_DEBUG    1

#if SIM900_DEBUG
#define SIM900_TRACE    rt_kprintf
#else
#define SIM900_TRACE(...)
#endif
#define MAX_ADDR_LEN    6


//private sem locker
static struct rt_semaphore  sem_sim900_lock;    //lock for sim900 OP
static struct rt_semaphore  sem_rb_lock;        //lock for rb OP
#define rt_sim900_lock      rt_sem_take(&sem_sim900_lock,RT_WAITING_FOREVER);
#define rt_sim900_unlock    rt_sem_release(&sem_sim900_lock);



//default ip & port
static rt_uint8_t   default_remoteIPADDR1[] = "202.142.21.85";
static rt_uint8_t   default_remotePORT1[]   = "8003";
static rt_uint8_t   default_connTYPE1[]     = "TCP";
//second default ip & port
static rt_uint8_t   default_remoteIPADDR2[] = "192.168.1.100";
static rt_uint8_t   default_remotePORT2[]   = "80";
static rt_uint8_t   default_connTYPE2[]     = "TCP";


static  rt_sim900_device*   sim900;

/*ring buffer for IP DATA*/
static  rt_uint8_t* IPdata_buff;
static  struct rb   IPdata_rb;

rt_size_t rt_sim900_write(const void* buffer, rt_size_t size);


//<CR><LF><response><CR><LF>
// /r  /n  response  /r /n
// 0D  0A  response  0D 0A
//
////查询数据传输状态
//rt_uint8_t sim900_TCPPack(void)
//{
//    rt_uint8_t TCPpackcmd[] = "AT+CIPACK\r\n";//len=11
//    rt_device_write(sim900_device.device,
//                    0,
//                    TCPpackcmd[0],
//                    rt_strlen(TCPpackcmd[0]) );
//    return 1;
//}
//
////设施本地端口
//rt_uint8_t sim900_TCPSetPort(rt_uint8_t* cmd)
//{
//    rt_uint8_t len = rt_strlen(cmd);
//    rt_uint8_t TCPsetportcmd[16+len+2];
//    sprintf(TCPsetportcmd,
//            "%s%s%s",
//            "AT+CLPORT=\"TCP\",",
//            cmd,
//            "\r\n");
//    rt_device_write(sim900_device.device,
//                    0,
//                    TCPsetportcmd[0],
//                    rt_strlen(TCPsetportcmd[0]) );
//    return 1;
//}
//
////配置域名服务器
//rt_uint8_t sim900_TCPDnsConf(rt_uint8_t* DNS)
//{
//    rt_uint8_t len = rt_strlen(DNS);
//    rt_uint8_t cmd[12+len+3];
//    sprintf(cmd,
//            "%s%s%s\r\n",
//            "AT+CDNSCFG=\"",//12
//            DNS,
//            "\"");
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
////域名解析
//rt_uint8_t sim900_TCPDNSParse(rt_uint8_t* DOMAIN)
//{
//    rt_uint8_t len = rt_strlen(DOMAIN);
//    rt_uint8_t cmd[12+len+3];
//    sprintf(cmd,
//            "%s%s%s\r\n",
//            "AT+CDNSGIP=\"",//12
//            DOMAIN,
//            "\"");
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
////设置自动发送时间
//rt_uint8_t sim900_TCPAutosendConf(rt_uint8_t MODE,rt_uint8_t TIME)
//{
//    rt_uint8_t cmd[] = "";
//    sprintf(cmd,
//            "%s%s,%s\r\n",
//            "AT+CIPATS=",//13
//            MODE,
//            TIME);
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
////配置为服务器
//rt_uint8_t sim900_TCPServerConf(rt_uint8_t MODE, rt_uint16_t PORT)
//{
//    rt_uint8_t cmd[] = "";
//    sprintf(cmd,
//            "%s%s,%s\r\n",
//            "AT+CIPSERVER=",//13
//            MODE,
//            PORT);
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
////配置TCP/IP应用模式；透明/非透明
//rt_uint8_t sim900_TCPModeConf(rt_uint8_t mode)
//{
//    rt_uint8_t cmd[] = "";
//    sprintf(cmd,
//            "%s%s\r\n",
//            "AT+CIPMODE=",//11
//            mode);
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
////配置透明传输模式
//rt_uint8_t sim900_TCPTransConf(rt_uint8_t NmRetry, rt_uint8_t WaitTm, rt_uint16_t SendSz,rt_uint8_t esc)
//{
//    rt_uint8_t cmd[] = "";
//    sprintf(cmd,
//            "%s%s,%s,%s,$s\r\n",
//            "AT+CIPCCFG=",//11
//            NmRetry,
//            WaitTm,
//            SendSz,
//            esc);
//    rt_device_write(sim900_device.device,
//                    0,
//                    cmd[0],
//                    rt_strlen(cmd[0]) );
//    return 1;
//}
//
//void ESC_monitor(void)
//{
////(1) if "CLOSED" recieved
////(2) stop data transfer for 1000ms
////    send "+++"
////    idle for 500ms
////    get "OK"
//}
//void IncomingCALL_monitor(void)
//{
////    RI_pin == LW for 120ms
////    send incoming call event
//}
//
//void IncomingSMS_monitor(void)
//{
////    RI_pin == LW for 50ms
////    send incoming sms event
//}

//void EnterTransparentMode(void)
//{
////send ATs to config for trans mode
////send ATs to establish GPRS TCP connection
////get "CONNECTED"
////switch mode=TRANSP_MODE
//}
//
//void return2Trans(void)
//{
////send "+++" to esc
////send "ATO"
////get "CONNECTED"
////switch mode=TRANSP_MODE
//}
//
//void Trans2NonTrans_active(void)
//{//closed actively
////stop data transfer for 1000ms
////send "+++"
////idle for 500ms
////get "OK"
////switch mode=NONTRANSP_MODE
//}
//
//void Trans2NonTrans_passive(void)
//{//closed passively by remote server/client
////wait for and get "CLOSED"
////switch mode=NONTRANSP_MODE
//}
//
//void Tran2NonTrans(void)
//{
////ESC monitor
////switch mode=NONTRANSP_MODE
//}
//

//
////call control api CALLTYPE
//void do_call(void)
//{
//    //if(INCOMING_CALL)
//    {
//        //answercall
//    }
//    //else if(OUTGOING_CALL)
//    {
//        //dailcall
//    }
//    
//    //if(call success)
//    //{//in talking
//        //waiting for call finish
//    //}
//    //else if (failed)
//    //{no answer/busy/no carrier
//        //hang up
//    //}
//    //if(hang up)
//    //{
//        //sim900_HangUpCall()
//    //}
//    //else
//    //{waiting for hanged by remote}
//    
//    //add calldone hook func
//}
////sms sontrol api
//void do_sms(void)
//{
//    //
//}
//
//void switch_toESC(void)
//{
//    //if(in transp mode)
//    //idle for 1000ms
//    //usart send "+++"
//    //idle for 500ms
//    //return in_esc
//}
//
//void switch_fromESC(void)
//{
//    //if(in_esc)
//    //usart send "ato"
//    //waiting for "connect"
//    //set to in transp mode
//}
//
//rt_uint8_t sim900_ConnectGPRS(void)
//{
//}
//void sim900_Setup(void)
//{
//}
//rt_uint8_t sim900_checkcall(void)
//{
//}
//netconn_write(struct netconn *conn, const void *dataptr, size_t size, u8_t apiflags);
//netconn_read();




rt_uint32_t sim900_cmd_parse(rt_uint8_t cellular_cmd)
{
    rt_uint32_t sim900_cmd;
    switch(cellular_cmd)
    {
        case 1:
            sim900_cmd = TCPUDP_CONN;
        break;
        case 2:
            sim900_cmd = TCPUDP_DISCONN;
        break;
        case 3:
            sim900_cmd = TCPUDP_CLOSE;
        break;
        case 11:
            sim900_cmd = OUTGOING_CALL;
        break;
        case 12:
            sim900_cmd = INCOMING_CALL;
        break;
        case 13:
            sim900_cmd = HANGUPCALL;
        break;
        case 21:
            sim900_cmd = OUTGOING_SMS;
        break;
        case 22:
            sim900_cmd = INCOMING_SMS;
        break;
        case 31:
            sim900_cmd = SIG_CHECK;
        break;
        default:
        break;
    }
    return sim900_cmd;
}


/*
 * RT-Thread modem threads Driver
 * 2013-05-30 ben added for armfly board
 * sim900 rt device for armfly board
 */

void ATresp_Process(rt_uint8_t* ATresp_str)
{
    rt_uint8_t* ATRes_buff;
    rt_uint8_t  bufSz = 64;
    
    RT_ASSERT(sim900->device != RT_NULL);

    ATRes_buff = rt_malloc_align(bufSz, 4);
    rt_memcpy(ATRes_buff, ATresp_str, rt_strlen(ATresp_str));

    if(rt_strncmp( (char*)(&ATRes_buff[0]), "CLOSED", 6)==0)
    {
        rt_event_send(sim900->ATResp_event, CLOSED);
        //rt_sim900_control(TCPUDP_CLOSED);
    }
    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "RING", 4)==0)
    {
        rt_event_send(sim900->ATResp_event, RING);
        //rt_sim900_control(INCOMING_CALL);
    }
    else
        rt_mb_send(sim900->ATMailBox, (rt_uint32_t)&ATRes_buff[0]);
    
    
//    if(rt_strncmp( (char*)(&ATRes_buff[0]), "OK", 2)==0)
//        send_event(sim900->device, OK);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "ERROR", 5)==0)
//        send_event(sim900->device, ERROR);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "CONNECT", 7)==0)
//        send_event(sim900_device, CONNECT);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "NO ANSWER", 9)==0)
//        send_event(sim900_device, NO_ANSWER);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "NO CARRIER", 10)==0)
//        send_event(sim900_device, NO_CARRIER);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), ">", 1)==0)
//        send_event(sim900_device, PROMOT_MARK);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "SEND OK", 7)==0)
//        send_event(sim900_device, SEND_OK);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "SEND FAIL", 9)==0)
//        send_event(sim900_device, SEND_FAIL);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "CLOSE OK", 8)==0)
//        send_event(sim900_device, CLOSE_OK);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "+CPAS: 0", 8)==0)
//        send_event(sim900_device, CALLDONE);
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "+CIPSEND:", 9)==0)
//    {//send info
//        rt_mq_send(&(sim900_device->msg_mq), &ATRes_buff[0], rt_strlen(&ATRes_buff[0]));
//        send_event(sim900_device, DATA_SEND);
//    }
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "+CME ERROR", 10)==0)
//    {//cme error info
//        rt_mq_send(&(sim900_device->msg_mq), &ATRes_buff[0], rt_strlen(&ATRes_buff[0]));
//        send_event(sim900_device, CME_ERROR);
//    }
//    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "+CSQ:", 5)==0)
//    {//signal quality info
//        if( (ATRes_buff[6]-0x30)*10 + (ATRes_buff[7]-0x30) < 99)
//            rt_sem_release(&sim900_device.connect_sem);
//        send_event(sim900_device, SIG_OK);
//    }

}

rt_uint8_t WaitATResp(const rt_uint8_t* expect_resp_str, rt_uint32_t max_tout )
{
    rt_uint8_t* ATRes_buff;
    rt_uint8_t  bufSz = 64;

    RT_ASSERT(sim900->device != RT_NULL);

    ATRes_buff = rt_malloc_align(bufSz, 4);
    rt_memcpy(ATRes_buff, expect_resp_str, rt_strlen(expect_resp_str));

    if(rt_strncmp( (char*)(&ATRes_buff[0]), "CLOSED", 6)==0)
    {
        rt_event_recv(sim900->ATResp_event, CLOSED);
        //rt_sim900_control(TCPUDP_CLOSED);
    }
    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "RING", 4)==0)
    {
        rt_event_recv(sim900->ATResp_event, RING);
        //rt_sim900_control(INCOMING_CALL);
    }
    else if(rt_strncmp( (char*)(&ATRes_buff[0]), "OK", 2)==0)
    {
    	rt_event_recv(sim900->ATResp_event, OK);
    }

}


/****************************************
*   rt_thread_sim900_rx_entry           *
*   bottom op of sim900 read            *
*                                       *
*   (1) handle rx byte                  *
*   (2) modem mode:read at respnse      *
*       send AT event to proc thread    *
*   (3)	ppp mode:fill ip data into buf  *
*       send sem to sio_read            *
*                                       *
****************************************/
static void rt_thread_sim900_rx_entry(void* parameter)
{
    rt_uint8_t  i = 0;
    rt_uint8_t  j = 0;
    rt_uint8_t  k = 0;
    rt_uint8_t  thischar;
    rt_uint8_t  ch[5];
    rt_uint8_t  bufSz = 64;

    rt_uint8_t  recv_type;
    rt_uint8_t  ATRes_buff[64];     //ring buffer
    rt_uint16_t ip_data_len = 0;

    rt_memset(&ATRes_buff[0], 0, bufSz);

    while(1)
    {
        if(rt_sem_take(sim900->rx_semaphore, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        while(rt_device_read(sim900->device, 0, &thischar, 1) == 1)
        {// <CR><LF>RESPONSE<CR><LF> //0d 0d 0a XXXX 0d 0d 0a
            switch(recv_type)
            {
                case recv_ats:
                {	//recieve at
                    ch[4] = ch[3];
                    ch[3] = ch[2];
                    ch[2] = ch[1];
                    ch[1] = ch[0];
                    ch[0] = thischar;
                    if(i < bufSz)
                        ATRes_buff[i++] = thischar;
                    /* check CR key */
                    if( (ch[1] == 0x0D) && (ch[0] == 0x0A) )
                    {//DO AT RESPONSE PARSE
                        while(ATRes_buff[i-1-k] == 0x0D && ATRes_buff[i-1-k] == 0x0A)
                        {
                            ATRes_buff[i-k] = 0;//remove 0d 0a
                            if(k < i-1)
                                k++;
                        }
                        k=0;
                        if(ATRes_buff[0] > 0 && ATRes_buff[0] < 128)
                            ATresp_Process(&ATRes_buff[0]);//resolve at response and send RT event
                        rt_memset(&ATRes_buff[0], 0, bufSz);
                        i = 0;
                    }
                    /*check IP data, using IP head before ip package */
                    else if((ch[4] == '+') && (ch[3] == 'I') && (ch[2] == 'P') && (ch[1] == 'D') && (ch[0] == ','))
                    {// switch to handle ip data"+IPD,"
                        recv_type = recv_ipdata_len;
                        rt_memset(&ch[0], 0, 4);
                        rt_memset(&ATRes_buff[0], 0, bufSz);
                        i = 0;
                    }
                    else if(thischar == '>')
                    {//
                        rt_event_send(sim900->promotion_mark, PROMOT_MARK);
                        rt_memset(&ATRes_buff[0], 0, bufSz);
                        i = 0;
                    }
                }
                break;
                case recv_ipdata_len:
                {   //(1)get data lenth
                    if(thischar != ':')
                    {
                         if(thischar >= '0'&& thischar <= '9')
                            ip_data_len = ip_data_len*10 + (thischar-'0');
                         else
                            rt_kprintf("ip data len error\r\n");
                    }
                    else
                    {
                        rt_kprintf("ip data len %d \r\n", ip_data_len);
                        recv_type = recv_ipdata;
                        j = 0;
                    }
                }
                break;
                case recv_ipdata:
                {   //(2)get ip data
                    if(j < ip_data_len)
                    {
                        rt_sem_take(&sem_rb_lock,RT_WAITING_FOREVER);
                        rb_put(&IPdata_rb, &thischar, 1);
                        rt_sem_release(&sem_rb_lock);
                        j++;
                    }
                    else
                    {   //get data accomplished
                        recv_type = recv_ats;
                        rt_kprintf("ip data recieved\r\n");
                        rt_sem_release(sim900->frame_sem);
                    }
                }
                default:
                break;
            }
        }
    }
}


/****************************************
*   rt_thread_sim900_app_entry          *
*   a thread for app of sim900          *
*   a serial operation to sim900        *
*   recieve event from other thread     *
*   to do the app ops                   *
*   ops for setup,dailcall,answercall   *
*   sms send and read, and gprs,tcpip   *
*   fault handle for these ops          *
****************************************/
//static void rt_thread_sim900_setup_entry(void* parameter)
//{
//    rt_uint16_t i = 0;
//    rt_uint8_t  j = 1;
//
//    rt_uint8_t  GPRSconnected = 0;
//    rt_uint8_t  CallInProgress = 0;
//    rt_err_t    errstatus;
//    rt_uint32_t e = 0;
//
//    if(sim900->device == RT_NULL)//if usart1 is not asigned
//        return;
//
//    sim900_device->status = INITIAL;
//    {/*INIT STATUS-->STANDBY STATUS*/
//        rt_thread_delay( RT_TICK_PER_SECOND*10 ); //delay 10s
//        sim900_Setup();         //setup sim900
//        //sim900_check_signal_quality();
//    }
//    sim900_device->status = STANDBY;
//
//    while(1)
//    {
//        if( (sim900_device->TCPUDP_autoconn == 1)&&(sim900_device->status == STANDBY) )
//        {
//            rt_event_send(&(sim900_device->OPreq_event), TCPUDP_CONN);
//        }
////1. waiting for request of call/sms/gprs/tcp/udp
//        rt_event_recv(  sim900_device->OPreq_event,
//                        INCOMING_CALL|OUTGOING_CALL|SIG_CHECK|TCPUDP_CONN||TCPUDP_DISCONN|TCPUDP_CLOSED,
//                        RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
//                        RT_WAITING_FOREVER, &e);
//
////(A). STANDBY-->IN_GPRS-->IN_TCPUDP_NONTRANSP
//        if( (sim900_device->status == STANDBY) && ((e & TCPUDP_CONN) == TCPUDP_CONN) )
//        {   //1.attach to GPRS   STANDBY-->IN_GPRS
//            while(sim900_device->status != IN_GPRS)      //if gprs is not connected
//                sim900_device->status = sim900_ConnectGPRS(); //keep trying until connect gprs
//            rt_kprintf("gprs connected\n"); //if connect success, got "CONNECT", GPRSconnected = 1
//            //2. tcp/ip connection IN_GPRS-->IN_TCPUDP_TRANSP
//            if(sim900_device->status  == IN_GPRS)
//            {//connect tcp/ip
//                if(sim900_device->remote_addr == RT_NULL)
//                    rt_strncpy(sim900_device->remote_addr, default_remoteIPADDR1, rt_strlen(default_remoteIPADDR1));//using default
//                //check remote ip
//                if(sim900_device->remote_port == RT_NULL)
//                    rt_strncpy(sim900_device->remote_port, default_remotePORT1, rt_strlen(default_remotePORT1));//using default
//                //check remote port
//                if(sim900_device->conn_type == RT_NULL)
//                    rt_strncpy(sim900_device->conn_type, default_connTYPE1, rt_strlen(default_connTYPE1));
//                //check conn type tcp/udp
//                rt_kprintf("connecting: 202.142.21.85:8080\n");
//                while(sim900_device->status != IN_TCPUDP_TRANSP)
//                  //sim900_device.status = sim900_connectTCPUDP(TCP);   //try tcp connect
//                rt_kprintf("tcp connected\r\n");//tcp_conn new success
//                rt_sem_release(&sim900_device->tcp_connected_sem);
//            }
//        }
//
//        /*(B). IN_TCPUDP_NONTRANSP-->STANDBY*/
//        if( ((e & TCPUDP_CLOSED) == TCPUDP_CLOSED) && (sim900_device->status == IN_TCPUDP_TRANSP) )
//        {   //this event comes from USART "CLOSED" in transparent mode
//            //TCP/UDP CONNECTION HAS BEEN CLOSED BY REMOTE
//            sim900_device->status = STANDBY; //sim900 switch t non-transparent mode automatically
//            rt_event_send(&(sim900_device->OPreq_response), TCPUDP_CLOSED);
//        }
//        if((e & TCPUDP_DISCONN) == TCPUDP_DISCONN)
//        {//dis connect tcp/udp by app
//            if(sim900_device->status == IN_TCPUDP_TRANSP)
//            {//go to esc mode
//            
//            }
//            //do disconn
//            sim900_device->status = STANDBY;
//            rt_event_send(&(sim900_device->OPreq_response), TCPUDP_DISCONN);
//        }
//
//        /*(C). STANDBY/IN_TCPUDP_TRANSP-->IN_CALL*/
//        if( (e & (INCOMING_CALL | OUTGOING_CALL) )> 0)
//        {
//            if(sim900_device->status == STANDBY)
//            {//do call
//            }
//            else if(sim900_device->status == IN_TCPUDP_TRANSP)
//            {//go to esc mode
//            //do call
//            //return from esc mode
//            }
//        }
//
//        /*(D). STANDBY/IN_TCPUDP_TRANSP-->SMS*/
//        if( ( e & (INCOMING_SMS| OUTGOING_SMS) ) > 0)
//        {
//            if(sim900_device->status == STANDBY)
//            {//do SMS
//            }
//            else if(sim900_device->status == IN_TCPUDP_TRANSP)
//            {//go to ecs mode
//            //do SMS
//            //return from esc mode
//            }
//        }
//
//        /*(E). STANDBY/IN_TCPUDP_TRANSP-->SIG_CHECK*/
//        if((e & SIG_CHECK) == SIG_CHECK)
//        {
//            if(sim900_device->status == STANDBY)
//            {//do SIG CHECK
//            }
//            else if(sim900_device->status == IN_TCPUDP_TRANSP)
//            {//go to ecs mode
//            //do SIG CHECK
//            //return from esc mode
//            }
//        }
//        
//        
//        
////4. if tcp/udp terminated by remote server, do reconnect
//        if( ( e & (TCPUDP_CLOSED) > 0)&&(sim900_device->status == IN_TCPUDP_TRANSP) )
//        {
//        }
////5. tcp/udp is not closed remotely
//        else
//        {//tcp/udp discconnect first
//            if(sim900_device->status == IN_TCPUDP_TRANSP)
//            {
//                //do esc procedure
//                sim900_device->status = IN_ESC;//set esc mode
//                rt_kprintf("switch to AT mode\n");
//            }
//            if(e & (INCOMING_CALL | OUTGOING_CALL)> 0)
//            {//do call
//                if( sim900_DailCall() == 1)
//                     CallInProgress = 1;
//                else
//                    rt_kprintf("dail failed\n");
//                while(CallInProgress == 1)//check call in progress
//                {
//                    CallInProgress = sim900_checkcall();
//                    rt_thread_delay(10000);
//                }
//                rt_sem_release(&(sim900_device->calldone_sem));//if call finished
//                rt_kprintf("call finished\n");
//            }
//            if( e & (INCOMING_SMS| OUTGOING_SMS) > 0)
//            {//turn to do sms
//            }
//            if( (e & SIG_CHECK) == SIG_CHECK)
//            {//  DO SIG_CHECK
//            }
//            if(sim900_device->status == IN_ESC)
//            {
//                //return to tcp conn:"ATO"
//                //sim900_device.status == IN_TCPUDP_NONTRANSP;
//            }
////                if( e & (TCP_DISCONN | UDP_DISCONN) > 0)
////                {//  do disconn
////                    sim900_sim900_disconnectTCPUDP();
////                    sim900_device.status ==STANDBY;
////                    rt_kprintf("tcp connection closed\n");
////                }
//
//        }
////7. switch sim900 to modem mode
//        
//
//    }
//
//}


/****************************************
*   sim900_HW_reset_entry               *
*   a thread for reset sim900 device    *
*   1. delete all threads if exist      *
*   2. H/W reset, include delay         *
*   3. init and startup threads         *
****************************************/
void sim900_reset_entry(void* parameter)
{
//	rt_err_t result;
    rt_thread_t tid;
//    GPIO_InitTypeDef GPIO_InitStructure;

//1. delete all threads if exist
//  if( (thread_sim900_rx.stat == RT_THREAD_INIT) || (thread_sim900_rx.stat == RT_THREAD_READY) ||
//      (thread_sim900_rx.stat == RT_THREAD_SUSPEND) || (thread_sim900_rx.stat == RT_THREAD_RUNNING) )
//      rt_thread_detach(&thread_sim900_rx);
//  if(thread_sim900_msgproc.stat !=RT_THREAD_CLOSE)
//      rt_thread_detach(&thread_sim900_msgproc);
//  if( (thread_sim900_setup.stat == RT_THREAD_INIT) || (thread_sim900_setup.stat == RT_THREAD_READY) ||
//      (thread_sim900_setup.stat == RT_THREAD_SUSPEND) || (thread_sim900_setup.stat == RT_THREAD_RUNNING) )
//      rt_thread_detach(&thread_sim900_setup);

//2. H/W reset, include delay
//rcc init
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//gpio init
//    GPIO_InitStructure.GPIO_Pin =  GPIOC_sim900_START;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    GPIO_ResetBits(GPIOC, GPIOC_sim900_START);
//    rt_thread_delay(100);
//    GPIO_SetBits(GPIOC, GPIOC_sim900_START);

//3. init and startup threads
    //add here: startup sim900 init process entry
    tid = rt_thread_create( "s900_rx",
                            rt_thread_sim900_rx_entry,
                            RT_NULL,
                            512, 8, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);

//    tid = rt_thread_create( "s900_stp",
//                            rt_thread_sim900_setup_entry,
//                            RT_NULL,
//                            1024, 3, 10);
//    if (tid != RT_NULL)
//        rt_thread_startup(tid);
}








/****************************************
*   rt_sim900_rx_ind                    *
*   hook func in usart isr              *
****************************************/
static rt_err_t rt_sim900_rx_ind(rt_device_t dev, rt_size_t size)
{
    RT_ASSERT(sim900 != RT_NULL);
//    rt_kprintf("#");
    /* release semaphore to notice thread parser rx data */
    rt_sem_release( sim900->rx_semaphore );
    
    return RT_EOK;
}

/************************************************
* This func set the I/O device for sim900       *
*   set usart device as virtual sim900          *
************************************************/
void rt_sim900_set_device(const char* device_name)
{
    rt_device_t dev = RT_NULL;
    RT_ASSERT(sim900 != RT_NULL);
    /* find usart device */
    dev = rt_device_find(device_name);

    if (dev != RT_NULL && rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        if (sim900->device != RT_NULL)
        {   /* close old sim900 virtual device */
            rt_device_close(sim900->device);
        }
        /* set new sim900 virtual device */
        sim900->device = dev;
        rt_device_set_rx_indicate(dev, rt_sim900_rx_ind);
    }
    else
    {
        rt_kprintf("sim900: can not find device:%s\n", device_name);
    }
}

/********************************************************
* This function returns current sim900 ported device.   *
* @return the sim900 ported device name is returned.    *
********************************************************/
const char* rt_sim900_get_device(void)
{
    RT_ASSERT(sim900 != RT_NULL);
    return sim900->device->parent.name;
}

/****************************************
*   rt_sim900_thread_startup            *
*   startup the sim900 reset entry      *
*   normal startup procedure:           *
*   1. sim900_device_init               *
*   2. rt_sim900_set_device             *
*   3. rt_sim900_thread_startup         *
****************************************/
void rt_sim900_thread_startup(void)
{
    rt_thread_t tid;
    tid= rt_thread_create(  "s900_rst",
                            &sim900_reset_entry,
                            RT_NULL,
                            512,2,10);
    if(tid != RT_NULL)
        rt_thread_startup(tid);
}

/*RT-Thread Device Driver Interface*/
/****************************************
*   rt_sim900_init                      *
*   AT CMD setup sim900 moduler         *
*   1. echo off                         *
*   2. ip head on                       *
*   3. read info                        *
****************************************/
static rt_err_t rt_sim900_init(void)
{
    rt_uint8_t  i = 0;
    rt_uint8_t  cmd[8];
    rt_uint8_t  CMDresp[8];
    rt_err_t    err;

    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*set eoch off*/
    rt_sprintf(cmd, "ATE0\r\n");
    do
    {
        rt_sim900_write(&cmd[0], rt_strlen(&cmd[0]));
        err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_TICK_PER_SECOND);//timeout
    }while(err != RT_EOK);
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        rt_kprintf("echo set off\r\n");
        return 1;//failed
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
    {
        rt_kprintf("echo set error\r\n");
        return 0;//failed
    }
    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*IP DATA HEAD set on*/
    rt_sprintf(cmd, "AT+CIPHEAD=1\r\n");
    do
    {
        rt_sim900_write(&cmd[0], rt_strlen(&cmd[0]));
        err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],RT_TICK_PER_SECOND);//timeout
    }while(err != RT_EOK);
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        rt_kprintf("ip head set on\r\n");
        return 1;//failed
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
    {
        rt_kprintf("ip head set error\r\n");
        return 0;//failed
    }
    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*add get sim900 info funcs*/

    return RT_EOK;
}

static rt_err_t rt_sim900_open(void)
{
    return RT_EOK;
}

static rt_err_t rt_sim900_close(void)
{
    return RT_EOK;
}

/********************************************
* This function read ipdata from sim900     *
* block type, use it in app thread          *
* paras: ip data buffer pointer             *
* @return the byte number of sim900 read    *
********************************************/
rt_size_t rt_sim900_read(void* buffer)
{
    rt_size_t   read_len;
    rt_uint8_t  i = 0;
    rt_uint8_t* buf = (rt_uint8_t*)buffer;

    if(sim900->status != IN_TCPUDP_NONTRANSP)
    {//check tcp/udp connnection
        rt_kprintf("sim900_read: tcp/udp conn error\r\n");
        return 0;
    }

    /*wait for ipdata recv*/
    //rt_event_recv(sim900->OPreq_event, IPDATA_IN, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
    rt_sem_take(sim900->frame_sem, RT_WAITING_FOREVER);

    /*read all byte in ring buffer*/
    rt_sem_take(&sem_rb_lock, RT_WAITING_FOREVER);
    while(rb_get(&IPdata_rb, &buf[i], 1) == RT_TRUE)
        i++;
    rt_sem_release(&sem_rb_lock);

    /*read done*/
    read_len = i;
    return read_len;
}

/********************************************
* This function write ATcmd to sim900       *
* @return the byte number of sim900 sent.   *
********************************************/
static rt_size_t rt_sim900_write(const void* buffer, rt_size_t size)
{
    rt_size_t write_len;
    rt_sem_take(&sem_sim900_lock,RT_WAITING_FOREVER);
    write_len = rt_device_write(sim900->device, 0, buffer, size);
    rt_sem_release(&sem_sim900_lock);
    return write_len;
}

/********************************************
* This function send ipdata to sim900       *
* non transparent mode                      *
* @return the byte number of sim900 sent.   *
********************************************/
static rt_size_t rt_sim900_DATAsend(const void* buffer, rt_size_t size)
{
    rt_size_t   write_len;
    rt_uint32_t e;

    /* check tcp/udp connnection */
    if(sim900->status != IN_TCPUDP_NONTRANSP)
    {
        rt_kprintf("sim900_send: tcp/udp conn error\r\n");
        return 0;
    }

    /*send ip data pack*/
    {
        rt_uint8_t  CMDresp[32];
        rt_uint8_t  TCPsendcmd[] = "AT+CIPSEND\r\n";//len=12
        rt_sim900_write(TCPsendcmd, strlen(TCPsendcmd));
        //wait for PROMOT_MARK
        rt_event_recv(  sim900->promotion_mark,
                        PROMOT_MARK,
                        RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                        RT_WAITING_FOREVER,
                        &e);
        rt_sim900_write(buffer, size);
        rt_sim900_write("\1A", 1);
        //wait for "send ok"
        rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_TICK_PER_SECOND);//timeout
        if(rt_strncmp( (char*)(&CMDresp[0]), "SEND OK", 7) == 0)
        {
            return write_len;
        }
        else if(rt_strncmp( (char*)(&CMDresp[0]), "SEND FAIL", 9) == 0)
        {
            return 0;
        }
    }
}





//拨打电话
rt_uint8_t sim900_DailCall(void)
{
    rt_uint8_t  i=0;
    rt_uint8_t  phone_number_len = 0;
    char str[25];
    rt_uint8_t  CMDresp[8];

    rt_memset(str, 0, sizeof(str));

    RT_ASSERT(sim900->CenterPhoneNumber != RT_NULL);
    phone_number_len = rt_strlen(sim900->CenterPhoneNumber);    
    RT_ASSERT(phone_number_len>=8);//at least 8 digi
    
    for(i=0; i<phone_number_len; i++)
    {
        if( (sim900->CenterPhoneNumber[i]>='0') && (sim900->CenterPhoneNumber[i]<='9') )
        {   //phone number is valid
            rt_kprintf("%c",sim900->CenterPhoneNumber[i]);
        }
        else
        {
            rt_kprintf("phone#%d: %c is invalid\n", i, sim900->CenterPhoneNumber[i]);
            return 0;
        }
     }

    rt_sprintf(str, "ATD%s;\r\n", sim900->CenterPhoneNumber);
    /*dail number 5 times*/
    for(i=0; i<5; i++)
    {
        rt_sim900_write(str, rt_strlen(str));
        rt_kprintf("dailing %s; %02d\n", sim900->CenterPhoneNumber, i);
        rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],5*RT_TICK_PER_SECOND);//timeout
        if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        {
            rt_kprintf("dail ok\r\n");
            return 1;//failed
        }
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        {
            rt_kprintf("dail error\r\n");
            return 0;//failed
        }
    }
}

rt_uint8_t sim900_FinishCall(void)
{
    rt_uint8_t  CMDresp[16];
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],RT_WAITING_FOREVER);//timeout
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        rt_kprintf("call hang up\r\n");
        return 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "BUSY", 4) == 0)
    {
        rt_kprintf("call busy\r\n");
        return 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "NO ANSWER", 9) == 0)
    {
        rt_kprintf("no answer\r\n");
        return 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "NO CARRIER", 10) == 0)
    {
        rt_kprintf("call finished\r\n");
        return 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
    {
        rt_kprintf("call finished error\r\n");
        return 0;
    }
}

//接电话
rt_uint8_t sim900_AnswerCall(void)
{
    rt_uint8_t  cmd[8] = "ATA\r\n";//len=5
    rt_uint8_t  CMDresp[8];
    rt_sim900_write(cmd, rt_strlen(cmd));
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        return 1;
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;
}

//挂断电话
rt_uint8_t sim900_HangUpCall(void)
{
    rt_uint8_t  cmd[8] = "ATH\r\n";//len=5
    rt_uint8_t  CMDresp[8];
    rt_sim900_write(cmd, rt_strlen(cmd));
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        return 1;
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;
}

/********************************************
* This function START TASK                  *
* set apn,username,password                 *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CSTT(rt_uint8_t* APN, rt_uint8_t* USER, rt_uint8_t* PASSWORD)
{
    rt_uint8_t  cmd[32];//len=10
    rt_uint8_t  CMDresp[8];
    
    rt_sprintf( cmd,
                "%s\"%s\",\"%s\",\"%s\"\r\n,"
                "AT+CSTT=",
                APN,
                USER,
                PASSWORD);
    rt_sim900_write(cmd, rt_strlen(cmd));
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        return 1;
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;
}

/********************************************
* This function bring up gprs               *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIICR(void)
{
    rt_uint8_t  cmd[16] = "AT+CIICR\r\n";//len=10
    rt_uint8_t  CMDresp[8];
    rt_sim900_write(cmd, rt_strlen(cmd));
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        return 1;
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;
}

/********************************************
* This function get local ip addr           *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIFSR(void)
{
    rt_uint8_t  cmd[16] = "AT+CIFSR\r\n";//len=10
    rt_uint8_t  CMDresp[16];
    rt_sim900_write(cmd, rt_strlen(cmd));
    rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout

//    rt_memcpy();

    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
        return 1;
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;
}

/********************************************
* This function query current conn status   *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIPStatus(void)
{
    rt_uint8_t cmd[32] = "AT+CIPSTATUS\r\n";
    rt_sim900_write(cmd, rt_strlen(cmd));
    
    return 1;
}

/********************************************
* This function establish tcp/udp conn      *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_ConnStart(rt_uint8_t* TYPE, rt_uint8_t* IP, rt_uint8_t* PORT)
{
    rt_uint8_t  cmd[64];
    rt_uint8_t  CMDresp[32];
    rt_err_t    err;

    rt_sprintf( cmd,
                "%s\"%s\",\"%s\",\"%s\"\r\n,"
                "AT+CIPSTART=",
                TYPE,
                IP,
                PORT);
    rt_kprintf("sim900_ConnStart:%s\r\n", cmd);

    do//wait for "OK"
    {
        rt_sim900_write(cmd, rt_strlen(cmd));
        err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], 5 * RT_TICK_PER_SECOND);//timeout
    }while(err != RT_EOK);
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        rt_memset(&CMDresp[0],0,sizeof(&CMDresp[0]));
        rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], 10 * RT_TICK_PER_SECOND);
        //wait for if "connect ok" then conn ok
        if(rt_strncmp( (char*)(&CMDresp[0]), "CONNECT OK", 10) == 0)
        {
            rt_kprintf("sim900_ConnStart: CONNECT OK\r\n");
            return 1;
        }
        else if(rt_strncmp( (char*)(&CMDresp[0]), "STATE:", 6) == 0)
        {   //if "state: tcp closed" and then "connect fail" then conn fail
            rt_memset(&CMDresp[0],0,sizeof(&CMDresp[0]));
            rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], 120*RT_TICK_PER_SECOND);
            if(rt_strncmp( (char*)(&CMDresp[0]), "CONNECT FAIL", 12) == 0)
                rt_kprintf("sim900_ConnStart: CONNECT FAIL\r\n");
            return 0;
        }
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
        return 0;   //failed
}

/********************************************
* This function close tcpudp                *
* use before cip shut                       *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIPCLOSE(void)
{
    rt_uint8_t cmd[16] = "AT+CIPCLOSE\r\n";//len=12
    rt_sim900_write(cmd, rt_strlen(cmd));
    //wait for "OK"
    return 1;
}

/********************************************
* This function deactive gprs               *
* use before tcp/ucp connection             *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIPSHUT(void)
{
    rt_uint8_t cmd[16] = "AT+CIPSHUT\r\n";//len=12
    rt_sim900_write(cmd, rt_strlen(cmd));
    //wait for "OK"
    return 1;
}

/********************************************
* This function signal qualicy check        *
* call by sim900_control funcs              *
* @return the number of signal quality.     *
********************************************/
rt_uint8_t sim900_SigCheck(void)
{
    rt_uint8_t  SQ = 0; //signal quality
    rt_err_t    err = 0;
    rt_uint8_t  cmd[8];
    rt_uint8_t  CMDresp[32];

    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*send sq cmd*/
    rt_sprintf(cmd, "AT+CSQ\r\n");
    do
    {
        rt_sim900_write(&cmd[0], rt_strlen(&cmd[0]));
        err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)(&CMDresp[0]), 5*RT_TICK_PER_SECOND);//timeout
    }while(err != RT_EOK);
    if(rt_strncmp( (char*)(&CMDresp[0]), "+CSQ:", 5) == 0)
    {
        sscanf( (char*)(&CMDresp[0]), "%*[^:]:%[^,]d", &SQ);//"+CSQ: 24,0"
        rt_kprintf("signal:%d\r\n", SQ);
        err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],5* RT_TICK_PER_SECOND);//timeout
        if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2))
            rt_kprintf("signal read OK\r\n");
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "+CME", 4) == 0)//"+CME ERROR:"
    {
        rt_kprintf("signal: %s\r\n", &CMDresp[0]);
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)//"ERROR"
    {
        rt_kprintf("signal read CMD error\r\n");
    }
    return SQ;  //sq
}

static rt_err_t rt_sim900_control(rt_uint8_t cmd, void *args)
{
    rt_uint8_t  result;
    rt_uint32_t sim900_cmd;
    rt_uint8_t* ATcmd;

    sim900_cmd = sim900_cmd_parse(cmd);

    switch(sim900_cmd)
    {
        case TCPUDP_CONN:   //do connect
            result = sim900_ConnStart(sim900->conn_type, sim900->remote_addr, sim900->remote_port);
            if(result == 1)
        break;
        case TCPUDP_DISCONN:
        
        break;
        case TCPUDP_CLOSED:
            //ATcmd[] = "AT+CIPSHUT\r\n";
            //rt_sim900_write(ATcmd , rt_strlen(ATcmd));
            //rt_kprintf("at:%s",ATcmd );
            //if(!isOK(500))
            //    return 0;
        break;
        case INCOMING_CALL://this is a callback branch by usart thread
            //send incoming call to app
            //send answer call cmd
            //wait for cmd ok
            //rt_event_send(sim900->OPreq_event, ANSWER_CALL);//send OP request to thread
            //err = rt_event_recv(sim900->OPreq_response, DONE);//wait for done
        break;
        case OUTGOING_CALL:
            result = sim900_DailCall();//send dial call cmd
            if(result = 1)//if dail ok, wait for call done
                result = sim900_FinishCall();
            //return call done status
        break;
        case HANGUP_CALL:
            result = sim900_HangUpCall();
        break;
        case SIG_CHECK:
            result = sim900_SigCheck();
            sim900->signalDB = 2*result - 114;  //save SQ as -114db ~ -52db
        break;
        default:
        break;
    }
    return result;
}

/****************************************
*   rt_hw_sim900_init                   *
*   init virtual device sim900          *
*   1. allocate mem                     *
*   2. init semaphore and msg queue     *
****************************************/
void rt_hw_sim900_init(const char* device_name)
{
    /*init sim device mem*/
    if(sim900 == RT_NULL)
        sim900 = (rt_sim900_device*)rt_malloc_align(sizeof(rt_sim900_device),RT_ALIGN_SIZE);
    rt_memset(sim900, 0, sizeof(rt_sim900_device));
    //sim900->status = INITIAL;

    /*init ip data buffer mem*/
    IPdata_buff = rt_malloc_align(1460, RT_ALIGN_SIZE);
    RT_ASSERT(IPdata_buff != RT_NULL);
    RT_ASSERT(&IPdata_rb != RT_NULL);
    rb_init(&IPdata_rb, &IPdata_buff[0], sizeof(IPdata_buff));

    /*init sem*/
    RT_ASSERT( rt_sem_init(&sem_sim900_lock, "s9_lock", 1, RT_IPC_FLAG_FIFO) == RT_EOK );
    RT_ASSERT( rt_sem_init(&sem_rb_lock, "rb_lock", 1, RT_IPC_FLAG_FIFO) == RT_EOK );

    RT_ASSERT( rt_sem_init(sim900->rx_semaphore, "s9_RXsem", 0, RT_IPC_FLAG_FIFO) == RT_EOK );
    RT_ASSERT( rt_sem_init(sim900->frame_sem, "s9_Fsem", 0, RT_IPC_FLAG_FIFO) == RT_EOK );

//    rt_sem_init(&(sim900->ok_sem), "s9_ok_sem", 0, RT_IPC_FLAG_FIFO);
//    rt_sem_init(&(sim900->err_sem), "s9_err_sem", 0, RT_IPC_FLAG_FIFO);
//    rt_sem_init(&(sim900->ring_sem), "s9_ring_sem", 0, RT_IPC_FLAG_FIFO);
//    rt_sem_init(&(sim900->connect_sem), "s9_cnnt_sem", 0, RT_IPC_FLAG_FIFO);
//    rt_sem_init(sim900->tcp_connected_sem, "C_CONNsem", 0, RT_IPC_FLAG_FIFO);
//    rt_sem_init(sim900->calldone_sem, "C_CALsem", 0, RT_IPC_FLAG_FIFO);
    
    RT_ASSERT( rt_event_init(sim900->ATResp_event, "C_RSPevt", RT_IPC_FLAG_FIFO) == RT_EOK );
    RT_ASSERT( rt_event_init(sim900->promotion_mark, "s9_PMsem", RT_IPC_FLAG_FIFO) == RT_EOK );
//    rt_event_init(sim900->OPreq_event, "C_REQevt", RT_IPC_FLAG_FIFO);

    sim900->ATMB_pool = rt_malloc_align(128, RT_ALIGN_SIZE);
    RT_ASSERT(sim900->ATMB_pool != RT_NULL);
    RT_ASSERT( rt_mb_init(sim900->ATMailBox, "s9_AT_MB", sim900->ATMB_pool, size(sim900->ATMB_pool)/4, RT_IPC_FLAG_FIFO) == RT_EOK);


//(2)set device
    rt_sim900_set_device(device_name);
}

















rt_int8_t getCCI(rt_uint8_t* cci)
{
	;
}

rt_int8_t getIMEI(rt_uint8_t* imei)
{

}


rt_int8_t sendSMS(const rt_uint8_t* to, const rt_uint8_t* msg)
{

}

rt_int8_t readSMS(rt_uint8_t* msg, rt_uint8_t msg_len, rt_uint8_t* number, rt_uint32_t nlen)
{

}


rt_int8_t readCall(rt_uint8_t* number, rt_int8_t len)
{

}

rt_int8_t Call(rt_uint8_t* number, rt_uint32_t milliseconds)
{

}

rt_uint8_t force(void)
{

}

rt_int32_t read(rt_uint8_t result, rt_int32_t len)
{

}

rt_int32_t readCellData(rt_uint8_t result, rt_int32_t len)
{

}

rt_int32_t SimpleRead(void)
{

}

rt_int32_t WhileSimpleRead(void)
{

}

void SimpleWrite(rt_uint8_t* comm)
{

}

void SimpleWriteln(rt_uint8_t* comm)
{

}

//private

rt_uint8_t configandwait(rt_uint8_t* pin)
{

}

rt_uint8_t setPIN(rt_uint8_t* pin)
{

}

rt_uint8_t changeNSIPmode(rt_uint8_t* pin)
{

}




