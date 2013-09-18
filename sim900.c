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
static struct rt_semaphore  sim900_lock;    //lock for sim900 OP
#define SIM900_LOCK()		rt_sem_take(&sim900_lock,RT_WAITING_FOREVER)
#define SIM900_UNLOCK()		rt_sem_release(&sim900_lock)

/*ring buffer for IP DATA*/
static  rt_uint8_t* IPdata_buff;
static  struct rb   IPdata_rb;
static struct rt_semaphore  rb_lock;        //lock for ipdata rb OP
#define RINGBUFFER_LOCK()	rt_sem_take(&rb_lock,RT_WAITING_FOREVER)
#define RINGBUFFER_UNLOCK()	rt_sem_release(&rb_lock)

//default ip & port
static rt_uint8_t   default_remoteIPADDR1[] = "202.142.21.85";
static rt_uint8_t   default_remotePORT1[]   = "8003";
static rt_uint8_t   default_connTYPE1[]     = "TCP";
//second default ip & port
static rt_uint8_t   default_remoteIPADDR2[] = "192.168.1.100";
static rt_uint8_t   default_remotePORT2[]   = "80";
static rt_uint8_t   default_connTYPE2[]     = "TCP";


static  rt_sim900_device*   sim900;

rt_size_t rt_sim900_write(const void* buffer, rt_size_t size);


/*
 * RT-Thread modem threads Driver
 * 2013-05-30 ben added for armfly board
 * sim900 rt device for armfly board
 */


rt_uint8_t GetCommLineStatus(void)
{
    return sim900->CommLineStatus;
}

void SetCommLineStatus(rt_uint8_t status)
{
    sim900->CommLineStatus = status;
}

//AT response are send to msg queue
//call by rt_thread_sim900_rx_entry
//parse "RING","CLOSED" response and send event seperately
//return err code
rt_err_t SendATResp(rt_uint8_t* at_resp_msg)
{
    rt_err_t ret;

    RT_ASSERT(sim900->AT_resp_MQ != RT_NULL);

    if(rt_strncmp( (char*)(&at_resp_msg[0]), "CLOSED", 6)==0)
    {//rt_sim900_control(TCPUDP_CLOSED);
        rt_event_send(sim900->ATResp_event, CLOSED);

    }
    else if(rt_strncmp( (char*)(&at_resp_msg[0]), "RING", 4)==0)
    {   //rt_sim900_control(INCOMING_CALL);
        if(GetCommLineStatus() != IN_CALL)
            rt_event_send(sim900->ATResp_event, RING);
    }
    else if(rt_strncmp( (char*)(&at_resp_msg[0]), "BUSY", 4)==0)
    {
        rt_event_send(sim900->ATResp_event, BUSY);
        //rt_sim900_control(INCOMING_CALL);
    }
    else if(rt_strncmp( (char*)(&at_resp_msg[0]), "NO CARRIER", 10)==0)
    {
        rt_event_send(sim900->ATResp_event, NO_CARRIER);
        //rt_sim900_control(INCOMING_CALL);
    }
    else if(rt_strncmp( (char*)(&at_resp_msg[0]), "NO ANSWER", 9)==0)
    {
        rt_event_send(sim900->ATResp_event, NO_ANSWER);
        //rt_sim900_control(INCOMING_CALL);
    }
    else
        ret = rt_mq_send(sim900->AT_resp_MQ, at_resp_msg, rt_strlen((char*)&at_resp_msg[0]));

    if(ret == -RT_EFULL)
        rt_kprintf("AT response MQ is full\r\n");

    return ret;
}
//AT response in msg queue are captured by thread call this function
//must call by a thread
//will block thread for max_tout
//return length of response string
rt_int8_t WaitATResp(rt_uint8_t* get_resp_str, rt_uint32_t max_tout )
{
    rt_uint8_t	ATResp_buff[SIM900_AT_RESP_MQ_MSG_SIZE];// buffer for nsg recv from MQ
    rt_int8_t	i = 0;      // less than SIM900_AT_RESP_MQ_MSG_SIZE:32
    rt_err_t	ret;

    //ATResp_buff = rt_malloc_align(SIM900_AT_RESP_MQ_MSG_SIZE, RT_ALIGN_SIZE);
    rt_memset(ATResp_buff, 0, sizeof(ATResp_buff));

    //read at response msg queue
    RT_ASSERT(sim900->AT_resp_MQ != RT_NULL);
    ret = rt_mq_recv(sim900->AT_resp_MQ, ATResp_buff, SIM900_AT_RESP_MQ_MSG_SIZE,  max_tout);

    if( ret == RT_EOK )
    {
        i = rt_strlen(ATResp_buff);//get string length
        rt_strncpy((char*)get_resp_str, (char*)ATResp_buff, i);
        //rt_free_align(ATResp_buff);
    }
    else
    {
        rt_kprintf("AT response recieve failed\r\n");
        i = -1;
    }

    return i;
}

//simple function: wrap the event op for ipdata recvieved
rt_err_t ipdata_recieved(void)
{
    return rt_sem_release(sim900->frame_sem);
}
rt_uint8_t ipdata_arrived(void)
{
    if(rt_sem_take(sim900->frame_sem, RT_WAITING_FOREVER) == RT_EOK)
    	return 1;
    else
    	return 0;
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
    rt_uint8_t  thischar = 0;
    rt_uint8_t  ch[5];

    rt_uint8_t  recv_type = recv_ats;
    rt_uint8_t* ATRes_buff;
    rt_uint8_t  bufSz = 32;
    rt_uint16_t ip_data_len = 0;

    ATRes_buff = (rt_uint8_t*)rt_malloc_align(bufSz, RT_ALIGN_SIZE);
    rt_memset(&ATRes_buff[0], 0, bufSz);

    rt_memset(&ch[0], 0, sizeof(&ch[0]));

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
                        if(ATRes_buff[0] > 0 && ATRes_buff[0] < 128)//content is a ascii char
                            SendATResp(&ATRes_buff[0]); //send at cmd response
                            //ATresp_Process(&ATRes_buff[0]);//resolve at response and send RT event
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
                        SetCommLineStatus(IN_TCPUDP_NONTRANSP);
                    }
                    else if(thischar == '>')
                    {//
                        SendATResp(&thischar);
                        //rt_event_send(sim900->promotion_mark, PROMOT_MARK);
                        rt_memset(&ATRes_buff[0], 0, bufSz);
                        i = 0;
                    }
                }
                break;
                case recv_ipdata_len:
                {   //(1)get data length
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
                        RINGBUFFER_LOCK();
                        rb_put(&IPdata_rb, &thischar, 1);
                        RINGBUFFER_UNLOCK();
                        j++;
                    }
                    else
                    {   //get data accomplished
                    	SetCommLineStatus(STANDBY);
                        recv_type = recv_ats;
                        rt_kprintf("ip data recieved\r\n");
                        ipdata_recieved();
                    }
                }
                default:
                break;
            }
        }
    }//while
    rt_free_align(ATRes_buff);
}




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

    /*setup sim900 CommLine*/
    rt_sim900_init();
}


/****************************************
*   rt_sim900_thread_startup            *
*   startup the sim900 reset entry      *
*   normal startup procedure:           *
*   1. rt_sim900_hw_init                *
*   2. rt_sim900_thread_startup         *
****************************************/
void rt_sim900_thread_startup(void)
{
    rt_thread_t tid;
    tid= rt_thread_create(  "s900_rst",
                            &sim900_reset_entry,
                            RT_NULL,
                            512, 2, 10);
    if(tid != RT_NULL)
        rt_thread_startup(tid);
}






/****************************************
*   sim900 tcp/udp functions            *
*   send data, recv data,connect tcp/udp*
*   return 0 failed, 1 success          *
****************************************/




/********************************************
* This function read ipdata from sim900     *
* block type, use it in app thread          *
* paras: ip data buffer pointer             *
* @return the byte number of sim900 read    *
********************************************/
rt_size_t sim900_readDATA(void* buffer)
{
    rt_size_t   read_len;
    rt_uint8_t  i = 0;
    rt_uint8_t* buf = (rt_uint8_t*)buffer;

    /*wait for ipdata recv*/
    rt_memset(&buf[0], 0, sizeof(&buf[0]));
    if(ipdata_arrived())
    {/*read all byte in ring buffer*/
        RINGBUFFER_LOCK();
        while(rb_get(&IPdata_rb, &buf[i], 1) == RT_TRUE)
            i++;
        RINGBUFFER_UNLOCK();
        /*read done*/
    }

    read_len = i;
    return read_len;
}
//example thread for app read ipdata
static void rt_thread_sim900_ipdataread_entry(void* parameter)
{
    rt_uint8_t   buffer[1460];//maximum
    rt_size_t    len;
    while(1)
    {
        len = sim900_readDATA(&buffer[0]);
        //do something with the data;
    }
}

/********************************************
* This function send ipdata to sim900       *
* non transparent mode                      *
* @return the byte number of sim900 sent.   *
********************************************/
rt_size_t sim900_sendDATA(const void* buffer, rt_size_t size)
{
    rt_size_t   write_len;
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t  TCPsendcmd[] = "AT+CIPSEND\r\n";//len=12

    /*send ip data pack*/
    rt_sim900_write(TCPsendcmd, rt_strlen((char*)TCPsendcmd));

    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0], RT_TICK_PER_SECOND) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), ">", 1) == 0)//wait for PROMOT_MARK
        {
        	SetCommLineStatus(IN_TCPUDP_NONTRANSP);
            rt_sim900_write(buffer, size);
            rt_sim900_write("\1A", 1);
        }
    }

    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
    {//wait for "send ok"
        if(rt_strncmp( (char*)(&CMDresp[0]), "SEND OK", 7) == 0)
        {
        	SetCommLineStatus(IN_TCPUDP_NONTRANSP);
            return write_len;
        }
        else if(rt_strncmp( (char*)(&CMDresp[0]), "SEND FAIL", 9) == 0)
        {
            SetCommLineStatus(IN_TCPUDP_NONTRANSP);
            return 0;
        }
    }
    return 0;
}







/****************************************
*   sim900 call functions               *
*   Call outgoing operation             *
*   return 0 failed, 1 success          *
****************************************/
rt_uint8_t sim900_Call(rt_uint8_t* number_str)
{
    rt_uint8_t  i=0;
    rt_uint8_t  phone_number_len = 0;
    rt_uint8_t  number[16];
    char str[25];
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];

    /* comm line is ARLREADY IN CALL */
    if( GetCommLineStatus() == IN_CALL)
        return 0;

    SetCommLineStatus(IN_CALL);//set comm line occupied

    rt_memset(str, 0, sizeof(str));
    if(number_str != RT_NULL)
    {//use input phone number
        phone_number_len = rt_strlen((char*)number_str);
        RT_ASSERT(phone_number_len>=5);//at least 8 digi
        rt_memcpy( number, number_str, phone_number_len );
    }
    else
    {//use default call center number
        RT_ASSERT(sim900->CenterPhoneNumber != RT_NULL);
        phone_number_len = rt_strlen((char*)sim900->CenterPhoneNumber);
        RT_ASSERT(phone_number_len>=5);//at least 8 digi
        rt_memcpy( number, sim900->CenterPhoneNumber, phone_number_len );
    }

    /*number string --> number & check number validation*/
    for(i=0; i<phone_number_len; i++)
    {
        if( (number[i]>='0') && (number[i]<='9') )
        {   //phone number is valid
            rt_kprintf("%c",number[i]);
        }
        else
        {
            rt_kprintf("phone#%d: %c is invalid\n", i, number[i]);
            SetCommLineStatus(STANDBY);//set comm line free
            return 0;
        }
     }
    rt_sprintf(str, "ATD%s;\r\n", number);

    /*dail number 5 times*/
    for(i=0; i<5; i++)
    {
        rt_sim900_write(str, rt_strlen(str));	//SEND dial CMD
        rt_kprintf("dailing %s; %02d\n", number, i);

        //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],5*RT_TICK_PER_SECOND);//timeout
        //	wait for response string
        if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
        {
            if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            {// if "OK", dail success
                rt_kprintf("dail ok\r\n");
                return 1;//
            }
            else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            {//if "error", dail failed
                rt_kprintf("dail error\r\n");
                SetCommLineStatus(STANDBY);//set comm line free
                return 0;//failed
            }
            else if(rt_strncmp( (char*)(&CMDresp[0]), "NO DIALTONE", 11) == 0)
            {//if "error", dail failed
                rt_kprintf("no dialtone\r\n");
                SetCommLineStatus(STANDBY);//set comm line free
                return 0;//failed
            }
            else
            {
                rt_kprintf("ATD RECV: %s\r\n",&CMDresp[0]);
            }
        }
    }
    SetCommLineStatus(STANDBY);//set comm line free
    return 0; //for exception
}

//simple function for waiting for in coming call
rt_uint8_t sim900_Call_ComeIn(void)
{
    rt_uint32_t e = 0;
    rt_uint8_t  state;
    rt_err_t    err;
    //block and waiting for in coming call event
    err = rt_event_recv(sim900->ATResp_event,
                        RING,
                        RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                        RT_WAITING_FOREVER, &e);

    if (err == RT_EOK)
        state = 1;
    else
        state = 0;
    return state; // 1=success
}

//answer in coming call op
//return 0 failed
//return 1 success
rt_uint8_t sim900_PickUp(void)
{
    rt_uint8_t  cmd[8] = "ATA\r\n";//len=5
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t	state;

    if(GetCommLineStatus() == IN_CALL)//if already in call
        return 0;

    rt_sim900_write(cmd, rt_strlen((char*)cmd));
    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND);		//	wait for response string
    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        SetCommLineStatus(IN_CALL);
        state = 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
    {
    	state = 0;
    }
    return state;
}

//hang up call op
//send hangup cmd
//response handled by finish call func
void sim900_HangUp(void)
{
    rt_uint8_t  cmd[8] = "ATH\r\n";//len=5
//    rt_uint8_t  CMDresp[8];

    if(GetCommLineStatus() == IN_CALL)
        rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
//    WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND);	//	wait for response string
//    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
//        return 1;
//    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
//        return 0;
}

//finish call handler
//handle all call finish status
//return 0 failed
//return 1 success
rt_uint8_t sim900_FinishCall(void)
{
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t	status = 0;

    if(GetCommLineStatus() != IN_CALL)
    	return 0;	// not in call mode

    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],RT_WAITING_FOREVER);//timeout
    WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND);	//	wait for response string

    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
    {
        rt_kprintf("call hang up\r\n");
        status = 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "BUSY", 4) == 0)
    {
        rt_kprintf("call busy\r\n");
        status = 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "NO ANSWER", 9) == 0)
    {
        rt_kprintf("no answer\r\n");
        status = 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "NO CARRIER", 10) == 0)
    {
        rt_kprintf("call finished\r\n");
        status = 1;
    }
    else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
    {
        rt_kprintf("call finished error\r\n");
        status = 0;
    }
    SetCommLineStatus(STANDBY);//set comm line free
    return status;
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
//example thread for incoming call handleing
static void rt_thread_sim900_IncomingCal_entry(void* parameter)
{
    while(1)
    {
        if( sim900_Call_ComeIn() )
        {
        	//pickup or reject
            sim900_PickUp();//sim900_HangUp();
            //notice: CommLineStatus = IN_CALL
            sim900_FinishCall();
        }
    }
}

//example thread for out going call handleing
static void rt_thread_sim900_OutgoingCall_entry(void* parameter)
{
    while(1)
    {
        sim9000_call("10086");
        //notice: CommLineStatus = IN_CALL
        sim900_FinishCall();
    }
}








/********************************************
* This function START TASK                  *
* set apn,username,password                 *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CSTT(rt_uint8_t* APN, rt_uint8_t* USER, rt_uint8_t* PASSWORD)
{
    rt_uint8_t  cmd[32];//len=10
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t	state = 0;
    
    rt_sprintf( (char*)&cmd[0],
                "%s\"%s\",\"%s\",\"%s\"\r\n,"
                "AT+CSTT=",
                APN,
                USER,
                PASSWORD);
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    if(WaitATResp(&CMDresp[0],RT_WAITING_FOREVER) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            state = 1;
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            state = 0;
    }
    return state;
}

/********************************************
* This function bring up gprs               *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIICR(void)
{
    rt_uint8_t  cmd[16] = "AT+CIICR\r\n";//len=10
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t	state = 0;

    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0],RT_WAITING_FOREVER) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            state = 1;
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            state = 0;
    }
    return state;
}

/********************************************
* This function get local ip addr           *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIFSR(void)
{
    rt_uint8_t  cmd[16] = "AT+CIFSR\r\n";//len=10
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t	state = 0;

    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    //rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_WAITING_FOREVER);//timeout
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0],RT_WAITING_FOREVER) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            state = 1;
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            state = 0;
    }
    return state;
}

/********************************************
* This function query current conn status   *
*                                           *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIPStatus(void)
{
    rt_uint8_t cmd[32] = "AT+CIPSTATUS\r\n";
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    
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
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t  state = 0;
    rt_uint8_t  i;
    rt_sprintf( (char*)cmd[0],
                "%s\"%s\",\"%s\",\"%s\"\r\n,"
                "AT+CIPSTART=",
                TYPE,
                IP,
                PORT);
    rt_kprintf("sim900_ConnStart:%s\r\n", cmd);

    for (i = 0; i < 10; i++)
    {//try 5 times
    	rt_kprintf("sim900 Connecting: %d\r\n", i);
        rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
        rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
        //err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], 5 * RT_TICK_PER_SECOND);//timeout
        if(WaitATResp(&CMDresp[0], 10*RT_TICK_PER_SECOND) > 0)
        {
            if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            {//wait for "OK"
                rt_memset(&CMDresp[0],0,sizeof(&CMDresp[0]));
                if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
                {//wait for if "connect ok" then conn ok
                    if(rt_strncmp( (char*)(&CMDresp[0]), "CONNECT OK", 10) == 0)
                    {
                        rt_kprintf("sim900_ConnStart: CONNECT OK\r\n");
                        return 1;
                    }
                    else if(rt_strncmp( (char*)(&CMDresp[0]), "ALREADY CONNECT", 15) == 0)
                    {
                        rt_kprintf("sim900_ConnStart: ALREADY CONNECT\r\n");
                        return 0;
                    }
                    else if(rt_strncmp( (char*)(&CMDresp[0]), "STATE:", 6) == 0)
                    {   //if "state: tcp closed" and then "connect fail" then conn fail
                        rt_memset(&CMDresp[0],0,sizeof(&CMDresp[0]));
                        if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
                        {
                            if(rt_strncmp( (char*)(&CMDresp[0]), "CONNECT FAIL", 12) == 0)
                                rt_kprintf("sim900_ConnStart: CONNECT FAIL\r\n");
                            return 0;
                        }
                    }
                }
                state = 1;
        	}
        	else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
       			state = 0;
        	else if(rt_strncmp( (char*)(&CMDresp[0]), "+CME", 4) == 0)
        		state = 0;
        }
    }

    return state;   //
}

/********************************************
* This function close tcpudp                *
* use before cip shut                       *
* @return the byte number of sim900 sent.   *
********************************************/
rt_uint8_t sim900_CIPCLOSE(void)
{
    rt_uint8_t cmd[16] = "AT+CIPCLOSE\r\n";//len=12
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
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
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));
    //wait for "OK"
    return 1;
}

//simple function to transfer rssi 2 signal DB number
//return: -115~-52
rt_int8_t rssi2db(rt_uint8_t rssi)
{
    rt_int8_t db = 0;
    if(rssi == 0)
        db = -115;
    else if (rssi == 1)
        db = -111;
    else if (rssi >=2 && rssi <=30)
        db = 2*rssi-114;
    else if (rssi >= 31 && rssi < 99)
        db = -52;
    else
        db = 0;
    return db;
}
/********************************************
* This function signal qualicy check        *
* call by sim900_control funcs              *
* @return the number of signal quality.     *
********************************************/
rt_uint8_t sim900_SigCheck(void)
{
    rt_uint8_t  SQ = 0; //signal quality
    rt_uint8_t  state = 0;
    rt_uint8_t  i = 0;
    rt_uint8_t  cmd[8];
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];

    /*send sq cmd*/
    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_sprintf((char*)&cmd[0], "AT+CSQ\r\n");

    for(i = 0; (i<5)&&(state!=1); i++)
    {
        rt_sim900_write(&cmd[0], rt_strlen((char*)&cmd[0]));
        rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
        //err = rt_mq_recv(sim900->AT_resp_MQ, &CMDresp[0], sizeof(CMDresp), 5*RT_TICK_PER_SECOND);//timeout
        if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
        {
            if(rt_strncmp( (char*)(&CMDresp[0]), "+CSQ:", 5) == 0)
            {
                sscanf( (char*)(&CMDresp[0]), "%*[^:]:%[^,]d", &SQ);//etc:"+CSQ: 24,0"
                rt_kprintf("signal:%d\r\n", SQ);
                sim900->rssi = SQ;//SQ:0--99
                sim900->signalDB = rssi2db(SQ);
                rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
                //err = rt_mq_recv(sim900->AT_resp_MQ, &CMDresp[0], sizeof(CMDresp), 5* RT_TICK_PER_SECOND);//timeout
                if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
                {
                    if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
                        rt_kprintf("signal read OK\r\n");
                    state = 1;
                }
            }
            else if(rt_strncmp( (char*)(&CMDresp[0]), "+CME", 4) == 0)//"+CME ERROR:"
            {
                rt_kprintf("signal: %s\r\n", &CMDresp[0]);
            }
            else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)//"ERROR"
            {
                rt_kprintf("signal read CMD error\r\n");
            }
        }
    }//for
    return state;  //sq
}

//simple function for set device ID info
void sim900_setID_Info(rt_uint8_t* id, rt_uint8_t* info)
{
    if(id != RT_NULL)
    {
        rt_free_align(id);
    }
    id = (rt_uint8_t*)rt_malloc_align(rt_strlen((char*)info), RT_ALIGN_SIZE);
    rt_strncpy((char*)id, (char*)info, rt_strlen((char*)info));
}

/********************************************
* This function ask for sim900 ID info      *
* use when sim900 startup                   *
* @return 1=success, 0 = failed.            *
********************************************/
rt_uint8_t sim900_getID_Info(void)
{
    rt_uint8_t  cmd[8];//len=8
    rt_uint8_t  buffer[30];
    rt_uint8_t  CMDresp[SIM900_AT_RESP_MQ_MSG_SIZE];
    rt_uint8_t  state = 0;

    rt_memset(&buffer[0], 0, sizeof(&buffer[0]));
    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_sprintf((char*)&cmd[0], "ATI\r\n");
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));

    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), "SIM", 3) == 0)
        {//ID starts with "SIM"
        	sim900_setID_Info(sim900->id, &CMDresp[0]);

            rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
            if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
            {
                if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
                    rt_kprintf("ID info read OK\r\n");
                state = 1;
            }//wait for ok
        }//wait for ID
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 2) == 0)
        {
             rt_kprintf("ID info read error\r\n");
             state = 0;
        }
    }

    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_sprintf((char*)&cmd[0], "AT+GSV\r\n");
    rt_sim900_write(cmd, rt_strlen((char*)&cmd[0]));

    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
    if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
    {
        if(rt_strncmp( (char*)(&CMDresp[0]), "SIM", 3) == 0)
        {//1.manufacturer start with "SIM"
            sim900_setID_Info(sim900->manufacturer, &CMDresp[0]);

            rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
            if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
            {
                if(rt_strncmp( (char*)(&CMDresp[0]), "SIM", 3) == 0)
                {//2.type start with"SIM"
                    sim900_setID_Info(sim900->type, &CMDresp[0]);

                    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
                    if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
                    {
                        if(rt_strncmp( (char*)(&CMDresp[0]), "Rev", 3) == 0)
                        {//3. revision start with "Rev"
                            sscanf( (char*)(&CMDresp[0]), "Revision:%s", &buffer[0]);//"Revision:1137B...
                            sim900_setID_Info(sim900->sn, &buffer[0]);

                            rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));
                            if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
                            {
                                if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
                                {//4. OK
                                    rt_kprintf("ID info read ok\r\n");
                                    state = 1;
                                }
                            }//WAIT FOR OK
                        }
                    }//WAIT FOR REVISION
                }
            }//WAIT FOR TYPE
        }//WAIT FOR MANUFACTURER
        else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 2) == 0)
        {
             rt_kprintf("ID info read error\r\n");
             state = 0;
        }
    }
    return state;
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
    rt_uint8_t  cmd[8];
    rt_uint8_t  CMDresp[8];
    rt_uint8_t	state = 0;

    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*set eoch off*/
    rt_sprintf((char*)&cmd[0], "ATE0\r\n");
    do
    {
        rt_sim900_write(&cmd[0], rt_strlen((char*)&cmd[0]));
        if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
        {// get response
            if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            {
                rt_kprintf("echo set off\r\n");
                state = 1;
            }
            else if (rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            {
                rt_kprintf("echo set off error, check comm line\r\n");
                return 0;	//failed
            }
        }
        //err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0], RT_TICK_PER_SECOND);//timeout
    }while(state != 1);

    state = 0;
    rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*IP DATA HEAD set on*/
    rt_sprintf((char*)&cmd[0], "AT+CIPHEAD=1\r\n");
    do
    {
        rt_sim900_write(&cmd[0], rt_strlen((char*)&cmd[0]));
        if(WaitATResp(&CMDresp[0], 5*RT_TICK_PER_SECOND) > 0)
        {// get response
            if(rt_strncmp( (char*)(&CMDresp[0]), "OK", 2) == 0)
            {
                rt_kprintf("ip head set on\r\n");
                state = 1;
            }
            else if(rt_strncmp( (char*)(&CMDresp[0]), "ERROR", 5) == 0)
            {
                rt_kprintf("ip head set error\r\n");
                return 0;//failed
            }
        }
        //err = rt_mb_recv(sim900->ATMailBox, (rt_uint32_t*)&CMDresp[0],RT_TICK_PER_SECOND);//timeout
    }while(state != 1);

    //state = 0;
    //rt_memset(&cmd[0], 0, sizeof(&cmd[0]));
    //rt_memset(&CMDresp[0], 0, sizeof(&CMDresp[0]));

    /*add get sim900 info funcs*/

    return state;
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
* This function write ATcmd to sim900       *
* @return the byte number of sim900 sent.   *
********************************************/
static rt_size_t rt_sim900_write(const void* buffer, rt_size_t size)
{
    rt_size_t write_len;
    SIM900_LOCK();
    write_len = rt_device_write(sim900->device, 0, buffer, size);
    SIM900_UNLOCK();
    return write_len;
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
            if(result == 1)//if dail ok, wait for call done
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

//simple function for IP data buffer init
void IPDataBuff_init(void)
{
    IPdata_buff = (rt_uint8_t*)rt_malloc_align(1460, RT_ALIGN_SIZE);
    RT_ASSERT(IPdata_buff != RT_NULL);
    RT_ASSERT(&IPdata_rb != RT_NULL);
    rb_init(&IPdata_rb, &IPdata_buff[0], sizeof(IPdata_buff));
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

    SetCommLineStatus(INITIAL);

    /*init ip data buffer mem*/
    IPDataBuff_init();

    /*init sem*/
    RT_ASSERT( rt_sem_init(sim900->rx_semaphore, "s9_RXsem", 0, RT_IPC_FLAG_FIFO) == RT_EOK );

    RT_ASSERT( rt_sem_init(&sim900_lock, "s9_lock", 1, RT_IPC_FLAG_FIFO) == RT_EOK );
    RT_ASSERT( rt_sem_init(&rb_lock, "rb_lock", 1, RT_IPC_FLAG_FIFO) == RT_EOK );
    RT_ASSERT( rt_sem_init(sim900->frame_sem, "s9_Fsem", 0, RT_IPC_FLAG_FIFO) == RT_EOK );
    
    RT_ASSERT( rt_event_init(sim900->ATResp_event, "C_RSPevt", RT_IPC_FLAG_FIFO) == RT_EOK );


    //at response msg queue.
    sim900->AT_resp_MQ_poll = (rt_uint8_t*)rt_malloc_align(SIM900_AT_RESP_MQ_POOL_SIZE, RT_ALIGN_SIZE);
    RT_ASSERT(sim900->AT_resp_MQ_poll != RT_NULL);
    RT_ASSERT( rt_mq_init(sim900->AT_resp_MQ, "s9_ATRMQ", sim900->AT_resp_MQ_poll, SIM900_AT_RESP_MQ_MSG_SIZE-sizeof(void*), SIM900_AT_RESP_MQ_POOL_SIZE, RT_IPC_FLAG_FIFO) == RT_EOK );

    //(2)set device
    rt_sim900_set_device(device_name);
}





















