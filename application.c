/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date             Author          Notes
 * 2009-01-05       Bernard         the first version
 * 2011-09-19       benjamin        rtc app added
 * 2012-09-19       benjamin_Zhao   nand flash fs added
 */
#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include "dfs_def.h"
#include "dfs_posix.h"
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/driver.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
extern void panel_init(void);
extern void workbench_init(void);
#endif

#include <stdio.h>
#include <time.h>
#include "stm32f10x.h"


#include "tft_lcd.h"
#include "touch.h"
#include "ch376.h"
#include "spi_flash.h"
#include "bsp_button.h"
#include "gps.h"
#include "led.h"
#include "cellular.h"

//struct rt_semaphore   rtc_disp_sem;
struct  rt_thread   thread_usb;
void rt_thread_cellular_app_entry(void* parameter);


/************************
*   init thread         *
* file system init now	*
************************/
void rt_init_thread_entry(void *parameter)
{
    uint8_t err;
    rt_device_t devicep;
    rt_thread_t tid;
    int temp = 0;//0 surpose success
   
    /* Filesystem Initialization */
    #ifdef RT_USING_DFS
    {
        /* init dfs */
        dfs_init();
        /* init the elm FAT filesystam*/
        elm_init();

        /* mount spi flash fat as root dir */
        #if (SPI_FLASH_TYPE > 0)
        {   /*init spi flash*/
            rt_hw_spi_flash_init();
        }
        #endif
        /*temp = mkdir("/spi0",0);*/
        if (dfs_mount("spi0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("SPI File System initialized!\n");
        }
        else
        {
            rt_kprintf("SPI File System init failed!\n");
        }

/*space for nand falsh fs*/


/*space for nand falsh fs*/

        /* mount SD CARD fat as sd0 directory */
        #if (STM32_USE_SDIO == 1)   // init sdcard
        {   /*using sdio driver*/
            rt_hw_sdcard_init();
        }
        #elif (STM32_USE_SDIO == 2)
        {
            rt_hw_msd_init();
        }
        #endif
        #if (STM32_USE_SDIO > 0)
        /*try open dir "/sd0" */
        if(opendir("/sd0") == RT_NULL)
        {   /* if dir does not exist, make it 0: create success*/
            temp = mkdir("/sd0",0);
        }
        else
        {   /* if dir exists, mark it*/
            temp = 0;
        }

        /*check if dir "sd0" is ready? and do the mount*/
        if(temp == 0)
        {   /*if "/sd0" dir exists, mount sdcard to "/sd0" dir*/
            if (dfs_mount("sd0", "/sd0/", "elm", 0, 0) == 0)
            {   /*if mount success,print mount success*/
                rt_kprintf("SD File System initialized!\n");
            }
            else
            {   /*if mount failed,print mount failed*/
                rt_kprintf("SD File System init failed!\n");
            }
        }
        else
        {   /*if make dir failed,print make failed*/
            rt_kprintf("mkdir /sd0 failed!\n");
        }
        temp = 0;
        #endif

        /*init ch376 and usb host*/
        #if (STM32_USE_USBHOST > 0)
        {   /*if use usb host,do the init*/
            err = mInitCH376Host();
        }
        if(err != USB_INT_SUCCESS)
        {   /*if ch376 usb host init failed*/
            /*print failed and suspend "usb thread"*/
            rt_kprintf("ch376 init failed, err code:%X\n",err);
            rt_thread_suspend(&thread_usb);
            //rt_schedule();//stop this thread
        }
        /*else "usb thread" will start running automatically*/
        /* monitoring and mounting USB host fat at "/usb0" dir */
        /*DO NOT MOUNT USB HERE*/
        #endif
    }
    #endif

    /* STARTUP workbench */
    #ifdef USING_LCD
        rt_hw_lcd_init();
        rt_hw_touch_init();
        rt_device_init_all();
    #endif
    #ifdef RT_USING_RTGUI
        devicep = rt_device_find("TFT LCD");
        rtgui_graphic_set_device(devicep);
        /* GUI系统初始化 */
        rtgui_system_server_init();
        /* 各个面板初始化 */
        panel_init();
        workbench_init();
    #endif
    
    /*enable rtc sec interrupt*/
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    
    /*add for buttons*/
    rt_hw_button_init();

    /*init thread and driver for GPS*/
    rt_gps_init();
    /*config GPS IO*/
    rt_gps_set_device(GPS_DEVICE);

/*init celluar and startup*/
    rt_hw_cellular_init(CELLULAR_DEVICE);
    rt_cellular_thread_startup();
    
    tid= rt_thread_create(  "cell_app",
                            &rt_thread_cellular_app_entry,
                            RT_NULL,
                            1024, 5, 10);
    if(tid != RT_NULL)
        rt_thread_startup(tid);
        


/*startup LED thread*/
    tid = rt_thread_create( "led",
                            rt_thread_ledbreath2_entry,
                            RT_NULL,
                            256, 12, 50);
    rt_thread_startup(tid);
}

/************************
*	thread lcd brt adj	*
*	using tim3 & pwm	*
************************/
//ALIGN(RT_ALIGN_SIZE)
//static char                   thread_destop_stack[512];
//static struct rt_thread       thread_destop;
//static void rt_thread_destop_entry(void* parameter)
//{
//  char str[30];
//  char str1[30];
//  char str2[30];
//  char str3[30];
//  char str4[30];
//  FONT_T tFont;		/* 定义一个字体结构体变量，用于设置字体参数 */
//  FONT_T tFont1;
//  touchPosition touchpos;
//  memset(str, 0, 30);
//  memset(str1, 0, 30);
//  memset(str2, 0, 30);
//  memset(str3, 0, 30);
//  memset(str4, 0, 30);
//  memset(&touchpos, 0, sizeof(struct touchPosition));
//
//  tFont.usFontCode = FC_ST_16X16;		/* 字体选择宋体16点阵，高16x宽15) */
//  tFont.usTextColor = CL_WHITE;		/* 字体颜色设置为白色 */
//  tFont.usBackColor = CL_MASK;	 	/* 文字背景颜色，透明 */
//  tFont.usSpace = 2;
//
//  tFont1.usFontCode = FC_ST_16X16;    /* 字体选择宋体16点阵，高16x宽15) */
//  tFont1.usTextColor = CL_BLACK;      /* 字体颜色设置为白色 */
//  tFont1.usBackColor = CL_MASK;       /* 文字背景颜色，透明 */
//  tFont1.usSpace = 2;
//
//  //LCD_DispOn();
//  while(1)
//  {   //don't use LCD_ClrScr to refreash
//      //LCD_SetBGLight(brt);
//      //rt_touch_read( RT_NULL, 0, &touchpos, 0);
//
//      LCD_DispStr(10,130,str,&tFont1);
//      sprintf(str, "XY RAW POS:%04d,%04d", touchpos.rawx, touchpos.rawy);
//      LCD_DispStr(10,130,str,&tFont);
//
//      LCD_DispStr(10,150,str1,&tFont1);
//      sprintf(str1, "XY PIX POS:%04d,%04d", touchpos.px, touchpos.py);
//      LCD_DispStr(10,150,str1,&tFont);
//
//      LCD_DispStr(10,170,str2,&tFont1);
//      sprintf(str2, "Z  RAW POS:%04d,%04d", touchpos.z1, touchpos.z2);
//      LCD_DispStr(10,170,str2,&tFont);
//
//      LCD_DispStr(10,190,str3,&tFont1);
//      sprintf(str3, "temp&area:%04d,%05d", touch_ReadTemperature(), TSC2046_ReadArea());
//      LCD_DispStr(10,190,str3,&tFont);
//
//      brt++;
//      cpu_usage_get(&cpuuseagemajor, &cpuuseageminor);
//      LCD_DispStr(10,210,str4,&tFont1);
//      sprintf(str4, "cpu usage:%02d.%02d%% %03d", cpuuseagemajor, cpuuseageminor, brt);
//      LCD_DispStr(10,210,str4,&tFont);
//      rt_thread_delay(5);
//      // add lcd brt adj func
//  }
//}


/************************
* thread cellular task  *
* test cellular func    *
************************/
void rt_thread_cellular_app_entry(void* parameter)
{
    
    rt_cellular_init();
    //rt_cellular_control();
    while(1)
    {
      
      rt_thread_delay(RT_TICK_PER_SECOND);
    }
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
//example thread for incoming call handleing
static void rt_thread_Celluar_IncomingCal_entry(void* parameter)
{
	rt_uint8_t ret = 0;
    while(1)
    {
        if( sim900_Call_ComeIn() )
        {
        	//pickup or reject
        	rt_celluar_control(PICKUP_CALL,RT_NULL);
            ret = sim900_PickUp();//sim900_HangUp();
            //notice: CommLineStatus = IN_CALL
            if(ret == 1)
                sim900_FinishCall();
                rt_celluar_control(FINISHCALL,RT_NULL);
        }
    }
}

//example thread for out going call handleing
static void rt_thread_Celluar_OutgoingCall_entry(void* parameter)
{
	rt_uint8_t ret = 0;
    while(1)
    {
    	rt_celluar_control(DO_CALL,"10086");
        ret = sim9000_call("10086");
        //notice: CommLineStatus = IN_CALL
        if(ret == 1)
            sim900_FinishCall();
            rt_celluar_control(FINISHCALL,RT_NULL);
    }
}
/************************
*   thread usb detect   *
**
************************/
ALIGN(RT_ALIGN_SIZE)
static char                 thread_usb_stack[1024];
static struct rt_thread     thread_usb;
static void rt_thread_usb_entry(void* parameter)
{
    uint8_t err, i, j, ok, temp;
    uint8_t sector[DEF_SECTOR_SIZE];
    ok = 0;
    temp = 0;
    while(1)
    {
        rt_kprintf( "Waiting for USB device\n" );
        while(1)
        {   /*查询CH376中断(INT#引脚为低电平)*/
            if( Query376Interrupt() )
            {   /*检查U盘是否连接*/
                if(CH376DiskConnect() == USB_INT_SUCCESS)
                    rt_kprintf("Device plugin\n");
                    break;
            }
            rt_thread_delay( 100 );
        }
        rt_thread_delay( 200 );

        /*try to mount USB设备,最多等待10*200mS*/
        for ( i = 0; i < 10; i++ )
        {   /*最长等待时间,10*200mS*/
            rt_thread_delay( 200 );
            //rt_kprintf( "Ready ?\n" );

            /*初始化磁盘并测试磁盘是否就绪*/
            err = CH376DiskMount();
            //rt_kprintf( "err: %X\n",err );
            if(err == USB_INT_SUCCESS)
            {   /*准备好*/
                break;
            }
            else if(err == ERR_DISK_DISCON)
            {   /* 检测到断开,重新检测并计时*/
                break;
            }
            if( (CH376GetDiskStatus() >= DEF_DISK_MOUNTED) && (i >= 5) )
            {   /*有的U盘总是返回未准备好,不过可以忽略,只要其建立连接MOUNTED且尝试5*50mS*/
                break;
            }
        }
        if(err == ERR_DISK_DISCON)
        {
            rt_kprintf("Device unpluged\n");
            continue;
        }
        if (CH376GetDiskStatus( ) < DEF_DISK_MOUNTED)
        {   /*未知USB设备,例如USB键盘、打印机等*/
            rt_kprintf("unknown device\n");
            rt_kprintf("please retry\n");
            goto WaitForRemove;
        }
        /* else if udisk mounted */
        rt_kprintf("Device mounted: Udisk\n");
        i = CH376ReadBlock( sector );
        if ( i == sizeof( INQUIRY_DATA ) )
        {   /* U盘的厂商和产品信息 */
            i=0;
            USBDeviceInfo.DeviceType = sector[i++];         /* 00H, 设备类型 */
            USBDeviceInfo.RemovableMedia = sector[i++];     /* 01H, 位7为1说明是移动存储 */
            USBDeviceInfo.Versions = sector[i++];           /* 02H, 协议版本 */
            USBDeviceInfo.DataFormatAndEtc = sector[i++];   /* 03H, 指定返回数据格式 */
            USBDeviceInfo.AdditionalLength = sector[i++];   /* 04H, 后续数据的长度 */
            USBDeviceInfo.Reserved1 = sector[i++];
            USBDeviceInfo.Reserved2 = sector[i++];
            USBDeviceInfo.MiscFlag = sector[i++];           /* 07H, 一些控制标志 */
            for (j = 0; j < 8;j++)
            {   /* 08H-0FH, 厂商信息 */
                USBDeviceInfo.VendorIdStr[j] = sector[i++];
            }
            for (j = 0; j < 16;j++)
            {    /* 10H-1FH, 产品信息 */
                USBDeviceInfo.ProductIdStr[j] = sector[i++];
            }
            for (j = 0; j < 4;j++)
            {   /* 20H-23H, 产品版本 */
                USBDeviceInfo.ProductRevStr[j] = sector[i++];
            }
            memset(sector, 0, DEF_SECTOR_SIZE);
            rt_kprintf( "Udisk Info: %s\n", USBDeviceInfo.VendorIdStr );
        }
        err = CH376DiskCapacity(&USBDiskTotalCapcity);
        if(err != USB_INT_SUCCESS)
        {
            rt_kprintf("Udisk read total capacity failed\n");
        }
        err = CH376DiskQuery(&USBDiskFreeCapcity);
        if(err != USB_INT_SUCCESS)
        {
            rt_kprintf("Udisk read free capacity failed\n");
        }
        err = CH376DiskReadSec(sector, 0, 1);
        /*status = CH376SecRead((uint8_t*)sector[0],1,NULL);*/
        if (err == USB_INT_SUCCESS)
        {
            /* gt the first partition */
            if (dfs_filesystem_get_partition(&part, sector, 0) != 0)
            {/* there is no partition */
                part.offset = 0;
                part.size   = 0;
            }
        }
        else
        {
            /* there is no partition table */
            part.offset = 0;
            part.size   = 0;
        }
        /*check if usb0 registed*/
        /*register usb0*/
        /*mount usb to elmfatfs*/
        rt_hw_ch376_init();

        if(opendir("/usb0") == RT_NULL)//check if dir exists
            temp = mkdir("/usb0",0);
        if(temp == 0)
        {
            if (dfs_mount("usb0", "/usb0/", "elm", 0, 0) == 0)
            {
                ok=1;
                rt_kprintf("USB host File System initialized!\n");
            }
            else
            {
                ok=0;
                rt_kprintf("USB host File System init failed!\n");
            }
        }
        else
        {
            rt_kprintf("mkdir /usb0 failed\n");
            temp = 0;
        }

WaitForRemove:
    /*rt_kprintf( "wait for remove\n" );*/
    while ( CH376DiskConnect( ) == USB_INT_SUCCESS )
        {  /* 检查U盘是否连接,等待U盘拔出 */
        rt_thread_delay( 500 );
        }
        /*deregister usb0*/
        /*unmount usb to elmfatfs*/
        if(ok)
        {
            if (dfs_unmount("/usb0/") == 0 && SafeRemoveDisk() == USB_INT_SUCCESS)
            rt_kprintf( "usb device safe removed\n" );
            else
                rt_kprintf( "usb device unsafe removed\n" );
        }
        ok=0;
        rt_thread_delay(200 );
    }
}

/************************
*   app threads init    *
*   app entry of RTOS   *
************************/
int rt_application_init()
{
    rt_thread_init(&thread_usb,
                   "usb",
                   rt_thread_usb_entry,
                   RT_NULL,
                   &thread_usb_stack[0],
                   sizeof(thread_usb_stack), 20, 10);
    rt_thread_startup(&thread_usb);

    rt_thread_startup(rt_thread_create( "init",
                                        rt_init_thread_entry,
                                        RT_NULL,
                                        1024, 4, 10));

    return 0;
}




