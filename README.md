sim900
======

This folder holds the C source code for sim900 GSM/GPRS module works on RT-THREAD OS.
The source code is suitable for EON BSP with RT-THREAD OS.

Hardware platform: ARMFLY 3.0;
CPU:  STM32F103ZE;
OS kernel version: RT-THREAD 1.0.2;


Stage(1):(in progress, 2013/05/30--)
    basic functions for sim900 operation;
    COM read & write OPs;
    OS thread support;
    AT commands OPs;
    sim900 status check OPs;
    
Stage(2):(TBD)
    GSM: Call handler;
    GPRS: TCP/IP & UDP data handler;

Stage(3):(TBD)
    GSM: SMS handler;
    GPRS: MMS handler;
    
Stage(4):(TBD)
    GPRS: HTTP/FTP handler;
    others;
