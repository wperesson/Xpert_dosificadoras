#ifndef _UART_H
#define _UART_H

#include "IHart_glb_defs.h"


//用户接收缓冲区结构体
typedef struct
{
    USIGN8 UartRevBuffer[USER_MAX_UART_RX_BUFFER];
    USIGN8 UartRevCounter;
    USIGN8 RevTimeOut;  
}USER_UARTREVDATA;

//用户发送缓冲区结构体
typedef struct
{
    USIGN8 UartSendBuffer[USER_MAX_UART_TX_BUFFER];
    USIGN8 DataLength;
    USIGN8 UartSendCounter;  
}USER_UARTSENDDATA;

USIGN8 DataParityErr(USIGN8 ucUARTNum);
void ResetUartRx(USIGN8 nComNum);
void ResetUartTx(USIGN8 nComNum);
void UartSendData(USIGN8 *pData, USIGN8 Length, USIGN8 nComNum);
void UartRevData(USIGN8 Data, USIGN8 nComNum);

extern USER_UARTREVDATA  userUartRevDataBuf[2];  /*下标0 用于和模块通信，下标1 用于和用户接口通信*/
extern USER_UARTSENDDATA userUartSendDataBuf[2];


#endif