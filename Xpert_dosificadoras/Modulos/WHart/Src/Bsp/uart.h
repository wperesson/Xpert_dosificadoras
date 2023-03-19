#ifndef _UART_H
#define _UART_H

#include "IHart_glb_defs.h"


//�û����ջ������ṹ��
typedef struct
{
    USIGN8 UartRevBuffer[USER_MAX_UART_RX_BUFFER];
    USIGN8 UartRevCounter;
    USIGN8 RevTimeOut;  
}USER_UARTREVDATA;

//�û����ͻ������ṹ��
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

extern USER_UARTREVDATA  userUartRevDataBuf[2];  /*�±�0 ���ں�ģ��ͨ�ţ��±�1 ���ں��û��ӿ�ͨ��*/
extern USER_UARTSENDDATA userUartSendDataBuf[2];


#endif