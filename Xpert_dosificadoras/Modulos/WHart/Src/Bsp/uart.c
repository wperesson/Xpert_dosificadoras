//#include <msp430x14x.h>
#include "IHart_glb_defs.h"
#include "uart.h"
#include "IHart_DataAcquisition.h"

//�û��˽��ջ�����
USER_UARTREVDATA  userUartRevDataBuf[2]={0};
//�û��˷��ͻ�����
USER_UARTSENDDATA userUartSendDataBuf[2]={0};
 
/******************************************************************************
 * Name: DataParityErr
 * Description: Device deals with messages that have parity errors
 * Parameters: USIGN8 ucUARTNum
 * Return: True/FALSE
 *****************************************************************************/
USIGN8 DataParityErr(USIGN8 ucUARTNum)
{
//    if((ucUARTNum==0) && (URCTL0 & RXERR))
//    {
//        return TRUE;
//    }
//    else if((ucUARTNum==1) && (URCTL1 & RXERR))
//    {
//        if((URCTL1 & FE)&&(gsParityInfo.ErrState == 0))//Framing Error
//        {
//            gsParityInfo.ErrState = HART_FRAME_ERR;
//            gsParityInfo.ErrCounter  = userUartRevDataBuf[ucUARTNum].UartRevCounter + 1;
//            return TRUE;
//        }
//        if((URCTL1 & PE)&&(gsParityInfo.ErrState == 0))//Vertical Parity
//        {
//            gsParityInfo.ErrState = HART_PARITY_VTC_ERR;
//            gsParityInfo.ErrCounter  = userUartRevDataBuf[ucUARTNum].UartRevCounter + 1;
//            return TRUE;
//        }
//    }
    return FALSE;
}

/*************************************************
 Ŀ�ģ�  ��λ���ڽ�������
 ������  USIGN8 nComNum,���ں�
 ����ֵ����
****************************************************/
void ResetUartRx(USIGN8 nComNum)
{
    memset(&userUartRevDataBuf[nComNum],0,sizeof(USER_UARTREVDATA)); 
}

/*************************************************
 Ŀ�ģ�  ��λ���ڷ�������
 ������  USIGN8 nComNum,���ں�
 ����ֵ����
****************************************************/
void ResetUartTx(USIGN8 nComNum)
{
    memset(&userUartSendDataBuf[nComNum],0,sizeof(USER_UARTSENDDATA));
}
/*************************************************
 Ŀ�ģ� ���ڽ��յ�һ���ֽڵĴ���
 ������ USIGN8 Data��������һ���ֽڵ�����
        USIGN8 nComNum,���ں�
 ����ֵ����
****************************************************/
#ifdef FUN_MONITOR_RTC
USIGN8  RTCPreamNum=0;
#endif
void UartRevData(USIGN8 Data, USIGN8 nComNum)
{
    //��ֹ�������
    if(userUartRevDataBuf[nComNum].UartRevCounter >= USER_MAX_UART_RX_BUFFER)
	{
	    userUartRevDataBuf[nComNum].UartRevCounter = 0;	
	}
    
	userUartRevDataBuf[nComNum].UartRevBuffer[userUartRevDataBuf[nComNum].UartRevCounter] = Data;
	userUartRevDataBuf[nComNum].UartRevCounter++;

    if(nComNum)
        userUartRevDataBuf[1].RevTimeOut = HART_MOD_TIMEOUT;
    else
        userUartRevDataBuf[0].RevTimeOut = M1100_TIMEOUT; 
    
#ifdef FUN_MONITOR_RTC  
    if(nComNum==0)
    {
        if(Data==0xFF)
        {
            RTCPreamNum++;
        }
        else
        {
            if(RTCPreamNum>2)
            {
                MyNewRtc = TBR;
                RTCPreamNum=0;
            }
        }  
    }    
#endif
}



