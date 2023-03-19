//#include <msp430x14x.h>
#include "IHart_glb_defs.h"
#include "IHart_Uart.h"
#include "uart.h"
#include "IHart_cfg.h"
//#include "msp430.h"
#include "Definiciones.h"
#include "IHart_DataAcquisition.h"

volatile USIGN8 nDelayRTSTime = 0;

extern xTimerHandle xTimerNetId, xTimerJoinKey, xTimerUpdRate;

USIGN8 UART1_Receive_finish = 0;
USIGN8 UART0_Receive_finish = 0;

/*************************************************
 Ŀ�ģ�  ������1����
 ����ֵ����
 ****************************************************/
void Deal_USRT1_Receive_Fun(void) {
	IHART_UART_FUNTION_ENUM nReturnTemp;

	nReturnTemp = IHartDealUartRecData(1, userUartRevDataBuf[1].UartRevBuffer,
			userUartRevDataBuf[1].UartRevCounter);

	if (nReturnTemp == IHART_PROCESS_CONFLIT) {
		UART1_Receive_finish = 1;
	} else {

	}
	ResetUartRx(1);
}

/*************************************************
 Ŀ�ģ�  ������0����
 ����ֵ����
 ****************************************************/
//void Deal_USRT0_Receive_Fun(void) {
//	IHART_UART_FUNTION_ENUM nReturnTemp;
////char i = 0;
////���������ɵ�����
//	nReturnTemp = IHartDealUartRecData(0, userUartRevDataBuf[0].UartRevBuffer,
//			userUartRevDataBuf[0].UartRevCounter);
//
//	ResetUartRx(0);
//}

IHART_UART_FUNTION_ENUM Deal_USRT0_Receive_Fun(void) {
	IHART_UART_FUNTION_ENUM nReturnTemp;

	nReturnTemp = IHartDealUartRecData(0, userUartRevDataBuf[0].UartRevBuffer,
			userUartRevDataBuf[0].UartRevCounter);

	ResetUartRx(0);

	return nReturnTemp;
}

/*******************************************************************
 �ӿڹ���  : �û����ͻ������е����ݡ����û�ʵ��
 �������: USIGN8 nComNum���跢�����ݵ�UART�ţ�0��������ģ�������Ķ˿�; 1��ά���˿ڡ�
 USIGN8* pData���������ݵ���ʼ��ַ��
 USIGN8 nDataLength���������ݵĳ��ȡ�
 �������: ��
 ����ֵ: I_UART_FUNTION_ENUM����I_UART_FUNTION_ENUMö�ٽṹ�塣
 ********************************************************************/

IHART_UART_FUNTION_ENUM IHartDealUartSendData(USIGN8 nComNum, USIGN8* pData,
		USIGN8 nDataLength) {
	IHART_UART_FUNTION_ENUM nReturnTemp = IHART_PROCESS_NO_ERR;
	uint8_t i = 0, j;
	char *ptr;
	uint32_t temp_int;
	uint16_t temp_short;
	if (nComNum)/* ����1�������� ��ά���˿�*/
	{

		userUartSendDataBuf[1].DataLength = nDataLength;
		memcpy(userUartSendDataBuf[1].UartSendBuffer, pData,
				userUartSendDataBuf[1].DataLength);

		if (debug_whart) {
			Buf485[50] = nDataLength;
			memcpy(&Buf485[51], pData, nDataLength);
//			data_TX_avail = true;
			xTaskNotifyGive(tsk_modb_rs485_rx_handler);
		}

		while (userUartSendDataBuf[1].UartSendBuffer[i] == 0xff) {
			i++;
		}
		if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x86) { //0x86: Respuesta del modulo
			i += 6;

			if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x1f) { //Comando mayor a 255
				i += 4;
				if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x03) {
					i++;

					//RES: 768, 0X300 Write Join Key  ===================================================================
					if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x00) {

						ptr = (char *) &SP.WHART_JoinKey;
						i += 16;
						for (j = 0; j < 16; j++) {
							*ptr++ =
									userUartSendDataBuf[1].UartSendBuffer[i - j];
						}
						xTimerStop(xTimerJoinKey, 0);
						hart_WR_JOIN_KEY_ret_cnt = 0;
					}
					//RES: 769, 0X301 read join status  ===================================================================
					else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x01) {
						i++;
						Var.whart_stat = ((0x0f
								& userUartSendDataBuf[1].UartSendBuffer[i])
								<< 16)
								+ (userUartSendDataBuf[1].UartSendBuffer[i+1]
										<< 8)
								+ userUartSendDataBuf[1].UartSendBuffer[i+2];


					}
					//RES: 773, 0X305 Write Net Id  ===================================================================
					else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x05) {
						i++;
						temp_short = (userUartSendDataBuf[1].UartSendBuffer[i]
								<< 8)
								+ userUartSendDataBuf[1].UartSendBuffer[i+1];

						if (temp_short != SP.WHART_NetId) {
							SP.WHART_NetId = temp_short;
							if (xSemaphoreTake(mtxI2C1, 30) == pdTRUE) {
								eeprom_write((char*) &SP, OFF_SP, sizeof(SP));
								xSemaphoreGive(mtxI2C1);
							}
						}
						xTimerStop(xTimerNetId, 0);
						hart_WR_NET_ID_ret_cnt = 0;
					}
					//RES: 774, 0X306 Read Net Id  ===================================================================
					else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x06) {
						i++;
						temp_short = (userUartSendDataBuf[1].UartSendBuffer[i]
								<< 8)
								+ userUartSendDataBuf[1].UartSendBuffer[i+1];
						if (temp_short != SP.WHART_NetId) {
							SP.WHART_NetId = temp_short;
							if (xSemaphoreTake(mtxI2C1, 30) == pdTRUE) {
								eeprom_write((char*) &SP, OFF_SP, sizeof(SP));
								xSemaphoreGive(mtxI2C1);
							}
						}
					}
					//RES: 771, 0X303Write Join Mode configuration  ===================================================================
					else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x03) {

					}
					//RES: 772X 0X304 Read Join Mode configuration  ===================================================================
					else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x04) {

					}
				}
			} else { //Comando menor a 255

				//RES: 103, 0X67 Write Burst Period  ===================================================================
				if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x67) {
					i += 8;
					ptr = (char *) &temp_int;
					for (j = 0; j < 4; j++) {
						*ptr++ = userUartSendDataBuf[1].UartSendBuffer[i--];
					}

					temp_short = temp_int / (32 * 1000);

					if (temp_short != SP.WHART_RefRate) {
						SP.WHART_RefRate = temp_short;
						if (xSemaphoreTake(mtxI2C1, 30) == pdTRUE) {
							eeprom_write((char*) &SP, OFF_SP, sizeof(SP));
							xSemaphoreGive(mtxI2C1);
						}
					}
					xTimerStop(xTimerUpdRate, 0);
					hart_WR_UPD_RATE_ret_cnt = 0;
				}

				//RES: 105, 0X69 Read Burst Mode Configuration  ===================================================================
				else if (userUartSendDataBuf[1].UartSendBuffer[i] == 0x69) {
					i += 18;
					ptr = (char *) &temp_int;

					*ptr++ = userUartSendDataBuf[1].UartSendBuffer[32];
					*ptr++ = userUartSendDataBuf[1].UartSendBuffer[31];
					*ptr++ = userUartSendDataBuf[1].UartSendBuffer[30];
					*ptr = userUartSendDataBuf[1].UartSendBuffer[29];

					SP.WHART_RefRate = temp_int / (32 * 1000);

				}
			}
			ResetUartTx(1);
		}
	} else/*����0���͸�ģ��*/
	{
		/*ֻ��ת����ʱ�򣬲���Ҫ�ж�WHM_CD*/
		if (WHM_CD) {
			/*��������*/
			WHM_RTS_L;
			uart0_tx_busy = true;

			while (!(LPC_UART0->LSR & UART_LSR_THRE)) {

			}
			WHM_RTS_L;
			nDelayRTSTime = IUserRTSLowTimer; /*������ʱʱ��*/
			while (nDelayRTSTime != 0)
				;

			userUartSendDataBuf[0].DataLength = nDataLength;
			memcpy(userUartSendDataBuf[0].UartSendBuffer, pData,
					userUartSendDataBuf[0].DataLength);

			if (WHM_CD) {
				if (UART0_Receive_finish == 0) {
					userUartSendDataBuf[0].UartSendCounter = 1;
					Chip_UART_SendByte(LPC_UART0,
							userUartSendDataBuf[0].UartSendBuffer[0]);
					return IHART_PROCESS_NO_ERR;
				}
			}
		}
		nReturnTemp = IHART_PROCESS_CONFLIT;
	}
	return nReturnTemp;
}

