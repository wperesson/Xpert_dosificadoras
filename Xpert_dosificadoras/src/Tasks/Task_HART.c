/*
 * Task_CommMB_HMI.c
 *
 *  Created on: 30/01/2014
 *      Author: Lisa
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"

#include "IHart_glb_defs.h"
#include "IHart_DataAcquisition.h"
#include "IHart_Uart.h"
#include "IHart_DataSave.h"
#include "IHart_UserCommand.h"

/* Librerias de C includes. */
#include "Definiciones.h"

#define T15			750E-6	//750 uS
#define T35			1.75E-3//1.75 mS
#if !MOTOR_ELEC

#define UART				LPC_UART0
#define UART_IRQ_SELEC		UART0_IRQn
#define UART_HANDLER_NAME 	UART0_IRQHandler
#define BAUDRATE			9600


#define BUF_SIZE			100

#define TIME_RESET_CNT		1000

bool Flag_Seg;
uint8_t ReadB, BufWH[BUF_SIZE], Index;

USIGN8 gsUART1_Hart_Delay_Flag;
//USIGN16 gnSTO_Time_Rx, gnSTO_Time_Tx;

uint16_t ret_count, init_count;

xTimerHandle TimerA, xTimerNetId, xTimerJoinKey, xTimerUpdRate;

bool User_command(char *buff, user_command_type command, char *Msg, char Len) {
	uint8_t i;
	char *ptr, *ptr_2;
	EvtToHART_Type Evento;

	if (uart0_tx_busy == true) //Si esta ocupado retornar sin hacer nada
		return false;

	ptr = buff;
	ptr_2 = buff;
	for (i = 0; i < gsHartSlaveInfo.ucNumOfRequestPreambles; i++) {
		*ptr++ = 0xff;
	}
	*ptr++ = 0x82;

	*ptr++ = IUserLongAddr[0]; // 0xe2;
	*ptr++ = IUserLongAddr[1]; //0xf6;
	*ptr++ = IUserLongAddr[2]; // 0x00;
	*ptr++ = IUserLongAddr[3]; //0x09;
	*ptr++ = IUserLongAddr[4]; //0x88;

	if (command > 255) {
		*ptr++ = 0x1f;

		*ptr++ = Len + 2;
		*ptr++ = command >> 8;
		*ptr++ = 0xff & command;

		i = 0;
		while (i < Len) {
			*ptr++ = Msg[i++];
		}
	} else {

		*ptr++ = 0xff & command;
		*ptr++ = Len;			//Byte count: 9 bytes a enviar
		i = 0;
		while (i < Len) {
			*ptr++ = Msg[i++];
		}
	}

	buff += gsHartSlaveInfo.ucNumOfRequestPreambles;
	*ptr = 0;
	while (buff < ptr) {
		*ptr ^= *buff++;
	}
	i = ptr - ptr_2 + 1;
	userUartRevDataBuf[1].UartRevCounter = i;
	memcpy(userUartRevDataBuf[1].UartRevBuffer, ptr_2, i);

	if (debug_whart) {
		Buf485[50] = userUartRevDataBuf[1].UartRevCounter;
		memcpy(&Buf485[51], userUartRevDataBuf[1].UartRevBuffer,
				userUartRevDataBuf[1].UartRevCounter);
//		data_RX_avail = true;
		xTaskNotifyGive(tsk_modb_rs485_rx_handler);
		vTaskDelay(10);
	}

	Evento.Src = comm1;
	xQueueSend(queEvtToHART, &Evento, 0);
	return true;
}

void Tmr_Retry_Callback(xTimerHandle pxTimer) {
	EvtToHART_Type Evento;
	int32_t Idx;

	Idx = (int32_t) pvTimerGetTimerID(pxTimer);

	if (Idx == 0) {			//Net Id retry timer
		hart_WR_NET_ID_ret_cnt++;
		Evento.Src = write_NET_ID;
		Evento.Val = SP.WHART_NetId;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (Idx == 1) {			//Update rate retry timer
		hart_WR_UPD_RATE_ret_cnt++;
		Evento.Src = write_UPD_RATE;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (Idx == 2) {			//Update rate retry timer
		hart_WR_JOIN_KEY_ret_cnt++;
		Evento.Src = write_JOIN_KEY;
		xQueueSend(queEvtToHART, &Evento, 0);
	}
}

void vTmrA_Callback(xTimerHandle pxTimer) {
	static USIGN8 nClearWdog = 0;
	static USIGN8 nUpdateRate = 0;
	static USIGN16 nAD_Execute = 0;
	nClearWdog++;
	nUpdateRate++;
	nAD_Execute++;
	EvtToHART_Type Evento;
//	static LED;
//
//	LED = !LED;
//	if (LED)
//		SGN_O1_ON;
//	else
//		SGN_O1_OFF;

	configASSERT(pxTimer);

	IHartUserTime();
	if (nClearWdog >= 10) {
		nClearWdog = 0;
	}

	/*��λ�豸*/
	if (gucResetFlag) {
		nResetDelayTime++;
		if ((nResetDelayTime >= 10000)
				|| ((gsHartSlaveInfo.ucNoticeResetFlag == 2)
						&& (nResetDelayTime >= 2000))) {
			gucResetFlag = 0;
			nResetDelayTime = 0;
			gsHartSlaveInfo.ucNoticeResetFlag = 0;
			IHartWriteEeprom(
					(USIGN8*) &gsStatusData.ucRemoteConfigChangedCounter,
					EE_RMOTE_CON_CHA_COUNTER, 2);
		}
	}

	/*���±���*/
	if (nUpdateRate >= IUserVariableUpdateRate) {
		nUpdateRate = 0;
		gsHartSlaveInfo.gsUpdateDynamicFlag = 1;
	}
	if (nAD_Execute >= 1000) {
		nAD_Execute = 0;
		gsHartSlaveInfo.gsAD_Execute_Flag = 1;

		//���MoreStatus 1s
	}
	if (Act_Page == 20 && ret_count) {
		ret_count--;
		if (ret_count == 0) {
			Evento.Src = read_JOIN_STATUS;
			xQueueSend(queEvtToHART, &Evento, 0);

		}
	}

	if (Act_Page == 20 && ret_count == 0) {
		ret_count = 500;
	}

	//===================================================
	//Contador para configuracion inicial
	if (init_count == TIME_RESET_CNT) {
		//Send write_join_key
		Evento.Src = reset_whart_module;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (init_count == 10000) {
		//Send write_join_key
		Evento.Src = write_JOIN_KEY;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (init_count == 14000) {
		//Send Net_id
		Evento.Src = write_NET_ID;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (init_count == 17000) {
		//Send Join_now
		Evento.Src = write_FORCE_JOIN_MODE;
		xQueueSend(queEvtToHART, &Evento, 0);
	} else if (init_count == 19000) {
//		Send write Update Rate
		Evento.Src = write_UPD_RATE;
		xQueueSend(queEvtToHART, &Evento, 0);

	}

	if (init_count != TIME_RESET_CNT && init_count < 20000)
		init_count++;

	/*����0-����ǰ������RTS������ʱ*/
	if (nDelayRTSTime) {
		nDelayRTSTime--;
	}
	/*����0�����ж�*/
	if (userUartRevDataBuf[0].RevTimeOut) {
		userUartRevDataBuf[0].RevTimeOut--;
		if (userUartRevDataBuf[0].RevTimeOut == 0) {
			Evento.Src = comm0;
			xQueueSendToFront(queEvtToHART, &Evento, 0);
		}
	}

	/*����1�����ж�*/
	if (userUartRevDataBuf[1].RevTimeOut) {
		userUartRevDataBuf[1].RevTimeOut--;
		if (userUartRevDataBuf[1].RevTimeOut == 0) {
			Evento.Src = comm1;
			xQueueSend(queEvtToHART, &Evento, 0);
		}
	}

}

/*-----------------------------------------------------------
 * 	 */
static portTASK_FUNCTION(vCommWHART, pvParameters) {

	char buff[40];
	char Msg[32], *ptr;
	uint8_t i;
	EvtToHART_Type Evento;
	uint32_t temp_int;
	uint16_t temp_short;

	memset(buff, 0xff, 40);
	memset(Msg, 0xff, 16);

	// UART_0: COMUNICACION CON MÓDULO WHART //////////////////
	Chip_UART_Init(UART);
	Chip_UART_SetBaud(UART, BAUDRATE);
	Chip_UART_ConfigData(UART,
			UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN
					| UART_LCR_PARITY_EVEN);
	Chip_UART_SetupFIFOS(UART,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV1);

	Chip_UART_TXEnable(UART);

	/* Enable UART interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	//TimerA
	TimerA = xTimerCreate("TmrA_Hart", 1, pdTRUE, (void*) 0, vTmrA_Callback);
	xTimerNetId = xTimerCreate("Tmr Net Id Retry", 3000, pdFALSE, (void*) 0,
			Tmr_Retry_Callback);
	xTimerUpdRate = xTimerCreate("Tmr Upd Rate Retry", 3000, pdFALSE, (void*) 1,
			Tmr_Retry_Callback);
	xTimerJoinKey = xTimerCreate("Tmr Join Key Retry", 3000, pdFALSE, (void*) 2,
			Tmr_Retry_Callback);

	WHART_RST_ON;

	mem_Net_Id = SP.WHART_NetId;
	mem_Ref_Rate = SP.WHART_RefRate;

	init_count = 0;

	Evento.Src = reset_whart_module;
	xQueueSend(queEvtToHART, &Evento, 0);

	while (1) {
		if (xQueueReceive(queEvtToHART, &Evento, 1000) == pdPASS) {
			switch (Evento.Src) {
			case reset_whart_module:
				//"L1S251710300"

				//Seteo la información de ID
				sscanf(&id_SN[7], "%d", &temp_short);

				IUserLongAddr[3] = temp_short >> 8;
				IUserLongAddr[4] = temp_short;

				WHART_RST_ON;
				vTaskDelay(2000);
				WHART_RST_OFF;
				vTaskDelay(500);

				Chip_UART_IntEnable(UART, UART_IER_RBRINT);
				Chip_UART_IntEnable(UART, UART_IER_THREINT);

				xTimerStart(TimerA, 0);
				vTaskDelay(500);

				/* Initialize the Device information */
				IHartDeviceInit();

				/* Initialize the Stack */
				IHartStackInit();

				/* Execute self testing */
				IHartSelfTest();

				WHM_RTS_H;

				init_count = TIME_RESET_CNT + 1;

				break;
			case config_changed:
				init_count = 0;
				break;
			case send_cmd:			//Para debugging del sistema WHART
				i = 0;
				if (Evento.Val > 1) {			//Viene c argumentos
					i = Evento.Val - 1;
					memcpy(Msg, &Buf485[202], i);
				}
				temp_short = (Buf485[201] << 8) + Buf485[200];
				User_command(buff, temp_short, Msg, i);
				break;
			case comm0:
				Deal_USRT0_Receive_Fun();
				break;
			case comm1:
				Deal_USRT1_Receive_Fun();
				break;
			case write_NET_ID:		//Cambio el NET ID? Escribir el nuevo valor
				Msg[0] = SP.WHART_NetId >> 8;
				Msg[1] = 0xff & SP.WHART_NetId;
				User_command(buff, write_net_id, Msg, 2);
				break;
			case read_NET_ID:
				User_command(buff, read_net_id, Msg, 0);
				break;
			case write_FORCE_JOIN_MODE:
				ptr = Msg;
				*ptr++ = 0x02;//Attempt to join inmediately on Powerup or reset
				*ptr++ = 0x00;
				*ptr++ = 0xff;
				*ptr++ = 0xff;
				*ptr++ = 0xff;
				*ptr++ = 0x0a;	//10 intentos de joinning
				User_command(buff, force_join_mode, Msg, 6);
				break;

			case read_BURST_MODE:
				Msg[0] = 0;	//Burst message
				User_command(buff, read_busrt_mode_config, Msg, 1);
				break;
			case write_UPD_RATE:
				temp_int = SP.WHART_RefRate * 32 * 1000;
				Msg[0] = 0;
				Msg[1] = 0xff & (temp_int >> 24);
				Msg[2] = 0xff & (temp_int >> 16);
				Msg[3] = 0xff & (temp_int >> 8);
				Msg[4] = 0xff & temp_int;

				Msg[5] = 0xff & (temp_int >> 24);
				Msg[6] = 0xff & (temp_int >> 16);
				Msg[7] = 0xff & (temp_int >> 8);
				Msg[8] = 0xff & temp_int;
				User_command(buff, write_burts_period, Msg, 9);
				break;
			case write_JOIN_KEY:
				memcpy(buff, SP.WHART_JoinKey, 16);
				for (i = 0; i < 16; i++) {
					Msg[i] = buff[15 - i];
				}
				User_command(buff, write_join_key, Msg, 16);
				break;
			case write_DEV_VAR_UNIT:
				memcpy(buff, SP.WHART_JoinKey, 16);
				for (i = 0; i < 16; i++) {
					Msg[i] = buff[15 - i];
				}
				User_command(buff, write_join_key, Msg, 16);
				break;
			case write_LONG_TAG:
				memset(Msg, 0x00, 32);
//				sprintf(Msg, "%s_%s", id_Field, id_well);
				sprintf(Msg, "SMARTC25_V1");
				User_command(buff, write_long_tag, Msg, 32);
				break;
			case read_JOIN_STATUS:
				User_command(buff, read_join_status, Msg, 0);
				ret_count = 3000;
				break;
			case request_ACT_ADV:
				temp_int = 100;
				ptr = (char*) &temp_int;
				for (i = 0; i < 4; i++) {
					Msg[3 - i] = *ptr++;
				}
				User_command(buff, request_active_Advertising, Msg, 4);
				break;
			default:
				break;
			}

		} else {
			/*Synchronization device status information, reporting battery level and reading signal strength of WH-M*/
			gsHartSlaveInfo.gsHartPowerInfo.fBatV = Var.VBatt;
			IHartUserActiveRequest();

			/* Update Dynamic Variable*/
			if (gsHartSlaveInfo.gsUpdateDynamicFlag) {
				gsHartSlaveInfo.gsUpdateDynamicFlag = 0;
				IHartUpdateDynamic();
			}
			/* Data collecting */
			if (gsHartSlaveInfo.gsAD_Execute_Flag) {
				gsHartSlaveInfo.gsAD_Execute_Flag = 0;

				gsDeviceVars[0].unValue.f = Var.LinePress;/*Mapped to PV*/

				gsDeviceVars[1].unValue.f = Var.tank_vol_perc;/*Mapped to SV*/

				gsDeviceVars[2].unValue.f = PumpOn ? sp_pump.PumpRate : 0; /*Mapped to TV*/

				gsDeviceVars[3].unValue.f = Var.VBatt; /*Mapped to QV*/

				/*Variables alarm*/
				IsDevVarsOutofLimit();
			}
		}
	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por el UART0 - Comunicacion WIRELESS HART*/
void UART_HANDLER_NAME(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t Interr;

	Interr = Chip_UART_ReadIntIDReg(UART);
	if (/*(Interr & UART_IIR_INTID_RDA) || */(Interr & UART_IIR_INTID_CTI)) {
		while ((Chip_UART_ReadLineStatus(UART) & UART_LSR_RDR) != 0) {
			UartRevData(Chip_UART_ReadByte(UART), 0);
		}
	} else if (Interr & UART_IIR_INTID_RDA) {
		while ((Chip_UART_ReadLineStatus(UART) & UART_LSR_RDR) != 0) {
			UartRevData(Chip_UART_ReadByte(UART), 0);
		}
	} else if (Interr & UART_IIR_INTID_THRE) {
		if (userUartSendDataBuf[0].UartSendCounter
				< userUartSendDataBuf[0].DataLength) {
			Chip_UART_SendByte(LPC_UART0,
					userUartSendDataBuf[0].UartSendBuffer[userUartSendDataBuf[0].UartSendCounter++]);
		} else/*�����������RTS�ź���*/
		{
			WHM_RTS_H;
			IHartSendEnd(0);
			ResetUartTx(0);
			uart0_tx_busy = false;
		}
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_CommWHART_Task(void) {
	xTaskCreate(vCommWHART, (char*) "WHART TSK", 128, NULL,
	Prio_CommWHART_Task, &tsk_whart_handler); /* "Comunicación con el HMI*/
}

#endif
