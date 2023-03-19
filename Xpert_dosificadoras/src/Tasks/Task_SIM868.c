/*
 * TaskINPUT.c
 *
 *  Created on: 09/03/2014
 *      Author: admin
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LTC2946.h"

#include "Definiciones.h"
#include "GSM.h"

#define UART				LPC_UART3
#define UART_IRQ_SELEC		UART3_IRQn
#define UART_HANDLER_NAME 	UART3_IRQHandler
#define BAUDRATE			115200
#define REF_RATE			20000

StreamBufferHandle_t stream_SIM868;

static portTASK_FUNCTION(vCOMM_SIM868_Task,pvParameters) {
	uint8_t i;
	float tempf;
	EvtToGSM_Type EvtToGSM;
	char buff_SIM868[GSM_BUFF_SIZE];
	char* ptr;
	static uint16_t ReadB, ReadC;
	Parameter_upd_Type newParameter;

	SIM868_PWRKEY_OFF; //Apagar el módulo
	vTaskDelay(2000);
//	GSM_RST_OFF;
	SIM868_PWRKEY_ON; //Encender el módulo
	vTaskDelay(1000);
//	GSM_KEY_OFF;
	SIM868_GPS_EN_ON;
	vTaskDelay(1000);

	stream_SIM868 = xStreamBufferCreate(GSM_BUFF_SIZE, 1);
	//Debo crear el stream buffer antes de configurar el puerto para evitar que un dato entrante
	//intente enviar al buffer str que no ha sido creado

	// UART_3: COMUNICACION CON GSM MODULE //////////////////
	Chip_UART_Init(UART);
	Chip_UART_SetBaud(UART, BAUDRATE);
	Chip_UART_ConfigData(UART,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(UART,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV1);

	/* Enable UART3 interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC,
	mainSOFTWARE_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	Chip_UART_TXEnable(UART);
	Chip_UART_ReadIntIDReg(UART);
	Chip_UART_ReadLineStatus(UART);
	Chip_UART_IntEnable(UART, UART_IER_RBRINT);

	Create_COMM_SMS_task();
//	Create_COMM_UBI_task();

	while (1) {

		memset(buff_SIM868, 0, GSM_BUFF_SIZE);

		ReadB = xStreamBufferReceive(stream_SIM868, buff_SIM868, GSM_BUFF_SIZE,
		portMAX_DELAY);

		do {
			ReadC = xStreamBufferReceive(stream_SIM868, &buff_SIM868[ReadB],
			GSM_BUFF_SIZE, 20);
			ReadB += ReadC;
		} while (ReadC);

		if (sniff_gsm || debug_gsm)
			if (ReadB != 0)
				xStreamBufferSend(stream_Sniffer, buff_SIM868, ReadB, 10);

		if (debug_gsm)
			xTaskNotifyGive(vCOMM_MODB_RS485Handle);
		if (ptr = strstr(&buff_SIM868[5], "v1.6/devices")) {
			//Dato proviene desde UBIDOTS.

			if (ptr = strstr(&buff_SIM868[5], "setrate/lv")) {
				ptr += strlen(ptr) + 2;
				if (sscanf(ptr, "%f", &tempf) == 1) {
					//								SP.PumpRate = tempf;
					newParameter.Val = tempf;
					newParameter.parameter = SP_PumpRate;
					newParameter.Src = src_web;
					xQueueSend(xQueHEAD_FLOW_SETT_upd, &newParameter, 0);
					//Publicar inmediatamente un nuevo dato
					xTaskNotifyGive(vCOMM_UBI_Handle);
				}

			} else if (ptr = strstr(&buff_SIM868[5], "pwrcont/lv")) {
				ptr += strlen(ptr) + 2;
				if (sscanf(ptr, "%f", &tempf) == 1) {
					(tempf == 1) ? (newParameter.Val = 1) : (newParameter.Val =
											0);
					newParameter.parameter = PUMP_ON;
					newParameter.Src = src_web;
					xQueueSend(xQueHEAD_FLOW_SETT_upd, &newParameter, 0);
					//Publicar inmediatamente un nuevo dato
					xTaskNotifyGive(vCOMM_UBI_Handle);
				}
			}

		} else if (ptr = strstr(buff_SIM868, "+CMTI:")) {
			//Dato proviene desde SMS.
			if (!debug_gsm) {
				EvtToGSM.Src = read_process_sms;
				xQueueSend(xQueEvtToGSM, &EvtToGSM, 0);
			}

		} else {
			for (i = 0; i < ReadB; i++) {
				if (xQueueSend(xQueMsgFromSIM868, &buff_SIM868[i],
						1) == errQUEUE_FULL) {
					break;
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por el UART - Comunicacion c módulo GPRS/GSM+GPS*/
void UART_HANDLER_NAME(void) {
#define LOC_BUFF_SZ	30

	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t Interr, lsr;
	static uint8_t ReadB, i;
	static uint8_t Buffer[LOC_BUFF_SZ];
	EvtToGSM_Type EvtToGSM;

	Interr = Chip_UART_ReadIntIDReg(UART) & 0xff;
	if ((Interr & UART_IIR_INTID_RDA) || (Interr & UART_IIR_INTID_CTI)) {

		ReadB = Chip_UART_Read(UART, Buffer, LOC_BUFF_SZ);

		if (ReadB > 0)
			xStreamBufferSendFromISR(stream_SIM868, Buffer, ReadB,
					&xHigherPriorityTaskWoken);

	} else if (Interr & UART_IIR_INTID_THRE) {

	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_COMM_SIM868_task(void) {
	xTaskCreate(vCOMM_SIM868_Task, (char *) "SIM868 TSK", 400,
	NULL, Prio_COMM_SIM868_Task, &vCOMM_SIM868_Handle);
}

