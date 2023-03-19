/*
 * TaskINPUT.c
 *
 *  Created on: 09/03/2014
 *      Author: admin
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Definiciones.h"

#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn
#define _GPDMA_CONN_ADC GPDMA_CONN_ADC

#define _LPC_ADC_IRQ ADC_IRQn

#define CANALES	6
#define PROF	15

static ADC_CLOCK_SETUP_T ADCSetup;
static uint8_t muestra_nro; //Indice del nro de muestra en curso
static uint16_t array[CANALES][PROF];

static portTASK_FUNCTION(vAN_IN_Task,pvParameters) {
	uint8_t i, j;
//	uint16_t buffer[CANALES];

//	uint16_t ADC_Result;
	uint32_t Result;

//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();

	/*ADC Init */

	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);

	//Configuro el ADC
	/* Setting ADC interrupt, ADC Interrupt must be disable in DMA mode */
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_SetSampleRate(_LPC_ADC_ID, &ADCSetup, CANALES * PROF);

	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH0, ENABLE); 	//An4
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH1, ENABLE); 	//An3
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH2, ENABLE);	//An2
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH3, ENABLE);	//An1
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH4, ENABLE);	//An6
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH5, ENABLE);	//An5
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH6, DISABLE);
	Chip_ADC_EnableChannel(_LPC_ADC_ID, ADC_CH7, DISABLE);

	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, ADC_CH5, ENABLE);

	Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);

	/* Enable ADC Interrupt */
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
	NVIC_SetPriority(_LPC_ADC_IRQ, mainSOFTWARE_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(_LPC_ADC_IRQ);

	muestra_nro = 0;

	while (1) {

		ulTaskNotifyTake(TRUE, portMAX_DELAY);

		muestra_nro++;
		if (muestra_nro >= PROF) {
			muestra_nro = 0;

			//Promediamos las muestras
			for (i = 0; i < CANALES; i++) {
				Result = 0;
				for (j = 0; j < PROF; j++)
					Result += array[i][j];
				Var.an[i].ADC_Un = Result / PROF;
			}
		}

#define HS_0_5V		5
#define LS_0_5V		0
#define HA_0_5V		4090
#define LA_0_5V		0

#define HS_0_10V	10
#define LS_0_10V	0
#define HA_0_10V	4090
#define LA_0_10V	0

#define HS_4_20mA	20
#define LS_4_20mA	4
#define HA_4_20mA	3722
#define LA_4_20mA	740

		//TRADUCIR UNIDADES ADC A SEÑAL ELÉCTRICA mA o V
		//TRADUCIR mA o V a unidades de Ingeniería
		for (i = 0; i < CANALES; i++) {
//			if (SP.An[i].Src != Not_Used) {
				switch (SP.An[i].InTyp) {
				case R4_20mA:

//					Var.an[i].Signal = ((float) (HS_4_20mA - LS_4_20mA)
//							/ (float) (HA_4_20mA - LA_4_20mA))
//							* Var.an[i].ADC_Un;

					Var.an[i].Signal = (float) Var.an[i].ADC_Un * 22 / 4096;

					Var.an[i].Eng_Un = (Var.an[i].Signal - SP.An[i].LS)
							* ((float) (SP.An[i].HE - SP.An[i].LE)
									/ (float) (SP.An[i].HS - SP.An[i].LS));

					break;
				case R0_5V:
					Var.an[i].Signal = ((float) (HS_0_5V - LS_0_5V)
							/ (float) (HA_0_5V - LA_0_5V))
							* Var.an[i].ADC_Un+ LS_0_5V;

					Var.an[i].Eng_Un = (Var.an[i].Signal - SP.An[i].LS)
							* ((float) (SP.An[i].HE - SP.An[i].LE)
									/ (float) (SP.An[i].HS - SP.An[i].LS));

					break;
				case R0_10V:
					Var.an[i].Signal = ((float) (HS_0_10V - LS_0_10V)
							/ (float) (HA_0_10V - LA_0_10V))
							* Var.an[i].ADC_Un+ LS_0_10V;

					Var.an[i].Eng_Un = (Var.an[i].Signal - SP.An[i].LS)
							* ((float) (SP.An[i].HE - SP.An[i].LE)
									/ (float) (SP.An[i].HS - SP.An[i].LS));

					break;
				default:
					break;
				}

//			} else {
//				Var.an[i].Signal = 0; //No se esta usando el canal
//				Var.an[i].Eng_Un = 0;
//			}
		}

		//LINE PRESSURE
		i = SP.AnChannelAssig[Line_Press];
		if (Var.an[i].Eng_Un < 0) //No publico valores menores a cero
			Var.LinePress = 0;
		else
			Var.LinePress = (uint16_t) Var.an[i].Eng_Un;

	}
}

void ADC_IRQHandler(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, ADC_CH5, DISABLE);

	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH3, &array[0][muestra_nro]);
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH2, &array[1][muestra_nro]);
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH1, &array[2][muestra_nro]);
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH0, &array[3][muestra_nro]);
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH5, &array[4][muestra_nro]);
	Chip_ADC_ReadValue(_LPC_ADC_ID, ADC_CH4, &array[5][muestra_nro]);

	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, ADC_CH5, ENABLE);

	vTaskNotifyGiveFromISR(tsk_an_in_handler, &xHigherPriorityTaskWoken);
}

void Create_AN_IN_task(void) {
	xTaskCreate(vAN_IN_Task, (char*) "ANALOG INP TSK", 128, NULL,
	Prio_AN_IN_Task, &tsk_an_in_handler);
}

