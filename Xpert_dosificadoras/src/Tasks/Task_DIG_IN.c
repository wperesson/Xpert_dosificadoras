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

#define FILTER_CNT_LIM	4

static portTASK_FUNCTION(vIO_Task,pvParameters) {
	portTickType xLastWakeTime;

	uint32_t Puerto;
	static uint8_t cont_door_open, cont_door_closed;

	xLastWakeTime = xTaskGetTickCount();
	Puerto = 0;
	cont_door_open = 0;
	cont_door_closed = 0;

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_RATE_MS));
		Puerto = Chip_GPIO_ReadValue(LPC_GPIO, 0);
		//Leo el sensor de la puerta abierta

		if (((1 << DI_0) & Puerto)) {
			Var.DI |= (1 << 0);
		} else {
			Var.DI &= ~(1 << 0);
		}

		if (!((1 << DI_1) & Puerto))
			Var.DI |= (1 << 1);
		else
			Var.DI &= ~(1 << 1);

		if (!((1 << DI_2) & Puerto))
			Var.DI |= (1 << 2);
		else
			Var.DI &= ~(1 << 2);

		if (!((1 << DI_3) & Puerto))
			Var.DI |= (1 << 3);
		else
			Var.DI &= ~(1 << 3);

		if (!((1 << DI_4) & Puerto))
			Var.DI |= (1 << 4);
		else
			Var.DI &= ~(1 << 4);

		/*
		 * Leo el estado de la puerta para saber si paso a manual o auto
		 */
		if (Var.DI & 0x01) { //DOOR OPEN, pasar a local
			cont_door_closed = 0;
			if (cont_door_open > 20) {
				Var.local_remoto = TRUE;
			} else {
				cont_door_open++;
			}

		} else {
			cont_door_open = 0;
			if (cont_door_closed > 20) {
				Var.local_remoto = FALSE;
			} else {
				cont_door_closed++;
			}
		}

//		/*
//		 * Analizo el estado de las entradas digitales 3 y 4 que son START y STOP
//		 */
//		if (Var.DI & 0x01) { //DOOR OPEN
//			cont_start = 0;
//			cont_stop = 0;
//		} else {	//DOOR CLOSED
//
//			//STOP
//			if (Var.DI & DIn_4) {
//				if (cont_stop == FILTER_CNT_LIM) {
//					//Enviar comando de stop
//					newParameter.Val = 0;
//					newParameter.parameter = PUMP_ON;
//					newParameter.Src = src_dig_input;
//					xQueueSend(queHEAD_FLOW_SETT_upd, &newParameter, 0);
//					//Resetear contador de start
//					cont_start = 0;
//					cont_stop++;
//				} else if (cont_stop < FILTER_CNT_LIM) {
//					cont_stop++;
//				}
//			} else {
//				//Reseteo el contador
//				cont_stop = 0;
//
//				//START
//				if (Var.DI & DIn_3) {
//					if (cont_start == FILTER_CNT_LIM) {
//						//Enviar comando de start
//						newParameter.Val = 1;
//						newParameter.parameter = PUMP_ON;
//						newParameter.Src = src_dig_input;
//						xQueueSend(queHEAD_FLOW_SETT_upd, &newParameter, 0);
//						//Resetear contador de stop
//						cont_stop = 0;
//						cont_start++;
//					} else if (cont_start < FILTER_CNT_LIM) {
//						cont_start++;
//					}
//				} else {
//					//Reseteo el contador
//					cont_start = 0;
//				}
//			}
//		}
	}
}

void Create_DIG_IN_task(void) {
	xTaskCreate(vIO_Task, (char*) "DIGITAL INP TSK", 64, NULL,
	Prio_DI_Task, &tsk_di_handler);
}

