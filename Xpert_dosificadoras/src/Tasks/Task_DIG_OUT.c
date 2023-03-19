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

static portTASK_FUNCTION(vDIG_OUT_Task,pvParameters) {
	portTickType xLastWakeTime;
//	static Bool LED1 = FALSE;
//	static Bool LED2 = FALSE;
//
//	uint8_t i, cont;
//	uint8_t Puerto;

	xLastWakeTime = xTaskGetTickCount();
//	Puerto = 0;

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));

		//PUMP STATUS
		/*
		 * Esta salida indica el estado de la bomba. Cuando la misma esta funcionando debe estar activa.
		 * Si la bomba esta parada, ya sea por un comando de un operario o un corte de energía.
		 * La lógica de la misma debe ser directa de modo que si se corta la energía el relé de salida indique
		 * que la bomba esta parada
		 *
		 * ACTIVADO: 	1 = RUNNING
		 * DESACTIVADO: 0 = STOPPED
		 */
//		if (PumpOn) {
//			SGN_O3_ON;
//		} else {
//			SGN_O3_OFF;
//		}

		//AUTO MANUAL Indicator
		/*
		 * Cuando un operario abre la puerta, la bomba pasa a manual automáticamente.
		 * Como la puerta estará normalmente cerrada, el relé lo voy a activar solo cuando se
		 * abre la puerta, de modo que la lógica de salida será:
		 *
		 * ACTIVADO: 	1 = MANUAL
		 * DESACTIVADO: 0 = AUTO
		 *
		 * ***************PRECAUCIÓN: EN LA LÓGICA DEL SCADA ELLOS IMPLEMENTARON******************
		 *
		 * 				ACTIVADO		->	AUTO
		 * 				DESACTIVADO		->	MANUAL
		 *****************************************************************************************
		 */
//		if (Var.Auto_mode) { //DOOR OPEN, pasar a manual
//			SGN_O4_ON;
//		} else {
//			SGN_O4_OFF;
//		}

		(Var.DO & DOut_1_MASK) ? DO_O1_ON : DO_O1_OFF;

		(Var.DO & DOut_2_MASK) ? DO_O2_ON : DO_O2_OFF;

		(Var.DO & DOut_3_MASK) ? DO_O3_ON : DO_O3_OFF;

		(Var.DO & DOut_4_MASK) ? DO_O4_ON : DO_O4_OFF;

		(Var.DO & DOut_rl1_MASK) ? DO_RL1_ON : DO_RL1_OFF;

//		(Var.DO & DOut_rl2_MASK) ? DO_RL2_ON : DO_RL2_OFF;
	}
}

void Create_DIG_OUT_task(void) {
	xTaskCreate(vDIG_OUT_Task, (char*) "DIGITAL OUTP TSK", 60,
	NULL, Prio_DO_Task, &tsk_do_handler);
}

