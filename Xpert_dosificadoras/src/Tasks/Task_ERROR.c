/*
 * Task_ReadADC.c
 *
 *  Created on: 30/01/2014
 *      Author: Lisa
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Librerias de C includes. */
#include "Definiciones.h"

static void vERROR(void *pvParameters);

/*-----------------------------------------------------------
 * 	PROCESAMIENTO DE ERRORES DEL INSTRUMENTO. */
static portTASK_FUNCTION(vERROR, pvParameters) {
	portTickType xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

//	RegErrores = 0; //Resetea el valor del registro

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_RATE_MS));

//		Temp = ErrorCode;

	}

}

void Create_ERROR_Task(void) {
	xTaskCreate(vERROR, (char *) "ERROR TSK ", 128, NULL,
	Prio_ERROR_Task, &tsk_error_handler); /* "Procesamiento de ERRORES*/
}

