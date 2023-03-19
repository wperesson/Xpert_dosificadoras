/*
 * Task_EVENTS_LOG.c
 *
 *  Created on: 23/06/2018
 *      Author: W. Peressón
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Librerias de C includes. */
#include "Definiciones.h"

static void vEVLOG(void *pvParameters);

/*-----------------------------------------------------------
 * 	GESTIO DEL LOG DE EVENTOS. */

/* El sistema gestionará un log de eventos codificados y almacenados en EEPROM. Para leer el log se
 * recurrirá a esta tarea ya que la información estará ordenada en memoria EEPROM y codificada. Para
 * hacerla legible se debe reinterpretar el comando y el tiempo en el cual sucedió el mismo.
 *
 * Cada registro del log consta de un byte de código de operación y 4 bytes de tiempo almacenado en
 * segundos en formato time_t
 *
 * La cantidad de memoria EEPROM que destinaremos a este LOG será de 3 bloques consecutivos o sea serán
 * 256 * 3 bytes = 768 bytes
 *
 * Si cada registro del LOG es de 5 bytes, el total de registros que podremos almacenar será de 153
 *
 * El reporte del LOG estará disponible a petición del usuario mediante SMS.
 *
 * PONER ESPECIAL CUIDADO DE NO ESCRIBIR MAS ALLA DE LOS LIMITES DE 3 BLOQUES PARA EVITAR SOBREESCRIBIR
 * DATOS RELEVANTES.
 *
 * Byte 1: Cola
 * Byte 2: Cabeza
 *
 */
static portTASK_FUNCTION(vEVLOG, pvParameters) {
//	portTickType xLastWakeTime;

	Events_to_log_Type event_to_log;

//	static uint8_t Cont_Alarm_Lev_up, Cont_Alarm_Lev_dn, Cont_Warning_Lev_up,
//			Cont_Warning_Lev_dn, Cont_Low_Batt_up, Cont_Low_Batt_dn,
//			Cont_Very_Low_Batt;

	vTaskDelay(10000); //Espero 10 segundos hasta q arranquen las otras tareas

	while (1) {

		if (xQueueReceive(queEV_LOG, &event_to_log, portMAX_DELAY) != pdFAIL) {

			//Almacenar en EEPROM nueva entrada del log
//			guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
		}
	}
}

void Create_EVLOG_Task(void) {
	xTaskCreate(vEVLOG, (char*) "EVENTS LOG TSK", 128, NULL, Prio_EV_LOG_Task,
			&tsk_events_log_handler); /* "Gestion del Log de eventos*/
}

