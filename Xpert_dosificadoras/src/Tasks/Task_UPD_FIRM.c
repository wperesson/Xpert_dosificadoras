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
#include "FTP_SIM868.h"

/* Librerias de C includes. */
#include "Definiciones.h"

#define T15			750E-6	//750 uS
#define T35			1.75E-3//1.75 mS

#define BUF_SIZE			100

#define TIME_RESET_CNT		1000

#define UART				LPC_UART3

/* Size of each sector */
//#define SECTOR_SIZE             1024
/* FTP UPGRADE SECTOR */
//#define IAP_FTP_START_SECTOR    24
/* Start sector address */
//#define START_ADDR_FTP_SECTOR  0x00078000
/* LAST SECTOR */
//#define IAP_LAST_SECTOR         29
const unsigned sector_start_map[MAX_FLASH_SECTOR] = { SECTOR_0_START,
SECTOR_1_START, SECTOR_2_START, SECTOR_3_START, SECTOR_4_START, SECTOR_5_START,
SECTOR_6_START, SECTOR_7_START, SECTOR_8_START, SECTOR_9_START, SECTOR_10_START,
SECTOR_11_START, SECTOR_12_START, SECTOR_13_START, SECTOR_14_START,
SECTOR_15_START,
SECTOR_16_START, SECTOR_17_START, SECTOR_18_START, SECTOR_19_START,
SECTOR_20_START,
SECTOR_21_START, SECTOR_22_START, SECTOR_23_START, SECTOR_24_START,
SECTOR_25_START,
SECTOR_26_START, SECTOR_27_START, SECTOR_28_START, SECTOR_29_START };

const unsigned sector_end_map[MAX_FLASH_SECTOR] = { SECTOR_0_END, SECTOR_1_END,
SECTOR_2_END, SECTOR_3_END, SECTOR_4_END, SECTOR_5_END, SECTOR_6_END,
SECTOR_7_END,
SECTOR_8_END, SECTOR_9_END, SECTOR_10_END, SECTOR_11_END, SECTOR_12_END,
SECTOR_13_END, SECTOR_14_END, SECTOR_15_END, SECTOR_16_END,
SECTOR_17_END,
SECTOR_18_END, SECTOR_19_END, SECTOR_20_END, SECTOR_21_END,
SECTOR_22_END,
SECTOR_23_END, SECTOR_24_END, SECTOR_25_END, SECTOR_26_END,
SECTOR_27_END, SECTOR_28_END, SECTOR_29_END };

/* Number elements in array */
#define ARRAY_ELEMENTS       (IAP_NUM_BYTES_TO_WRITE / sizeof(uint32_t))
#define BUFFER_SIZE			(ARRAY_ELEMENTS * sizeof(uint32_t))
/*-----------------------------------------------------------
 * 	 */
static portTASK_FUNCTION(vUPD_FIRM, pvParameters) {

	uint8_t ret_code;
	uint32_t part_id;
	static uint16_t idx;
	/* Data array to write to flash */
	static uint32_t Buffer[ARRAY_ELEMENTS];
	static bool FLAG;

	while (1) {

//		ulTaskNotifyTake(TRUE, portMAX_DELAY);
		while (!FLAG) {
			vTaskDelay(5000);
		}

		while (xSemaphoreTake(xMtxSIM868, 10000) != pdTRUE) {
			//Espero que se libere totalmente el recurso
		}
		//Ver el estado de la red, si esta activado el GPRS y si esta conectado a internet
		if (!GPRS_get_status(UART, (char*) &Buffer[0],
		BUFFER_SIZE)) {
			//Encender GPRS
		}

		//Ver el estado de la sesion FTP
		if (!ftp_sim868_get_stat(UART, Buffer,
		ARRAY_ELEMENTS * sizeof(uint32_t))) {
			//Establecer conexion con el servidor FTP, setear UN, PASS, server
			ftp_sim868_set_conn(UART, Buffer, BUFFER_SIZE, APN, FTP_SERV, "21",
					FTP_USR, FTP_PASS);
		}

		//Chequeo si existe un archivo nuevo para bajar. Para ello bajo el archivo FVer.txt que me indica
		//que version esta disponible para bajar. Luego comparo con la actual y si es mayor procedo a bajar el archivo completo.
		ftp_sim868_GET_FILE(UART, (char*) Buffer, BUFFER_SIZE, "Fver.txt", "/",
		IAP_NUM_BYTES_TO_WRITE);

//Bajar el archivo a la flash.

//Verificar el archivo recibido

		/* ===================================
		 * PROGRAMAR LA FLASH
		 *
		 * Verificar:
		 * 		1- La bateria este a un nivel seguro
		 * 		2- El código sea válido. Para esto se puede generar un archivo extra txt con
		 * 		informacion para contrastar in situ
		 */

//Desactivar todas las interrupciones y las otras tareas o elevar la prioridad de esta al maximo
		/* Disable interrupt mode so it doesn't fire during FLASH updates */
		taskENTER_CRITICAL();
		//		__disable_irq();
//		/* IAP Flash programming */
//		/* Prepare to write/erase the last sector */
//		ret_code = Chip_IAP_PreSectorForReadWrite(USER_START_SECTOR,
//		MAX_USER_SECTOR);
//
//		/* Error checking */
//		if (ret_code != IAP_CMD_SUCCESS) {
//			DEBUGOUT(
//					"Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n",
//					ret_code);
//		}
//
//		/* Erase the last sector */
//		ret_code = Chip_IAP_EraseSector(USER_START_SECTOR, MAX_USER_SECTOR);
//
//		/* Error checking */
//		if (ret_code != IAP_CMD_SUCCESS) {
//			DEBUGOUT(
//					"Chip_IAP_EraseSector() failed to execute, return code is: %x\r\n",
//					ret_code);
//		}
//
//		/* Prepare to write/erase the last sector */
//		ret_code = Chip_IAP_PreSectorForReadWrite(USER_START_SECTOR,
//		MAX_USER_SECTOR);
//
//		/* Error checking */
//		if (ret_code != IAP_CMD_SUCCESS) {
//			DEBUGOUT(
//					"Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n",
//					ret_code);
//		}
//
//		/* Write to the last sector */
//		ret_code = Chip_IAP_CopyRamToFlash(USER_FLASH_START, buffer,
//		IAP_NUM_BYTES_TO_WRITE);
//
//		/* Error checking */
//		if (ret_code != IAP_CMD_SUCCESS) {
//			DEBUGOUT(
//					"Chip_IAP_CopyRamToFlash() failed to execute, return code is: %x\r\n",
//					ret_code);
//		} else {
//			idx += IAP_NUM_BYTES_TO_WRITE;
//		}
//
//		/* Re-enable interrupt mode */
//		__enable_irq();
		taskEXIT_CRITICAL();

//			/* Start the signature generator for the last sector */
//			Chip_FMC_ComputeSignatureBlocks(USER_FLASH_START,
//					(SECTOR_SIZE / 16));
//
//			/* Check for signature geenration completion */
//			while (Chip_FMC_IsSignatureBusy()) {
//			}
//
//			/* Get the generated FLASH signature value */
//			DEBUGOUT("Generated signature for the last sector is: %x \r\n",
//					Chip_FMC_GetSignature(0));
		xSemaphoreGive(xMtxSIM868);

	}

}

void Create_UPD_FIRM_Task(void) {
	xTaskCreate(vUPD_FIRM, (char *) "UPDT FIRM TSK", 400, NULL,
	Prio_UPD_FIRM_Task, &vCOMM_WHARTHandle); /* "Comunicación con el HMI*/
}

