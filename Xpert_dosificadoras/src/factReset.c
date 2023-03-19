/*
 * factReset.c
 *
 *  Created on: 07/10/7
 *      Author: WPeresson
 */

#include "Definiciones.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
void set_factory_data(uint16_t Code) {

	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_write((char*) &SP, OFF_FACT_DATA, sizeof(SP));
		xSemaphoreGive(mtxI2C1);
	}
}

void read_factory_data(uint16_t Code) {

	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_read((char*) &SP, OFF_FACT_DATA, sizeof(SP));
		xSemaphoreGive(mtxI2C1);
	}
	//SP.key = (uint16_t) 0xabcd0001;
}
