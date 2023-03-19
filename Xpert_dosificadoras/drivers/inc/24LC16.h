/*****************************************************************************
 *   eeprom.h:  Header file for the 24LC08 EEPROM
 *
 *   Copyright(C) 2009, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************/
#ifndef __EEPROM_H
#define __EEPROM_H

#include "board.h"

#define I2CDEV I2C1

#define EEPROM_I2C_ADDR1    (0x50)
#define EEPROM_I2C_ADDR2    (0x51)
#define EEPROM_I2C_ADDR3    (0x52)
#define EEPROM_I2C_ADDR4    (0x53)

#define EEPROM_TOTAL_SIZE 2048
#define EEPROM_BLOCK_SIZE  256
#define EEPROM_PAGE_SIZE    16

void eeprom_init(void);
int16_t eeprom_read(char* buf, uint16_t offset, uint16_t len);
int16_t eeprom_write(char* buf, uint16_t offset, uint16_t len);

#endif /* end __EEPROM_H */
/****************************************************************************
 **                            End Of File
 *****************************************************************************/
