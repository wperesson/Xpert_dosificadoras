/*
 * CRC_Calc.h
 *
 *  Created on: 25 abr. 2017
 *      Author: BANGHO 3
 */

#ifndef SRC_CRC_CALC_H_
#define SRC_CRC_CALC_H_

uint16_t Crc16(uint8_t*, uint16_t);
void MBCrc16(uint8_t*, uint16_t);
unsigned int crc_16(unsigned char *buffer, unsigned int length);

#endif /* SRC_CRC_CALC_H_ */
