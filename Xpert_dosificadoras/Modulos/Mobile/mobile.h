/*
 * mobile.h
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#ifndef MOBILE_H_
#define MOBILE_H_

#include "Mobile_types.h"

void limpioBuffer(MOBILE_T*);
bool mobile_get_AT_replay(MOBILE_T*);
bool isMobilePoweredON(MOBILE_T*);
bool mobilePowerOn(MOBILE_T*);
bool mobilePowerOff(MOBILE_T*);
bool mobilePowerOnRoutine(MOBILE_T*);
bool mobile_set_RF(MOBILE_T*, PHONE_FUNCTIONALITY_ENUM, uint8_t);
bool mobile_set_apn_manually(MOBILE_T*);
bool mobile_get_info(MOBILE_T*, MOBILE_INFO_ENUM, uint32_t);
bool mobile_get_rssi(MOBILE_T*);
bool mobile_get_register_status(MOBILE_T*);
bool mobile_get_operator(MOBILE_T*);
time_t mobile_get_time(MOBILE_T*);
bool mobile_gprs_attach(MOBILE_T*);
uint16_t GetReply(MOBILE_T*, uint32_t);
uint16_t sendGetReply(MOBILE_T*, char*, uint32_t);
uint16_t send_RAW_GetReply(MOBILE_T*, char*, uint16_t, uint32_t);
bool sendCheckReply(MOBILE_T*, char*, const char*, uint32_t);

#endif /* MOBILE_H_ */
