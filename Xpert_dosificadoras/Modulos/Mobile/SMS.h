/*
 * SMS.h
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#ifndef SMS_H_
#define SMS_H_

#include "Mobile_types.h"



bool sms_init(MOBILE_T*);
bool sms_send(MOBILE_T*, char*);
bool sms_delete(MOBILE_T*, uint8_t);
bool readSMS(MOBILE_T*, PHONE_DATA_T*);

#endif /* SMS_H_ */
