/*
 * gps.h
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#ifndef SRC_GPS_H
#define SRC_GPS_H

#include "Mobile_types.h"

bool isGpsPoweredOn(MOBILE_T*);
bool setGpsPowerTo(MOBILE_T*, uint8_t);
bool requestGpsCoordinates(MOBILE_T*, GPS_DATA_T*);
bool readGpsCoordinates(MOBILE_T *, GPS_DATA_T *);

#endif /* SRC_GPS_H */
