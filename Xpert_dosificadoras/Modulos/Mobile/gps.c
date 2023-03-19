/*
 * gps.c
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#include "gps.h"
#include "mobile.h"

bool isGpsPoweredOn(MOBILE_T *_mobile) {

	if (sendCheckReply(_mobile, "AT+CGNSPWR?", "+CGNSPWR: 1", 1500))
		return true; 	//GPS encendido
	else
		return false;	//GPS apagado
}

bool setGpsPowerTo(MOBILE_T *_mobile, uint8_t status) {

	sprintf(_mobile->data.bufferRX, "AT+CGNSPWR=%d", status);
	if (sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1500)) {
		return true;
	}

	return false;
}

bool requestGpsCoordinates(MOBILE_T *_mobile, GPS_DATA_T *gps) {
	char *ptr;
	char *ptr_start;
	char *ptr_end;
	bool status;
	uint8_t len;

	char localBuff[4];

	/* Reinicio valores para asegurar*/
	gps->gps_run = 0;

	/* Pido la info del GPS */
	status = sendGetReply(_mobile, "AT+CGNSINF", 3000);
	if (status) {
		//+CGNSINF: 1,,,0.000000,0.000000,-18.000,,,1,,0.1,0.1,0.1,,,,9999000.0,6000.0\r\n\r\nOK\r\n"
		ptr = strstr(_mobile->data.bufferRX, "+CGNSINF: ");
		if (!ptr)
			return false;

		ptr_start = strstr(ptr, ":") + 1;
		ptr_end = strstr(ptr_start, ",");
		len = ptr_end - ptr_start;
		if (len > 0) {
			memset(localBuff, 0x00, 4);
			// is GPS Running?
			strncat(localBuff, ptr_start, len);
			gps->gps_run = atoi(localBuff);
		} else
			gps->gps_run = 0;

		ptr_start = ptr_end + 1;
		ptr_end = strstr(ptr_start, ",");
		len = ptr_end - ptr_start;
		if (len > 0) {
			memset(localBuff, 0x00, 4);
			// grab fix status
			strncat(localBuff, ptr_start, len);
			gps->gps_fix = atoi(localBuff);
		} else
			gps->gps_fix = 0;
	} else
		return false;
	return true;
}

bool readGpsCoordinates(MOBILE_T *_mobile, GPS_DATA_T *_gps) {
	char *ptr;
	char local_buff[5];

	/* Reinicio valores para asegurar*/
	_gps->gps_run = 0;

	/* Pido la info del GPS */
	requestGpsCoordinates(_mobile, _gps);

	if (_gps->gps_fix == false)
		return false;

	ptr = strtok(_mobile->data.bufferRX, ":");

	//GPS Running status
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;

	// GPS fix status
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;

	// grab the date & time
	ptr = strtok(NULL, ".");
	if (!ptr)
		return false;

	//grab time info
	if (strlen(ptr) == 14) {

		local_buff[4] = 0;
		strncpy(local_buff, ptr, 4);

		_gps->GPS_time.tm_year = atoi(local_buff) - 1900;
		local_buff[2] = 0;
		strncpy(local_buff, ptr + 4, 2);
		_gps->GPS_time.tm_mon = atoi(local_buff) - 1;

		strncpy(local_buff, ptr + 6, 2);
		_gps->GPS_time.tm_mday = atoi(local_buff);

		strncpy(local_buff, ptr + 8, 2);
		_gps->GPS_time.tm_hour = atoi(local_buff);

		strncpy(local_buff, ptr + 10, 2);
		_gps->GPS_time.tm_min = atoi(local_buff);

		strncpy(local_buff, ptr + 12, 2);
		_gps->GPS_time.tm_sec = atoi(local_buff);

		/* Convert tm structure to seconds */
		_gps->UTC_time = mktime(&_gps->GPS_time);

	}
	ptr = strtok(NULL, ",");

	// grab LAT
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;
	_gps->Lat = atof(ptr);
	// grab LON
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;
	_gps->Lon = atof(ptr);

	// grab ALTITUDE
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;
	_gps->Altitude = atof(ptr);

	// grab SPEED
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;
	_gps->Speed_kph = atof(ptr);

	// grab Course Over Ground
	ptr = strtok(NULL, ",");
	if (!ptr)
		return false;
	_gps->course_over_gnd = atoi(ptr);

	return true;
}
