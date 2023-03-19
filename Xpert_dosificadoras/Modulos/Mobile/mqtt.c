/*
 * mqtt.c
 *
 *  Created on: 3 ene. 2022
 *      Author: Walter Peresson
 */

#include "mqtt.h"
#include "mobile.h"

/*
 * Funciones locales
 */

bool mqtt_check_broker_connection(MOBILE_T *_mobile) {

	return sendCheckReply(_mobile, "AT+SMSTATE?", "+SMSTATE: 1", 3000);
}

/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
BROKER_CONN_REPLY_ENUM mqtt_config_conn_to_broker(MOBILE_T *_mobile,
		uint16_t port, uint16_t keepalive) {

	limpioBuffer(_mobile);

	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"URL\",\"%s\",%d",
			_mobile->credential.broker_url, port);

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;

	vTaskDelay(200);
	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"KEEPTIME\",%d", keepalive);

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;
	vTaskDelay(200);
	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"CLEANSS\",1");

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;
	vTaskDelay(200);
	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"CLIENTID\",%s",
			_mobile->credential.ClientID);

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;
	vTaskDelay(200);
	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"USERNAME\",%s",
			_mobile->credential.access_key);

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;
	vTaskDelay(200);
	sprintf(_mobile->data.bufferRX, "AT+SMCONF=\"PASSWORD\",%s",
			_mobile->credential.access_secret);

	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return false;
	vTaskDelay(200);

	return true;
}

/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool mqtt_connect_to_broker(MOBILE_T *_mobile) {

	limpioBuffer(_mobile);

	if (sendCheckReply(_mobile, "AT+SMCONN", "OK", 4 * 60000))
		return true;
	else
		return false;

}

/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool mqtt_disconnect_from_broker(MOBILE_T *_mobile) {

	return sendCheckReply(_mobile, "AT+SMDISC", "OK", 1000);
}

/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow.
 * @return	true if the connection was established, false if refused
 *
 */
bool mqtt_publish(MOBILE_T *_mobile, uint16_t pack_len, char *ptrToDeviceId) {
	/*
	 * msg1: SMPUB
	 * msg2: json
	 * */

	sprintf(_mobile->data.bufferRX, "AT+SMPUB=\"losant/%s/state\",%d,0,0",
			ptrToDeviceId, pack_len);
	if (sendCheckReply(_mobile, _mobile->data.bufferRX, ">", 2000)) {
		if (sendCheckReply(_mobile, _mobile->data.bufferTX, OK_CHAR, 20000)) {
			return true;
		}
	} else
		return false;
	return false;
}

/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool mqtt_subscribe(MOBILE_T *_mobile, char *topic, uint8_t qos) {

	sprintf(_mobile->data.bufferTX, "AT+SMSUB=\"%s\",%d", topic, qos);
	return sendCheckReply(_mobile, _mobile->data.bufferTX, "OK", 30000);
}


/**
 * @brief	Set a new connection mqtt broker
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool mqtt_unsubscribe(MOBILE_T *_mobile, char *topic) {

	sprintf(_mobile->data.bufferTX, "AT+SMUNSUB=\"%s\"", topic);
	return sendCheckReply(_mobile, _mobile->data.bufferTX, "OK", 30000);
}
