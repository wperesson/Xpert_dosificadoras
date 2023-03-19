/*
 * TcpIp.c
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#include "TcpIp.h"
#include "mobile.h"

/**
 * @brief	Set a new connection to LTE network
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the ip was assigned, false if no ip
 *
 */
bool tcpip_PDP_configure(MOBILE_T *_mobile) {

	//assert(strlen(_mobile->network.gprs_apn) >= 1);

	sprintf(_mobile->data.bufferRX, "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",0",
			_mobile->network.gprs_apn, _mobile->network.gprs_usr,
			_mobile->network.gprs_pass);

	return sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000);
}

/**
 * @brief	Get the network status
 * @param
 * @return
 */
uint32_t tcpipGetNetworkStatus(MOBILE_T *_mobile) {
	char *ptr;
	uint32_t status = 0;

	limpioBuffer(_mobile);

	mobile_get_info(_mobile, CNACT, 4000);
	//"+CNACT: 0,1,\"10.46.176.230\"
	ptr = strstr(_mobile->data.bufferRX, "+CNACT:"); //+CNACT: 0,1,...
	if (ptr != NULL) {
		ptr = strtok(ptr, ",");	//0
		ptr = strtok(NULL, ",");	//1
		if (ptr != NULL)
			status = atoi(ptr);
	}
	return status;
}

/**
 * @brief	Set a new connection to LTE network
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the ip was assigned, false if no ip
 */
bool tcpip_get_ip(MOBILE_T *_mobile) {
	char *ptr;

	limpioBuffer(_mobile);

	//aux_ptr = buffer + buffer_size - 20;
	mobile_get_info(_mobile, CNACT, 4000);
	//"+CNACT: 0,1,\"10.46.176.230\"\r\n+CNACT: 1,0,\"0.0.0.0\"\r\n+CNACT: 2,0,\"0.0.0.0\"\r\n+CNACT: 3,0,\"0.0.0.0\"\r\n\r\nOK\r\n"
	ptr = strstr(_mobile->data.bufferRX, "+CNACT:");
	if (ptr != NULL) {
		ptr = strtok(ptr, ",");
		ptr = strtok(NULL, ",");
		ptr = strtok(NULL, "\r\n");
		if (ptr != NULL)
			/*Dejo almacenado en el buffer el IP*/
			strcpy(_mobile->data.bufferRX, ptr);

		if (strcmp(_mobile->data.bufferRX, "\"0.0.0.0\"") == 0) {
			return false;
		} else
			return true;
	}
	return false;
}

/**
 * @brief	Set a new connection to LTE network
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool tcpip_network_active_control(MOBILE_T *_mobile, char status) {
	char *ptr;

	limpioBuffer(_mobile);

	/*Open wireless connection parameter 0 is PDP Index, parameter 1 means active*/
	sprintf(_mobile->data.bufferRX, "AT+CNACT=0,%d", status);
	sendGetReply(_mobile, _mobile->data.bufferRX, 4000);

	ptr = strstr(_mobile->data.bufferRX, "ERROR");
	if (ptr != NULL)
		return false;

	ptr = strstr(_mobile->data.bufferRX, "OK");
	if (ptr != NULL)
		return true;

	ptr = strstr(_mobile->data.bufferRX, "+APP PDP:");
	if (ptr != NULL)
		return true;
	return false;
}

/**
 * @brief	Set a new connection to LTE network
 * @param	buffer	: pointer to a buffer for storing the string to send and receive from LTE module
 * @param	buffer_size : max buffer size to avoid overflow
 * @param	apn	: ACCESS POINT NAME. Up to 64 bytes length name. Provided by the LTE mobile operator
 * @param	user: string parameter which indicates the LTE user name.
 * @param	pwd	: string parameter which indicates the LTE password.
 * @return	true if the connection was established, false if refused
 *
 */
bool tcpip_activate_connection(MOBILE_T *_mobile) {

	limpioBuffer(_mobile);

	if (tcpip_get_ip(_mobile))
		return true;

	//No IP assigned. We should check if RF is active
	if (sendCheckReply(_mobile, "AT+CFUN?", "+CFUN: 0", 10000)) {

		/*Enable RF*/
		mobile_set_RF(_mobile, FUNCT_FULL, 0);
	}

	/*Open wireless connection parameter 0 is PDP Index, parameter 1 means active*/
	if (!tcpip_network_active_control(_mobile, ON))
		return false;

	if (tcpip_get_ip(_mobile))
		return true;
	return false;
}

/* @brief
 * @param
 * @return
 */
bool tcpipActivateNetworkConnection(MOBILE_T *_mobile) {
	uint16_t i, j;

	if (tcpip_get_ip(_mobile))
		return true;

	tcpip_PDP_configure(_mobile);

	tcpip_network_active_control(_mobile, ON);

	i = 0;
	j = 0;
	while (tcpip_get_ip(_mobile) == false) {
		if (i == 10) {
			i = 0;
			if (tcpipGetNetworkStatus(_mobile) == 0) {
				tcpip_network_active_control(_mobile, OFF);
				vTaskDelay(2000);
				tcpip_network_active_control(_mobile, ON);
			}
		}
		i++;
		j++;
		if (j >= 120)
			return false;
		vTaskDelay(1000);
	}
	return true;
}

/* @brief	Connect to server by url and post the json string
 * @param
 * @return	numbers of bytes received. 0 indicates an issue.
 */
uint16_t tcpipConnectToServer(MOBILE_T *_mobile, char *url, char *path,
		char *json) {

	uint16_t i, serverStatusCode, receivedBytesCount;
	char *ptr, *str;

	if (tcpipActivateNetworkConnection(_mobile) == false)
		return 0;

	/*If HTTP is connected, then disconnect*/
	if (sendCheckReply(_mobile, "AT+SHSTATE?", "+SHSTATE: 1", 1000))
		/*Terminate HTTP service*/
		sendCheckReply(_mobile, "AT+SHDISC", OK_CHAR, 4000);

	/*Set connect server parameter*/
	str = &_mobile->data.bufferRX[_mobile->data.bufferRX_size / 4];
	sprintf(str, "AT+SHCONF=\"URL\",\"%s\"", url);
	if (!sendCheckReply(_mobile, str, OK_CHAR, 1000))
		return 0;

	/*Set max body length*/
	if (!sendCheckReply(_mobile, "AT+SHCONF=\"BODYLEN\",1024", OK_CHAR, 1000))
		return 0;

	/*Set max header length*/
	if (!sendCheckReply(_mobile, "AT+SHCONF=\"HEADERLEN\",350", OK_CHAR, 1000))
		return 0;

	/*Configure SSL/TLS version*/
	if (!sendCheckReply(_mobile, "AT+CSSLCFG=\"sslversion\",0,3", OK_CHAR,
			1000))
		return 0;

	/*Connect HTTP server, make 3 attempts */
	i = 0;
	while (sendCheckReply(_mobile, "AT+SHCONN", OK_CHAR, 30000) == false) {
		i++;
		if (i > 3)
			return 0;
	}

	/*Get HTTP status*/
	if (!sendCheckReply(_mobile, "AT+SHSTATE?", "+SHSTATE: 1", 1000))
		return 0;

	/*Clear HTTP header*/
	if (!sendCheckReply(_mobile, "AT+SHCHEAD", OK_CHAR, 1000))
		return 0;

	/*Add header content*/
	if (!sendCheckReply(_mobile,
			"AT+SHAHEAD=\"Content-Type\",\"application/json\"", OK_CHAR, 1000))
		return 0;

	/*Add header content*/
	if (!sendCheckReply(_mobile, "AT+SHAHEAD=\"Cache-control\",\"no-cache\"",
	OK_CHAR, 1000))
		return 0;

	/*Add header content*/
	if (!sendCheckReply(_mobile, "AT+SHAHEAD=\"Connection\",\"keep-alive\"",
	OK_CHAR, 1000))
		return 0;

	/*Add header content*/
	if (!sendCheckReply(_mobile, "AT+SHAHEAD=\"Accept\",\"*/*\"", OK_CHAR,
			1000))
		return 0;

	/*Add header content*/
	sprintf(_mobile->data.bufferRX, "AT+SHAHEAD=\"Authorization\",\"%s\"", JWT);
	if (!sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000))
		return 0;

	/*Armo la cadena JSON con el SN y demas info para crear el device*/

	if (strlen(json) > 0) {
		/*POST the json data*/
		sprintf(_mobile->data.bufferRX, "AT+SHBOD=%d,10000", strlen(json));
		if (sendCheckReply(_mobile, _mobile->data.bufferRX, ">", 1000)) {
			if (!sendCheckReply(_mobile, json, OK_CHAR, 15000)) {
				/*No recibí el OK del comando*/
				return 0;
			}
		} else
			return 0;
	}
	/*Espero respuesta del servidor*/
	sprintf(_mobile->data.bufferRX, "AT+SHREQ=\"%s\",3", path);
	if (sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 5000)) {

		GetReply(_mobile, 2000);
		//		+SHREQ: "POST",400,48
		ptr = strstr(_mobile->data.bufferRX, "+SHREQ:");
		if (ptr != NULL) {
			ptr = strtok(_mobile->data.bufferRX, " :");
			ptr = strtok(NULL, " ,");
			ptr = strtok(NULL, " ,");
			serverStatusCode = atoi(ptr);
			ptr = strtok(NULL, " ,\r");
			receivedBytesCount = atoi(ptr);
		}
		//	+CME ERROR: operation not allowed
		ptr = strstr(_mobile->data.bufferRX, "+CME ERROR:");
		if (ptr != NULL) {
			return 0;
		}

		/* 200 OK
		 *
		 * 400 Bad Request
		 * 401 Unauthorized
		 * 402 Payment Required
		 * 403 Forbidden
		 * 404 Not Found
		 *
		 * 408 Request timeout
		 *
		 * 600 Not HTTP PDU
		 * 601 Network Error
		 * 602 No memory
		 */

		switch (serverStatusCode) {
		case 200:
			/*Request OK*/
			break;
		case 601:
			/*Network Error*/
			return 0;
		case 400:
			/*Bad Request*/
			break;
		default:
			//Not ok
			return 0;
			break;
		}
	} else
		return 0;

	sprintf(_mobile->data.bufferRX, "AT+SHREAD=0,%d", receivedBytesCount);
	if (!sendGetReply(_mobile, _mobile->data.bufferRX, 1000)) {
		/*El servidor envió una respuesta. Ahora debo leerlas y guardarlas en EEPROM*/
		ptr = strstr(_mobile->data.bufferRX, "\"error\"");
		if (ptr) {
			/*El servidor envió un mensaje de error */
			return 0;
		}
	}
	return receivedBytesCount;
}

/* @brief	Get endpoint from LOSANT using https service
 * @param
 * @return	numbers of bytes received. 0 indicates an issue.
 */
uint16_t https_get_endpoint(MOBILE_T *_mobile) {

	return tcpipConnectToServer(_mobile, "https://edgerouting.onlosant.com",
			"/routing", NULL);
}

/* @brief	Get credentials from LOSANT using https service
 * @param
 * @return	numbers of bytes received. 0 indicates an issue.
 */
uint16_t https_get_credentials(MOBILE_T *_mobile, volatile DEVICE_T *_device) {
	uint16_t len;

	/*Armo la cadena JSON con el SN y demas info para crear el device*/
	len = sprintf(_mobile->data.bufferTX, ""
			"{"
			"\"serial\":\"%s\","
			"\"MAC\":\"%s\","
			"\"cont_type\":\"%s\","
			"\"dev_type\":\"LP\","
			"\"owner\":\"%s\","
			"\"pow_type\":\"solar\""
			"}", _device->id.serialNumber, _device->id.xbeeMac, _device->type,
			_device->owner);

	assert(len <= _mobile->data.bufferTX_size);
	return tcpipConnectToServer(_mobile, _mobile->credential.endpoint_url,
			_mobile->credential.endpoint_path, _mobile->data.bufferTX);
}
