/*
 * mobile.c
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#include "mobile.h"
#include "TcpIp.h"
//#include "peripherals.h"

/*
 * 	FUNCIONES PRIVADAS
 */
void power_cycle(void) {
	/*PWRKEY should be pulled low at
	 least 1 second and then released to
	 power on/down the module.*/

	/*Intento ciclo de 1 seg de Pwr Key para encender el módulo*/
	PWRKEY_4G_ON; //PWR_KEY=0
	vTaskDelay(1100);
	PWRKEY_4G_OFF; //PWR_KEY=1
}

/**
 * @brief Enciende el módulo celular.
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	timeout : time to wait to get the reply
 * @return	True: mobile turned ON, false: error on turning ON
 */
bool mobilePowerOnSequence(MOBILE_T *_mobile) {

	char *ptr;
	uint32_t baud;
	bool ready = false;

	/* Deshabilito interrupciones por el uart*/
#if USE_K32L2B3
	UART_EnableRx(UART2, false);
#elif USE_LPC1769
	//COMPLETAR CON LAS FUNCIONES PARA EL LPC1769
#endif

	power_cycle();

	/* Habilito interrupciones por el uart*/
#if USE_K32L2B3
	UART_ClearStatusFlags(UART2, kUART_RxOverrunFlag);
	UART_ReadByte(UART2);
	UART_EnableRx(UART2, true);
#elif USE_LPC1769
	//COMPLETAR CON LAS FUNCIONES PARA EL LPC1769
#endif

	limpioBuffer(_mobile);

	/*Espero el mensaje de bienvenida del MOBILE
	 * (12:47:01.1) 0AhRDY
	 * (12:47:01.1) 0Ah
	 * (12:47:01.4) 0Ah+CFUN: 1
	 * (12:47:01.4) 0Ah
	 * (12:47:01.5) 0Ah+CPIN: READY
	 * (12:47:01.5) 0Ah
	 * (12:47:01.7) 0AhSMS Ready
	 */

	while (GetReply(_mobile, 3000) > 0) {
		/* check for RDY message */
		ptr = strstr(_mobile->data.bufferRX, "RDY");
		if (ptr != NULL)
			ready = true;
		else {
			/* check for +CFUN message */
			ptr = strstr(_mobile->data.bufferRX, "+CFUN");
			if (ptr != NULL)
				ready = true;
			else
				ready = false;
		}
	}

	if (false == ready) {
		/* Puede que este en auto-baud*/
		uint8_t i;
		for (i = 0; i < 5; i++) {
			if (isMobilePoweredON(_mobile)) {
				ready = true;
				break;
			}
		}
	}

	/* Chequeo que el baud rate este fijo en 115200*/
	if (mobile_get_info(_mobile, IPR, 1000)) {
		ptr = strstr(_mobile->data.bufferRX, "+IPR:");
		if (ptr != NULL) {
			ptr = strtok(ptr, ":");
			ptr = strtok(NULL, "\r\n");
			if (ptr != NULL) {
				baud = atoi(ptr);
			}

			if (baud != 115200) {
				sendCheckReply(_mobile, "AT+IPR=115200", OK_CHAR, 10);
			}
		}
	} //todo: chequear 	que a veces ewntra a grabar esta info de nuevo aunque el IPR este bien seteado

	return ready;
}

/*
 * 	FUNCIONES PUBLICAS
 */

void limpioBuffer(MOBILE_T *_mobile) {
	memset(_mobile->data.bufferRX, 0, _mobile->data.bufferRX_size);
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_get_AT_replay(MOBILE_T *_mobile) {

	limpioBuffer(_mobile);
	return sendCheckReply(_mobile, "AT", OK_CHAR, 500);
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool isMobilePoweredON(MOBILE_T *_mobile) {
	return mobile_get_AT_replay(_mobile);
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_get_info(MOBILE_T *_mobile, MOBILE_INFO_ENUM info_id,
		uint32_t timeout) {
	char message[20];

	switch (info_id) {
	case IMEI:
		strcpy(message, "AT+GSN");
		break;
	case CIMI:
		strcpy(message, "AT+CIMI");
		break;
	case CSQ:
		strcpy(message, "AT+CSQ");
		break;
	case CREG:
		strcpy(message, "AT+CREG?");
		break;
	case CGATT:
		strcpy(message, "AT+CGATT?");
		break;
	case COPS:
		strcpy(message, "AT+COPS?");
		break;
	case CNACT:
		strcpy(message, "AT+CNACT?");
		break;
	case CFUN:
		strcpy(message, "AT+CFUN?");
		break;
	case IPR:
		strcpy(message, "AT+IPR?");
		break;
	case CLTS:
		strcpy(message, "AT+CLTS?");
		break;
	case CCLK:
		strcpy(message, "AT+CCLK?");
		break;
	case GMR:
		strcpy(message, "AT+GMR");
		break;
	case CMGF:
		strcpy(message, "AT+CMGF?");
		break;
	default:
		break;
	}

	return sendCheckReply(_mobile, message, OK_CHAR, timeout);
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_get_rssi(MOBILE_T *_mobile) {

	char *ptr;
	/*Escala de señal RSSI
	 * <10: Marginal
	 * 10 to 14: OK
	 * 15 to 19: Good
	 * 20 to 30: Excellent
	 * 99 Not known or not detectable
	 */

	/*Escala de ber
	 * 0 to 7
	 * 99 Not known or not detectable
	 */

	/*Request signal quality*/
	if (mobile_get_info(_mobile, CSQ, 2000))

		ptr = strstr(_mobile->data.bufferRX, "+CSQ:");
	if (ptr != NULL) {
		ptr = strtok(ptr, ": ");

		ptr = strtok(NULL, ",");
		if (ptr != NULL)
			_mobile->status.rssi = atoi(ptr);
		ptr = strtok(NULL, ",");
		if (ptr != NULL)
			_mobile->status.ber = atoi(ptr);
		return true;
	}
	return false;
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if the UE is already registered, False if it isn´t registered
 */
bool mobile_get_register_status(MOBILE_T *_mobile) {
	char *ptr;

	mobile_get_info(_mobile, CREG, 4000);
	ptr = strstr(_mobile->data.bufferRX, "+CREG:");
	if (ptr != NULL) {
		ptr = strtok(ptr, ": ");

		ptr = strtok(NULL, ",");

		ptr = strtok(NULL, "\r\n");

		if (ptr != NULL) {
			_mobile->status.reg_on_netw = atoi(ptr);
			if ((_mobile->status.reg_on_netw == REGISTERED_HN)
					| (_mobile->status.reg_on_netw == REGISTERED_ROAM))
				return true;
		}
	}
	return false;
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if the UE is already registered, False if it isn´t registered
 */
bool mobile_get_operator(MOBILE_T *_mobile) {
	char *ptr;

	/*Request OPER name*/
	if (mobile_get_info(_mobile, COPS, 4000)) {
		if (strstr(_mobile->data.bufferRX, OK_CHAR)) {

			/*Leo el nombre de la operadora*/
			ptr = strstr(_mobile->data.bufferRX, "+COPS:");
			if (ptr != NULL) {

				ptr = strtok(ptr, ","); //+COPS: 0

				ptr = strtok(NULL, ",");	//0

				ptr = strtok(NULL, ",");	//"Movistar Movistar"
				if (ptr != NULL)
					sscanf(ptr, "\"%[^\"]s", _mobile->status.oper);

				ptr = strtok(NULL, " ,\r\n");
				if (ptr != NULL) {
					_mobile->status.accessTechnology = atoi(ptr);
					return true;
				}
			}
		}
	}
	return false;
}

/**
 * @brief Enciende el módulo celular.
 * @param
 * @return	True: mobile turned ON, false: error on turning ON
 */
bool mobilePowerOn(MOBILE_T *_mobile) {

	char i = 0;

	/*Chequeo la presencia del módulo. Lo enciendo*/
	while (isMobilePoweredON(_mobile) == false) {
		mobilePowerOnSequence(_mobile);
		i++;
		if (i > 5)
			return false;
	}
	return true;
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobilePowerOff(MOBILE_T *_mobile) {
	char i = 0;
	char localBuff[2];

	/*Envío un Ctrl+Z para asegurar que ek SIM no este esperando mensaje*/
	localBuff[0] = 0x1A;
	localBuff[1] = 0;
	sendCheckReply(_mobile, localBuff, OK_CHAR, 1000);

	if (sendCheckReply(_mobile, "AT+CPOWD=1", "NORMAL POWER DOWN", 1000)) {
		vTaskDelay(1000);
	} else {
		power_cycle();	//todo: poner un ctrl+c en lugar de power cycle
		GetReply(_mobile, 10000);
	}

	while (mobile_get_AT_replay(_mobile)) {
		vTaskDelay(1000);
		i++;
		if (i > 10)
			return false;
	}
	_mobile->status.accessTechnology = 0;
	_mobile->status.rssi = 0;
	_mobile->status.ber = 0;
	_mobile->status.isAttachedToGPRS = 0;
	_mobile->status.reg_on_netw = NOT_REGISTERED;
	return true;
}

/**
 * @brief	Set Phone Functionality
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_set_RF(MOBILE_T *_mobile, PHONE_FUNCTIONALITY_ENUM fun,
		uint8_t reset) {
	char *ptr;

	/*     */
	sprintf(_mobile->data.bufferRX, "AT+CFUN=%d,%d", fun, reset);

	sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000);

	ptr = strstr(_mobile->data.bufferRX, "OK:");
	if (ptr != NULL)
		return true;

	ptr = strstr(_mobile->data.bufferRX, "+CPIN:");
	if (ptr != NULL)
		return true;

	return false;
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_set_apn_manually(MOBILE_T *_mobile) {
	uint8_t cont = 0;

	mobile_get_info(_mobile, CFUN, 4000);
	if (strstr(_mobile->data.bufferRX, "+CFUN: 1"))
		/*Disable RF*/
		mobile_set_RF(_mobile, FUNCT_MINIMUM, 0);

	while ((!mobile_get_AT_replay(_mobile)) & (cont < 10)) {
		cont++;
	}

	/*Set the APN manually*/
	sprintf(_mobile->data.bufferRX, "AT+CGDCONT=1,\"IP\",\"%s\"",
			_mobile->network.gprs_apn);
	sendCheckReply(_mobile, _mobile->data.bufferRX, OK_CHAR, 1000);

	/*Enable RF*/
	mobile_set_RF(_mobile, FUNCT_FULL, 0);

	cont = 0;
	while ((!mobile_get_AT_replay(_mobile)) & (cont < 10)) {
		cont++;
	}

	for (cont = 0; cont < 20; cont++) {
		if (mobile_get_info(_mobile, CGATT, 4000))
			break;
		vTaskDelay(1000);
	}

	sendCheckReply(_mobile, "AT+CGNAPN", OK_CHAR, 5000);

	sprintf(_mobile->data.bufferTX, "AT+CNCFG=0,1,\"%s\",\"%s\",\"%s\",0",
			_mobile->network.gprs_apn, _mobile->network.gprs_usr,
			_mobile->network.gprs_pass);
//todo: ver que ewsta SUCEDEIDNO ACA
	sendCheckReply(_mobile, _mobile->data.bufferTX, OK_CHAR, 5000);

//	if (!tcpip_network_active_control(_mobile, ON))
//		return false;

	return true;
}

/**
 * @brief	Solicita el tiempo actual dado por la red a la cual se conectó el sistema
 * @param
 * @return
 */
time_t mobile_get_time(MOBILE_T *_mobile) {
	char *ptr;
	char local_buff[5];
	struct tm dateTime;
	time_t timeInSeconds;
	int16_t timeZoneSeconds;

// "\r\r\n+CCLK: \"22/05/22,13:03:40-12\"\r\n\r\nOK\r\n"
	mobile_get_info(_mobile, CCLK, 4000);

	ptr = strstr(_mobile->data.bufferRX, "+CCLK:");
	if (ptr != NULL) {
		ptr = strtok(_mobile->data.bufferRX, ":");	//+CCLK

		ptr = strtok(NULL, " \"/");	//22
		dateTime.tm_year = 2000 + atoi(ptr) - 1900;
		ptr = strtok(NULL, "/");	//05
		dateTime.tm_mon = atoi(ptr) - 1;
		ptr = strtok(NULL, ",");	//22
		dateTime.tm_mday = atoi(ptr);
		ptr = strtok(NULL, ":");	//13
		dateTime.tm_hour = atoi(ptr);
		ptr = strtok(NULL, ":");	//03
		dateTime.tm_min = atoi(ptr);
		ptr = strtok(NULL, "\"");	//40-12
		memset(local_buff, 0x00, 5);
		strncpy(local_buff, ptr, 2);
		dateTime.tm_sec = atoi(local_buff);
		memset(local_buff, 0x00, 5);
		strncpy(local_buff, ptr + 2, 3);
		timeZoneSeconds = (3600 / 4) * atoi(local_buff);//Calculo la zona horaria

		/* Convert tm structure to seconds */
		timeInSeconds = mktime(&dateTime);
		timeInSeconds -= timeZoneSeconds;
		return timeInSeconds;
	}
	return 0;
}

/**
 * @brief
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool mobile_gprs_attach(MOBILE_T *_mobile) {
	char *ptr;

	if (mobile_get_info(_mobile, CGATT, 4000)) {
		ptr = strstr(_mobile->data.bufferRX, "+CGATT:");
		if (ptr != NULL) {
			ptr = strtok(ptr, ":");
			if (ptr == NULL) {
				return false;
			}
			ptr = strtok(NULL, "\r\n");
			if (ptr != NULL) {
				_mobile->status.isAttachedToGPRS = atoi(ptr);
				return true;
			}
		}
	}
	return sendCheckReply(_mobile, "AT+CGATT=1", OK_CHAR, 75000);
}

/**
 * @brief	Wait a reply from the module.
 * @param	queue	: handler a la cola que recibe los datos desde el MOBILE
 * @param	Reply	: pointer to the Reply string to receive
 * @param	Reply_buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	the amount of bytes received
 */
uint16_t GetReply(MOBILE_T *_mobile, uint32_t timeout) {
	uint16_t j;
	uint16_t delay;
	char *ptrToReply;

	ptrToReply = _mobile->data.bufferRX;
	j = 0;
	delay = timeout;
	while (xQueueReceive(_mobile->data.fr_mobile, ptrToReply, delay) != pdFAIL) {

		if (j < 2)
			/*Los dos primeros bytes vamos a poner el timeout especificado*/
			delay = timeout;
		else
			/* los siguientes bytes, los leemos con timeout bajo*/
			delay = 60;

		j++;
		ptrToReply++;
		if (j >= (_mobile->data.bufferRX_size - 1))
			break;
	}
	return j;
}

/**
 * @brief	Transmit a command directly to UART. Then it will return the module response.
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to receive
 * @param	Reply_buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	the amount of bytes received
 */

uint16_t sendGetReply(MOBILE_T *_mobile, char *messageToSend, uint32_t timeout) {
	uint16_t i, j;
	uint16_t delay;
	UART_TX_TRANSFER_T dataToSend;
	char *ptrToReply;

	ptrToReply = _mobile->data.bufferRX;

	i = strlen((char*) messageToSend);
	if (i > 0) {
		dataToSend.ptr = messageToSend;
		dataToSend.Len = i;
		xQueueSend(_mobile->data.to_mobile, &dataToSend, 10);
	}
	j = 0;
	delay = timeout;
	while (xQueueReceive(_mobile->data.fr_mobile, ptrToReply, delay) != pdFAIL) {

		if (j < 2)
			/*Los dos primeros bytes vamos a poner el timeout especificado*/
			delay = timeout;
		else
			/* los siguientes bytes, los leemos con timeout bajo*/
			delay = 15;	//5;

		j++;
		ptrToReply++;
		if (j >= (_mobile->data.bufferRX_size - 1))
			break;
	}
	*ptrToReply = 0;	//Agrego el NULL al final de la cadena
	return j;
}

/**
 * @brief	Transmit a command directly to UART. Then it will Compare response.
 * @param	Buffer	: ptr to buffer for storing messages
 * @param	Message	: pointer to the message string to send
 * @param	Reply	: pointer to the Reply string to compare with
 * @param	Buff_size : max buffer size to avoid overflow. Should have
 * enough cap to allocate the response
 * @param	timeout : time to wait to get the reply
 * @return	True if match the Reply string, or False if the reply doesn't contain the string
 */
bool sendCheckReply(MOBILE_T *_mobile, char *Message,
		const char *stringToCompare, uint32_t timeout) {

	if (sendGetReply(_mobile, Message, timeout) == 0)
		return false;

	if (strstr(_mobile->data.bufferRX, stringToCompare)) {
		return true;
	} else
		return false;
}

