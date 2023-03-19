/*
 * SMS.c
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#include "SMS.h"
#include "mobile.h"

bool sms_init(MOBILE_T *_mobile) {

#if USE_SIM868 | USE_SIM7070G
	/*Select SMS Mode = TEXT*/
//	if (!sendCheckReply(data->to_mobile, data->fr_mobile, data->buffer,
//			"AT+CMGF=1", OK_CHAR, data->buffer_size, 4000))
	if (!sendCheckReply(_mobile, "AT+CMGF=1", OK_CHAR, 4000))
		return false;

	/*Select Character Set = GSM*/
//	if (!sendCheckReply(data->to_mobile, data->fr_mobile, data->buffer,
//			"AT+CSCS=\"GSM\"", OK_CHAR, data->buffer_size, 500))
	if (!sendCheckReply(_mobile, "AT+CSCS=\"GSM\"", OK_CHAR, 500))
		return false;

#endif

	return true;
}

//bool sms_delete(QueueHandle_t to_mobile, QueueHandle_t fr_mobile,
//		char *buffer, uint16_t Buff_size, uint8_t SMS_idx) {
bool sms_delete(MOBILE_T *_mobile, uint8_t SMS_idx) {

//Leo el resto del mensaje para vaciar la cola
//	while (xQueueReceive(fr_mobile, &buffer[cont], 100) != pdFAIL) {
//		cont++;
//		cont_total++;
//		if (cont >= Buff_size)
//			cont = 0;
//	}

	sprintf(_mobile->data.bufferRX, "AT+CMGD= %u", SMS_idx); //Delete SMS
//	if (sendCheckReply(data->to_mobile, data->fr_mobile, data->buffer,
//			data->buffer, "OK", data->buffer_size, 5000)) {
	if (sendCheckReply(_mobile, _mobile->data.bufferRX, "OK", 5000)) {
		return true;
	}
	return false;
}

/*
 * @brief	Transmit a SMS. buffer should contain at least 150 bytes
 */
//bool sms_send(QueueHandle_t to_mobile, QueueHandle_t fr_mobile, char *message,
//		char *reply, char *dest, uint16_t timeout) {
bool sms_send(MOBILE_T *_mobile, char *dest) {
	char buff[30];
	uint8_t len;

	len = sprintf(buff, "AT+CMGS= \"%s\"", dest);
//	sendGetReply(data->to_mobile, data->fr_mobile, buff, buff, 30, 2000);
	sendGetReply(_mobile, buff, 8000);
	if (strstr(_mobile->data.bufferRX, ">")) {
		len = strlen(_mobile->data.bufferTX);
		if (len > 140) {
			len = 140;
//			*(Message + 139) = 0; //put a null so the following strcat wont fail
		}
		_mobile->data.bufferTX[len] = '\r'; //Ctrl+Z
		_mobile->data.bufferTX[len+1] = 0x1a;
		_mobile->data.bufferTX[len+2] = 0x00;


		//Envio el mensaje
//		sendGetReply(data->to_mobile, data->fr_mobile, data->buffer,
//				data->buffer, 150, 500);
		sendGetReply(_mobile, _mobile->data.bufferTX, 500); //todo: deberia usar buff???

		//Espero confirmación de envío
//		GetReply(data->fr_mobile, data->buffer, 150, data->timeout);
		GetReply(_mobile, 10000);
		if (strstr(_mobile->data.bufferRX, "+CMGS"))
			return true;
	}

	/*No encontró el promoting mark “>”, por lo tanto envío Ctrl+Z para asegurar que el modulo no
	 * quede en ese el mensaje*/
	sprintf(_mobile->data.bufferTX, "---");
	_mobile->data.bufferTX[3] = 0x1b; //Ctrl+Z
	_mobile->data.bufferTX[4] = 0x00;
	sendCheckReply(_mobile, _mobile->data.bufferTX, "OK", 1000);
	return false;
}

/*
 * @brief	Receive SMS. Then it will process and answer or discard the SMS.
 */
bool readSMS(MOBILE_T *_mobile, PHONE_DATA_T *_gps) {
	char *ptr;

//Listar todos los mensajes
#if USE_SIM868
	sprintf(_mobile->buffer, "AT+CMGL=\"ALL\",1");
#elif USE_BG96
	sprintf(_mobile->buffer, "AT+CMGL=\"ALL\"");
#elif USE_SIM7070G
	sprintf(_mobile->data.bufferRX, "AT+CMGL=\"ALL\"");
#endif
//	sendGetReply(data->to_mobile, data->fr_mobile, data->buffer, data->buffer,
//			data->buffer_size, 20000); //Pido la lista de mensajes
	sendGetReply(_mobile, _mobile->data.bufferRX, 20000); //Pido la lista de mensajes

	ptr = strstr(_mobile->data.bufferRX, "+CMGL:");
	if (ptr) {
		//El SMS tiene respuesta válida
		ptr = strtok(ptr, " ");
		if (!ptr)
			false;

		// Grab SMS Index
		ptr = strtok(NULL, ",");
		if (!ptr)
			goto PROCESS;
		_mobile->status.SMS_idx = atoi(ptr);

		// skip SMS STATUS "REC UNREAD"
		ptr = strtok(NULL, ",");
		if (!ptr)
			goto PROCESS;

		// grab the Sender Nbr
		ptr = strtok(NULL, ",");
		if (!ptr)
			goto PROCESS;

		sscanf(ptr, "\"%[^\"]", _mobile->status.SMS_orig);

		// skip OA/DA
		ptr = strtok(NULL, ",");
		if (!ptr)
			goto PROCESS;

		// skip DATE & HOUR
		ptr = strtok(NULL, "\r\n");
		if (!ptr)
			goto PROCESS;

		// grab the MESSAGE
//		(0.000) 0Ah+CMGL: 0,"REC UNREAD","03517596858",,"22/05/20,16:29:31-12"
//		(0.000) 0AhReport
//		(0.000) 0Ah

		ptr = strtok(NULL, "\n");
		//sscanf(ptr, "\"%[^\"]", _mobile->data.bufferRX);
		strcpy(_mobile->data.bufferRX,ptr);
		PROCESS:
		/*Debo reenviar el mensaje?*/
		if (_mobile->status.sms_fwd) {
//			SendSMS(ptr, buffer, gsm->SMS_fwd_orig, 60 * 1000);

			sms_send(_mobile, _mobile->status.SMS_fwd_orig);
		}
		/*Procesar, o eliminar si no se reconoce*/


//	} else if (strstr(&buffer[Enviados], "ERROR")) {
//
//		//Vuelvo a inicializar el MOBILE
//		EvtToMOBILE.Src = MOBILE_Init;
//		xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
//		return true;

	} else if (strstr(_mobile->data.bufferRX, OK_CHAR)||strstr(_mobile->data.bufferRX, "ERROR")) {
		//La respuesta fue OK_CHAR por lo tanto no hay mas mensajes
		return false;

	} else {
		//No se reconoce la respuesta del SMS
		return true;
	}
	return true;
}
