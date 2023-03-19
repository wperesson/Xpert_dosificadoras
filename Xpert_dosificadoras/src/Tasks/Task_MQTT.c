/*
 * Task_MQTT.c
 *
 *  Created on: Febrero 2022
 *  Author: wperesson
 */

#include "Definiciones.h"

#define UART				LPC_UART3
#define LOC_BUFF_SZ			50
#define REF_RATE			60000
#define CTRL_REF_FACT		50
#define REF_COUNTER_LIM		10
#define RETRY_LIMIT_2		120

/*Timer indexes*/
typedef enum {
	tmr_ref_var_idx, tmr_ref_cise_idx
} tmr_ref_idx_type;

bool network_opened;
bool conected_to_broker;
//char Reg_status_flags = 0; //Flags que indican que estados u operaciones fueron relizadas

/* Envía un estado a la cola*/
void next_event(mqtt_event_type state) {

	//Enviar a la cola para su ejecución
	xQueueSend(queMQTT_ev, &state, 0);
}

/* Envía un estado al frente de la cola para que sea ejecutado inmediatamante*/
void next_inm_state(mqtt_event_type state) {

	//Enviar a la cola para su ejecución
	xQueueSendToFront(queMQTT_ev, &state, 0);
}

void vTmrRefDataCallback(xTimerHandle pxTimer) {

	configASSERT(pxTimer);
	int32_t tmr_idx;
//	mqtt_event_type loc_mqtt_st;

	tmr_idx = (int32_t) pvTimerGetTimerID(pxTimer);

	switch (tmr_idx) {
	case tmr_ref_var_idx:
		if (conected_to_broker) {
			next_event(mqtt_ev_publish_variables);
		}
		break;
	case tmr_ref_cise_idx:
		if (conected_to_broker) {
			next_event(mqtt_ev_publish_commands);
			next_event(mqtt_ev_publish_id);
			next_event(mqtt_ev_publish_sett);
			next_event(mqtt_ev_publish_alarms);
		}
		break;
	default:
		break;
	}
}

//bool ping_req(char *buff) {
//	uint16_t len;
//
//	/*SEND PING REQUEST*/
//	len = MQTT_PingReqPacket((uint8_t*) buff);
//	if (len > 0) {
//		/*El servidor devolvió una respuesta*/
//
//		/*PING RESP OK !!!!*/
//		return true;
//	}
//	return false;
//}

//bool mobile_init(MOBILE_DATA_T *data, GSM_DATA_T *gsm) {
bool mobile_init(MOBILE_T *_mobile) {
	char i = 0, retry = 0;
	char *ptr;
	/*
	 * Inicialización del MOBILE
	 */
	_mobile->status.reg_on_netw = NOT_REGISTERED;

	/*Chequeo la presencia del módulo. Lo enciendo*/
	while (isMobilePoweredON(_mobile) == false) {
		//mobilePowerOn(_mobile);
		i++;
		vTaskDelay(600);
		if (i > 3)
			return false;
	}

	if (!mobile_get_AT_replay(_mobile)) {
		_mobile->status.module_running = 0;
		return false;
	}

	/*Leer el IMEI \r\r\n865456053725444\r\n\r\nOK\r\n*/
	if (mobile_get_info(_mobile, IMEI, 4000)) {
		ptr = strtok(_mobile->data.bufferRX, "\n");
		if (ptr != NULL) {
			ptr = strtok(NULL, "\r");
		}
		strncpy(_mobile->status.imei, ptr, 17);
	}

	/*Comprobamos si esta sincronizado con el horario de la red*/
	mobile_get_info(_mobile, CLTS, 4000);
	ptr = strstr(_mobile->data.bufferRX, "+CLTS: 0");
	if (ptr != NULL) {
		sendCheckReply(_mobile, "AT+CLTS=1", OK_CHAR, 4000);
		/*Apago el módulo para reiniciar*/
		mobilePowerOff(_mobile);
		return false;
	}

	/*Set Preferred Mode Selection
	 * 2 Automatic
	 * 13 GSM only
	 * 38 LTE only
	 * 51 GSM and LTE only
	 */
	sendCheckReply(_mobile, "AT+CNMP=2", OK_CHAR, 2000);

	/*Set     */
	sendCheckReply(_mobile, "AT+CMCFG=1", OK_CHAR, 2000);

	/*Set Preferred mode: 1: CAT-M, 2: NB-Iot, 3: CAT-M and NB-IoT    */
#if USE_OLIVIA
//	sendCheckReply(queMsgToMOBILE, queMsgFromMOBILE, _mobile->data.buffer,
//			"AT+CMNB=2", OK_CHAR, data->buffer_size, 2000);
	sendCheckReply(_mobile, "AT+CMNB=2", OK_CHAR, 2000);
#else
	sendCheckReply(_mobile, "AT+CMNB=1",
			OK_CHAR, 2000);
#endif

	mobile_set_apn_manually(_mobile);

	/*Si todavia no se registro en la red...*/
	if ((_mobile->status.reg_on_netw != REGISTERED_HN)
			| (_mobile->status.reg_on_netw != REGISTERED_ROAM)) {

		/*Request Registered on network status*/
		while (retry < RETRY_LIMIT_2) {
			retry++;
			if (mobile_get_register_status(_mobile)) {
				break;
			} else
				vTaskDelay(1000);
		}

		if (retry >= RETRY_LIMIT_2) {
			//Si el móvil no se registra debo hacer un power cycle
			return false;

		}
	}
	return true;
}

void decodeCredentials(const MOBILE_DATA_T *data, char *ptr) {
/*
{
  "deviceId" : "62605a7360444f93b8484a52",
  "key" : "4eddcb4f-2b47-4c83-8f26-85a3ee9fc480",
  "secret" : "069e6015d821a565be3db4d52db083cd05c4ad764e006e2cdf185206cf512ac5",
  "error" : ""
}
 */
	ptr = strstr(data->bufferRX, "\"deviceId\"");
	if (ptr) {

		ptr = strtok(ptr, " :\"\n"); //deviceId
		ptr = strtok(NULL, " :\""); //62605a7360444f93b8484a52
		if (strlen(ptr) == (LEN_CLIENT_ID - 1)) {
			strncpy(clientID, ptr, LEN_CLIENT_ID);
			if (xSemaphoreTake(mtxI2C1,
					10) == pdTRUE) {
				eeprom_write(ptr, OFF_CLIENT_ID, LEN_CLIENT_ID);
				xSemaphoreGive(mtxI2C1);
			}
		}

		ptr = strtok(NULL, " ,\"\n"); //Key
		ptr = strtok(NULL, " :\""); //4eddcb4f-2b47-4c83-8f26-85a3ee9fc480
		if (strlen(ptr) == (LEN_ACCESS_KEY - 1)) {
			strncpy(access_key, ptr, LEN_ACCESS_KEY);
			if (xSemaphoreTake(mtxI2C1,
					10) == pdTRUE) {
				eeprom_write(ptr, OFF_ACCESS_KEY, LEN_ACCESS_KEY);
				xSemaphoreGive(mtxI2C1);
			}
		}

		ptr = strtok(NULL, " ,\"\n"); //secret
		ptr = strtok(NULL, " :\""); //069e6015d821a565be3db4d52db083cd05c4ad764e006e2cdf185206cf512ac5
		if (strlen(ptr) == (LEN_ACCESS_SECRET - 1)) {
			strncpy(access_secret, ptr, LEN_ACCESS_SECRET);
			if (xSemaphoreTake(mtxI2C1,
					10) == pdTRUE) {
				eeprom_write(ptr, OFF_ACCESS_SECRET, LEN_ACCESS_SECRET);
				xSemaphoreGive(mtxI2C1);
			}
		}
	}
}

Bool verifyNetworkRegistration(MOBILE_T *mobile2)
{	bool isInitIsOk;
	uint8_t retriesCounter = 0;
	isInitIsOk = false;
	while (isInitIsOk == false) {
		isInitIsOk = mobile_get_register_status(mobile2);
		if (isInitIsOk)
			break;
		retriesCounter++;
		if (retriesCounter > 2)
			break;
		vTaskDelay(2000);
	}

	if (isInitIsOk == false)
		isInitIsOk = mobile_init(mobile2);

	if (isInitIsOk == false)
		return false;

	mobile_get_operator(mobile2);
	mobile_get_rssi(mobile2);
	return true;
}

//Bool checkLosantCredentials(MOBILE_T *mobile2){
//	bool isInitIsOk;
//	if (SP.alreadyRegisteredOnLosant == false) {
//		isInitIsOk = askLosantForNewCredentials(mobile2, &device);
//	}
//
//	if (isInitIsOk == false)
//		return false;
//	else
//		return true;
//
//}
Bool networConnection(MOBILE_T *mobile2)
{
	if (mobile_get_AT_replay(mobile2) == false)
			return false;

		if (mobile_gprs_attach(mobile2) == false)
			return false;

		vTaskDelay(2000);

		if (tcpipActivateNetworkConnection(mobile2) == false)
			return false;
		return true;
}
Bool connectToLosant(MOBILE_T *mobile2){
	uint8_t retriesCounter = 0;

	mqtt_config_conn_to_broker(mobile2, 1883, 240);

		/* Abro la conexión con el broker */
		retriesCounter = 0;
		while (false == mqtt_connect_to_broker(mobile2)) {
			if (mqtt_check_broker_connection(mobile2))
				break;
			retriesCounter++;
			/* Cierro conexión con el broker */
			mqtt_disconnect_from_broker(mobile2);
			vTaskDelay(2000);
			if (retriesCounter > 2)
				return false;
		}
	return true;

}

//bool publishToLosant(){
//	bool isPublished;
//	/* Publico variables */
//	do {
//		if (xQueuePeek(quePOINT, &point, 0) == pdFAIL)
//			break;
//
//		isPublished = publicar_variables(mobile2, losant_state_head, &point,
//				payloadProfile);
//		sprintf(head, "losant/%s/state", _mobile->credential.masterClientID);
//		if (isPublished) {
//			/* Remuevo el punto de la cola */
//			xQueueReceive(quePOINT, &point, 0);
//		} else {
//			break;
//		}
//		vTaskDelay(500);
//	} while (isPublished);
//
//	/* Suscribo a una variable */
//
//	/* Cierro conexión con el broker */
//	mqtt_disconnect_from_broker(mobile2);
//	//todo: a veces falla la desconexion desde el broker
//
//	return isPublished;
//}

static portTASK_FUNCTION(vCOMM_MQTT_Task,pvParameters) {
	char buff_MQTT[MQTT_BUFF_SIZE],head[70],buff_Tx[300];
	static uint16_t len, mem_mqtt_ref_rate;
	static uint16_t msgID;
	static tcp_send_Type Recb;
	static Bool subscribed; //Flag de suscripción activa
	uint16_t attempt_cnt;
	uint8_t cont_coord_report_posit;
	mqtt_event_type mqtt_ev;
	mqtt_event_type mqtt_prev_ev; //Almacena el estado previo.
	char losant_state_head[50];
	TimerHandle_t timer_ref_var, timer_ref_cise;
	bool isInitIsOk;
	//MOBILE_DATA_T data;
	char *ptr;
	MOBILE_T mobile2;
	mem_mqtt_ref_rate = SP.mqtt_ref_rate;
	/*Timer para refrescar VARIABLES a enviar al broker*/
	timer_ref_var = xTimerCreate("Tmr_ref_var", mem_mqtt_ref_rate * 60 * 1000,
	true, (void*) tmr_ref_var_idx, vTmrRefDataCallback);
	if (timer_ref_var != 0) {

	}

	/*Timer para refrescar CONTROLES + ID + SETT + EVENTS a enviar al broker*/
	timer_ref_cise = xTimerCreate("Tmr_ref_cise",
	CTRL_REF_FACT * mem_mqtt_ref_rate * 60 * 1000,
	true, (void*) tmr_ref_cise_idx, vTmrRefDataCallback);
	if (timer_ref_cise != 0) {

	}

	/*Inicializo los valores de las variables*/
	subscribed = false;
	conected_to_broker = false;
	attempt_cnt = 0;
	cont_coord_report_posit = REF_COUNTER_LIM;
	msgID = 1;

//	/*Esperar a estar registrado en la red*/
//	while (mobile2.reg_on_netw != TRUE) {
//		vTaskDelay(5000);
//	}

	/*Arranco el timer para publicar variables*/
	if (xTimerIsTimerActive(timer_ref_var) == pdFALSE) {
		if (xTimerStart(timer_ref_var, 100) == pdPASS) {
			xTimerReset(timer_ref_var, 100);
		}
	}

	/* Leo la URL del endpoint "register" */
//	eeprom_read(endpoint_url, OFF_ENDPOINT_URL,
//	LEN_ENDPOINT_URL);
//	sprintf(clientID, "6282508b3073284d69ac02e4");
	/*Primer estado al que voy a entrar*/
	xQueueReset(queMQTT_ev); //Borro cualquier item dentro de la cola.
	mqtt_prev_ev = mqtt_ev_connect_to_broker;
	next_event(mqtt_ev_connect_to_broker);
	//next_event(mqtt_ev_req_keys_from_broker);

	mobile2.data.bufferRX = buff_MQTT;
	mobile2.data.bufferRX_size = MQTT_BUFF_SIZE;
	mobile2.data.bufferTX = buff_Tx;
	mobile2.data.bufferTX_size = 300;
	mobile2.data.fr_mobile = queMsgFromMOBILE;
	mobile2.data.to_mobile = queMsgToMOBILE;
	strcpy(mobile2.credential.broker_url,"broker.losant.com");
	strcpy(mobile2.network.gprs_apn, GPRS_APN);
	strcpy(mobile2.network.gprs_usr, GPRS_USER);
	strcpy(mobile2.network.gprs_pass, GPRS_PASS);
	//XP20-06-1006(ELEC EDGE)
		sprintf(mobile2.credential.masterClientID, "62c6f62fe5e14ef7561350b6");
		sprintf(mobile2.credential.access_key,
		"23051234-1bca-4867-9c87-5ae45128ef59");
		sprintf(mobile2.credential.access_secret,
		"fb4464ef2950c78b0e62ee275b3fc168522a1bb08c8c1f11b9580309f8feade2");
	//XP20-06-1004
//	sprintf(mobile2.credential.masterClientID, "62a1eb86d33740be22629d0b");
//	sprintf(mobile2.credential.access_key,
//	"a8b659e4-e217-4ef2-9900-d5c6b0402dfe");
//	sprintf(mobile2.credential.access_secret,
//	"4e8862a0684cd44a73e95ba1da4a53d0810f05d4c95f3b0860b9120f2a5f4c6b");
	//XP20-06-1005
//	sprintf(mobile2.credential.masterClientID, "62a1eba9788eeb581b8f9f14");
//	sprintf(mobile2.credential.access_key,
//	"afd2bd14-5687-4720-a9aa-1f60f34d7451");
//	sprintf(mobile2.credential.access_secret,
//	"0348b7a996fb07481448614b5f9d373fb640a876d2a31bf01c25a2d717bf411c");

	//XP20-06-1006
//	sprintf(mobile2.credential.masterClientID, "629f724398068ec5697ac6e9");
//	sprintf(mobile2.credential.access_key,
//	"8a2770ca-29b8-4461-a176-f89e23961587");
//	sprintf(mobile2.credential.access_secret,
//	"4ff5bfc42421bac0a65d2bdbc3f976124cb24a7c93a4fbe206f2b03524567e6d");
//xp20-06-1007
//	sprintf(mobile2.credential.masterClientID, "629f72543073284d69ac4a41");
//	sprintf(mobile2.credential.access_key,
//	"260caa77-670b-4b9a-9789-378efc42fa3c");
//	sprintf(mobile2.credential.access_secret,
//	"676f21a0f21c45b420079c843a82d04d2381fd53ab03a9396c6cdab7917b3144");
		//XP20-06-1015(ELEC ANGOLA)
//			sprintf(mobile2.credential.masterClientID, "6349ccd17d45bd309dc938cf");
//			sprintf(mobile2.credential.access_key,
//			"0fe08cd3-831c-44fa-b68d-fc0f9a8c87f1");
//			sprintf(mobile2.credential.access_secret,
//			"b5e30b87d561d023602daf64d2d7b985d956914a2bc9b4304a5509069c0a1d4b");
		//XP20-06-1019
//			sprintf(mobile2.credential.masterClientID, "63c44b642d06e7dae51da97d");
//			sprintf(mobile2.credential.access_key,
//			"fedf6bc7-6eaa-4ece-8d5b-bc0301508465");
//			sprintf(mobile2.credential.access_secret,
//			"f99b41e283c1db5895e62ed34a6d8e3e28504a09c2ea97fb6786614f8f31a7fb");
		//XP20-06-1020
//			sprintf(mobile2.credential.masterClientID, "63c4498af4f9ead7f48d759e");
//			sprintf(mobile2.credential.access_key,
//			"4ff22798-b564-4d8a-9719-32f1f8e07728");
//			sprintf(mobile2.credential.access_secret,
//			"c0af0e61897cae629e22c3eb7641d877c3becfbddc644b3f7ac44ccd94b2b2d3");
		//XP20-06-1018
//			sprintf(mobile2.credential.masterClientID, "63c44b262d06e7dae51da97b");
//			sprintf(mobile2.credential.access_key,
//			"8cf443a1-1a4d-4111-b542-b237d02e44eb");
//			sprintf(mobile2.credential.access_secret,
//			"b35fee7786c65449789f739b3b26446e5ddcc343614b81cc644a836bf6d1ab8e");
		//XP20-06-1021
//			sprintf(mobile2.credential.masterClientID, "63dd1307cf8730bff67fed06");
//			sprintf(mobile2.credential.access_key,
//			"92fa9843-01d7-49b1-bf51-377f7be86de0");
//			sprintf(mobile2.credential.access_secret,
//			"a5daff45841024cc77f4f6b6fa79091d86a827685fa5c3735e1abd19921e06c9");
		//XP20-06-1022
//			sprintf(mobile2.credential.masterClientID, "63dd3732bcfc09f21ed650d8");
//			sprintf(mobile2.credential.access_key,
//			"f4a0dfd1-9b50-46c2-bd93-77717a529476");
//			sprintf(mobile2.credential.access_secret,
//			"7692a4e74e2f253d5c450bb16be718ef5228c31c686ced673bf758e586fc00d8");
		//XP20-06-1023
			sprintf(mobile2.credential.masterClientID, "63dd375fcf8730bff67fed7d");
			sprintf(mobile2.credential.access_key,
			"292d95a8-ed9f-4522-a099-bff5b5f20ac4");
			sprintf(mobile2.credential.access_secret,
			"6625ab931fabe968f347f41d24cd50009d241b79fc449fca0aee12c8684391e2");

	while (1) {

		if (xQueueReceive(queMQTT_ev, &mqtt_ev, 30000) == pdPASS) {

			/*Chequeo si cambió el refresh rate*/
			if (mem_mqtt_ref_rate != SP.mqtt_ref_rate) {
				xTimerChangePeriod(timer_ref_var, SP.mqtt_ref_rate * 60 * 1000,
						0);
				xTimerChangePeriod(timer_ref_cise,
						CTRL_REF_FACT * SP.mqtt_ref_rate * 60*1000, 0);

				//Actualizo el nuevo valor
				mem_mqtt_ref_rate = SP.mqtt_ref_rate;
			}

			/*Si cae varias veces en el mismo estado, se interpondrá un delay de 3 segundos*/
			if (mqtt_ev != mqtt_ev_publish_variables
					&& mqtt_prev_ev == mqtt_ev) {
				vTaskDelay(3000); //Espero 3 segundos para repetir el comando
			}
			mqtt_prev_ev = mqtt_ev;

			if (!debug_gsm) {
				if (xSemaphoreTake(mtxMOBILE, portMAX_DELAY) == pdTRUE) {
					/* Reviso si el modulo esta registrado*/

					if (isMobilePoweredON(&mobile2)&&mobile_get_register_status(&mobile2))
						isInitIsOk = true;
					else
						isInitIsOk = mobile_init(&mobile2);
					switch (mqtt_ev) {

					/*=======================================================================
					 * 		---	GET CREDENTIALS FROM BROKER ---
					 */
					case mqtt_ev_req_keys_from_broker:

						SP.alreadyRegisteredOnLosant = false;

						/*Si estoy registrado en Losant, leo las credenciales*/
						if (SP.alreadyRegisteredOnLosant) {
//								/* Leo las credenciales almacenadas p LOSANT*/
//								leer_eeprom(clientID, OFF_CLIENT_ID,
//								LEN_CLIENT_ID); //Leo el client ID para Losant
//								leer_eeprom(access_key, OFF_ACCESS_KEY,
//								LEN_ACCESS_KEY); //Leo el access key para Losant
//								leer_eeprom(access_secret,
//								OFF_ACCESS_SECRET,
//								LEN_ACCESS_SECRET); //Leo el access secret para Losant

//							sprintf(clientID, "5fb66e9f5ddd6b0006bbb120");
//							sprintf(access_key,
//									"3f2813bc-dcfe-4fdf-826a-94dd8bc003c0");
//							sprintf(access_secret,
//									"b7404adcb5a786758785c471013b0d3efaf75129edb7dfc8154822c30211550a");

//							sprintf(clientID, "6203f73f34a9cc895bfaf4f0");
//							sprintf(access_key,
//									"96855b03-2f27-4bfb-b04f-596661119e13");
//							sprintf(access_secret,
//									"0332ec6e66c4b37deb5e396bd466d5a2a542e59589d4cc10d3a851c07c48876b");

							/*Ahora debo conectarme al broker*/
							next_event(mqtt_ev_connect_to_broker);
						} else {

							/* Tiene SN asignado??*/
							if (!strcmp(id_SN, NO_SN)) {
								/*No tiene SN asignado. Volver a intentar dentro de 30 seg.*/
								vTaskDelay(30 * 1000);
								xQueueReset(queMQTT_ev); //Borro cualquier item dentro de la cola.
								next_inm_state(mqtt_ev_req_keys_from_broker);
							} else {

								/*SN Asignado.*/
								memset(buff_MQTT, 0, MQTT_BUFF_SIZE);

//								if (https_get_credentials(&data, gprs_apn,
//										gprs_usr, gprs_pass, id_SN,
//										CONTROLLER_TYPE, endpoint_url))
								if(1){
									sprintf(mobile2.credential.masterClientID, "63dd375fcf8730bff67fed7d");
									sprintf(mobile2.credential.access_key,
									"292d95a8-ed9f-4522-a099-bff5b5f20ac4");
									sprintf(mobile2.credential.access_secret,
									"6625ab931fabe968f347f41d24cd50009d241b79fc449fca0aee12c8684391e2");

									//decodeCredentials(&data, ptr);
									/*Setear la bandera de que tengo las credenciales*/
									SP.alreadyRegisteredOnLosant = TRUE;

									/*Guardo SP en EEPROM*/
//									guardar_eeprom((char*) &SP, OFF_SP,
//											sizeof(SP));
									next_event(mqtt_ev_connect_to_broker);
								} else {
									//No consiguió las credenciales. Volver a intentar
									next_event(mqtt_ev_req_keys_from_broker);
								}
							}
						}

						if (strlen(clientID)) {
							/*Imprimo el encabezado que luego usaremos para publicar*/
							sprintf(losant_state_head, "/losant/%s/state",
									clientID);
//						ptr = losant_state_head;
//						for (; *ptr; ++ptr)
//							*ptr = tolower(*ptr);
						}
						break;
						/*=======================================================================
						 * 		---	CONNECT TO BROKER ---
						 */
					case mqtt_ev_connect_to_broker:

						/*Reseteo la cola para borrar posibles acciones*/
						xQueueReset(queMQTT_ev);
						if(verifyNetworkRegistration(&mobile2))
						{
							if(networConnection(&mobile2))
							{
								if(!connectToLosant(&mobile2))
								{
									next_event(mqtt_ev_disconnect_from_broker);

								}
								else
								{
									conected_to_broker=true;
									next_event(mqtt_ev_subscribe);
								}
							}
							else
							{
								next_event(mqtt_ev_connect_to_broker);
							}
						}
						else
						{
							next_event(mqtt_ev_connect_to_broker);
						}

						/*Cierro la conexion*/
//						LTE_TCP_Disconnect(buff_MQTT, MQTT_BUFF_SIZE);
						/*Intento iniciar nuevamente*/
//						if (LTE_TCP_Connect(buff_MQTT, MQTT_BUFF_SIZE, gprs_apn,
//								gprs_usr, gprs_pass)) {
//
//						}
//						if (LTE_Connect_to_broker(buff_MQTT, MQTT_BUFF_SIZE,
//								clientID, access_key, access_secret)) {
//							conected_to_broker = true;
//							next_event(mqtt_ev_subscribe);
//							subscribed = false;
//						} else {
//							vTaskDelay(500);
//							next_event(mqtt_ev_connect_to_broker);
//							conected_to_broker = false;
//						}
						break;

						/*=======================================================================
						 * 		---	DISCONNECT FROM BROKER ---
						 */
					case mqtt_ev_disconnect_from_broker:
						mqtt_disconnect_from_broker(&mobile2);
						next_event(mqtt_ev_connect_to_broker);
						break;

						/*=======================================================================
						 * 		---	PUBLISH VARIABLES, ID, EVENTS, CTRLS, SETT ---
						 */
					case mqtt_ev_publish_variables:
					case mqtt_ev_publish_id:
					case mqtt_ev_publish_events:
					case mqtt_ev_publish_commands:
					case mqtt_ev_publish_sett:
					case mqtt_ev_publish_alarms:

						if ((attempt_cnt < 3) && mqtt_check_broker_connection(&mobile2)) {

							memset(buff_MQTT, 0, MQTT_BUFF_SIZE);

							/*Publicar VARIBLES*/
							if (mqtt_ev == mqtt_ev_publish_variables) {

								/*Conformo el payload*/

								/*Incremento el contador de refresco de COORD*/
								if (gps.gps_fix)
									cont_coord_report_posit++;
								else
									cont_coord_report_posit = 0;

								/*Analizo si es hora de enviar las coordenadas*/
								if (cont_coord_report_posit > REF_COUNTER_LIM) {

									cont_coord_report_posit = 0;

									/*Envio los datos incluyendo COORDENADAS*/
									len = sprintf(mobile2.data.bufferTX, "{"
											"\"data\":{"
											"\"tleve\":%.2f,"
											"\"tlevp\":%d,"
											"\"cpan\":%.2f,"
											"\"vbat\":%.2f,"
											"\"di1\":%d,"
											"\"di2\":%d,"
											"\"di3\":%d,"
											"\"di4\":%d,"
											"\"od\":%d,"
											"\"ai1\":%6.2f,"
											"\"ai2\":%6.2f,"
											"\"ai3\":%6.2f,"
											"\"ai4\":%6.2f,"
											"\"ai5\":%6.2f,"
											"\"ai6\":%6.2f,"
											"\"posit\":\"%6.4f,%6.4f\","
											"\"lpres\":%d,"
											"\"rssi\":%d"
											"}}", Var.tank_vol,
											Var.tank_vol_perc, Var.CPanel,
											Var.VBatt,
											(Var.DI & DIn_1_MASK) ? 1 : 0,
											(Var.DI & DIn_2_MASK) ? 1 : 0,
											(Var.DI & DIn_3_MASK) ? 1 : 0,
											(Var.DI & DIn_4_MASK) ? 1 : 0,
											(Var.DI & DIn_OD_MASK) ? 1 : 0,
											Var.an[0].Signal, Var.an[1].Signal,
											Var.an[2].Signal, Var.an[3].Signal,
											Var.an[4].Signal, Var.an[5].Signal,
											gps.Lat, gps.Lon, Var.LinePress,
											gsm.rssi);
								} else {

									/* Envio los datos sin las coordenadas.*/
									len = sprintf(mobile2.data.bufferTX, "{"
											"\"data\":{"
											"\"tleve\":%.2f,"
											"\"tlevp\":%d,"
											"\"cpan\":%.2f,"
											"\"vbat\":%.2f,"
											"\"di1\":%d,"
											"\"di2\":%d,"
											"\"di3\":%d,"
											"\"di4\":%d,"
											"\"od\":%d,"
											"\"ai1\":%6.2f,"
											"\"ai2\":%6.2f,"
											"\"ai3\":%6.2f,"
											"\"ai4\":%6.2f,"
											"\"ai5\":%6.2f,"
											"\"ai6\":%6.2f,"
											"\"lpres\":%d,"
											"\"rssi\":%d"
											"}}", Var.tank_vol,
											Var.tank_vol_perc, Var.CPanel,
											Var.VBatt,
											(Var.DI & DIn_1_MASK) ? 1 : 0,
											(Var.DI & DIn_2_MASK) ? 1 : 0,
											(Var.DI & DIn_3_MASK) ? 1 : 0,
											(Var.DI & DIn_4_MASK) ? 1 : 0,
											(Var.DI & DIn_OD_MASK) ? 1 : 0,
											Var.an[0].Signal, Var.an[1].Signal,
											Var.an[2].Signal, Var.an[3].Signal,
											Var.an[4].Signal, Var.an[5].Signal,
											Var.LinePress, gsm.rssi);
								}
							}
							/*Publicar ALARMAS*/
							if (mqtt_ev == mqtt_ev_publish_alarms) {
								/*Conformo el payload*/
								len = sprintf(mobile2.data.bufferTX, "{"
										"\"data\":{"
										"\"al_do\":%d,"
										"\"al_gf\":%d,"
										"\"al_lb\":%d,"
										"\"al_ll\":%d,"
										"\"al_vll\":%d,"
										"\"al_ms\":%d"
										"}}", GetAlarm(Al_DoorOpen) ? 1 : 0,
										GetAlarm(Al_GenFail) ? 1 : 0,
										GetAlarm(Al_LowBatt) ? 1 : 0,
										GetAlarm(Al_advertencia_bajo_nivel_T1) ?
												1 : 0,
										GetAlarm(Al_alarma_bajo_nivel_T1) ?
												1 : 0,
										GetAlarm(Al_alarma_motor_atascado_B1) ?
												1 : 0);
							}
							/*Publicar ID*/
							else if (mqtt_ev == mqtt_ev_publish_id) {

								len = sprintf(mobile2.data.bufferTX, "{"
										"\"data\":{"
										"\"stg1\":\"%s\","
										"\"stg2\":\"%s\","
										"\"stg3\":\"%s\""
										"}}", &id_Tag[0], &id_Tag[1],
										&id_Tag[2]);
							}
							/*Publicar EVENTOS*/
							else if (mqtt_ev == mqtt_ev_publish_events) {

								len = sprintf(mobile2.data.bufferTX, "{"
										"\"data\":{"
										"\"evod\":\"1\","
										"\"evms\":\"1\""
										"}}");
							}
							/*Publicar COMANDOS*/
							else if (mqtt_ev == mqtt_ev_publish_commands) {
//								PRINT("PUBLISH CTRLS");
								len = sprintf(mobile2.data.bufferTX, "{"
										"\"data\":{"
										"\"pum\":%d,"
										"\"flow\":%.2f,"
										"\"vunit\":\"%s\","
										"\"funit\":\"%s\","
										"\"do1\":%d,"
										"\"do2\":%d,"
										"\"do3\":%d,"
										"\"do4\":%d,"
										"\"rl1\":%d,"
										"\"rl2\":%d"
										"}}", PumpOn ? 1 : 0, sp_pump.PumpRate,
										Var.vol_unit, Var.flow_unit,
										(Var.DO & DOut_1_MASK) ? 1 : 0,
										(Var.DO & DOut_2_MASK) ? 1 : 0,
										(Var.DO & DOut_3_MASK) ? 1 : 0,
										(Var.DO & DOut_4_MASK) ? 1 : 0,
										(Var.DO & DOut_REL1) ? 1 : 0,
										(Var.DO & DOut_REL2) ? 1 : 0);

							}
							/*Publicar SETTINGS*/
							else if (mqtt_ev == mqtt_ev_publish_sett) {

								len =
										sprintf(mobile2.data.bufferTX, "{"
												"\"data\":{"
												"\"vunit\":\"%s\","
												"\"funit\":\"%s\","
												"\"fl_cf\":%.2f,"
												"\"ref\":%d,"
												"\"tk_sen_hgt\":%.2f,"
												"\"tk_den\":%.2f,"
												"\"tk_shp\":%d,"
												"\"tk_d1\":%.2f,"
												"\"tk_d2\":%.2f,"
												"\"tk_d3\":%.2f,"
												"\"wrlev\":%d,"
												"\"allev\":%d,"
												"\"bt_wl\":%.2f"
												"}}", Var.vol_unit,
												Var.flow_unit, sp_pump.CF,
												SP.mqtt_ref_rate,
												sp_pump.tnk_sensor_hgt,
												sp_pump.tnk_dens,
												sp_pump.tnk_shp,
												sp_pump.tnk_d1_VT_HT_diam_RT_width,
												sp_pump.tnk_d2_VT_height_HT_lenght_RT_height,
												sp_pump.tnk_d3_RT_length,
												sp_pump.level_warning_limit_T1,
												sp_pump.level_alarm_limit_T1,
												gen.low_batt_limit);
							}

//							Recb = LTE_mqtt_publish(buff_MQTT, MQTT_BUFF_SIZE,
//									len, losant_state_head, &msgID);
							sprintf((char*) head, "losant/%s/state",
															mobile2.credential.masterClientID);
							mqtt_publish(&mobile2, len, head);

						} else {
							/*Superamos la cantidad de intentos para realizar la operacion c exito*/
							next_event(mqtt_ev_disconnect_from_broker);

							attempt_cnt = 0;
						}
						break;

						/*=======================================================================
						 * 		---	SUBSCRIBE ---
						 */
					case mqtt_ev_subscribe:
						memset(buff_MQTT, 0, MQTT_BUFF_SIZE);

						len = sprintf((char*) buff_MQTT, "losant/%s/command",
								mobile2.credential.masterClientID);
						mqtt_unsubscribe(&mobile2, buff_MQTT);
						len = sprintf((char*) buff_MQTT, "losant/%s/command",
														mobile2.credential.masterClientID);
						subscribed = mqtt_subscribe(&mobile2, buff_MQTT, 0);

						memset(buff_MQTT, 0, MQTT_BUFF_SIZE);
						len = sprintf((char*) buff_MQTT, "losant/%s/state",
								mobile2.credential.masterClientID);
						mqtt_unsubscribe(&mobile2, buff_MQTT);
						len = sprintf((char*) buff_MQTT, "losant/%s/state",
								mobile2.credential.masterClientID);
						subscribed = mqtt_subscribe(&mobile2, buff_MQTT, 0);
//						subscribed = LTE_mqtt_subscribe(buff_MQTT,
//						MQTT_BUFF_SIZE, len);

						if (subscribed) {
							/*Averiguo si ya esta corriendo el timer de CISE. En caso de que no este corriendo
							 * lo pongo a correr y agrego una serie de estados para que actualice la info de
							 * esos parámetros */

							/*Arranco el timer para publicar CISE*/
							if (xTimerIsTimerActive(timer_ref_cise) == pdFALSE) {
								if (xTimerStart(timer_ref_cise,
										100) == pdPASS) {
									xTimerReset(timer_ref_cise, 100);
									/*Incluyo todos los estados para q actualice todos los valores*/
									next_event(mqtt_ev_publish_id);
									next_event(mqtt_ev_publish_commands);
									next_event(mqtt_ev_publish_sett);
									next_event(mqtt_ev_publish_alarms);
								}
							}
						}
						break;
					default:
						break;
					}
				}
				xSemaphoreGive(mtxMOBILE);
			}
		} else {
			/* Paso 1 minuto desde el ultimo envio de datos.
			 * Pido un ping para mantener la conexion activa*/
			if (!mqtt_check_broker_connection(&mobile2)) {
				next_event(mqtt_ev_disconnect_from_broker);
			}
		}
	}
}

void Create_COMM_MQTT_task(void) {
	xTaskCreate(vCOMM_MQTT_Task, (char*) "MQTT TSK", 860,
	NULL, Prio_COMM_MQTT_Task, &tsk_mqtt_handler);
}
