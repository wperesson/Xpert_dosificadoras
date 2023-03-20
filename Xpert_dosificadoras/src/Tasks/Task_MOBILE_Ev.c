/*
 * Tsk_SMS.c
 *
 *  Created on: 08/11/2019
 *  Author: Walter Peresson
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Definiciones.h"

#define UART				LPC_UART3
#define UART_IRQ_SELEC		UART3_IRQn
#define UART_HANDLER_NAME 	UART3_IRQHandler
#define BAUDRATE			115200
#define REF_RATE			30000
#define WLC_MSG_PDAY		4
#define SAVE_COORD_TO_EEPROM_CNT	100
uint8_t simIns=0;

typedef enum {
	tmr_ref_gps_idx, tmr_ref_sms_idx
} tmr_ref_idx_type;

void vTmrRefGPSCallback(xTimerHandle pxTimer) {
	EvtToMOB_Type EvtToMOBILE;

	configASSERT(pxTimer);
//	EvtToMOBILE.Src = ref_gps;
//	xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
//	configASSERT(pxTimer);
	int32_t tmr_idx;
//	mqtt_event_type loc_mqtt_st;

	tmr_idx = (int32_t) pvTimerGetTimerID(pxTimer);

	switch (tmr_idx) {
	case tmr_ref_gps_idx:
			EvtToMOBILE.Src = ref_gps;
			xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
		break;
	case tmr_ref_sms_idx:

		if(simIns==1)
			{
			EvtToMOBILE.Src = ref_SMS;
			xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
			}
			break;
	default:
		break;
	}
}

//bool Send_message(char *buffer, GSM_Type *ptr_gsm) {
////Finally Send replay and DELETE the message
//	if (SendSMS(buffer, buffer, ptr_gsm->SMS_orig, 60 * 1000)) {
//		return true;
//	}
//	return false; //Si llegamos aca es porq no se pudo enviar el mensaje
//}

static portTASK_FUNCTION(vCOMM_MOB_Ev_Task,pvParameters) {
	uint8_t i, j;
	uint16_t cont_gps_fix=0; //Cont de retardo para almacenar coordenadas en EEPROM
	EvtToMOB_Type EvtToMOB;
	char buff_SMS[MOBILE_BUFF_SIZE],buffSmsTx[140];
	UART_TX_TRANSFER_T uart_data;
	char *ptr;
//	MOBILE_DATA_T data;
	MOBILE_T mobile;
	mqtt_event_type mqttSt;
	TimerHandle_t timerRefGps,timerRefSms;


	memset(gsm.imei, 0, sizeof(gsm.imei));
	memset(gsm.oper, 0, sizeof(gsm.oper));
	memset(gsm.SMS_scenter, 0, sizeof(gsm.SMS_scenter));

	EvtToMOB.Src = MOBILE_Init;
	xQueueSend(queEvtToMOBILE, &EvtToMOB, 0);

	cont_gps_fix = 0;
	sendingSMS = false;
	uint32_t current_day;
	uint8_t cont_welc_sent = WLC_MSG_PDAY;
	mobile.data.bufferRX = buff_SMS;
	mobile.data.bufferRX_size = MOBILE_BUFF_SIZE;
	mobile.data.bufferTX = buffSmsTx;
	mobile.data.bufferTX_size = 140;
	mobile.data.fr_mobile = queMsgFromMOBILE;
	mobile.data.to_mobile = queMsgToMOBILE;
	strcpy(mobile.status.SMS_fwd_orig,"3517596858");
	mobile.status.sms_fwd=0;
//	xLastWakeTime = xTaskGetTickCount();

	/*Timer para refrescar la lectura de GPS y chequear SMS pendientes
	 * Comienza con un valor de 3seg, para tratar de ver rápidamente cuando se
	 * registra el cel en el sistema. Luego sube a 30seg.
	 */
	timerRefGps = xTimerCreate("Tmr_ref_gps", 1*3600*1000, true, (void*)tmr_ref_gps_idx,
			vTmrRefGPSCallback);//GPS cada HS
	if (timerRefGps != 0) {

	}
	timerRefSms = xTimerCreate("Tmr_ref_sms", REF_RATE / 10, true, (void*)tmr_ref_sms_idx,
			vTmrRefGPSCallback);
	if (timerRefSms != 0) {

	}

	current_day = Local.time[RTC_TIMETYPE_DAYOFYEAR];
	/*Arranco el timer para publicar variables*/
	if (xTimerIsTimerActive(timerRefGps) == pdFALSE) {
		if (xTimerStart(timerRefGps, 100) == pdPASS) {
			xTimerReset(timerRefGps, 100);
		}
	}
	if (xTimerIsTimerActive(timerRefSms) == pdFALSE) {
		if (xTimerStart(timerRefSms, 100) == pdPASS) {
			xTimerReset(timerRefSms, 100);
		}
	}

#if USAR_SNIFFER
	vTaskDelay(2000); //Para dar tiempo a que inicie la RS485 task y el sniffer
#endif
	while (1) {

		//Reviso si alguna tarea me envió un pedido
		if (xQueueReceive(queEvtToMOBILE, &EvtToMOB, 1000) == pdPASS) {
			/*
			 * Semaforo para sincronizar la posesion del UART3
			 */
			if (xSemaphoreTake(mtxMOBILE, portMAX_DELAY) == pdTRUE) {
				switch (EvtToMOB.Src) {

				case MOBILE_Init: //Init the MOBILE settings

					i = 0;
					gsm.reg_on_netw = NOT_REGISTERED;

					/*Ahora si habilito la interrupcion*/
					Chip_UART_IntEnable(UART, UART_IER_RBRINT);

					/*Chequeo la presencia del módulo. Lo enciendo*/
					while (isMobilePoweredON(&mobile) == false) {
						mobilePowerOn(&mobile);
						i++;
						if (i > 5)
							break;
					}

//					if (!mobile_get_AT_replay(&mobile)) {
//						gsm.module_running = 0;

					if (isMobilePoweredON(&mobile) == false){

						if (!debug_gsm) {
							EvtToMOB.Src = MOBILE_Init;
							xQueueSend(queEvtToMOBILE, &EvtToMOB, 0);
						}

						break;
					}

#if USE_GPS
					/*Luego de 1seg, lo enciendo*/
					//MOBILE_GPS_EN_ON;
					vTaskDelay(1000);
#endif

					sms_init(&mobile);

					/*Leer el IMEI \r\r\n865456053725444\r\n\r\nOK\r\n*/
					if (mobile_get_info(&mobile, IMEI, 4000)) {
						ptr = strtok(buff_SMS, "\n");
						if (ptr != NULL) {
							ptr = strtok(NULL, "\r");
						}
						strncpy(gsm.imei, ptr, 17);
					}
					sendGetReply(&mobile, "AT+CPIN?", 1000);
					ptr = strstr(mobile.data.bufferRX, "SIM not inserted");
					if (ptr != NULL) {
						simIns=0;
					}
					else{
						ptr = strstr(mobile.data.bufferRX, "ERROR");
						if (ptr != NULL) {
							simIns=0;
						}
						else{
							simIns=1;
						}
					}

					/*Comprobamos si esta sincronizado con el horario de la red*/
					mobile_get_info(&mobile, CLTS, 4000);
					ptr = strstr(mobile.data.bufferRX, "+CLTS: 0");
					if (ptr != NULL) {
						sendCheckReply(&mobile, "AT+CLTS=1", OK_CHAR, 4000);
						/*Apago el módulo para reiniciar*/
						mobilePowerOff(&mobile);

						/*Limpio la cola de eventos*/
						xQueueReset(queEvtToMOBILE);

						/*Enciendo e inicio el módulo*/
						EvtToMOB.Src = MOBILE_Init;
						xQueueSend(queEvtToMOBILE, &EvtToMOB, 0);
						break;
					}

#if USAR_SNIFFER
					/*Verbose the message*/
					sendCheckReply(&mobile,
							"AT+CMEE=2", OK_CHAR, 4000);
#endif

#if (USE_SIM868 && USE_BLUETOOTH)
					/*Crear tarea de soporte Bluetooth*/
					if (!vCOMM_BLUE_Handle)
						Create_COMM_BLUE_task();
#endif
					/*Me aseguro de tener un SN especificado*/
					if (strncmp(id_SN, "No Serial Nbr", 14)) { //todo: unificar el string que comparo con L306 TaskMQTT
						/*Crear tarea de soporte MQTT*/
						if ((!tsk_mqtt_handler)&&(simIns==1)){
#if USE_SIM868
					Create_COMM_MQTT_SIM868_task();
#elif USE_BG96
					Create_COMM_MQTT_BG96_task();
#elif USE_SIM7070G
							Create_COMM_MQTT_task();
					}
#endif
							else if(simIns==1){
								mqttSt = mqtt_ev_disconnect_from_broker;
								xQueueSend(queMQTT_ev, &mqttSt, 0);
							}

					}

					break;
				case sendToMOBILE:
					//Send a raw message to MOBILE
					uart_data.ptr = EvtToMOB.Message;
					uart_data.Len = EvtToMOB.Val;
					xQueueSend(queMsgToMOBILE, &uart_data, 1);
					break;
				case sendSMS:
					//Send SMS. EvtToMOBILE.Val contains the dest nbr idx
					mobile.data.bufferTX=&EvtToMOB.Message;
					if (EvtToMOB.Val == 5) { //Mensaje a todos los autorizados
						for (i = 0; i < 4; i++) {
							if (SP.SP_GSM_REG_AUTH & (1 << i)) {
								if ((strlen(gprs_rpn[i]) > 6)
										&& (strlen(gprs_rpn[i]) <= 15)) { //
//									if (SendSMS(EvtToMOB.Message, buff_SMS,
//											gprs_rpn[i], 30 * 1000)) {
//									}
//									data.timeout = 30 * 1000;
									sms_send(&mobile, gprs_rpn[i]);
								}
							}
						}
					} else if (EvtToMOB.Val > 0 && EvtToMOB.Val < 5) {
//						SendSMS(EvtToMOB.Message, buff_SMS,
//								gprs_rpn[EvtToMOB.Val - 1], 60 * 1000);
						sms_send(&mobile, gprs_rpn[EvtToMOB.Val - 1]);
					}
					sendingSMS = false;
					memset(EvtToMOB.Message,0,sizeof(EvtToMOB.Message));
					break;
				case read_process_sms:
					//Process incoming SMS
					mobile.data.bufferTX=&EvtToMOB.Message;
					readSMS(&mobile, &gps);
					processSmsCommand(&mobile,mobile.data.bufferRX);

					break;
				case SMS_ServiceCenter:
					//Set SMS service center
					//Update SMS Service Center
					sprintf(buff_SMS, "AT+CSCA=\"%s\"", gsm.SMS_scenter);
					if (sendCheckReply(&mobile, buff_SMS, OK_CHAR, 500)) {
					} else {
						EvtToMOB.Src = SMS_ServiceCenter;
						xQueueSend(queEvtToMOBILE, &EvtToMOB, 0);
					}
					break;
				case cmd_Report:
					//SMS command.
					sprintf(mobile.data.bufferTX, "ID: %s, %s, %s\n"
							"Stat: %s\n"
							"Flow: %.1f%s\n"
							"Solar ch.: %.1fA\n"
							"Batt V: %.1fV\n"
							"Tnk Level: %.1f%s\n"
							"Press: %dpsi\n"
							"Pumped: %.1f%s", &id_Tag[0], &id_Tag[1],
							&id_Tag[2], PumpOn ? "On" : "Off", sp_pump.PumpRate,
							Var.flow_unit, Var.CPanel, Var.VBatt, Var.tank_vol,
							Var.vol_unit, Var.LinePress, Var.Total_pumped,
							Var.vol_unit /*&buff_SMS[250]*/);
//					Send_message(buff_SMS, &gsm);
					sms_send(&mobile,mobile.status.SMS_orig);
					break;
				case cmd_Location:
					//SMS command.
					if (gps.gps_fix) {
						sprintf(mobile.data.bufferTX,
								"Well ID: %s, %s, %s\n"
										"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
								&id_Tag[0], &id_Tag[1], &id_Tag[2], gps.Lat,
								gps.Lon);
					} else {
						//Lee en EEPROM la ultima locación
						leer_eeprom((char*) &gps.Lat, OFF_LAT, LEN_COORD);
						leer_eeprom((char*) &gps.Lon, OFF_LON, LEN_COORD);

						sprintf(mobile.data.bufferTX,
								"Well ID: %s, %s, %s\n"
										"GPS NOT FIXED.\n"
										"LAST KNOW LOC:\n"
										"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
								&id_Tag[0], &id_Tag[1], &id_Tag[2], gps.Lat,
								gps.Lon);
					}
//					Send_message(buff_SMS, &gsm);
					sms_send(&mobile, mobile.status.SMS_orig);
					break;
				case cmd_reset_totalizer:
					//SMS command.
					i = 0;
					guardar_eeprom("0000", OFF_TOTALIZER, LEN_TOTALIZER);
					Var.Total_gearmot_turns = 0;
					sprintf(mobile.data.bufferTX, "Totalizer cleared");
//					Send_message(buff_SMS, &gsm);
					sms_send(&mobile, mobile.status.SMS_orig);
					break;
				case cmd_Start_Notif:
					//SMS command. Enable notifications by SMS
					sprintf(mobile.data.bufferTX, "Notifications ENABLED");
					SP.SP_GSM_REG_AUTH |= (0x0f
							& (1 << (gsm.Send_SMS_to_idx - 1)));
//					Send_message(buff_SMS, &gsm);
					sms_send(&mobile, mobile.status.SMS_orig);
					break;
				case cmd_Stop_Notif:
					//SMS command. Disable notifications by SMS
					sprintf(mobile.data.bufferTX, "Notifications DISABLED");
					SP.SP_GSM_REG_AUTH &= ~(1 << (gsm.Send_SMS_to_idx - 1));
//					Send_message(buff_SMS, &gsm);
					sms_send(&mobile, mobile.status.SMS_orig);
					break;
				case cmd_Welcome:
					//Send welcome SMS to registered user
					//Pongo un contador de seguridad para que el sistema no envíe mas de 8 SMS de bienvenida por día
					if (cont_welc_sent) {
						sprintf(mobile.data.bufferTX,
								"Welcome to sms service\n"
										"Well ID: %s, %s, %s\n"
										"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
								&id_Tag[0], &id_Tag[1], &id_Tag[2], gps.Lat,
								gps.Lon);
//						SendSMS(buff_SMS, buff_SMS, gprs_rpn[EvtToMOB.Val - 1],
//								60 * 1000);
						sms_send(&mobile, gprs_rpn[EvtToMOB.Val - 1]);

					} else {
						if (current_day != Local.time[RTC_TIMETYPE_DAYOFYEAR])
							cont_welc_sent = WLC_MSG_PDAY;
						else
							cont_welc_sent--;
					}

					break;
				case ref_gps:


#if (USE_SIM868 && USE_GPS)
						getGPScoordinates(buff_SMS, sizeof(buff_SMS), &gps);
#elif (USE_BG96 && USE_GPS)
						getGNSScoordinates(buff_SMS, sizeof(buff_SMS), &gps);
#elif (USE_SIM7070G && USE_GPS)
						if(!isGpsPoweredOn(&mobile))
						{
							setGpsPowerTo(&mobile, 1);
						}
						while(!readGpsCoordinates(&mobile, &gps))
						{
							cont_gps_fix++;
							vTaskDelay(100);
							if(cont_gps_fix>=600)
							{
								cont_gps_fix = 0;
								break;
							}
						}
#endif
						if (gps.gps_fix) {
							guardar_eeprom((char*) &gps.Lat, OFF_LAT,
							LEN_COORD);
							guardar_eeprom((char*) &gps.Lon, OFF_LON,
							LEN_COORD);
							cont_gps_fix = 0;
						}
						vTaskDelay(100);
						setGpsPowerTo(&mobile, 0);
						vTaskDelay(1000);

						EvtToMOB.Src = MOBILE_Init;
						xQueueSend(queEvtToMOBILE, &EvtToMOB, 0);
						//Request UNREAD SMS
						break;
				case ref_SMS:
					//Refresh: sign strength, GPS, check APN
					if (!debug_gsm) {
						if(isGpsPoweredOn(&mobile))
						{
							setGpsPowerTo(&mobile, 0);
						}
						/*Request signal quality*/
						if (sendGetReply(&mobile, "\rAT+CSQ", 500) > 5) //Pido el nivel de señal
							sscanf(buff_SMS, "%*[^ ] %d,%d", &gsm.rssi,
									&gsm.ber);

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

						/*Si todavia no se registro en la red...*/
						if (!gsm.reg_on_netw) {

							/*Request Registered on network status*/
							if (sendCheckReply(&mobile, "AT+CREG?",
									"+CREG: 0,1", 500)) {

								/*Request OPER name*/
								if (mobile_get_info(&mobile, COPS, 4000)) {
									if (strstr(buff_SMS, OK_CHAR)) {

										/*Leo el nombre de la operadora*/
										ptr = strstr(buff_SMS, "+COPS:");
										if (strstr(ptr, "+COPS:")) {

											ptr = strtok(buff_SMS, ","); //+COPS: 0

											ptr = strtok(NULL, ",");	//0

											ptr = strtok(NULL, ",");//"Movistar Movistar"
											if (ptr != NULL)
												sscanf(ptr, "\"%[^\"]s",
														gsm.oper);

											ptr = strtok(NULL, " ,\r\n");
											if (ptr != NULL)
												gsm.accessTechnology = atoi(ptr);

										}

										/*Según el nombre de la operadora, configuro el APN en base a los perfiles almacenados en EEPROM*/
										for (i = 0; i < 4; i++) {
											memset(buff_SMS, 0x00, 100);
											leer_eeprom(buff_SMS,
													OFF_GPRS_APN_PROF1
															+ i * LEN_APN_PROF,
													LEN_APN_PROF);

											ptr = strstr(buff_SMS, gsm.oper);

											/* En el caso de que encuentre el nombre de la operadora antes del perfil 4, el for dejará de
											 * buscar y tomará ese perfil como válido. Si no encuentra coincidencia en ninguno de los
											 * 3 primeros, indefectiblemente cargara el Profile_4
											 */
											if ((ptr != 0) | (i == Profile_4)) {

												/*Leo el APN, usr y pass desde el MOBILE*/
												ptr = strtok(buff_SMS, ";");

												ptr = strtok(NULL, ";");
												if (ptr != NULL)
													strcpy(gprs_apn, ptr);
												else
													sprintf(gprs_apn, "no_apn");

												ptr = strtok(NULL, ";");
												if (ptr != NULL)
													strcpy(gprs_usr, ptr);
												else
													memset(gprs_usr, 0x00,
													LEN_GPRS_USR_PASS);

												ptr = strtok(NULL, ";");
												if (ptr != NULL)
													strcpy(gprs_pass, ptr);
												else
													memset(gprs_usr, 0x00,
													LEN_GPRS_USR_PASS);

												/*Guardo el indice del APN que estare usando*/
												Var.apn_prof = i;
												break;

											}
										}
									}
								}

								//Request SMS Service Center
								if (sendGetReply(&mobile, "AT+CSCA?", 500)
										> 5) {
									if (strstr(buff_SMS, OK_CHAR))
										sscanf(buff_SMS, "%*[^\"]\"%[^\"]",
												gsm.SMS_scenter);
									else
										break;
								}
							}
						}
						//Verificacion de estado de formato sms
						mobile_get_info(&mobile, CMGF, 4000);
						if (strstr(mobile.data.bufferRX, "+CMGF: 0"))
						{
							sprintf(mobile.data.bufferRX, "AT+CMGF=1");

							sendCheckReply(&mobile, mobile.data.bufferRX, OK_CHAR, 1000);
						}
						for (i = 0; i < 4; i++) {
							if (readSMS(&mobile, &gsm) == false)
								break;
							else
								processSmsCommand(&mobile, mobile.data.bufferRX);
						}
					}
					break;

				default:
					break;
				}

				xSemaphoreGive(mtxMOBILE);
			}
		}
	}
}

void Create_COMM_MOBILE_Ev_task(void) {
	xTaskCreate(vCOMM_MOB_Ev_Task, (char*) "MOBILE Ev TSK", 750,
	NULL, Prio_COMM_MOBILE_Ev_Task, &tsk_mobile_ev_handler);
}

