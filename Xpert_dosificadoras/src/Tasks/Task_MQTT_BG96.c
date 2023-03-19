/*
 * Task_MQTT_BG96.c
 *
 *  Created on: Mayo 2020
 *  Author: wperesson
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Definiciones.h"
//#include "GSM.h"
////#include "GPRS_MQTTClient.h"
//#include "LTE_CATM1.h"

#if USE_BG96

#define UART				LPC_UART3
#define LOC_BUFF_SZ			50
#define REF_RATE			60000
#define CTRL_REF_FACT		50
#define QOS					1
#define REF_COUNTER_LIM		10

///*Flag states*/
//typedef enum {
//	b_conected_to_broker, b_suscribed
//} Flags_state_type;

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

void print_to_RS485(char *str) {
	uint16_t len;
	char head[] = "-------->";
	char *ptr = head;

	if (sniff_gsm || debug_gsm) {
		len = strlen(ptr);
		while ((xQueueSend(queMsgToRS485, ptr++, 10) != errQUEUE_FULL)
				&& (len > 0)) {
			len--;
		}

		len = strlen(str);
		while ((xQueueSend(queMsgToRS485, str++, 10) != errQUEUE_FULL)
				&& (len > 0)) {
			len--;
		}

		ptr = head;
		sprintf(ptr, "\r\n");
		len = strlen(ptr);
		while ((xQueueSend(queMsgToRS485, ptr++, 10) != errQUEUE_FULL)
				&& (len > 0)) {
			len--;
		}
	}
}

#if DEBUG
#define PRINT(x)	print_to_RS485(x)
#else
#define PRINT(x)
#endif

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

bool connected_to_TCP(char *buff, uint16_t buff_size) {
	uint8_t i;

	for (i = 0; i < 2; i++) {
		if (LTE_TCP_Connect(buff, buff_size, gprs_apn, gprs_usr, gprs_pass)) {
			return TRUE;
		}
	}
	return FALSE;
}

bool ping_req(char *buff) {
	uint16_t len;

	/*SEND PING REQUEST*/
	len = MQTT_PingReqPacket((uint8_t*) buff);
	if (len > 0) {
		/*El servidor devolvió una respuesta*/

		/*PING RESP OK !!!!*/
		return true;
	}
	return false;
}

static portTASK_FUNCTION(vCOMM_MQTT_Task,pvParameters) {
	char buff_MQTT[MQTT_BUFF_SIZE];
	static uint16_t len, mem_mqtt_ref_rate;
	static uint16_t msgID;
	static tcp_send_Type Recb;
	static Bool subscribed; //Flag de suscripción activa
	uint16_t attempt_cnt;
	uint8_t cont_coord_report_posit;
	mqtt_event_type mqtt_ev;
	mqtt_event_type mqtt_prev_ev; //Almacena el estado previo.
	char losant_state_head[50];
	xTimerHandle timer_ref_var, timer_ref_cise;

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

	/*Esperar a estar registrado en la red*/
	while (gsm.reg_on_netw != TRUE) {
		vTaskDelay(5000);
	}

	/*Arranco el timer para publicar variables*/
	if (xTimerIsTimerActive(timer_ref_var) == pdFALSE) {
		if (xTimerStart(timer_ref_var, 100) == pdPASS) {
			xTimerReset(timer_ref_var, 100);
		}
	}

	/* Leo la URL del endpoint "register" */
	eeprom_read(endpoint_url, OFF_ENDPOINT_URL,
	LEN_ENDPOINT_URL);

	/*Primer estado al que voy a entrar*/
	xQueueReset(queMQTT_ev); //Borro cualquier item dentro de la cola.
	mqtt_prev_ev = mqtt_ev_connect_to_broker;
	next_event(mqtt_ev_req_keys_from_broker);

	while (1) {

		if (xQueueReceive(queMQTT_ev, &mqtt_ev, portMAX_DELAY) == pdPASS) {

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

					switch (mqtt_ev) {

					/*=======================================================================
					 * 		---	GET CREDENTIALS FROM BROKER ---
					 */
					case mqtt_ev_req_keys_from_broker:

						/*Si estoy registrado en Losant, leo las credenciales*/
						if (SP.already_registered_onlosant) {
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

							sprintf(clientID, "6203f73f34a9cc895bfaf4f0");
							sprintf(access_key,
									"96855b03-2f27-4bfb-b04f-596661119e13");
							sprintf(access_secret,
									"0332ec6e66c4b37deb5e396bd466d5a2a542e59589d4cc10d3a851c07c48876b");

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

								if (https_get_credentials(buff_MQTT,
								MQTT_BUFF_SIZE, gprs_apn, gprs_usr,
										gprs_pass)) {
									/*Setear la bandera de que tengo las credenciales*/
									SP.already_registered_onlosant = TRUE;

									/*Guardo SP en EEPROM*/
									guardar_eeprom((char*) &SP, OFF_SP,
											sizeof(SP));

									next_event(mqtt_ev_connect_to_broker);
								} else {
									//No consiguió las credenciales. Volver a intentar
									next_event(mqtt_ev_req_keys_from_broker);
								}
							}
						}

						/*Imprimo el encabezado que luego usaremos para publicar*/
						sprintf(losant_state_head, "/losant/%s/state",
								clientID);
//						ptr = losant_state_head;
//						for (; *ptr; ++ptr)
//							*ptr = tolower(*ptr);
						break;
						/*=======================================================================
						 * 		---	CONNECT TO BROKER ---
						 */
					case mqtt_ev_connect_to_broker:

						/*Reseteo la cola para borrar posibles acciones*/
						xQueueReset(queMQTT_ev);

						/*Cierro la conexion*/
						LTE_TCP_Disconnect(buff_MQTT, MQTT_BUFF_SIZE);

						/*Intento iniciar nuevamente*/
						if (LTE_TCP_Connect(buff_MQTT, MQTT_BUFF_SIZE, gprs_apn,
								gprs_usr, gprs_pass)) {

						}

						if (LTE_Connect_to_broker(buff_MQTT, MQTT_BUFF_SIZE,
								clientID, access_key, access_secret)) {
							conected_to_broker = true;
							next_event(mqtt_ev_subscribe);
							subscribed = false;
						} else {
							vTaskDelay(500);
							next_event(mqtt_ev_connect_to_broker);
							conected_to_broker = false;
						}
						break;

						/*=======================================================================
						 * 		---	DISCONNECT FROM BROKER ---
						 */
					case mqtt_ev_disconnect_from_broker:
						LTE_TCP_Disconnect(buff_MQTT, MQTT_BUFF_SIZE);
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

						if (attempt_cnt < 3) {

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
									len = sprintf((char*) buff_MQTT, "{"
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
									len = sprintf((char*) buff_MQTT, "{"
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
								len = sprintf((char*) buff_MQTT, "{"
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

								len = sprintf((char*) buff_MQTT, "{"
										"\"data\":{"
										"\"stg1\":\"%s\","
										"\"stg2\":\"%s\","
										"\"stg3\":\"%s\""
										"}}", &id_Tag[0], &id_Tag[1],
										&id_Tag[2]);
							}
							/*Publicar EVENTOS*/
							else if (mqtt_ev == mqtt_ev_publish_events) {

								len = sprintf((char*) buff_MQTT, "{"
										"\"data\":{"
										"\"evod\":\"1\","
										"\"evms\":\"1\""
										"}}");
							}
							/*Publicar COMANDOS*/
							else if (mqtt_ev == mqtt_ev_publish_commands) {
//								PRINT("PUBLISH CTRLS");
								len = sprintf((char*) buff_MQTT, "{"
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
										sprintf((char*) buff_MQTT, "{"
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

							Recb = LTE_mqtt_publish(buff_MQTT, MQTT_BUFF_SIZE,
									len, losant_state_head, &msgID);
							msgID++;
							if (msgID == 0)
								msgID = 1;

							if (Recb > 0) {

							} else {
								/*Sin respuesta del server.*/
								attempt_cnt++;
							}
						} else {
							/*Superamos la cantidad de intentos para realizar la operacion c exito*/
							next_event(mqtt_ev_connect_to_broker);
							attempt_cnt = 0;
						}
						break;

						/*=======================================================================
						 * 		---	SUBSCRIBE ---
						 */
					case mqtt_ev_subscribe:
						memset(buff_MQTT, 0, MQTT_BUFF_SIZE);

						len = sprintf((char*) buff_MQTT, "losant/%s/command",
								clientID);
						subscribed = LTE_mqtt_subscribe(buff_MQTT,
						MQTT_BUFF_SIZE, len);

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
			if (!ping_req(buff_MQTT)) {
				next_event(mqtt_ev_connect_to_broker);
			}
		}
	}
}

void Create_COMM_MQTT_BG96_task(void) {
	xTaskCreate(vCOMM_MQTT_Task, (char*) "MQTT TSK", 512,
	NULL, Prio_COMM_MQTT_Task, &tsk_mqtt_handler);
}

#endif

