/*
 * TaskINPUT.c
 *
 *  Created on: 09/03/2014
 *      Author: admin
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LTC2946.h"

#include "Definiciones.h"
#include "GSM.h"

#define UART				LPC_UART3
#define UART_IRQ_SELEC		UART3_IRQn
#define UART_HANDLER_NAME 	UART3_IRQHandler
#define BAUDRATE			115200
#define REF_RATE			20000

void vTmrRefGPSCallback(xTimerHandle pxTimer) {
	EvtToGSM_Type EvtToGSM;

	configASSERT(pxTimer);
	EvtToGSM.Src = ref_gps_sms;
	xQueueSend(xQueEvtToGSM, &EvtToGSM, 0);
}

static portTASK_FUNCTION(vCOMM_SMS_Task,pvParameters) {
	uint8_t i;
	uint8_t cont_gps_fix; //Cont de retardo para almacenar coordenadas en EEPROM
	EvtToGSM_Type EvtToGSM;
	gen_ctrl_ev_Type EventoGen;
	char buff_SMS[GSM_BUFF_SIZE];
	Parameter_upd_Type newParameter;

	xTimerHandle Timer_ref;

	memset(gsm.imei, 0, sizeof(gsm.imei));
	memset(gsm.oper, 0, sizeof(gsm.oper));
	memset(gsm.SMS_scenter, 0, sizeof(gsm.SMS_scenter));

	EvtToGSM.Src = gsm_init;
	xQueueSend(xQueEvtToGSM, &EvtToGSM, 0);
	cont_gps_fix = 0;

	//Timer para refrescar la lectura de GPS y chequear SMS pendientes
	Timer_ref = xTimerCreate("Tmr_ref_data", REF_RATE, true, (void*) 0,
			vTmrRefGPSCallback);
	if (Timer_ref != 0) {
		if (xTimerStart(Timer_ref, 100) == pdPASS) {
			xTimerReset(Timer_ref, 100);
		}
	}

	while (1) {

		//Reviso si alguna tarea me envió un pedido
		if (xQueueReceive(xQueEvtToGSM, &EvtToGSM, 1000) == pdPASS) {
			/*
			 * Semaforo para sincronizar la posesion del UART3
			 */
			if (xSemaphoreTake(xMtxSIM868, portMAX_DELAY) == pdTRUE) {

				switch (EvtToGSM.Src) {

				case gsm_init: //Inicializo el módulo. Leo la información importante

					//Envio ESC para salir de cualquier modo actual
					buff_SMS[0] = 0x1b;
					buff_SMS[1] = 0x00;
					sendCheckReply(UART, buff_SMS, buff_SMS, "OK",
							sizeof(buff_SMS), 4000);
					//Chequeo que el módulo responde
					if (!sendCheckReply(UART, buff_SMS, "AT", "OK",
							sizeof(buff_SMS), 4000))
						break;

					//Select SMS Mode = TEXT
					if (!sendCheckReply(UART, buff_SMS, "AT+CMGF=1", "OK",
							sizeof(buff_SMS), 4000))
						break;
					//Select Character Set = GSM
					if (!sendCheckReply(UART, buff_SMS, "AT+CSCS=\"GSM\"", "OK",
							sizeof(buff_SMS), 500))
						break;
					//Request IMEI
					if (sendGetReply(UART, "AT+GSN", buff_SMS, sizeof(buff_SMS),
							4000) > 10) { //Pido el IMEI
						if (strstr(buff_SMS, "OK"))
							sscanf(&buff_SMS[9], "%[^'\r']", gsm.imei);
						else
							break;
					}
					//todo: sacar esto del inicio y ponerlo en algun lugar donde lo consulte periodicamente
					//Request OPER name
					if (sendGetReply(UART, "AT+COPS?", buff_SMS,
							sizeof(buff_SMS), 500) > 5) { //Pido el nombre de la operadora
						if (strstr(buff_SMS, "OK"))
							sscanf(buff_SMS, "%*[^'\"']%s\"", gsm.oper);
						else
							break;
					}
					//Request SMS Service Center
					if (sendGetReply(UART, "AT+CSCA?", buff_SMS,
							sizeof(buff_SMS), 500) > 5) {
						if (strstr(buff_SMS, "OK"))
							sscanf(buff_SMS, "%*[^\"]\"%[^\"]",
									gsm.SMS_scenter);
						else
							break;
					}
					uart_gsm_rdy = true;
					break;
				case sendToGsm:
					//Debo chequear si antes tenemos algún mensaje?
					Chip_UART_SendBlocking(UART, EvtToGSM.Message,
							EvtToGSM.Val);

					break;
				case sendSMS:
					if (EvtToGSM.Val == 5) { //Mensaje a todos los autorizados
						for (i = 0; i < 4; i++) {
							if (SP.SP_GSM_REG_AUTH & (1 << i)) {
								if ((strlen(RPN[i]) > 6)
										&& (strlen(RPN[i]) <= 15)) { //
									if (SendSMS(UART, EvtToGSM.Message,
											buff_SMS, RPN[i], 30 * 1000)) {
									}
								}
							}
						}
					} else if (EvtToGSM.Val > 0 && EvtToGSM.Val < 5) {
						SendSMS(UART, EvtToGSM.Message, buff_SMS,
								RPN[EvtToGSM.Val - 1], 60 * 1000);
					}
					break;
				case read_process_sms:
					memset(buff_SMS, 0, sizeof(buff_SMS));
					readSMS(UART, buff_SMS, sizeof(buff_SMS), &gsm);
					processSmsCommand(_mobile, ptr);
					break;
				case SMS_ServiceCenter:
					//Update SMS Service Center
					sprintf(buff_SMS, "AT+CSCA=\"%s\"", gsm.SMS_scenter);
					if (sendCheckReply(UART, buff_SMS, buff_SMS, "OK",
							sizeof(buff_SMS), 500)) {
					} else {
						EvtToGSM.Src = SMS_ServiceCenter;
						xQueueSend(xQueEvtToGSM, &EvtToGSM, 0);
					}
					break;
				case cmd_Report:
					read_flow_unit(&SP, &Var);
					sprintf(buff_SMS, "Well ID: %s\n"
							"Status: %s\n"
							"Rate: %.2f %s\n"
							"Solar Ch.: %.2f A\n"
							"Battery: %.2f V\n"
							"Tank Level: %.1f %s\n"
							"Pressure: %d psi\n"
							"Total: %.2f %s", id_well, PumpOn ? "On" : "Off",
							SP.PumpRate, &Var.flow_unit, Var.CPanel, Var.VBatt,
							Var.tank_vol, &Var.vol_unit, Var.LinePress,
							Var.Total_pumped, &Var.vol_unit);
					Send_message(UART, buff_SMS, &gsm);
					break;
				case cmd_Ubidots_set_apn:

					if (GPRS_start_conn(UART, buff_SMS, GSM_BUFF_SIZE,
							EvtToGSM.Message, USER,
							PASS)) {
						strcpy(APN, EvtToGSM.Message);

						//Guardo APN en EEPROM
						if (xSemaphoreTake(xMtxI2C1, 10) == pdTRUE) {
							eeprom_write(APN, OFF_GPRS_APN, sizeof(APN));
							xSemaphoreGive(xMtxI2C1);
						}
						sprintf(buff_SMS, "New APN:\n%s", APN);
					} else {
						sprintf(buff_SMS,
								"Failed to connect with new APN %s\nPlease correct and try again.\n",
								APN);
					}

					Send_message(UART, buff_SMS, &gsm);

					break;
				case cmd_Location:
					if (gsm.gps_fix) {
						sprintf(buff_SMS, "Well ID: %s, %s, %s\n"
								"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
								id_Client, id_Field, id_well, gsm.Lat, gsm.Lon);
					} else {
						if (xSemaphoreTake(xMtxI2C1, 10) == pdTRUE) {
							//Lee en EEPROM la ultima locación
							eeprom_read((char*) &gsm.Lat, OFF_LAT,
							LEN_COORD);
							eeprom_read((char*) &gsm.Lon, OFF_LON,
							LEN_COORD);
							xSemaphoreGive(xMtxI2C1);
						}
						sprintf(buff_SMS, "Well ID: %s, %s, %s\n"
								"GPS NOT FIXED.\n"
								"LAST KNOW LOC:\n"
								"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
								id_Client, id_Field, id_well, gsm.Lat, gsm.Lon);
					}
					Send_message(UART, buff_SMS, &gsm);
					break;
//				case cmd_Flow:
//
//					break;
				case cmd_reset_totalizer:
//					i = 0;
//					if (xSemaphoreTake(xMtxI2C1, 10) == pdTRUE) {
//						eeprom_write((char*) &i, OFF_TOTALIZER,
//						LEN_TOTALIZER);
//						xSemaphoreGive(xMtxI2C1);
//					}
//					Var.Total_mot_turns = 0;
//					sprintf(buff_SMS, "Totalizer cleared");
//					Send_message(UART, buff_SMS, &gsm);

					newParameter.Val = 1;
					newParameter.parameter = RST_TOTAL;
					newParameter.Src = src_sms;
					xQueueSend(xQueHEAD_FLOW_SETT_upd, &newParameter, 0);
					break;
				case cmd_Start_Pump:

					break;
				case cmd_Stop_Pump:

					break;
				case cmd_Start_Gen:
					EventoGen = genEv_Gsm_On;
					gen.Sender = gsm.Send_SMS_to_idx;
					xQueueSend(xQueEvtGenStart, &EventoGen, 0);
					break;
				case cmd_Stop_Gen:
					EventoGen = genEv_Gsm_Off;
					gen.Sender = gsm.Send_SMS_to_idx;
					xQueueSend(xQueEvtGenStart, &EventoGen, 0);
					break;
				case cmd_Start_Notif:
					sprintf(buff_SMS, "Notifications ENABLED");
					SP.SP_GSM_REG_AUTH |= (0x0f
							& (1 << (gsm.Send_SMS_to_idx - 1)));
					Send_message(UART, buff_SMS, &gsm);
					break;
				case cmd_Stop_Notif:
					sprintf(buff_SMS, "Notifications DISABLED");
					SP.SP_GSM_REG_AUTH &= ~(1 << (gsm.Send_SMS_to_idx - 1));
					Send_message(UART, buff_SMS, &gsm);
					break;
				case cmd_Welcome:
					sprintf(buff_SMS, "Welcome to sms service\n"
							"Well ID: %s, %s, %s\n"
							"http://maps.google.com/?q=%f,%f", //31+10+1+10=52
							id_Client, id_Field, id_well, gsm.Lat, gsm.Lon);
					SendSMS(UART, buff_SMS, buff_SMS, RPN[EvtToGSM.Val - 1],
							60 * 1000);
					break;
				case ref_gps_sms:
					if (!debug_gsm) {
						//Request signal quality
						if (sendGetReply(UART, "\rAT+CSQ", buff_SMS,
								sizeof(buff_SMS), 500) > 5) //Pido el nivel de señal
							sscanf(buff_SMS, "%*[^ ] %d,%*d", &gsm.sstr);

						//Pido información del GPS
						getGPScoordinates(UART, buff_SMS, sizeof(buff_SMS),
								&gsm);

						if (gsm.gps_fix) {
							if (cont_gps_fix == 10) {
								if (xSemaphoreTake(xMtxI2C1, 10) == pdTRUE) {
									//Alamcena en EEPROM una locación para tener la ultima locación conocida
									eeprom_write((char*) &gsm.Lat, OFF_LAT,
									LEN_COORD);
									eeprom_write((char*) &gsm.Lon, OFF_LON,
									LEN_COORD);
									xSemaphoreGive(xMtxI2C1);
									cont_gps_fix++;
								}
							} else if (cont_gps_fix < 10)
								cont_gps_fix++;
						} else
							cont_gps_fix = 10;

						//Request UNREAD SMS
						readSMS(UART, buff_SMS, sizeof(buff_SMS), &gsm);
						ptr=bufSMS;
						processSmsCommand(_mobile, ptr);
					}
					break;
				default:
					break;
				}

				xSemaphoreGive(xMtxSIM868);
			}
		}
	}
}

void Create_COMM_SMS_task(void) {
	xTaskCreate(vCOMM_SMS_Task, (char *) "SMS TSK", 400,
	NULL, Prio_COMM_SMS_Task, &vCOMM_SMS_Handle);
}

