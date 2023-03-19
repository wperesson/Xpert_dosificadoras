/*
 * TaskINPUT.c
 *
 *  Created on: 09/03/2014
 *      Author: admin
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Definiciones.h"
#define  DELAY_TIME			1000
#define  CNT_TO_1_SEC		2 //CNT_TO_1_SEC = 1000/DELAY_TIME

struct tm *timeinfo;
time_t now;
xTimerHandle Timer_Gen_Off;
static gen_ctrl_ev_Type EvtTmoutSrc; //Evento q procesa el Timeout

void vTmrGenCallback(xTimerHandle pxTimer) {
//	long TimerId;
	gen_ctrl_ev_Type Evento;

	configASSERT(pxTimer);
	xTimerStop(pxTimer, 0);
	Evento = EvtTmoutSrc;
	xQueueSend(queEvtGenStart, &Evento, 0);
}

/* Reprograma la próxima fecha y hora del test de generador
 * days_to_next_test: Cantidad de dias para el próximo test.
 * return: nothing
 */
void reprogram_alarm(void) {

//	NVIC_DisableIRQ((IRQn_Type) RTC_IRQn);
	//Leo la hora local
	Chip_RTC_GetFullTime(LPC_RTC, &Local);
	time(&now); //Resetea la variable now a cero
	timeinfo = localtime(&now);
	timeinfo->tm_hour = (int) Local.time[RTC_TIMETYPE_HOUR];
	timeinfo->tm_min = (int) Local.time[RTC_TIMETYPE_MINUTE];
	timeinfo->tm_sec = (int) Local.time[RTC_TIMETYPE_SECOND];
	timeinfo->tm_mday = (int) Local.time[RTC_TIMETYPE_DAYOFMONTH];
	timeinfo->tm_mon = (int) Local.time[RTC_TIMETYPE_MONTH] - 1;
	timeinfo->tm_year = (int) Local.time[RTC_TIMETYPE_YEAR] - 1900;
	now = mktime(timeinfo); //Valido el tiempo actual. Actualizo now

	timeinfo = localtime(&gen.genNextStart); //Cargo el valor agendado del prox start

	while (now > gen.genNextStart) {
		timeinfo->tm_mday += gen.period;
		gen.genNextStart = mktime(timeinfo);
	}

	//chequeo que la alarma del proximo test no este mas asdelante que
	//el valor de Next Test.
	if (difftime(gen.genNextStart, now)
			> (uint32_t) timeinfo->tm_mday * 24 * 3600) {
		//El próximo test esta fuera de sincronismo
		timeinfo->tm_mday = (int) Local.time[RTC_TIMETYPE_DAYOFMONTH];
		gen.genNextStart = mktime(timeinfo);
	}

//	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
//		eeprom_write((char*) &gen, OFF_GEN, sizeof(gen));
//		xSemaphoreGive(mtxI2C1);
//	}	//todo: poner funcion "guardar_eeprom"
	guardar_eeprom((char*) &gen, OFF_GEN, sizeof(gen));
}

static portTASK_FUNCTION(vPWR_GEN_Task,pvParameters) {

	static uint16_t Cont_after_low_batt_ev, Cont_after_gen_start;
	gen_ctrl_ev_Type Evento;
	EvtToMOB_Type EvtToGSM;

	Cont_after_low_batt_ev = Cont_after_gen_start = 0;

	//Timers: TIMEOUT_ENCENDIDO, TIMEOUT_APAGADO
	Timer_Gen_Off = xTimerCreate("Tmr_TMOUT", 10 * 60 * 1000, false, (void*) 0,
			vTmrGenCallback);

	xQueueReset(queEvtGenStart);

	reprogram_alarm();

	/* Enable RTC interrupt in NVIC */
	NVIC_DisableIRQ((IRQn_Type) RTC_IRQn);
	NVIC_SetPriority((IRQn_Type) RTC_IRQn, mainSOFTWARE_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ((IRQn_Type) RTC_IRQn);

	SGN_O1_OFF;
	SGN_O2_OFF;
	gen.contact = false;
	gen.running = false;

	while (1) {
		if (xQueueReceive(queEvtGenStart, (void*) &Evento,
				(DELAY_TIME / portTICK_RATE_MS)) == pdTRUE) {

			switch (Evento) {
			case genEv_Manual_On:
				gen.contact = true;
				break;

			case genEv_Manual_Off:
				gen.contact = false;
				ClearAlarm(Al_GenFail);	//Borro la alarma para que se reinicie el ciclo
				break;

			case genEv_Gsm_On:
				if (Var.local_remoto == TRUE) {	//Modo MANUAL
					sprintf(EvtToGSM.Message,
							"Pump in manual mode. Command ignored.");

					EvtToGSM.Src = sendSMS;
					EvtToGSM.Val = gen.sender;
					xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
				} else {	//Modo AUTO

					if (!gen.contact) {
						//Enciendo el timer de TIMEOUT;
						xTimerChangePeriod(Timer_Gen_Off,
								gen.gsm_timeout * 60 * 1000, 0);
						xTimerStart(Timer_Gen_Off, 0);
						EvtTmoutSrc = genEv_Gsm_Off;
						gen.contact = true;
						ClearAlarm(Al_GenFail);	//Borro la alarma para que se reinicie el ciclo
					}

					if (gen.gsm_timeout >= 10 * 60) {
						gen.gsm_timeout = 10 * 60;
						sprintf(EvtToGSM.Message, "Generator started.\n"
								"MAX TIMEOUT VAL EXC or NOT VAL. "
								"Limited to %d min", gen.gsm_timeout);
					} else {
						sprintf(EvtToGSM.Message, "Generator started.\n"
								"Timeout: %d min", gen.gsm_timeout);
					}
					EvtToGSM.Src = sendSMS;
					EvtToGSM.Val = gen.sender;
					if (xQueueSend(queEvtToMOBILE, &EvtToGSM, 0) == pdFAIL) {
						Evento = genEv_Gsm_On;
						xQueueSend(queEvtGenStart, &Evento, 0);
					}

				}
				break;

			case genEv_Gsm_Off:

				if (Var.local_remoto == TRUE) {	//----- Modo MANUAL
					sprintf(EvtToGSM.Message,
							"Command ignored. Pump in manual mode.");

					EvtToGSM.Src = sendSMS;
					EvtToGSM.Val = gen.sender;
					xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
				} else {					//------ Modo AUTO
					gen.contact = false;
					sprintf(EvtToGSM.Message, "Generator stopped.");
					EvtToGSM.Src = sendSMS;
					EvtToGSM.Val = gen.sender;
					if (xQueueSend(queEvtToMOBILE, &EvtToGSM, 0) == pdFAIL) {
						Evento = genEv_Gsm_Off;
						xQueueSend(queEvtGenStart, &Evento, 0);
					}
				}

				break;

			case genEv_Modbus_On:
				if (Var.local_remoto == TRUE) {
					//Modo MANUAL. No Hacer nada
				} else {	//Modo AUTO
					if (!gen.contact) {
						//Enciendo el timer de TIMEOUT;
						xTimerChangePeriod(Timer_Gen_Off,
								gen.gsm_timeout * 60 * 1000, 0);
						xTimerStart(Timer_Gen_Off, 0);
						EvtTmoutSrc = genEv_Gsm_Off;
						gen.contact = true;
						ClearAlarm(Al_GenFail);	//Borro la alarma para que se reinicie el ciclo
					}
				}
				break;

			case genEv_Modbus_Off:
				if (Var.local_remoto == TRUE) {
					//Modo MANUAL. No Hacer nada
				} else {	//Modo AUTO
					gen.contact = false;
				}

				break;

			case genEv_Low_Batt_On:
				if (Var.local_remoto == FALSE) {				//----- Modo AUTO
					if (!gen.contact && !GetAlarm(Al_GenFail)) {
						gen.contact = true;
						//Enciendo el timer de TIMEOUT;
						xTimerChangePeriod(Timer_Gen_Off,
								gen.low_batt_timeout * 60 * 1000, 0);
						xTimerStart(Timer_Gen_Off, 0);
						EvtTmoutSrc = genEv_Gsm_Off;//Uso el mismo q para cuando lo enciendo por SMS
					}
				}
				break;

			case genEv_Sch_On:
				if (Var.local_remoto == FALSE) {				//----- Modo AUTO
					if (!gen.contact) {
						gen.contact = true;
						//Enciendo el timer de TIMEOUT;
						xTimerChangePeriod(Timer_Gen_Off,
								gen.test_timeout * 60 * 1000, 0);
						xTimerStart(Timer_Gen_Off, 0);
						EvtTmoutSrc = genEv_Sch_Off;
					}
				}
				break;

			case genEv_Sch_Off:
				gen.contact = false;
				break;

			case genEv_fail_start_manual:
			case genEv_fail_start_gsm:
			case genEv_fail_start_lowBatt:
			case genEv_fail_start_sched:

			default:
				break;
			}
		}

		if (gen.contact) {
			//=========================================================================
			//GENERADOR ENCENDIDO.
			PWR_RELAY_1_2_ON;//Conecto el RELE 2 que controla el ARRANQUE DEL GENERADOR
			if (!gen.running) {			//GENERATOR NOT RUNNING

				if (Cont_after_gen_start == 10 * CNT_TO_1_SEC) {
					//10 seg: Conecto el negativo del generador a la batería
					SGN_O2_ON;
					SGN_O1_OFF;
					Cont_after_gen_start++;
				} else if (Cont_after_gen_start == 30 * CNT_TO_1_SEC) {
					//30 seg: El generador fallo en su arranque
					SetAlarm(Al_GenFail);
					SGN_O1_OFF;
					SGN_O2_OFF;
					gen.contact = false;
					Cont_after_gen_start = 0;
					xTimerStop(Timer_Gen_Off, 0);	//Paro el conteo del TIMEOUT
				} else if (Cont_after_gen_start > 20 * CNT_TO_1_SEC) {
					//20 seg: Comienzo a chequear la tension del generador
					if (Var.DI & DIn_1) { //2. Señal digital en DIn1
						gen.running = true;
						Cont_after_gen_start = 0;
						ClearAlarm(Al_GenFail);
						SGN_O1_ON;
						SGN_O2_ON;
					} else {
						gen.running = false;
						Cont_after_gen_start++;
					}
				} else {
					gen.running = false;
					Cont_after_gen_start++;
				}
			} else { //GENERATOR STOPPED
				//Generador encendido. Chequeo tensión de entrada y entrada dig.
				if (Var.DI & DIn_1) { //2. Señal digital en DIn1
					gen.running = true;
					Cont_after_gen_start = 0;
				} else {
					Cont_after_gen_start++;
					if (Cont_after_gen_start == 10) {
						//El generador fallo durante el tiempo de carga
						gen.running = false;
						SetAlarm(Al_GenFail);
						gen.contact = false;
						xTimerStop(Timer_Gen_Off, 0); //Paro el conteo del TIMEOUT
						SGN_O1_OFF;
						SGN_O2_OFF;
					}
				}
			}
		} else {
			//=========================================================================
			//GENERADOR APAGADO.
			if (gen.running) {			//El gen todavia esta encendido. Apagar
				SGN_O1_OFF; 	//Desconecto (+)
				vTaskDelay(500);
				SGN_O2_OFF;		//Desconecto (-)
				vTaskDelay(2000);
			}
			PWR_RELAY_1_2_OFF; 	//Paro el generador
			gen.running = false;
			Cont_after_gen_start = 0;
		}

		//Chequeo condición de encendido: LOW BATTERY
		if (gen.low_batt_timeout == 0) {
			//Desactivo el Autoencendido por baja bateria

		} else {
			if (!gen.contact) {
				if ((Var.VBatt < gen.low_batt_limit) && (Var.VBatt > 0)
						&& (Cont_after_low_batt_ev == 0)) {
					Evento = genEv_Low_Batt_On;
					xQueueSend(queEvtGenStart, &Evento, 0);
					Cont_after_low_batt_ev =
							gen.low_batt_timeout * CNT_TO_1_SEC; //*60;
				} else if (Cont_after_low_batt_ev > 0) {
					Cont_after_low_batt_ev--;
				}
			}
		}

		//Chequeo condición de encendido: SCHEDULED TEST
		if (gen.test_timeout == 0) {
			//Desactivo el encendido por SCHEDULED Test
		} else {
			Chip_RTC_GetFullTime(LPC_RTC, &Local);
			timeinfo = localtime(&gen.genNextStart); //Cargo el valor agendado del prox start
			mktime(timeinfo); //Valido el tiempo actual. Actualizo now

			//Comparo Hora y minuto
			if ((timeinfo->tm_min == (int) Local.time[RTC_TIMETYPE_MINUTE])
					&& (timeinfo->tm_hour == (int) Local.time[RTC_TIMETYPE_HOUR])) {
				//Comparo fecha
				if ((timeinfo->tm_mday
						== (int) Local.time[RTC_TIMETYPE_DAYOFMONTH])
						&& (timeinfo->tm_mon
								== (int) Local.time[RTC_TIMETYPE_MONTH] - 1)
						&& (timeinfo->tm_year
								== (int) Local.time[RTC_TIMETYPE_YEAR] - 1900)) {
					//Estamos en momento de hacer el TEST del GEN
					reprogram_alarm();
					if (!gen.contact) {
						Evento = genEv_Sch_On;
						xQueueSend(queEvtGenStart, &Evento, 0);
					}
				}
			}
		}

		if (upd_next_gen_start_data) {
			//chequeo integridad de los datos
			if (gen.test_timeout < 0)
				gen.test_timeout = 0;

			if (gen.gsm_timeout < 0)
				gen.gsm_timeout = 0;

			if (gen.low_batt_timeout < 0)
				gen.low_batt_timeout = 0;

			if (gen.low_batt_limit > 30)
				gen.low_batt_limit = 30;
			else if (gen.low_batt_limit < 10)
				gen.low_batt_limit = 10;

			if (gen.period > 15)
				gen.period = 15;
			else if (gen.period < 0)
				gen.period = 0;

			reprogram_alarm();
			upd_next_gen_start_data = false;
		}
	}
}

void Create_PWR_GENtask(void) {
	xTaskCreate(vPWR_GEN_Task, (char*) "PWR GENERATOR TSK", 240,
	NULL, Prio_PWR_GEN_Task, &tsk_power_gen_handler);
}

