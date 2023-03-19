/*
 * Task_ALARM.c
 *
 *  Created on: 15/06/2015
 *  Author: W. Peressón
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Librerias de C includes. */
#include "Definiciones.h"

#define USE_WATCHDOG 0

/*msg_sent, posicion de cada bit*/
typedef enum {
	msg_LowBatt_sent,
	msg_Very_LowBatt_sent,
	msg_Warn_LowLev_sent,
	msg_Alarm_LowLev_sent,
	msg_DoorOpen_sent,
	msg_GenFail_sent,
	msg_Motor_stall_sent
} msg_sent_position_Typ;

xTimerHandle Tmr[Al_LAST];

static void vALARM(void *pvParameters);

/*********************************************************************//**
 * @brief		Revisa e valor de una variable y setea la alarma correspondiente
 * 				si se supera el valor límite por un determinado tiempo dado por el
 * @param[in]	alarm_id: identificador de la alarma
 * @param[in]	*cont: puntero al contador de segundos actuales
 * @param[in]	count_limit: valor límite del contador
 * @param[in]	set_point: valor límite de la variable
 * @param[in]	value: valor actual de la variable
 *
 * @return	Estado de la alarma
 * 				0: Sin alarma
 * 				1: Alarma disparada
 * 				2: Alarma establecida
 * 				3: Alarma reseteada
 **********************************************************************/
AlarmStat_Type process_alarm(AlarmPos_Type alarm_id, uint8_t *cont,
		uint8_t count_limit, float set_point, float value) {

	if (GetAlarm(alarm_id)) { //Alarma activada
		if (value >= (set_point * 1.1))
			(*cont)++;
		else
			*cont = 0;

		if (*cont >= count_limit) {
			ClearAlarm(alarm_id);
			*cont = 0;
			return al_st_ALARM_RESETED;
		}
		return al_st_ALARM_ON;
	} else { //No tengo alarma
		if (value < set_point)
			(*cont)++;
		else
			*cont = 0;

		if (*cont >= count_limit) {
			SetAlarm(alarm_id);
			*cont = 0;
			return al_st_ALARM_SHOOTED;
		}
		return al_st_NO_ALARM;
	}
	return al_st_NO_ALARM;
}

/*-----------------------------------------------------------
 * 	PROCESAMIENTO DE ERRORES DEL INSTRUMENTO. */
static portTASK_FUNCTION(vALARM, pvParameters) {
	portTickType xLastWakeTime;

	static uint8_t cnt_advertencia_bajo_nivel_T1, cnt_alarma_bajo_nivel_T1;

	static uint16_t Cont_Warning_DO,Cont_Alarm_Lev_dn,Cont_Alarm_Lev_up;
	EvtToMOB_Type EvtToSIM868;

	static bool vll_mem_PumpOn;	//Variable para memorizar el estado de la bomba ante una Very Low Level
	global_Very_low_battery = false;
	Parameter_upd_Type newParameter;
	char msg_sent_register;
//	mqtt_state_type mqtt_st;
	AlarmStat_Type alarm_status;//Almacena el estado de la alarma para permitir reaccionar según el estado
	uint32_t Cont_Low_Batt_dn=0,Cont_Low_Batt_up=0,Cont_Very_Low_Batt=0;
	Bool vlb_mem_PumpOn;
	xLastWakeTime = xTaskGetTickCount();

	RegAlarmas = 0; //Resetea el valor del registro

	vTaskDelay(10000); //Espero 10 segundos hasta q arranquen las otras tareas

	/*Ver si vengo de un reinicio por el WDT*/
	if (LPC_WWDT->MOD & (1 << 2)) {
		snd_head_flow_sett(&newParameter, (0x01 & LPC_RTC->GPREG[0]) ? 1 : 0,
				PUMP_ON, src_self_generated, 0);
	}

#if USE_WATCHDOG

	/* Initialize WWDT and event router */
	Chip_WWDT_Init(LPC_WWDT);

	Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_PCLK);
	wdtFreq = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_WDT) / 4;

	/* Set watchdog feed time constant to 6s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * 60); //todo: volver a 6 seg

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Clear watchdog warning and timeout interrupts */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);

	/* Enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

#endif
	cnt_advertencia_bajo_nivel_T1 = cnt_alarma_bajo_nivel_T1 = 0;
	Cont_Warning_DO = 0;

//	/*Refrescar el actual de las alarmas via web*/
//	mqtt_st = mqtt_publish_alarms;
//	xQueueSend(queMQTT_SM, &mqtt_st, 0);

	/*El registro de banderas de mensajes enviados debe iniciar en 0*/
	msg_sent_register = 0;
	while (1) {
		//ALIMENTO EL WATCHDOG
		Chip_WWDT_Feed(LPC_WWDT);

		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));

		/*Decremento el valor de global_Scada_Control_cnt de modo que si llega a cero
		 * es porque el SCADA no escribió mas datos. Esto me permite saber si el sistema
		 * esta en el estado "BAJO CONTROL DE SCADA" o "SIN CONTROL DE SCADA".
		 */
		if (global_Scada_Control_cnt > 0)
			//El sistema SCADA esta haciendo lecturas, por la tanto el controlador
			//esta BAJO CONTROL DEL SCADA
			global_Scada_Control_cnt--;
		else {
			//El sistema SCADA dejó de hacer lecturas, entonces estamos en modo
			//AUTONOMO
			if (Var.rst_totalizer)
				Var.rst_totalizer = false;
		}

		/*
		 ADVERTENCIA T1: Bajo nivel de tanque
		 ALARMA T1: Muy bajo nivel de tanque
		 ADVERTENCIA T2: Bajo nivel de tanque
		 ALARMA T2: Muy bajo nivel de tanque
		 ALARMA: Puerta abierta
		 ALARMA: Motor B1 atascado
		 ALARMA: Motor B2 atascado
		 */

		/*==========================================================================================*/
		/* ALARMA POR APERTURA DE PUERTA FRONTAL */
		if (Var.DI & 0x01) { //DOOR OPEN
			if (Cont_Warning_DO == 2) { //2 segundos para avisar q se abrio la puerta
				SetAlarm(Al_DoorOpen);
				if (!GetFlag(&msg_sent_register, msg_DoorOpen_sent)) {
					sprintf(EvtToSIM868.Message, "WARNING.\n"
							"Door Open.");
					EvtToSIM868.Src = sendSMS;
					EvtToSIM868.Val = 5; //A todos los miembros del phonebook
					if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
						SetFlag(&msg_sent_register, msg_DoorOpen_sent, true);
					Cont_Warning_DO++;
				}
			} else if (Cont_Warning_DO < 2) {
				Cont_Warning_DO++;

			}

			else
				Cont_Warning_DO = 0;

		} else if (!(Var.DI & 0x01)) { //DOOR CLOSED
			ClearAlarm(Al_DoorOpen);

			if (Cont_Warning_DO >= 600) { //10 minutos
				SetFlag(&msg_sent_register, msg_DoorOpen_sent, false);
				Cont_Warning_DO = 0;
			} else if (Cont_Warning_DO > 0) { //Si es cero no hago nada y queda esperando que se abra la puerta
				Cont_Warning_DO++;
			}

		}

		/*==========================================================================================*/
		/* ADVERTENCIA POR BAJO NIVEL EN TANQUE 1*/
		alarm_status = process_alarm(Al_advertencia_bajo_nivel_T1,
				&cnt_advertencia_bajo_nivel_T1, 100, sp_pump.level_warning_limit_T1,
				Var.tank_vol);
		switch (alarm_status) {
		case al_st_ALARM_SHOOTED:
			if (!GetFlag(&msg_sent_register, msg_Warn_LowLev_sent)) {
				sprintf(EvtToSIM868.Message, "WARNING.\n"
						"Low Level: %.1f %s", Var.tank_vol, Var.vol_unit);
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
					SetFlag(&msg_sent_register, msg_Warn_LowLev_sent, true);
			}
			break;
		case al_st_NO_ALARM:
		case al_st_ALARM_ON:
		case al_st_ALARM_RESETED:
		default:
			break;
		}

		/*==========================================================================================*/
		/* ALARMA POR BAJO NIVEL EN TANQUE*/
		alarm_status = process_alarm(Al_alarma_bajo_nivel_T1,
				&cnt_alarma_bajo_nivel_T1, 100, sp_pump.level_alarm_limit_T1,
				Var.tank_vol);
		switch (alarm_status) {
		case al_st_ALARM_SHOOTED:
			if (!GetFlag(&msg_sent_register, msg_Alarm_LowLev_sent)) {
				if (PumpOn) {
					/*Debo chequear que la puerta este cerrada para pausar el motor
					 * ya que si un operario esta calibrando la bomba es normal que salten
					 * las alarmas*/
					if (!(Var.DI & 0x01)) { //Chequeo que la puerta este cerrada
						/*Pongo la bomba en pausa para que no succione aire*/
						snd_head_flow_sett(&newParameter, 0, PUMP_ON,
								src_self_generated, 0);
						/*Memorizo el estado para encender la bomba cuando se restablezca el nivel*/
						vll_mem_PumpOn = PumpOn;

						sprintf(EvtToSIM868.Message, "ALARM.\n"
								"Vey Low Tank Level: %.1f %s\n"
								"PUMP STOPPED", Var.tank_vol, Var.vol_unit);
					}
				} else {
					sprintf(EvtToSIM868.Message, "ALARM.\n"
							"Very Low Level: %.1f %s", Var.tank_vol,
							Var.vol_unit);
				}
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
					SetFlag(&msg_sent_register, msg_Alarm_LowLev_sent, true);

			}
			break;
		case al_st_ALARM_RESETED:
			if (!PumpOn) { //Chequeo que la bomba este apagada

				/* La bomba estaba encendida???*/
				if (vll_mem_PumpOn) {
					/*Pongo la bomba en marcha ya que tenemos nivel nuevamente*/
					snd_head_flow_sett(&newParameter, 1, PUMP_ON,
							src_self_generated, 0);

					sprintf(EvtToSIM868.Message, "PUMP RESTARTED\n"
							"Level: %.1f %s", Var.tank_vol, Var.vol_unit);

					EvtToSIM868.Src = sendSMS;
					EvtToSIM868.Val = 5; //A todos los miembros del phonebook
					if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
						SetFlag(&msg_sent_register, msg_Alarm_LowLev_sent,
						true);
				}
			}
			break;
		case al_st_NO_ALARM:
		case al_st_ALARM_ON:
		default:
			break;
		}

		if (!GetAlarm(Al_alarma_bajo_nivel_T1)) {
			if (Var.tank_vol < sp_pump.level_alarm_limit_T1)
				Cont_Alarm_Lev_dn++;
			else
				Cont_Alarm_Lev_dn = 0;
		}

		if (GetAlarm(Al_alarma_bajo_nivel_T1)) {
			if (Var.tank_vol > (sp_pump.level_alarm_limit_T1 * 1.1))
				Cont_Alarm_Lev_up++;
			else
				Cont_Alarm_Lev_up = 0;
		}

		if (Cont_Alarm_Lev_dn >= 40) {
			SetAlarm(Al_alarma_bajo_nivel_T1);
			if (!GetFlag(&msg_sent_register, msg_Alarm_LowLev_sent)) {

				if (PumpOn) {
					/*Debo chequear que la puerta este cerrada para pausar el motor
					 * ya que si un operario esta calibrando la bomba es normal que salten
					 * las alarmas*/
					if (!(Var.DI & 0x01)) { //Chequeo que la puerta este cerrada
						/*Pongo la bomba en pausa para que no succione aire*/
						snd_head_flow_sett(&newParameter, 0, PUMP_ON,
								src_self_generated, 0);
						/*Memorizo el estado para encender la bomba cuando se restablezca el nivel*/
						vll_mem_PumpOn = PumpOn;

						sprintf(EvtToSIM868.Message, "ALARM.\n"
								"Vey Low Tank Level: %.1f %s\n"
								"PUMP STOPPED", Var.tank_vol, Var.vol_unit);
					}
				} else {
					sprintf(EvtToSIM868.Message, "ALARM.\n"
							"Very Low Level: %.1f %s", Var.tank_vol,
							Var.vol_unit);
				}
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
					SetFlag(&msg_sent_register, msg_Alarm_LowLev_sent, true);

			}
		}

		if (Cont_Alarm_Lev_up >= 120) {
			ClearAlarm(Al_alarma_bajo_nivel_T1);
			SetFlag(&msg_sent_register, msg_Alarm_LowLev_sent, false);
			Cont_Alarm_Lev_dn = 0;
			Cont_Alarm_Lev_up = 0;

			if (!PumpOn) { //Chequeo que la bomba este apagada

				/* La bomba estaba encendida???*/
				if (vll_mem_PumpOn) {
					/*Pongo la bomba en marcha ya que tenemos nivel nuevamente*/
					snd_head_flow_sett(&newParameter, 1, PUMP_ON,
							src_self_generated, 0);

					sprintf(EvtToSIM868.Message, "PUMP RESTARTED\n"
							"Level: %.1f %s", Var.tank_vol, Var.vol_unit);

					EvtToSIM868.Src = sendSMS;
					EvtToSIM868.Val = 5; //A todos los miembros del phonebook
					if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
						SetFlag(&msg_sent_register, msg_Alarm_LowLev_sent,
						true);
				}
			}
		}
#if !MOTOR_ELEC
		/*==========================================================================================*/
		/* ALARMA POR BAJA TENSION EN BATERIA */
		if (!GetAlarm(Al_LowBatt)) {
			if ((Var.VBatt < gen.low_batt_limit))
				Cont_Low_Batt_dn++;
			else
				Cont_Low_Batt_dn = 0;
		}

		if (GetAlarm(Al_LowBatt)) {
			if (Var.VBatt >= (gen.low_batt_limit + 1.0f))
				Cont_Low_Batt_up++;
			else
				Cont_Low_Batt_up = 0;
		}

		if (Cont_Low_Batt_dn >= 40) {
			SetAlarm(Al_LowBatt);
			if (!GetFlag(&msg_sent_register, msg_LowBatt_sent)) {

				sprintf(EvtToSIM868.Message, "ALARM.\n"
						"Low Battery: %.1f V", Var.VBatt);
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100) == pdPASS)
					SetFlag(&msg_sent_register, msg_LowBatt_sent, true);
			}
		}

		if (Cont_Low_Batt_up >= 120) {
			ClearAlarm(Al_LowBatt);
			SetFlag(&msg_sent_register, msg_LowBatt_sent, false);
			Cont_Low_Batt_dn = 0;
			Cont_Low_Batt_up = 0;
		}

		/*==========================================================================================*/
		/* VERY LOW BATTERY < WARNING_LOW_BATT */

		if ((Var.VBatt <= (gen.low_batt_limit - 0.5f)) && (Var.VBatt > 0)) {

			if (Cont_Very_Low_Batt == 60) {
				global_Very_low_battery = true;
				SetAlarm(Al_VeryLowBatt);
				if (PumpOn) {
					/*Memorizo el estado para encender la bomba cuando se restablezca la carga de la bateria*/
					vlb_mem_PumpOn = PumpOn;

					snd_head_flow_sett(&newParameter, 0, PUMP_ON,
							src_self_generated, 0);

					if (!GetFlag(&msg_sent_register, msg_Very_LowBatt_sent)) {
						sprintf(EvtToSIM868.Message, "ALARM.\n"
								"Vey Low Battery: %.1f V\n"
								"PUMP STOPPED", Var.VBatt);
						EvtToSIM868.Src = sendSMS;
						EvtToSIM868.Val = 5; //A todos los miembros del phonebook
						if (xQueueSend(queEvtToMOBILE, &EvtToSIM868,
								100) == pdPASS)
							SetFlag(&msg_sent_register, msg_Very_LowBatt_sent,
							true);
					}
				}
			} else {
				Cont_Very_Low_Batt++;
			}
		} else if (Var.VBatt > (gen.low_batt_limit + 0.5f)) {
			if (Cont_Very_Low_Batt == 0) {
				SetFlag(&msg_sent_register, msg_Very_LowBatt_sent, false);
				global_Very_low_battery = false;
				ClearAlarm(Al_VeryLowBatt);
				if (vlb_mem_PumpOn) {
					snd_head_flow_sett(&newParameter, 1, PUMP_ON,
							src_self_generated, 0);
					vlb_mem_PumpOn = false;

					sprintf(EvtToSIM868.Message, "PUMP RESTARTED\n"
							"Battery voltage: %.1f V", Var.VBatt);

					EvtToSIM868.Src = sendSMS;
					EvtToSIM868.Val = 5; //A todos los miembros del phonebook
					xQueueSend(queEvtToMOBILE, &EvtToSIM868, 100);
				}
			} else if (Cont_Very_Low_Batt > 0) {
				Cont_Very_Low_Batt--;
			}
		}
#endif
		/*==========================================================================================*/
		/* ALARMA POR FALLO EN GENERADOR */
		if (GetAlarm(Al_GenFail)) {
			if (!GetFlag(&msg_sent_register, msg_GenFail_sent)) {
				Chip_RTC_GetFullTime(LPC_RTC, &Local);
				sprintf(EvtToSIM868.Message, "Generator Failed to start.\n"
						"%02d:%02d %02d/%02d/%02d",
						Local.time[RTC_TIMETYPE_HOUR],
						Local.time[RTC_TIMETYPE_MINUTE],
						Local.time[RTC_TIMETYPE_MONTH],
						Local.time[RTC_TIMETYPE_DAYOFMONTH],
						Local.time[RTC_TIMETYPE_YEAR]);
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if ( xQueueSend(queEvtToMOBILE, &EvtToSIM868, 0) == pdPASS)
					SetFlag(&msg_sent_register, msg_GenFail_sent, true);
			}
		} else
			SetFlag(&msg_sent_register, msg_GenFail_sent, false);

		/*==========================================================================================*/
		/* ALARMA POR MOTOR ATASCADO */
		if (GetAlarm(Al_alarma_motor_atascado_B1)) {
			if (!GetFlag(&msg_sent_register, msg_Motor_stall_sent)) {
				Chip_RTC_GetFullTime(LPC_RTC, &Local);
				sprintf(EvtToSIM868.Message, "ALARM.\n"
						"Motor Stalled.");
				EvtToSIM868.Src = sendSMS;
				EvtToSIM868.Val = 5; //A todos los miembros del phonebook
				if (xQueueSend(queEvtToMOBILE, &EvtToSIM868, 0) == pdPASS)
					SetFlag(&msg_sent_register, msg_Motor_stall_sent, true);
			}
		} else
			SetFlag(&msg_sent_register, msg_Motor_stall_sent, false);
	}
}

void Create_ALARM_Task(void) {
	xTaskCreate(vALARM, (char*) "ALARM TSK", 210, NULL,
	Prio_ALARM_Task, &tsk_alarm_handler); /* "Procesamiento de ERRORES*/
}

