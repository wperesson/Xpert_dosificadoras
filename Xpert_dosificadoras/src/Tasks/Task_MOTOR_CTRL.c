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

#define step_analysis 		0		//Realiza un análisis de respuesta al escalón
#define USE_AC_MOTOR 		0		//Controla el Pwm DIRECTO sin realimentación

#define PWM_2          2 //P2_1 (2-3 Bits of PINSEL4)
#define PWM_3          4 //P2_2 (4-5 Bits of PINSEL4)
#define PWM_4          6 //P2_3 (6-7 Bits of PINSEL4)
#define MAX_PWM_CNT		600
#define WINDUP_GUARD	5300 //4000

#define TIMER_0 				LPC_TIMER0
#define TMR_HANDLER_NAME_0 	TIMER0_IRQHandler

#define TIMER_1 				LPC_TIMER1
#define TMR_HANDLER_NAME_1 	TIMER1_IRQHandler

#define TIMER_TON		1
#define TIMER_TCYCLE	2

#define SBIT_CNTEN     0
#define SBIT_PWMEN     2

#define SBIT_PWMMR0R   1

#define SBIT_LEN2      2
#define SBIT_LEN3      3

#define SBIT_PWMENA2   10
#define SBIT_PWMENA3   11

char dutyCycle;
static bool MotorCycle;

xTimerHandle Timer_Cycle_Hdl, Timer_TOn_Hdl;

static uint32_t ReadCapt_1;
//static uint32_t ReadCapt_2[3];

#if step_analysis

/*__attribute__ ((section(".RamAHB32")))*/float RPM[1500];

#endif

static uint8_t cont_ovr;

static uint32_t Memo_1;

static uint8_t Elap_pulses_1 = 0, Elap_Turns_Total = 0;
static uint16_t Elap_Turns_1 = 0, memElap_Turns_1 = 0;

void pid_zeroize(void) {
	// set prev and integrated error to zero
	Var.prev_error = 0;
	Var.int_error = 0;
}

/*
 * Calcula el Volumen Per Revolution en la unidad elegida: Lit o Gal
 */
void calcula_VPR(SP_Pump_Type *sp) {

	if ((SP.flow_unit == Galons_hour) || (SP.flow_unit == Galons_day)) {
		//1 USA GALLON = 231 in3
		sp->VPR = M_PI * pow(sp->plg_size, 2) * sp->plg_str * (float) sp->heads
				/ (4 * 231); //Gallon
	} else {
		//
		sp->VPR = M_PI * pow(sp->plg_size, 2) * sp->plg_str
				* (2.54 * 2.54 * 2.54) * (float) sp->heads / (4 * 1000); //Liters
	}
}

/*
 * Calcula el max caudal según la max RPM seteada. EL valor estara en la unidad seleccionada
 */
float calculate_MaxFlowRate(SP_Pump_Type *sp) {
	switch (SP.flow_unit) {
	case Liters_hour:
	case Galons_hour:
		return 60 * sp->VPR * (float) sp->maxSpd / ((float) sp->GR * sp->CF);

	case Liters_day:
	case Galons_day:
		return 1440 * sp->VPR * (float) sp->maxSpd / ((float) sp->GR * sp->CF);

	}
	return 0;
}

/*
 * Calcula las RPM del motor para una caudal dado
 */
float calculate_RPM(SP_Pump_Type *sp, float pumprate) {

	switch (SP.flow_unit) {
	case Liters_hour:
	case Galons_hour:
		return pumprate * sp->CF * sp->GR / (sp->VPR * (float) 60.0);
	case Liters_day:
	case Galons_day:
		return pumprate * sp->CF * sp->GR / (sp->VPR * (float) 1440.0);
	}
	return 0;
}

/*
 * Calcula el PumpRate según las unidades seleccionadas, y en base a las RPM
 */
float calcula_FlowRate(SP_Pump_Type *sp, float motorRPM) {

	switch (SP.flow_unit) {
	case Liters_hour:
	case Galons_hour:
		return sp->VPR * (float) 60 * motorRPM / (sp->GR * sp->CF);

	case Liters_day:
	case Galons_day:
		return sp->VPR * (float) 1440 * motorRPM / (sp->GR * sp->CF);

	default:
		break;
	}
	return 0;
}

static portTASK_FUNCTION(vMOTOR_CTRL_Task,pvParameters) {
	static uint8_t con_retardo_motor_stall;
	double diff;
	double p_term;
	double i_term;
	double d_term;
	static uint8_t Retardo_Elap_Turns = 100;
	uint32_t match;
	static bool MotorOn;
	EvtToMOB_Type EvtToGSM;
	static uint8_t Ultima_Hora;
	Parameter_upd_Type newParameter;
	bool param_ok = false;
	Bool newPumpOn;
	static float maxFlowRate, tempfloat;
	Flow_UNITS_Typ mem_SP_flow_unit;
	mqtt_event_type mqtt_st;

#if step_analysis
	uint16_t j, k;
#endif

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	/* TIMER 0 para CAPTURE de MOTOR 1 */
	Chip_TIMER_Init(TIMER_0);

//Elijo como src de la cuenta el PCLK = CCLK/4. Ahora el CCLK es de 96MHz
//por lo tanto el PCLK=96/4=24MHz
	Chip_TIMER_TIMER_SetCountClockSrc(TIMER_0, TIMER_CAPSRC_RISING_PCLK, 0);

//Para hacer una cuenta de 100KHz debo dividir por 240, por ello seteo el PR en 239 ya
//que el sistema contará en el pulso 239+1 un incremento en el TC
	Chip_TIMER_PrescaleSet(TIMER_0, 239); //
	Chip_TIMER_MatchDisableInt(TIMER_0, 0);
	Chip_TIMER_ClearMatch(TIMER_0, 0);
	Chip_TIMER_CaptureRisingEdgeEnable(TIMER_0, 0);
	Chip_TIMER_CaptureEnableInt(TIMER_0, 0);

	Chip_TIMER_ClearMatch(TIMER_0, 0);
	Chip_TIMER_ClearMatch(TIMER_0, 1);
	Chip_TIMER_SetMatch(TIMER_0, 0, 1000);
	Chip_TIMER_SetMatch(TIMER_0, 1, sp_pump.Cycle * 60 * 100e3);
	Chip_TIMER_ResetOnMatchEnable(TIMER_0, 1); //Resetear cuando llegamos a TCYCLE

	Chip_TIMER_Reset(TIMER_0);
	Chip_TIMER_Enable(TIMER_0);

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_SetPriority(TIMER0_IRQn, 0);
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_SetPriority(TIMER1_IRQn, 0);
	NVIC_EnableIRQ(TIMER1_IRQn);

//Init PWM Motor 1
	Chip_PWM_Init(LPC_PWM1);
	Chip_PWM_SetCountClockSrc(LPC_PWM1, PWM_CAPSRC_RISING_PCLK, 0);
	Chip_PWM_PrescaleSet(LPC_PWM1, 0);
	Chip_PWM_ResetOnMatchEnable(LPC_PWM1, 0);
	Chip_PWM_SetMatch(LPC_PWM1, 0, MAX_PWM_CNT - 1);

	Chip_PWM_SetMatch(LPC_PWM1, 2, 0); //Motor 1
	Chip_PWM_SetMatch(LPC_PWM1, 3, 0); //Motor 2

	Chip_PWM_LatchEnable(LPC_PWM1, 0, PWM_OUT_ENABLED);
	Chip_PWM_Reset(LPC_PWM1);
	Chip_PWM_SetControlMode(LPC_PWM1, 2, PWM_SINGLE_EDGE_CONTROL_MODE,
			PWM_OUT_ENABLED);
	Chip_PWM_Enable(LPC_PWM1);

	/* Disable PWM interrupt */
	NVIC_ClearPendingIRQ(PWM1_IRQn);
	NVIC_DisableIRQ(PWM1_IRQn);

//Inicializo el pid
//	windup_guard = 4000;
	Var.int_error = 0;
	Var.prev_error = 0;

	Memo_1 = 0;
	con_retardo_motor_stall = 0;

	calcula_VPR(&sp_pump);

	maxFlowRate = calculate_MaxFlowRate(&sp_pump);

	if ((sp_pump.PumpRate < sp_pump.Threshold) && (sp_pump.PumpRate > 0)) {

		Chip_TIMER_SetMatch(TIMER_0, 0,
				(sp_pump.PumpRate / (float) sp_pump.Threshold) * (float) (sp_pump.Cycle) * 60
						* 100e3);
		Chip_TIMER_SetMatch(TIMER_0, 1, sp_pump.Cycle * 60 * 100e3);
		Chip_TIMER_MatchEnableInt(TIMER_0, 0); //Habilita interr por TCYCLE y TON
		Chip_TIMER_MatchEnableInt(TIMER_0, 1); //Habilita interr por TCYCLE
		sp_pump.MotorRPM = calculate_RPM(&sp_pump, sp_pump.Threshold);

	} else if (sp_pump.PumpRate > 0) {

		Chip_TIMER_MatchDisableInt(TIMER_0, 0); //Deshabilita int por TCYCLE y TON
		Chip_TIMER_MatchDisableInt(TIMER_0, 1); //Deshabilita int por TCYCLE
		sp_pump.MotorRPM = calculate_RPM(&sp_pump, sp_pump.PumpRate);
	}

	ReadCapt_1 = 1;
	Ultima_Hora = 25;
	MotorCycle = TRUE; //Inicio en true, porque es
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_RATE_MS));

		//----------------------------------------------------------
		//Almacenar el valor de las cantidad de vueltas cada hora transcurrida
		if (Local.time[RTC_TIMETYPE_HOUR] != Ultima_Hora) {
			Ultima_Hora = Local.time[RTC_TIMETYPE_HOUR];

			if (!Var.rst_totalizer) {		//Si el Reset Totalizer esta suelto

				//Almacenar en EEPROM el valor del totalizador
				guardar_eeprom((char*) &Var.Total_gearmot_turns,
				OFF_TOTALIZER,
				LEN_TOTALIZER);
			}
		}

		Var.Total_pumped = Var.Total_gearmot_turns * sp_pump.VPR / sp_pump.CF;

		/* 	El SP.CF se calcula como la relacion entre el ActualPumpRate/CorrectedPumpRate
		 *    CF= FR/FRc = VPR/VPRc => VPRc= VPR/CF
		 *    CF= 	Correction factor
		 *    VPR= 	Volume per revolution
		 *    FR= 	Flow rate
		 */

		//-----------------------------------------------
#if  USE_AC_MOTOR
		Var.MotorRPM = SP.maxSpd * (abs(Var.An[5].ADC_Un - 745)) / (2972);
#else
		if (Elap_Turns_1 > memElap_Turns_1) {
			Retardo_Elap_Turns = 10;
			memElap_Turns_1 = Elap_Turns_1;
		} else if (memElap_Turns_1 > Elap_Turns_1) {
			memElap_Turns_1 = Elap_Turns_1;
			Retardo_Elap_Turns = 10;
		}

		if (Retardo_Elap_Turns) {
			Retardo_Elap_Turns--;
			Var.MotorRPM = 60.0 * 100e3 / (float) ReadCapt_1;
			con_retardo_motor_stall = 0;
		} else {

			Var.MotorRPM = 0;
			if (PumpOn) {
				//MOTOR STALL - ALARMA
				if (con_retardo_motor_stall > 200) {
					SetAlarm(Al_alarma_motor_atascado_B1);

					snd_head_flow_sett(&newParameter, 0, PUMP_ON,
							src_self_generated, 0);

					con_retardo_motor_stall = 0;
				} else {
					if (MotorCycle && sp_pump.PumpRate > 0)
						con_retardo_motor_stall++;
				}
			}
		}
#endif

#if step_analysis

		if (!SP.PumpOn) {
			k = 0;
			j = 0;
		} else if (SP.PumpOn && k == 0) {
			k = 1;
			j = 1;
		} else if (j >= 500) {
			k = 0;
		} else {
			j += k;
		}

		if (k)
		RPM[j] = Var.MotorRPM;
		else
		RPM[0] = Var.MotorRPM;

#endif

#if USE_AC_MOTOR
		Var.PumpRate = calcula_FlowRate(&sp_pump, Var.MotorRPM);
#else
		if (ReadCapt_1 > 0) {
			Var.PumpRate = calcula_FlowRate(&sp_pump, Var.MotorRPM);
		} else
			Var.PumpRate = 0;
#endif
		//CALCULO EL ERROR ACTUAL DE VELOCIDAD
		Var.curr_error = sp_pump.MotorRPM - Var.MotorRPM;

		// integration with windup guarding
		Var.int_error += Var.curr_error;
		if (Var.int_error < -(WINDUP_GUARD))
			Var.int_error = -(WINDUP_GUARD);
		else if (Var.int_error > WINDUP_GUARD)
			Var.int_error = WINDUP_GUARD;

		// differentiation
		diff = Var.curr_error - Var.prev_error;

		// scaling
		p_term = (sp_pump.PropGain * Var.curr_error);
		i_term = (sp_pump.IntGain * Var.int_error);
		d_term = (sp_pump.DerGain * diff);

		// summation of terms
		Var.control = p_term + i_term + d_term;

		// save current error as previous error for next iteration
		Var.prev_error = Var.curr_error;

#if step_analysis
		Chip_PWM_SetMatch(LPC_PWM1, 2, MAX_PWM_CNT-1); //Step

#else
		if(PumpOn)
			PumpOn = verifyPumpOk();

		LPC_RTC->GPREG[0] = PumpOn ? 1 : 0;
		if (PumpOn) { //Bomba encendida
			//			ClearAlarm(Al_Motor_stall); //Reseteo la alarma de STALL

#if USE_AC_MOTOR
//			match = SP.PumpRate;
			match = (uint32_t)((sp_pump.MotorRPM + Var.control) / 3.05);
#else
			match = (uint32_t) (((sp_pump.MotorRPM + Var.control) / 5.977) + 9.706);
#endif
			if (match > MAX_PWM_CNT - 1)
				match = MAX_PWM_CNT - 1;
			else if (match < 0)
				match = 0;

			if (!MotorOn) { //El motor esta apagado?
				//Conectar energía y setar el Vin a 0
				Chip_PWM_SetMatch(LPC_PWM1, 2, 0); //Motor 1
				vTaskDelay(300);
				PWR_MOTOR_1_ON;
				MotorOn = true;
				Retardo_Elap_Turns = 10;
			}

			if (sp_pump.PumpRate == 0) { //Paro el motor con el SP
				Chip_PWM_SetMatch(LPC_PWM1, 2, 0);
			} else if (sp_pump.PumpRate < sp_pump.Threshold) { //Por debajo del umbral tambien verifico que todo este ok para que funcione la bomba
				MotorCycle ?
						Chip_PWM_SetMatch(LPC_PWM1, 2, match) :
						Chip_PWM_SetMatch(LPC_PWM1, 2, 0); //Motor 1
			} else {
				Chip_PWM_SetMatch(LPC_PWM1, 2, match);
			}
		} else if (global_Override) { // Boton de override
			PWR_MOTOR_1_ON;
			vTaskDelay(100);

			match = (uint32_t) ((sp_pump.Threshold * sp_pump.CF * sp_pump.GR
					/ (sp_pump.VPR * (float) 1440.0) / 5.977) + 9.706);
			Chip_PWM_SetMatch(LPC_PWM1, 2, match); //Motor 1
			Chip_PWM_LatchEnable(LPC_PWM1, 2, PWM_OUT_ENABLED);
			cont_ovr = 0;
			while (global_Override && cont_ovr < 15) {
				vTaskDelay(100);
				cont_ovr++;
			}
			global_Override = false;

		} else { //Apago el motor
			PWR_MOTOR_1_OFF;
			MotorOn = false;
			Chip_PWM_SetMatch(LPC_PWM1, 2, 0); //Motor 1
//						vTaskDelay(30);
		}

#endif
		Chip_PWM_LatchEnable(LPC_PWM1, 2, PWM_OUT_ENABLED);

		//
		//RECIBO NUEVO VALOR DE ALGUN PARAMETRO
		//
		if (xQueueReceive(queHEAD_FLOW_SETT_upd, &newParameter, 0) != pdFAIL) {
			param_ok = false;
			switch (newParameter.parameter) {
			case PUMP_ON:
				/* Salvo el valor del totalizador para evitar que se pierda antes de
				 * que transcurra la hora
				 */
				//Almacenar en EEPROM el valor del totalizador
				guardar_eeprom((char*) &Var.Total_gearmot_turns,
				OFF_TOTALIZER,
				LEN_TOTALIZER);

				/*Chequeo si el comando proviene de "src_self_generated".
				 * Si es asi se debe haber disparado una alarma de bajo nivel
				 * de bateria, o muy bajo nivel de tanque o un atascamiento del motor*/
				if (newParameter.src == src_self_generated) {
					newPumpOn = false;
				}

				/*Por defecto leemos el estado de PumpOn. */
				newPumpOn = PumpOn;
				/*
				 * MODO MANUAL
				 */
				if (Var.local_remoto == TRUE) {

					/*Solo puedo modificar el estado desde el panel frontal*/

					if (newParameter.src == src_front_panel) {
						newPumpOn = newParameter.value == 1;
					} else if (newParameter.src == src_sms) {
						sprintf(EvtToGSM.Message,
								"Command ignored. Pump in manual mode");
						EvtToGSM.Src = sendSMS;
						EvtToGSM.Val = newParameter.sender;
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);

					} else if (newParameter.src == src_modbus) {
						//El controlador esta en modo manual, no hacer nada
					} else if (newParameter.src == src_web) {
						//Avisar que la bomba esta en modo manual, alguien abrio la puerta
					}
				}

				/*
				 * MODO AUTONOMO
				 */
				else {

					/* Se puede modificar el estado desde SMS o Web, siempre y cuando
					 * no este bajo control de scada por las lineas digitales*/
					if (newParameter.src == src_sms) {
						if (global_Scada_Control_cnt == 0) {
							//"SIN CONTROL DE SCADA"
							newPumpOn = (newParameter.value == 1);
							if (newPumpOn) {
								read_flow_Vol_unit(&SP, &Var);
								sprintf(EvtToGSM.Message, "Pump started.\n"
										"Flow %.1f %s", sp_pump.PumpRate,
										&Var.flow_unit);
							} else {
								sprintf(EvtToGSM.Message, "Pump Stopped");
							}
						} else {
							//"BAJO CONTROL DE SCADA"
							sprintf(EvtToGSM.Message,
									"Command ignored. Pump under SCADA control");
						}
						EvtToGSM.Src = sendSMS;
						EvtToGSM.Val = newParameter.sender;
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
					} else if (newParameter.src == src_modbus) {
						//El controlador esta en modo AUTO
						if (!global_Very_low_battery) {
							//En caso de que el motor este atascado no lo enciendo por este medio
							if (!GetAlarm(Al_alarma_motor_atascado_B1))
								newPumpOn = newParameter.value == 1;
						}
					} else if (newParameter.src == src_front_panel) {
						//NO SE PUEDE OPERAR LA BOMBA CON LA PUERTA CERRADA
					} else
						newPumpOn = newParameter.value == 1;
				}

				/*Unico lugar donde modifico PumpOn*/
				PumpOn = newPumpOn;

				/*Si el comando es para encender el motor, reseteo el MOTOR_STALL*/
				if (PumpOn)
					ClearAlarm(Al_alarma_motor_atascado_B1); //Reseteo la alarma de STALL

				break;

			case SP_PumpRate:
				//Chequeo los límites del nuevo valor de caudal

				if (newParameter.value > maxFlowRate) { //======== Controlo límite superior ===============

					sp_pump.PumpRate = maxFlowRate;

					//Chequeo si la fuente del nuevo valor es el SMS.
					if (newParameter.src == src_sms) {
						read_flow_Vol_unit(&SP, &Var);
						sprintf(EvtToGSM.Message, "MAX VALUE EXCEEDED.\n"
								"Limited to Flow: %.1f %s", sp_pump.PumpRate,
								&Var.flow_unit);
					}

				} else if (newParameter.value < 0) { // ======== Controlo que sea > 0 ===============
					//Chequeo si la fuente del nuevo valor es el SMS.
					if (newParameter.src == src_sms) {
						read_flow_Vol_unit(&SP, &Var);
						sprintf(EvtToGSM.Message, "Invalid flow value.\n"
								"Flow: %.1f %s", sp_pump.PumpRate, &Var.flow_unit);
					}
					break;

				} else {// ======= El valor esta dentro de los límites ==============

					sp_pump.PumpRate = newParameter.value;
					//Chequeo si la fuente del nuevo valor es el SMS.
					if (newParameter.src == src_sms) {
						read_flow_Vol_unit(&SP, &Var);
						sprintf(EvtToGSM.Message, "New Flow Value: %.1f %s",
								sp_pump.PumpRate, &Var.flow_unit);
					}
				}
				if (newParameter.src == src_sms) {
					EvtToGSM.Src = sendSMS;
					EvtToGSM.Val = newParameter.sender;
					xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
				}
				param_ok = true;
				break;

			case SP_HEADS:
				//Chequeo el valor de la cantidad de cabezas sea 1, 2 o 4
				if ((newParameter.value == 1) || (newParameter.value == 2)
						|| (newParameter.value == 4)) {
					sp_pump.heads = (uint16_t) newParameter.value;
					param_ok = true;
				}
				break;

			case SP_THRESH:
				//Chequeo que el valor de Threshold no sea mayor al 50% del max valor de caudal
				tempfloat = 0.5 * calculate_MaxFlowRate(&sp_pump);
				if (newParameter.value <= tempfloat) {
					//Chequeo que el valor de Threshold no sea menor a 500 RPM del motor
					tempfloat = calculate_RPM(&sp_pump, newParameter.value);

					if ((tempfloat >= 500)) {
						sp_pump.Threshold = (uint16_t) newParameter.value;
					} else {
						sp_pump.Threshold = calcula_FlowRate(&sp_pump, 500);
					}

				} else
					sp_pump.Threshold = 0.5 * calculate_MaxFlowRate(&sp_pump);
				param_ok = true;
				break;

			case SP_PLG_SIZE:
				//Chequeo que el plunger size sea una de las tres opciones
				if ((newParameter.value == 0.5)				//  1/2
				|| (newParameter.value == 0.375) 		//	3/8
						|| (newParameter.value == 0.25)) {	//	1/4
					sp_pump.plg_size = newParameter.value;
					param_ok = true;
				}
				break;

			case SP_PLG_STR:
				//Chequeo que el plunger stroke sea una de las tres opciones
				if ((newParameter.value == 0.5)				//	1/2
				|| (newParameter.value == 0.75)		//	3/4
						|| (newParameter.value == 1)) {		//	1
					sp_pump.plg_str = newParameter.value;
					param_ok = true;
				}
				break;

			case SP_CYCLE_T:
				//Chequeo el valor del Cycle esté entre 0 y 60 min
				if (newParameter.value < 1) {
					sp_pump.Cycle = 1;
				} else if (newParameter.value > 60) {
					sp_pump.Cycle = 60;
				} else {
					sp_pump.Cycle = (uint16_t) newParameter.value;
				}
				param_ok = true;
				break;

			case SP_CF:
				//Chequeo el valor del CF sea mayor que 0
				if (newParameter.value > 0) {
					sp_pump.CF = newParameter.value;
					param_ok = true;
				}
				break;

			case SP_UNITS:
				//Almaceno temporalmente el valor de SP.vol_unit asi puedo convertir el SP.Pumprate segun las nuevas unidades
				mem_SP_flow_unit = SP.flow_unit;

				//Chequeo el valor de las unidades sea una de las posibles
				if ((newParameter.value > 0) && (newParameter.value <= 4)) {
					SP.flow_unit = newParameter.value;

					read_flow_Vol_unit(&SP, &Var);

					//Actualizo el VPR ya que cambian las Undades
					calcula_VPR(&sp_pump);

					//Pongo el nuevo Threshold en un valor acorde al motor, 500RPM
					sp_pump.Threshold = calcula_FlowRate(&sp_pump, 500);

					//Calculo el SP.PumpRate de acuerdo a las nuevas unidades
					/* Para pasar de Lit a Gal: 1 Galon  -> 3.78541 Litros
					 * Para pasar de Hora a Dia x24
					 */

					//==========================================================
					//Paso de GALON -> LITRO o de LITRO -> GALON
					if ((mem_SP_flow_unit == Galons_hour)
							|| (mem_SP_flow_unit == Galons_day)) {
						//Estaba en Galones--------------------
						if ((SP.flow_unit == Galons_hour)
								|| (SP.flow_unit == Galons_day)) {
							//Esta en Galones, no escalar

						} else {
							//Pasar a Litros
							sp_pump.PumpRate = sp_pump.PumpRate * 3.78541;
						}

					} else {
						//Estaba en Litros--------------------
						if ((SP.flow_unit == Galons_hour)
								|| (SP.flow_unit == Galons_day)) {
							//Pasar a Galones
							sp_pump.PumpRate = sp_pump.PumpRate / 3.78541;
						} else {
							//Esta en Litros, no escalar

						}
					}
					//----------------------------------------------------------

					//==========================================================
					//Paso de HORA -> DIA o de DIA -> HORA
					if ((mem_SP_flow_unit == Liters_hour)
							|| (mem_SP_flow_unit == Galons_hour)) {
						//Estaba en Horas--------------------
						if ((SP.flow_unit == Liters_hour)
								|| (SP.flow_unit == Galons_hour)) {
							//Esta en Horas, no escalar

						} else {
							//Pasar a Dia
							sp_pump.PumpRate = sp_pump.PumpRate * 24;
						}

					} else {
						//Estaba en Dias--------------------
						if ((SP.flow_unit == Liters_day)
								|| (SP.flow_unit == Galons_day)) {
							//Esta en Dias, no escalar

						} else {
							//Pasar a Horas
							sp_pump.PumpRate = sp_pump.PumpRate / 24;
						}
					}
					//----------------------------------------------------------

					param_ok = true;

					/*Envio tambien el nuevo parámetro para que se actualice el cálculo de
					 la capacidad del tanque que se expresa en las unidades elegidas*/
					snd_tank_sett(&newParameter, 0, SP_UNITS, src_front_panel);
				}
				break;

			case RST_TOTAL:
				/*Este comando puede provenir de diferentes fuentes. Si viene del modbus
				 * el estado de la variable "Var.rst_totalizer" lo controla el master. Por lo
				 * tanto si el estado de dicha variable es 1, el
				 *
				 */
				//Modo manual
				if (Var.local_remoto == TRUE) {
					if (newParameter.src == src_front_panel) {
						Var.Total_gearmot_turns = 0;
						//Almacenar en EEPROM el valor del totalizador
						guardar_eeprom((char*) &Var.Total_gearmot_turns,
						OFF_TOTALIZER,
						LEN_TOTALIZER);
					}
					if (newParameter.src == src_sms) {
						sprintf(EvtToGSM.Message,
								"Command ignored. Pump in manual mode");
						EvtToGSM.Src = sendSMS;
						EvtToGSM.Val = newParameter.sender;
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
					}
				}

				//Modo auto
				else {
					if (newParameter.src == src_sms) {
						if (global_Scada_Control_cnt == 0) {
							//"SIN CONTROL DE SCADA"
							Var.Total_gearmot_turns = 0;
							//Almacenar en EEPROM el valor del totalizador
							guardar_eeprom((char*) &Var.Total_gearmot_turns,
							OFF_TOTALIZER,
							LEN_TOTALIZER);
							sprintf(EvtToGSM.Message, "Totalizer cleared");

						} else {
							//"BAJO CONTROL DE SCADA"
							sprintf(EvtToGSM.Message,
									"Command ignored. Pump under SCADA control");
						}
						EvtToGSM.Src = sendSMS;
						EvtToGSM.Val = newParameter.sender;
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
					} else {
						Var.rst_totalizer = newParameter.value == 1;
						if (Var.rst_totalizer) {
							Var.Total_gearmot_turns = 0;
							//Almacenar en EEPROM el valor del totalizador
							guardar_eeprom((char*) &Var.Total_gearmot_turns,
							OFF_TOTALIZER,
							LEN_TOTALIZER);
						}
					}
				}
				break;

			default:
				break;
			}

			if (newParameter.parameter == SP_CF)
				/*Actualizar los valores de SETTINGS*/
				mqtt_st = mqtt_ev_publish_sett;
			else
				/*Actualizar los valores de CTRLS*/
				mqtt_st = mqtt_ev_publish_commands;

			xQueueSend(queMQTT_ev, &mqtt_st, 0);

			if (param_ok) {

				calcula_VPR(&sp_pump);

				maxFlowRate = calculate_MaxFlowRate(&sp_pump);
				if (newParameter.parameter != SP_THRESH)
				{
					sp_pump.Threshold = calcula_FlowRate(&sp_pump, 500);
				}

				if ((sp_pump.PumpRate < sp_pump.Threshold) && (sp_pump.PumpRate > 0)) {
					Memo_1 = 0;
					Chip_TIMER_Reset(TIMER_0);
					Chip_TIMER_ClearMatch(TIMER_0, 0);
					Chip_TIMER_ClearMatch(TIMER_0, 1);
					Chip_TIMER_SetMatch(TIMER_0, 0,
							(sp_pump.PumpRate / (float) sp_pump.Threshold)
									* (float) (sp_pump.Cycle) * 60 * 100e3);
					Chip_TIMER_SetMatch(TIMER_0, 1, sp_pump.Cycle * 60 * 100e3);
					Chip_TIMER_MatchEnableInt(TIMER_0, 0); //Habilita interr por TCYCLE y TON
					Chip_TIMER_MatchEnableInt(TIMER_0, 1); //Habilita interr por TCYCLE

					MotorCycle = TRUE;
					sp_pump.MotorRPM = calculate_RPM(&sp_pump, sp_pump.Threshold);

				} else if (sp_pump.PumpRate > 0) {
					Chip_TIMER_ClearMatch(TIMER_0, 0);
					Chip_TIMER_ClearMatch(TIMER_0, 1);
					Chip_TIMER_MatchDisableInt(TIMER_0, 0); //Deshabilita int por TCYCLE y TON
					Chip_TIMER_MatchDisableInt(TIMER_0, 1); //Deshabilita int por TCYCLE
					MotorCycle = TRUE;
					sp_pump.MotorRPM = calculate_RPM(&sp_pump, sp_pump.PumpRate);
				}

				//Almacenar en EEPROM los cambios en SP
				guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
				guardar_eeprom((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
			}
		}
	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por el TMR 0*/
void TMR_HANDLER_NAME_0(void) {
	uint32_t Temp;

	if (Chip_TIMER_CapturePending(TIMER_0, 0)) { //Pulso de Motor 1
		Chip_TIMER_ClearCapture(TIMER_0, 0);
		if (Elap_pulses_1 == 0) {
			Elap_pulses_1 = sp_pump.EPPR - 1;
			Elap_Turns_1++; //Una vuelta mas...

			Elap_Turns_Total++;
			if (Elap_Turns_Total == sp_pump.GR) {
				if (!Var.rst_totalizer) {
					Var.Total_gearmot_turns++;
				}
				Elap_Turns_Total = 0;
			}

			Temp = Chip_TIMER_ReadCapture(TIMER_0, 0);
			if (Temp > Memo_1) {
				ReadCapt_1 = Temp - Memo_1;
				Memo_1 = Temp;
			} else {
				Memo_1 = Temp;
			}
		} else {
			Elap_pulses_1--;
		}
	}

	if (Chip_TIMER_MatchPending(TIMER_0, 0)) { //Fin de TON
		Chip_TIMER_ClearMatch(TIMER_0, 0);
		MotorCycle = false;
	}
	if (Chip_TIMER_MatchPending(TIMER_0, 1)) { //Fin de TCYCLE
		Chip_TIMER_ClearMatch(TIMER_0, 1);
		MotorCycle = true;
	}
}

void Create_MOTOR_CTRL_task(void) {
	xTaskCreate(vMOTOR_CTRL_Task, (char*) "MOTOR CTRL TSK", 200, NULL,
	Prio_MOTOR_CTRL_Task, &tsk_motor_control_handler);
}

