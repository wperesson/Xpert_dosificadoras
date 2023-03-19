/*
 * Tsk_RS485.c
 *
 *  Created on: 08/11/2019
 *  Author: Walter Peresson
 */

/* FreeRTOS.org includes. */
#include <ATV312HU11.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"
#include "timers.h"
/* Librerias de C includes. */
#include "Definiciones.h"
#include "uart.h"
#if ATV312
typedef enum {
	MOD_tnk_HorCyl_diam = 0,
	MOD_tnk_HorCyl_lgt = 2,
	MOD_tnk_VertCyl_diam = 4,
	MOD_tnk_VertCyl_hgt = 6,
	MOD_tnk_Rect_lgt = 8,
	MOD_tnk_Rect_wid = 10,
	MOD_tnk_Rect_hgt = 12,
	MOD_tnk_Density = 14,
	MOD_tnk_Sensor_hgt = 16,
	MOD_tnk_shape = 18,

} MOD_Hold_Reg_Add_Type;		//MODBUS REGISTER RELATIVE ADDRESS

#define SL_VAR_ID	0xAC		//Slave ID variador

#define EXIT_TIME	(10 * 60 * 1000)

#define UART				LPC_UART0
#define UART_IRQ_SELEC		UART0_IRQn
#define UART_HANDLER_NAME 	UART0_IRQHandler

#define INPUT_REG_ARR_SIZE					54//OFF_FLOAT_TOTAL_PUMPD+4
#define HOLD_REG_ARR_SIZE_B1					88//OFF_FLOAT_TOTAL_PUMPD+4

#define MBTXEN_ON 	WHART_CD_ON
#define MBTXEN_OFF 	WHART_CD_OFF
#define MBRXEN_ON	WHART_RT_ON
#define MBRXEN_OFF	WHART_RT_OFF
#define STOP_ON		DO_O1_ON//DO_RL1_OFF
#define STOP_OFF	DO_O1_OFF//DO_RL1_ON

bool CyPumpOn;
StreamBufferHandle_t strMsgFromATV12;
xTimerHandle TimerCRL, TimmerTout;

uint16_t T_ON, T_CYCLE;

bool Send_ToVAR(uint8_t Fun, uint16_t Add, uint16_t Data) {
	uint8_t data[15], *ptData;
	uint32_t Len = 0;
	ptData = &data[0];
	VSD_bomba1.LastAddTX = Add;
	data[0] = VSD_bomba1.SLID;
	VSD_bomba1.LastOper = Fun;
	data[1] = Fun;
	data[2] = (Add >> 8) & 0xff;
	data[3] = Add & 0xff;
	data[4] = (Data >> 8) & 0xff;
	data[5] = Data & 0xff;
	MBCrc16(data, 6);
	Len = 7;
	while ((xQueueSend(queMsgToATV12, ptData++, 10) != errQUEUE_FULL)
			&& (Len > 0)) {
		Len--;
	}
	ulTaskNotifyTake(TRUE, portMAX_DELAY);
	if (VSD_bomba1.Rx_OK != 1) {

		VSD_bomba1.Status &= FAULT_MASK;
		xQueueReset(queMsgToATV12);
		xTaskNotifyGive(tsk_motor_elec_tx_handler);
		return false;
	} else {
		VSD_bomba1.Rx_OK = 0;
		return true;
	}
}
/*
 * Calcula el Volumen Per Revolution en la unidad elegida: Lit o Gal
 */
void ME_calcula_VPR(SP_Pump_Type *sp) {

	if ((SP.flow_unit == Galons_hour) || (sp->flow_unit == Galons_day)) {
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
float ME_calculate_MaxFlowRate(SP_Pump_Type *sp) {
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
float ME_calculate_RPM(SP_Pump_Type *sp, float pumprate) {

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
float ME_calcula_FlowRate(SP_Pump_Type *sp, float motorRPM) {

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

void init_uart_MasterMB(LPC_USART_T *uart) {

	MBTXEN_OFF;
	MBRXEN_OFF;
	// UART_1: COMUNICACION RS485 //////////////////
	Chip_UART_Init(uart);
	Chip_UART_SetBaud(uart, 19200);
	Chip_UART_ConfigData(uart,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(uart,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3);

	//Seteo el delay en periodos del baud clock
	Chip_UART_SetRS485Delay(uart, 10);

	//Activo la direccion de control del puerto automática
	Chip_UART_SetRS485Flags(uart,
	UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_SEL_DTR | UART_RS485CTRL_OINV_1);

	Chip_UART_TXEnable(uart);

	/* Enable UART interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(UART_IRQ_SELEC);
	MBTXEN_ON;
	Chip_UART_IntEnable(uart, UART_IER_RBRINT);
}

uint32_t wait_Byte(char *data, TickType_t delay) {

	TickType_t _delay = delay;
	uint32_t ReadB, ReadC;

	ReadB = 0;
	ReadC = 0;
	do {
		ReadC = xStreamBufferReceive(strMsgFromATV12, data + ReadB,
		BUF485_SIZE, _delay);
		ReadB += ReadC;

		/* Delay para esperar por si el UART recibe una interrupción por
		 * una cadena de bytes que alcanzaron el trigger level u otro byte
		 * que llego y se activo el Character Timeout Indicator que es 3.5 a 4.5
		 * veces el tiempo de cada caracter.
		 *
		 * delay = (10 bits /9600 bps) x TRIGGER LEVEL*/
		_delay = 15;

	} while (ReadC && (ReadB < BUF485_SIZE - 16));
	return ReadB;

}
/*-----------------------------------------------------------
 *
 */
static portTASK_FUNCTION(vCommMOTOR_ELEC_Tx, pvParameters) {

	uint8_t data, i;
	TickType_t delay;
	char Buf485_Rx[BUF485_SIZE]__attribute__ ((aligned (32)));
	MB_Function_Code Oper;
	static uint16_t ReadB, ReadC;

	strMsgFromATV12 = xStreamBufferCreate(BUF485_SIZE, 1);
	/*Inicializo el puerto */
	init_uart_MasterMB(UART);
	VSD_bomba1.SLID = 14;
	Create_CommMOTOR_ELEC_Ctrl_Task();

	while (1) {
		/*Espero el primer dato*/
		delay = portMAX_DELAY;
		while (xQueueReceive(queMsgToATV12, &data, delay) == pdPASS) {
			/*Hasta que se acabe la cola debo leer con cero delay*/
			delay = 0;
			while ((Chip_UART_ReadLineStatus(UART) & UART_LSR_THRE) == 0) {
				/*Espero que la interrupción por THRE me avise que puedo seguir*/
				ulTaskNotifyTake(TRUE, portMAX_DELAY);
			}
			/*Enciendo el TXEN*/
			MBTXEN_ON;
			MBRXEN_OFF;
			/*Ahora si, envío un byte*/
			Chip_UART_SendByte(UART, data);
			/*Habilito la int por THRE*/
			Chip_UART_IntEnable(UART, UART_IER_THREINT);
		}
		ulTaskNotifyTake(TRUE, portMAX_DELAY);
		/*Apago el TXEN*/
		MBTXEN_OFF;
		MBRXEN_ON;
		memset(Buf485_Rx, 0xff, BUF485_SIZE); //Preparo el buffer donde voy a alojar los datos.
		delay = portMAX_DELAY;
		ReadB = 0;
		do {
			ReadC = wait_Byte(Buf485_Rx, delay);
			ReadB += ReadC;
			delay = 20;
		} while (ReadC);

		//Trampa para acomodar la cadena recibida
		if (Buf485_Rx[0] == 0) {
			for (i = 0; i < ReadB; i++) {
				Buf485_Rx[i] = Buf485_Rx[i + 1];
			}
			ReadB--;
		}

		if (Check_CRC(Buf485_Rx, ReadB/*Index*/)) { //Check CRC

			if (Buf485_Rx[0] == VSD_bomba1.SLID) {
				if (VSD_bomba1.LastOper == Buf485_Rx[1]) {
					Oper = Buf485_Rx[1];
					switch (Oper) {
					case FC03_Read_Hold_Reg:

						switch (VSD_bomba1.LastAddTX) {
						case CMD:
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						case LFRD:
							VSD_bomba1.RPMset = (Buf485_Rx[3] << 8)
									+ Buf485_Rx[4];
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						case RFRD:
							VSD_bomba1.RPMOut = (Buf485_Rx[3] << 8)
									+ Buf485_Rx[4];
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						case LFT:
							VSD_bomba1.FaultCode = Buf485_Rx[4];
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						case ETA:
							VSD_bomba1.Status = Buf485_Rx[4];
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						}
						break;
					case FC06_Write_Sing_Reg:
						switch (VSD_bomba1.LastAddTX) {
						case CMD:
							xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
							VSD_bomba1.Rx_OK = 1;
							break;
						case LFRD:
							if ((int) (sp_pump.MotorRPM)
									== ((Buf485_Rx[4] << 8) + Buf485_Rx[5])) {
								VSD_bomba1.RPMOut = (Buf485_Rx[4] << 8)
										+ Buf485_Rx[5];
								xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);
								VSD_bomba1.Rx_OK = 1;
							}
							break;
						}
						break;
					default:
						break;
					}
				}
				xTimerReset(TimmerTout, 0);
			}
		}
	}
}

/*-----------------------------------------------------------
 *
 */
void vTouTmr_Callback(xTimerHandle TimmerTout) {
	SetAlarm(Al_alarma_motor_atascado_B1);
	VSD_bomba1.Status &= FAULT_MASK;
	xTaskNotifyGive(tsk_motor_elec_Ctrl_handler);

}

/*-----------------------------------------------------------
 *
 */
void vMBTmr_Callback(xTimerHandle TimerCRL) {
	static uint16_t Estado = 0, T = 0;
	if ((T_CYCLE > 0) && ((VSD_bomba1.Status & FAULT_MASK) != FAULT_MASK)
			&& (!GetAlarm(Al_alarma_bajo_nivel_T1))) {
		if (!PumpOn) {
			Estado = 0;
			T = 0;
		}
		switch (Estado) {
		case 0:
			if (T >= T_ON) {
				Estado = 1;
				CyPumpOn = false;
				T++;
			} else {
				T++;
				CyPumpOn = true;
			}
			break;
		case 1:
			if (T >= T_CYCLE) {
				Estado = 0;
				CyPumpOn = true;
				T = 1;
			} else {
				T++;
				CyPumpOn = false;
			}
			break;
		}
	} else {
		T = 0;
//		CyPumpOn= false;
		Estado = 0;
	}
}

/*-----------------------------------------------------------
 *
 */
static portTASK_FUNCTION(vCommMOTOR_ELEC_Rx, pvParameters) {

	Create_CommMOTOR_ELEC_Tx_Task();
	Create_CommMOTOR_ELEC_Ctrl_Task();
	VSD_bomba1.SLID = 14;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_RATE_MS));
	}
}

/*-----------------------------------------------------------
 *
 */
static portTASK_FUNCTION(vCommMOTOR_ELEC_Ctrl, pvParameters) {

	EvtToMOB_Type EvtToGSM;
	static uint8_t Ultima_Hora;
	Parameter_upd_Type newParameter;
	bool param_ok = false;
	Bool newPumpOn;
	uint8_t cont_ovr;
	static float  tempfloat, tempfloat2;
	Flow_UNITS_Typ mem_SP_flow_unit;
	mqtt_event_type mqtt_st;
	uint8_t Stat = 0;
	uint32_t start = xTaskGetTickCount(), difTyck;

	//Calculo de volumen p revolucion y masx flow

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump, sp_pump.PumpRate);
	TimerCRL = xTimerCreate("Tmr_Control", pdMS_TO_TICKS(1000), pdTRUE,
			(void*) 0, vMBTmr_Callback);
	TimmerTout = xTimerCreate("Tmr_Modbus", pdMS_TO_TICKS(3000), pdTRUE,
			(void*) 0, vTouTmr_Callback);
	xTimerStart(TimmerTout, 0);
	ME_calcula_VPR(&sp_pump);
	sp_pump.maxFlow = ME_calculate_MaxFlowRate(&sp_pump);
	if ((sp_pump.PumpRate < sp_pump.Threshold) && (sp_pump.PumpRate > 0)) {
		T_ON = (sp_pump.PumpRate / sp_pump.Threshold) * sp_pump.Cycle * 60;
		T_CYCLE = sp_pump.Cycle * 60;
		CyPumpOn = false;
		sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump, sp_pump.Threshold);
		xTimerStart(TimerCRL, 0);

	} else if (sp_pump.PumpRate > 0) {

		T_CYCLE = 0;
		CyPumpOn = true;
		sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump, sp_pump.PumpRate);
		xTimerStop(TimerCRL, 0);
	}
	vTaskDelay(100);
	Send_ToVAR(3, ETA, 1);
	while ((VSD_bomba1.Status & FAULT_MASK) == FAULT_MASK) //Se veriica si existe alguna falla
	{
		Stat = 0;
		Send_ToVAR(6, CMD, 128);
		vTaskDelay(10);
		Send_ToVAR(3, ETA, 1);
		vTaskDelay(10);
	}
	snd_head_flow_sett(&newParameter, !(Var.DI & 0x01), PUMP_ON,src_self_generated, 0);//Encendido apenas inicia el sistema
	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_RATE_MS));
		Send_ToVAR(3, ETA, 1);
		if ((VSD_bomba1.Status & FAULT_MASK) == FAULT_MASK) //Se veriica si existe alguna falla
		{
			Stat = 0;
			Send_ToVAR(6, CMD, 128);
			SetAlarm(Al_alarma_motor_atascado_B1);
		}

		vTaskDelay(10);
		switch (Stat) {
		case 0: //Verificacion  de estado
			Send_ToVAR(6, CMD, 0);
			vTaskDelay(100);
			Send_ToVAR(3, LFT, 1);
			Var.MotorRPM = 0;
			if (VSD_bomba1.FaultCode != nOF) {
				if ((VSD_bomba1.Status & FAULT_MASK) != FAULT_MASK) //me quedo en este punto hasta que se levante el error
				{
					ClearAlarm(Al_alarma_motor_atascado_B1);
					if (PumpOn && CyPumpOn) {
						STOP_OFF;
						Stat = 1;
					} else {
						if (global_Override) {
							Stat = 4;
							STOP_OFF;
						}
					}
				}

			} else if (PumpOn && CyPumpOn) {
				STOP_OFF;
				Stat = 1;
			} else {
				if (global_Override) {
					Stat = 4;
					STOP_OFF;
				}
			}
			break;
		case 1:
			ME_calcula_VPR(&sp_pump);
			sp_pump.maxFlow = ME_calculate_MaxFlowRate(&sp_pump);
			if ((sp_pump.PumpRate < sp_pump.Threshold)
					&& (sp_pump.PumpRate > 0)) {
				T_ON = (sp_pump.PumpRate / sp_pump.Threshold) * sp_pump.Cycle
						* 60;
				T_CYCLE = sp_pump.Cycle * 60;
				xTimerStart(TimerCRL, 0);
				sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump,
						sp_pump.Threshold);

			} else if (sp_pump.PumpRate > 0) {
				T_CYCLE = 0;
				sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump, sp_pump.PumpRate);
				xTimerStop(TimerCRL, 0);
			}
			VSD_bomba1.RPMset = sp_pump.MotorRPM;
			Send_ToVAR(6, LFRD, VSD_bomba1.RPMset);
			vTaskDelay(10);
			Send_ToVAR(6, CMD, 6);
			vTaskDelay(10);
			Send_ToVAR(6, CMD, 7);
			vTaskDelay(10);
			Send_ToVAR(6, CMD, 15);
			vTaskDelay(10);
			Stat = 2;
			break;
		case 2: //Relevamiento de parametro RPMOut
			Send_ToVAR(3, RFRD, 1);
			Var.MotorRPM = VSD_bomba1.RPMOut;
			if (VSD_bomba1.RPMset != sp_pump.MotorRPM) {
				VSD_bomba1.RPMset = sp_pump.MotorRPM;
				Send_ToVAR(6, LFRD, VSD_bomba1.RPMset);
				vTaskDelay(10);
			}
			if ((!PumpOn) || (!CyPumpOn)) {
				Stat = 3;
			}
			break;
		case 3: //Stop
			Var.MotorRPM = 0;
			Send_ToVAR(6, CMD, 7);
			vTaskDelay(10);
			Send_ToVAR(6, CMD, 6);
			vTaskDelay(10);
			STOP_ON;
			Stat = 0;
			break;
		case 4:
			sp_pump.MotorRPM = 50;
			Send_ToVAR(6, LFRD, sp_pump.MotorRPM);
			vTaskDelay(50);
			Send_ToVAR(6, CMD, 6);
			vTaskDelay(50);
			Send_ToVAR(6, CMD, 7);
			vTaskDelay(50);
			Send_ToVAR(6, CMD, 15);
			vTaskDelay(50);
			cont_ovr = 0;
			while (global_Override) { //&& cont_ovr < 15) {
				Send_ToVAR(3, RFRD, 1);
				vTaskDelay(10);
				cont_ovr++;
			}
			//global_Override = false;
			Stat = 3;
			break;
		}

		//Control de caudal
		difTyck = xTaskGetTickCount() - start;
		tempfloat2 += (float) Var.MotorRPM * difTyck
				/ (60 * (float) sp_pump.GR * 1000.0);
		Var.Total_gearmot_turns = tempfloat2;
		start = xTaskGetTickCount();
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

		if (VSD_bomba1.RPMOut > 0) {
			Var.PumpRate = ME_calcula_FlowRate(&sp_pump, Var.MotorRPM);
		} else
			Var.PumpRate = 0;

		//Fin control de caudal
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
				if ((VSD_bomba1.Status & FAULT_MASK) == FAULT_MASK)	//me quedo en este punto hasta que se levante el error
				{
					break;
				}

				/*Por defecto leemos el estado de PumpOn. */
				newPumpOn = PumpOn;
				/*
				 * MODO LOCAL
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
							newPumpOn = newParameter.value == 1;
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

				if (newParameter.value > sp_pump.maxFlow) { //======== Controlo límite superior ===============

					sp_pump.PumpRate = sp_pump.maxFlow;

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
								"Flow: %.1f %s", sp_pump.PumpRate,
								&Var.flow_unit);
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
				tempfloat = 0.5 * ME_calculate_MaxFlowRate(&sp_pump);
				if (newParameter.value <= tempfloat) {
					//Chequeo que el valor de Threshold no sea menor a 200 RPM del motor
					tempfloat = ME_calculate_RPM(&sp_pump, newParameter.value);

					if ((tempfloat >= (sp_pump.maxSpd*0.25))) {
						sp_pump.Threshold = (uint16_t) newParameter.value;
					} else {
						sp_pump.Threshold = ME_calcula_FlowRate(&sp_pump, (sp_pump.maxSpd*0.25));
					}
				} else
					sp_pump.Threshold = 0.5
							* ME_calculate_MaxFlowRate(&sp_pump);
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
			case SP_UNITS://TODO:Tener en cuenta que el cambio de variables puede llegar a tener un error de hasta 10L lahacer un cambio y volver a la unudad original
				//Almaceno temporalmente el valor de sp_pump.vol_unit asi puedo convertir el sp_pump.Pumprate segun las nuevas unidades
				mem_SP_flow_unit = SP.flow_unit;

				//Chequeo el valor de las unidades sea una de las posibles
				if ((newParameter.value > 0) && (newParameter.value <= 4)) {
					SP.flow_unit = newParameter.value;

					read_flow_Vol_unit(&SP, &Var);

					//Actualizo el VPR ya que cambian las Undades
					ME_calcula_VPR(&sp_pump);

					//Pongo el nuevo Threshold en un valor acorde al motor, 500RPM
					sp_pump.Threshold = ME_calcula_FlowRate(&sp_pump, 500);

					//Calculo el sp_pump.PumpRate de acuerdo a las nuevas unidades
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
				//Modo local
				if (Var.local_remoto == TRUE) {
					if (newParameter.src == src_front_panel) {
						Var.Total_gearmot_turns = 0;
						tempfloat2 = 0;
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

				//Modo remoto
				else {
					if (newParameter.src == src_sms) {
						if (global_Scada_Control_cnt == 0) {
							//"SIN CONTROL DE SCADA"
							Var.Total_gearmot_turns = 0;
							tempfloat2 = 0;
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
							tempfloat2 = 0;
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

				ME_calcula_VPR(&sp_pump);

				sp_pump.maxFlow = ME_calculate_MaxFlowRate(&sp_pump);
				if ((sp_pump.PumpRate < sp_pump.Threshold)
						&& (sp_pump.PumpRate > 0)) {
					T_ON = (sp_pump.PumpRate / sp_pump.Threshold)
							* sp_pump.Cycle * 60;
					T_CYCLE = sp_pump.Cycle * 60;
					CyPumpOn = false;
					sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump,
							sp_pump.Threshold);
					xTimerStart(TimerCRL, 0);

				} else if (sp_pump.PumpRate > 0) {

					T_CYCLE = 0;
					CyPumpOn = true;
					sp_pump.MotorRPM = ME_calculate_RPM(&sp_pump,
							sp_pump.PumpRate);
					xTimerStop(TimerCRL, 0);
				}

				//Almacenar en EEPROM los cambios en SP o sp_pump
				if (newParameter.parameter == SP_UNITS)
					guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
				else
					guardar_eeprom((char*) &sp_pump, OFF_sp_pump,
							sizeof(sp_pump));
			}
		}
	}
}
/*-----------------------------------------------------------*/
/* Atención de INT por el UART1 - Comunicación MODBUS SERVER + DEBUGGING*/
void UART_HANDLER_NAME(void) {
#define LOC_BUFF_SZ	30

	static signed portBASE_TYPE xHigherPriorityTaskWoken =
	pdFALSE;
	static uint32_t Interr;
	static uint8_t ReadB;
	static uint8_t Buffer[LOC_BUFF_SZ];

	Interr = Chip_UART_ReadIntIDReg(UART);
	if ((Interr & UART_IIR_INTID_RDA) || (Interr & UART_IIR_INTID_CTI)) {
		ReadB = Chip_UART_Read(UART, Buffer, LOC_BUFF_SZ);

		if (ReadB > 0) {
			xStreamBufferSendFromISR(strMsgFromATV12, Buffer, ReadB,
					&xHigherPriorityTaskWoken);
		}

	} else if (Interr & UART_IIR_INTID_THRE) {
		Chip_UART_IntDisable(UART, UART_IER_THREINT);
		vTaskNotifyGiveFromISR(tsk_motor_elec_tx_handler,
				&xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_CommMOTOR_ELEC_Rx_Task(void) {
	xTaskCreate(vCommMOTOR_ELEC_Rx, (char*) "Motor Elec Rx TSK", 10,
	NULL,
	Prio_CommMOTOR_ELEC_Rx_Task, &tsk_motor_elec_rx_handler);
}
void Create_CommMOTOR_ELEC_Tx_Task(void) {
	xTaskCreate(vCommMOTOR_ELEC_Tx, (char*) "Motor Elec Tx TSK", 220,
	NULL,
	Prio_CommMOTOR_ELEC_Tx_Task, &tsk_motor_elec_tx_handler);
}
void Create_CommMOTOR_ELEC_Ctrl_Task(void) {
	xTaskCreate(vCommMOTOR_ELEC_Ctrl, (char*) "Motor Elec Ctrl TSK", 230,
	NULL,
	Prio_CommMOTOR_ELEC_Ctrl_Task, &tsk_motor_elec_Ctrl_handler);
}

#endif
