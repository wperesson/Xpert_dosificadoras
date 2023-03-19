/*
 * Task_CommMB_HMI.c
 *
 *  Created on: 30/01/2014
 *      Author: Lisa
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"

//todo: Quitar los dos siguientes includes
#include "IHart_cfg.h"
#include "IHart_DataAcquisition.h"

/* Librerias de C includes. */
#include "Definiciones.h"
#include "SIN_V20.h"
//#include "rtc_17xx_40xx.h"
#if USE_SIM231_TP_SCREEN
#define T15			750E-6	//750 uS
#define T35			1.75E-3//1.75 mS

#define UART 			LPC_UART2
#define TIMER 			LPC_TIMER2
#define TMR_IRQ_SELEC 	TIMER2_IRQn
#define UART_IRQ_SELEC	UART2_IRQn
#define UART_HANDLER_NAME 	UART2_IRQHandler
#define TMR_HANDLER_NAME 	TIMER2_IRQHandler
#define BAUDRATE			115200

//#define USE_BUFFER_ADDRESS_TO_DEBUG

Status Send_UART(uint32_t Len, uint32_t *PtData) {
	Status Stat;

	Chip_UART_IntEnable(UART, UART_IER_THREINT);
	if (Len == Chip_UART_SendBlocking(UART, PtData, Len)) {
		Stat = SUCCESS;
	} else {
		Stat = ERROR;
	}
	ulTaskNotifyTake(TRUE, portMAX_DELAY);
	return Stat;
}

uint32_t ToBf(char *Ptr, uint32_t Size, char *Buffer) {
	char i;

	*(Buffer + 2) = Size; //bytes a transmitir
	for (i = 0; i < Size; i++) {
		*(Buffer + 3 + i) = *(Ptr + Size - 1 - i);
	}
	MBCrc16((uint8_t*) Buffer, Size + 3);
	return Size + 5;
}

uint32_t FromBuffer(char *Ptr, uint32_t Size, uint8_t *Buffer) {
	char i;

	*(Buffer + 2) = Size; //bytes a transmitir
	for (i = 0; i < Size; i++) {
		*(Buffer + 3 + i) = *(Ptr + Size - 1 - i);
	}
	MBCrc16(Buffer, Size + 3);
	return Size + 5;
}

uint32_t unpack_ascii(char *Src, char *Dest, uint8_t Len) {
	uint8_t i, j;
	uint32_t Val;
	char *ptr;

	j = 0;

	while (3 * j < Len) {
		ptr = ((char*) &Val) + 2;
		Val = 0;
		for (i = 0; i < 3; i++) {
			*ptr-- = *Src++;
		}

		for (i = 0; i < 4; i++) {
			if (Val & 0x20) //El 6to bit es uno?
				*(Dest + 3 - i) = (0x3f & Val);
			else
				//El 6to bit es cero
				*(Dest + 3 - i) = 0x40 | (0x3f & Val);
			Val = Val >> 6;
		}
		Dest += 4;

		j++;
	}
	return j * 4;
}

/*-----------------------------------------------------------
 * 	 */
static portTASK_FUNCTION(vCommMB_HMI, pvParameters) {

	uint32_t Len;
	char Buff[100];
#if USE_BUFFER_ADDRESS_TO_DEBUG
	char buff_address[100]; //temporal
	uint8_t k = 0;
#endif
	uint8_t ReadB;
	Modb_RegAdd Address;
	MB_Function_Code Oper;
	EvtToMOB_Type EvtToGSM;
	EvtToHART_Type EvtToHART;
	portTickType xLastWakeTime;
	static uint8_t An_Idx = 0, CharTemp, i;
	TickType_t delay;
	uint16_t FactResetCode = 0;
	uint16_t shortTemp;
	float floatTemp;
	double double_temp;
	gen_ctrl_ev_Type EventoGen;
	struct tm *timeinfo;
//	static S_Typ mem_SP_vol_unit;
//	static Flow_UNITS_Typ mem_SP_flow_unit;
	Parameter_upd_Type newParameter;
	mqtt_event_type mqtt_st;

	xLastWakeTime = xTaskGetTickCount();

	char *Ptr_Char;

	semExpT15_HMI = xSemaphoreCreateBinary();
	semExpT35_HMI = xSemaphoreCreateBinary();

	/* Enable timer clock */
	Chip_TIMER_Init(TIMER);
	/* Timer rate is system clock rate */
	Chip_Clock_GetSystemClockRate();
	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(TIMER);

	Chip_TIMER_SetMatch(TIMER, 0, 36000); //750us t15
	Chip_TIMER_SetMatch(TIMER, 1, 84000); //1.75ms t35

	Chip_TIMER_MatchEnableInt(TIMER, 0);
	Chip_TIMER_MatchEnableInt(TIMER, 1);

	Chip_TIMER_ResetOnMatchEnable(TIMER, 1);

	// UART_2: COMUNICACION CON HMI //////////////////
	Chip_UART_Init(UART);
	Chip_UART_SetBaud(UART, BAUDRATE);
	Chip_UART_ConfigData(UART,
	UART_LCR_WLEN8 | UART_LCR_SBS_2BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(UART,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV1);

	Chip_UART_TXEnable(UART);

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TMR_IRQ_SELEC);
	NVIC_SetPriority(TMR_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 3);
//	NVIC_EnableIRQ(TMR_IRQ_SELEC);
	/* Enable UART2 interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 3);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	Chip_UART_IntEnable(UART, UART_IER_RBRINT);
	xSemaphoreTake(semExpT15_HMI, 0);
	xSemaphoreTake(semExpT35_HMI, 0);

	upd_next_gen_start_data = false;
//	mem_SP_vol_unit = SP.vol_unit;
//	mem_SP_flow_unit = SP.flow_unit;

	while (1) {
		//RECEPTION
		ReadB = 0;
		memset(Buff, 0xff, 100);
		delay = portMAX_DELAY;
		while (xQueueReceive(queMsgFromHMI, (void*) &Buff[ReadB], delay)
				!= pdFAIL && ReadB < 100) {
			ReadB++;
			delay = 2;
		};

		//Check CRC
		if (Check_CRC(Buff, ReadB)) {
//++++++++++		SB <--- HMI		+++++++++++++++++++++++++++++++++
			if ((Buff[0] == HMI_SL_VAR_ID) || (Buff[0] == HMI_SL_SP_ID)) {

				Address = 0;
				Address |= (Buff[2] << 8) | Buff[3];
				Oper = Buff[1];

				if (Address == Var_DI) {
					Address = Var_DI;
				}
#if USE_BUFFER_ADDRESS_TO_DEBUG
				buff_address[k++] = Address;
				if (k > 100)
					k = 0;
#endif
				switch (Oper) {
				case FC01_Read_Coils:
				case FC02_Read_Disc_Inp:
				case FC03_Read_Hold_Reg:
				case FC04_Read_Inp_Reg:
					switch (Address) {
					case PUMP_ON:
						Len = ToBf((char*) &PumpOn, sizeof(PumpOn), Buff);
						break;
					case OVERRIDE:
						Len = ToBf((char*) &global_Override,
								sizeof(global_Override), Buff);
						break;
					case GEN_START:
						Len = ToBf((char*) &gen.contact, sizeof(gen.contact),
								Buff);
						break;
					case RST_TOTAL:
						Len = ToBf((char*) &Var.rst_totalizer,
								sizeof(Var.rst_totalizer), Buff);
						break;
					case SP_RS485_WL:
						Len = ToBf((char*) &SP.RS485_WL, sizeof(SP.RS485_WL),
								Buff);
						break;
					case SP_FACT_RESET:
						Len = ToBf((char*) &FactResetCode,
								sizeof(FactResetCode), Buff);
						break;
					case SP_SET_FACT_RESET_DATA:
						Len = ToBf((char*) &FactResetCode,
								sizeof(FactResetCode), Buff);
						break;
					case SP_UseGpsTime:
						Len = ToBf((char*) &SP.UseGpsTime,
								sizeof(SP.UseGpsTime), Buff);
						break;
					case GEN_PERIOD:
						Len = ToBf((char*) &gen.period, sizeof(gen.period),
								Buff);
						break;
					case GEN_LOW_BATT_TIMEOUT:
						Len = ToBf((char*) &gen.low_batt_timeout,
								sizeof(gen.low_batt_timeout), Buff);
						break;
					case GEN_TEST_TIMEOUT:
						Len = ToBf((char*) &gen.test_timeout,
								sizeof(gen.test_timeout), Buff);
						break;
					case GEN_LOW_BATT_LIM:
						Len = ToBf((char*) &gen.low_batt_limit,
								sizeof(gen.low_batt_limit), Buff);
						break;
					case GEN_RUNNING:
						Len = ToBf((char*) &gen.running, sizeof(gen.running),
								Buff);
						break;
					case Var_DI:
						shortTemp = Var.DI;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case Var_ALARM:
						Len = ToBf((char*) &RegAlarmas, sizeof(RegAlarmas),
								Buff);
						break;
					case Var_AN_SIG:
						Len = ToBf((char*) &Var.an[An_Idx].Signal,
								sizeof(Var.an[An_Idx].Signal), Buff);
						break;
					case Var_AN_ENG:
						Len = ToBf((char*) &Var.an[An_Idx].Eng_Un,
								sizeof(Var.an[An_Idx].Eng_Un), Buff);
						break;
					case SP_AN_HS:
						Len = ToBf((char*) &SP.An[An_Idx].HS,
								sizeof(SP.An[An_Idx].HS), Buff);
						break;
					case SP_AN_LS:
						Len = ToBf((char*) &SP.An[An_Idx].LS,
								sizeof(SP.An[An_Idx].LS), Buff);
						break;
					case SP_AN_HE:
						Len = ToBf((char*) &SP.An[An_Idx].HE,
								sizeof(SP.An[An_Idx].HE), Buff);
						break;
					case SP_AN_LE:
						Len = ToBf((char*) &SP.An[An_Idx].LE,
								sizeof(SP.An[An_Idx].LE), Buff);
						break;
					case gps_fix:
						Len = ToBf((char*) &gps.gps_fix, sizeof(gps.gps_fix),
								Buff);
						break;
					case SP_PumpRate:
						Len = ToBf((char*) &sp_pump.PumpRate, sizeof(sp_pump.PumpRate),
								Buff);
						break;
					case SP_Motor_RPM:
						Len = ToBf((char*) &sp_pump.MotorRPM, sizeof(sp_pump.MotorRPM),
								Buff);
						break;
					case SP_PAGE:
						Len = ToBf((char*) &Act_Page, sizeof(Act_Page), Buff);
						break;
					case Var_Motor_Err:
						Len = ToBf((char*) &Var.curr_error,
								sizeof(Var.curr_error), Buff);
						break;
					case SP_PROP:
						Len = ToBf((char*) &sp_pump.PropGain, sizeof(sp_pump.PropGain),
								Buff);
						break;
					case SP_DER:
						Len = ToBf((char*) &sp_pump.DerGain, sizeof(sp_pump.DerGain),
								Buff);
						break;
					case SP_INT:
						Len = ToBf((char*) &sp_pump.IntGain, sizeof(sp_pump.IntGain),
								Buff);
						break;

						//todo: quitar todo esto de abajo. Lo dejo para mantener la letura desde la pantalla
//					case SP_PROP:
//					case SP_DER:
//					case SP_INT:
//						double_temp = 0;
//						Len = ToBf((char*) &double_temp, sizeof(double_temp),
//								Buff);
//						break;
					case SP_CF:
						Len = ToBf((char*) &sp_pump.CF, sizeof(sp_pump.CF), Buff);
						break;
					case SP_VPR:
						Len = ToBf((char*) &sp_pump.VPR, sizeof(sp_pump.VPR), Buff);
						break;
					case SP_PLG_SIZE:
						Len = ToBf((char*) &sp_pump.plg_size, sizeof(sp_pump.plg_size),
								Buff);
						break;
					case SP_PLG_STR:
						Len = ToBf((char*) &sp_pump.plg_str, sizeof(sp_pump.plg_str),
								Buff);
						break;
					case SP_HEADS:
						shortTemp = sp_pump.heads;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_UNITS:
						shortTemp = SP.flow_unit;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_GR:
						shortTemp = sp_pump.GR;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_EPPR:
						shortTemp = sp_pump.EPPR;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_MAXSPD:
						//Len = ToBf((char*) &sp_pump.nomSpd, sizeof(sp_pump.nomSpd), Buff);
						Len = ToBf((char*) &sp_pump.maxSpd, sizeof(sp_pump.maxSpd), Buff);
						break;
					case SP_CYCLE_T:
						Len = ToBf((char*) &sp_pump.Cycle, sizeof(sp_pump.Cycle), Buff);
						break;
					case SP_THRESH:
						Len = ToBf((char*) &sp_pump.Threshold, sizeof(sp_pump.Threshold),
								Buff);
						break;
					case SP_TankCap:
						Len = ToBf((char*) &sp_pump.TankCap, sizeof(sp_pump.TankCap),
								Buff);
						break;
					case SP_SpanTankCap:
						Len = ToBf((char*) &sp_pump.SpanTankCap,
								sizeof(sp_pump.SpanTankCap), Buff);
						break;
					case SP_ZeroTankCap:
						Len = ToBf((char*) &sp_pump.ZeroTankCap,
								sizeof(sp_pump.ZeroTankCap), Buff);
						break;
					case SP_SpanTankEng:
						Len = ToBf((char*) &sp_pump.SpanTankEng,
								sizeof(sp_pump.SpanTankEng), Buff);
						break;
					case SP_ZeroTankEng:
						Len = ToBf((char*) &sp_pump.ZeroTankEng,
								sizeof(sp_pump.ZeroTankEng), Buff);
						break;
					case SP_War_TankLevel:
						Len = ToBf((char*) &sp_pump.level_warning_limit_T1,
								sizeof(sp_pump.level_warning_limit_T1), Buff);
						break;
					case SP_AL_TankLevel:
						Len = ToBf((char*) &sp_pump.level_alarm_limit_T1,
								sizeof(sp_pump.level_alarm_limit_T1), Buff);
						break;
					case SP_RS485_BAUD:
						Len = ToBf((char*) &SP.RS485_BAUD,
								sizeof(SP.RS485_BAUD), Buff);
						break;
					case SP_RS485_SLID:
						Len = ToBf((char*) &SP.RS485_SLID,
								sizeof(SP.RS485_SLID), Buff);
						break;
					case SP_WHART_RATE:
						Len = ToBf((char*) &SP.WHART_RefRate,
								sizeof(SP.WHART_RefRate), Buff);
						break;
					case SP_WHART_NETID:
						Len = ToBf((char*) &SP.WHART_NetId,
								sizeof(SP.WHART_NetId), Buff);
						break;
					case Var_WHART_SSTR:
						Len = ToBf((char*) &Var.whart_sstr,
								sizeof(Var.whart_sstr), Buff);
						break;
					case Var_WHART_STAT:
						Len = ToBf((char*) &Var.whart_stat,
								sizeof(Var.whart_stat), Buff);
						break;
					case SP_PIN_ADMIN:
						Len = ToBf((char*) &PIN[0], sizeof(PIN[0]), Buff);
						break;
					case SP_PIN_USR1:
						Len = ToBf((char*) &PIN[1], sizeof(PIN[1]), Buff);
						break;
					case SP_PIN_USR2:
						Len = ToBf((char*) &PIN[2], sizeof(PIN[2]), Buff);
						break;
					case SP_GSM_REG_AUTH:
						Len = ToBf((char*) &SP.SP_GSM_REG_AUTH,
								sizeof(SP.SP_GSM_REG_AUTH), Buff);
						break;
					case Var_GSM_SSTRG:
						shortTemp = gsm.rssi;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_GSM_SEND_SMS:
						shortTemp = 0;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_TimeZone:
						shortTemp = SP.TimeZone;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;

					case Var_AN1:
						Len = ToBf((char*) &Var.an[0].Signal,
								sizeof(Var.an[0].Signal), Buff);
						break;
					case Var_AN2:
						Len = ToBf((char*) &Var.an[1].Signal,
								sizeof(Var.an[1].Signal), Buff);
						break;
					case Var_AN3:
						Len = ToBf((char*) &Var.an[2].Signal,
								sizeof(Var.an[2].Signal), Buff);
						break;
					case Var_AN4:
						Len = ToBf((char*) &Var.an[3].Signal,
								sizeof(Var.an[3].Signal), Buff);
						break;
					case Var_AN5:
						Len = ToBf((char*) &Var.an[4].Signal,
								sizeof(Var.an[4].Signal), Buff);
						break;
					case Var_AN6:
						Len = ToBf((char*) &Var.an[5].Signal,
								sizeof(Var.an[5].Signal), Buff);
						break;

					case Var_PumpRate:
						Len = ToBf((char*) &Var.PumpRate, sizeof(Var.PumpRate),
								Buff);
						break;
					case Var_Total_PUMPED:
						Len = ToBf((char*) &Var.Total_pumped,
								sizeof(Var.Total_pumped), Buff);
						break;
					case Var_Motor_RPM:
						Len = ToBf((char*) &Var.MotorRPM, sizeof(Var.MotorRPM),
								Buff);
						break;
					case Var_TankLevel:
						Len = ToBf((char*) &Var.tank_Lev, sizeof(Var.tank_Lev),
								Buff);
						break;
					case Var_TankVolume:
						Len = ToBf((char*) &Var.tank_vol, sizeof(Var.tank_vol),
								Buff);
						break;
					case Var_LinePress_Eng:
						Len = ToBf((char*) &Var.LinePress,
								sizeof(Var.LinePress), Buff);
						break;
					case Var_TankLevel_P:
						Len = ToBf((char*) &Var.tank_vol_perc,
								sizeof(Var.tank_vol_perc), Buff);
						break;
					case Var_TankLevel_Eng:
						Len =
								ToBf(
										(char*) &Var.an[SP.AnChannelAssig[Tank_Level]].Eng_Un,
										sizeof(float), Buff);
						break;

					case Var_VPannel:
						Len = ToBf((char*) &Var.VPanel, sizeof(Var.VPanel),
								Buff);
						break;
					case Var_CPannel:
						Len = ToBf((char*) &Var.CPanel, sizeof(Var.CPanel),
								Buff);
						break;
					case Var_VExtGen:
						Len = ToBf((char*) &Var.VExtGen, sizeof(Var.VExtGen),
								Buff);
						break;
					case Var_PPannel:
						Len = ToBf((char*) &Var.PPanel, sizeof(Var.PPanel),
								Buff);
						break;
					case Var_EPannel:
						Len = ToBf((char*) &Var.EPannel, sizeof(Var.EPannel),
								Buff);
						break;
					case Var_ChPannel:
						Len = ToBf((char*) &Var.ChPannel, sizeof(Var.ChPannel),
								Buff);
						break;
					case Var_VBatt:
						Len = ToBf((char*) &Var.VBatt, sizeof(Var.VBatt), Buff);
						break;
					case Lat:
						Len = ToBf((char*) &gps.Lat, sizeof(gps.Lat), Buff);
						break;
					case Lon:
						Len = ToBf((char*) &gps.Lon, sizeof(gps.Lon), Buff);
						break;
					case Alt:
						Len = ToBf((char*) &gps.Altitude, sizeof(gps.Altitude),
								Buff);
						break;
					case Speed_kph:
						Len = ToBf((char*) &gps.Speed_kph,
								sizeof(gps.Speed_kph), Buff);
						break;
					case SP_tnk_Shp:
						shortTemp = sp_pump.tnk_shp;
						Len = ToBf((char*) &shortTemp, sizeof(shortTemp), Buff);
						break;
					case SP_d1_tnk_VT_HT_diam_RT_width:
						Len = ToBf((char*) &sp_pump.tnk_d1_VT_HT_diam_RT_width,
								sizeof(sp_pump.tnk_d1_VT_HT_diam_RT_width), Buff);
						break;
					case SP_d2_tnk_VT_height_HT_lenght_RT_height:
						Len =
								ToBf(
										(char*) &sp_pump.tnk_d2_VT_height_HT_lenght_RT_height,
										sizeof(sp_pump.tnk_d2_VT_height_HT_lenght_RT_height),
										Buff);
						break;
					case SP_d3_tnk_RT_length:
						Len = ToBf((char*) &sp_pump.tnk_d3_RT_length,
								sizeof(sp_pump.tnk_d3_RT_length), Buff);
						break;
					case SP_tnk_dens:
						Len = ToBf((char*) &sp_pump.tnk_dens, sizeof(sp_pump.tnk_dens),
								Buff);
						break;
					case SP_tnk_sensor_hgt:
						Len = ToBf((char*) &sp_pump.tnk_sensor_hgt,
								sizeof(sp_pump.tnk_sensor_hgt), Buff);
						break;
					default:
						break;
					}
					break;
				case FC05_Write_Sing_Coil:
					switch ((Modb_RegAdd) Address) {
					case PUMP_ON:
						snd_head_flow_sett(&newParameter,
								(0xff & Buff[4]) ? 1 : 0, PUMP_ON,
								src_front_panel, 0);
						break;
					case OVERRIDE:
						global_Override = 0xff & Buff[4];
						break;
					case GEN_START:
						EventoGen =
								(0xff & Buff[4]) ?
										genEv_Manual_On : genEv_Manual_Off;
						xQueueSend(queEvtGenStart, &EventoGen, 0);
						break;
					case RST_TOTAL:
						snd_head_flow_sett(&newParameter, 1, RST_TOTAL,
								src_front_panel, 0);
						break;
					case SP_RS485_WL:
						if (Buff[4] == 0xff) { // WIRELESS
							SP.RS485_WL = TRUE;
							RXEN1_OFF;
							TXEN1_OFF;
							RST_WPRO_OFF;
						} else { // WIRED
							SP.RS485_WL = FALSE;
							RST_WPRO_ON;
							RXEN1_ON;
							TXEN1_OFF;
						}
						break;
					case SP_UseGpsTime:
						SP.UseGpsTime = 0xff & Buff[4];
						break;
					default:
						break;
					}
					break;
				case FC06_Write_Sing_Reg:
					switch ((Modb_RegAdd) Address) {
					case SP_CYCLE_T:
					case SP_THRESH:
					case SP_HEADS:
					case SP_UNITS:
					case SP_War_TankLevel:
					case SP_AL_TankLevel:
					case SP_tnk_Shp:
					case SP_GSM_SEND_SMS:
						Ptr_Char = (char*) &shortTemp;
						break;

					case SP_RS485_SLID:
						Ptr_Char = (char*) &SP.RS485_SLID;
						break;
					case SP_WHART_RATE:
						Ptr_Char = (char*) &SP.WHART_RefRate;
						break;
					case SP_WHART_NETID:
						Ptr_Char = (char*) &SP.WHART_NetId;
						break;
					case SP_GSM_REG_AUTH:
						Ptr_Char = (char*) &SP.SP_GSM_REG_AUTH;
						break;

					case SP_TimeZone:
						Ptr_Char = (char*) &SP.TimeZone;
						break;

						//Analog Input  ========================
					case SP_AN_HS:
						Ptr_Char = (char*) &SP.An[An_Idx].HS;
						break;
					case SP_AN_LS:
						Ptr_Char = (char*) &SP.An[An_Idx].LS;
						break;

					case SP_GR:
						Ptr_Char = (char*) &sp_pump.GR;
						break;
					case SP_PAGE:
						Ptr_Char = (char*) &Act_Page;
						break;
					case SP_EPPR:
						Ptr_Char = (char*) &sp_pump.EPPR;
						break;
					case SP_MAXSPD:
#if MOTOR_ELEC
						Ptr_Char = (char*) &sp_pump.nomSpd;
#else
						Ptr_Char = (char*) &sp_pump.maxSpd;
#endif
						break;
					case GEN_PERIOD:
						Ptr_Char = (char*) &gen.period;
						break;
					case GEN_LOW_BATT_TIMEOUT:
						Ptr_Char = (char*) &gen.low_batt_timeout;
						break;
					case GEN_TEST_TIMEOUT:
						Ptr_Char = (char*) &gen.test_timeout;
						break;
					case SP_FACT_RESET:
						Ptr_Char = (char*) &FactResetCode;
						break;
					case SP_SET_FACT_RESET_DATA:
						Ptr_Char = (char*) &FactResetCode;

						//=====================================================
						//  Llamada a ejecutar la tarea de firmware upgrade
						//=====================================================

						break;
					case SP_PIN_ADMIN:
						Ptr_Char = (char*) &PIN[0];
						break;
					case SP_PIN_USR1:
						Ptr_Char = (char*) &PIN[1];
						break;
					case SP_PIN_USR2:
						Ptr_Char = (char*) &PIN[2];
						break;
					default:
						break;
					}
					*Ptr_Char++ = Buff[5];
					*Ptr_Char++ = Buff[4];
					if(Address==SP_MAXSPD)
					{
						calcMaxSpeed(&sp_pump);
					}
					//Aca ejecuto acciones si cambió algun valor interesante
					break;
				case FC10_Write_Mult_Reg:
					switch (Address) {
					case SP_PumpRate:
					case SP_PLG_SIZE:
					case SP_PLG_STR:
					case SP_CF:
					case SP_d3_tnk_RT_length:
					case SP_d1_tnk_VT_HT_diam_RT_width:
					case SP_d2_tnk_VT_height_HT_lenght_RT_height:
					case SP_tnk_dens:
					case SP_tnk_sensor_hgt:
						Ptr_Char = (char*) &floatTemp;
						break;

					case SP_AN_HE:
						Ptr_Char = (char*) &SP.An[An_Idx].HE;
						break;
					case SP_AN_LE:
						Ptr_Char = (char*) &SP.An[An_Idx].LE;
						break;
					case SP_SpanTankEng:
						Ptr_Char = (char*) &sp_pump.SpanTankEng;
						break;
					case SP_ZeroTankEng:
						Ptr_Char = (char*) &sp_pump.ZeroTankEng;
						break;
					case SP_TankCap:
						Ptr_Char = (char*) &sp_pump.TankCap;
						break;
					case SP_SpanTankCap:
						Ptr_Char = (char*) &sp_pump.SpanTankCap;
						break;
					case SP_ZeroTankCap:
						Ptr_Char = (char*) &sp_pump.ZeroTankCap;
						break;
					case SP_PROP:		//todo enviar esto por la cola HEAD_FLOW
						Ptr_Char = (char*) &sp_pump.PropGain;
						break;
					case SP_DER:
						Ptr_Char = (char*) &sp_pump.DerGain;
						break;
					case SP_INT:
						Ptr_Char = (char*) &sp_pump.IntGain;
						break;
//					case SP_PROP:
//					case SP_DER:
//					case SP_INT:
//						double_temp = 0;
//						Ptr_Char = (char*) &double_temp;
//						break;
					case SP_RS485_BAUD:
						Ptr_Char = (char*) &SP.RS485_BAUD;
						break;
					case SP_VPR:
						Ptr_Char = (char*) &sp_pump.VPR;
						break;
					case GEN_LOW_BATT_LIM:
						Ptr_Char = (char*) &gen.low_batt_limit;
						break;
					default:
						break;
					}
					switch (Address) {
					case SP_DER:
					case SP_INT:
					case SP_PROP:
						*Ptr_Char++ = Buff[14];
						*Ptr_Char++ = Buff[13];
						*Ptr_Char++ = Buff[12];
						*Ptr_Char++ = Buff[11];
						*Ptr_Char++ = Buff[10];
						*Ptr_Char++ = Buff[9];
						*Ptr_Char++ = Buff[8];
						*Ptr_Char++ = Buff[7];
						break;
					default:
						*Ptr_Char++ = Buff[10];
						*Ptr_Char++ = Buff[9];
						*Ptr_Char++ = Buff[8];
						*Ptr_Char++ = Buff[7];
						break;
					}
					break;
				case FC6D_String_Read:
					switch ((Modb_RegAdd) Address) {
					case SP_AN_INTYP_1: //Analog Input 1 ========================
						Len = sprintf(&Buff[3], "%d", SP.An[An_Idx].InTyp);
						break;
					case SP_AN_SRC_1:
						Len = sprintf(&Buff[3], "%d", SP.An[An_Idx].Src);
						break;
					case AN_IDX:
						Len = sprintf(&Buff[3], "%d", An_Idx);
						break;
					case UtcTime:
						if (gps.gps_fix)
							Len = sprintf(&Buff[3], asctime(gmtime(&utcTime)));
						else
							Len = sprintf(&Buff[3], "Waiting GPS time info...");
						break;
					case TimeDate:
						Chip_RTC_GetFullTime(LPC_RTC, &Local);
						Len = sprintf(&Buff[3], "%04d%02d%02d%02d%02d%02d",
								Local.time[RTC_TIMETYPE_YEAR],
								Local.time[RTC_TIMETYPE_MONTH],
								Local.time[RTC_TIMETYPE_DAYOFMONTH],
								Local.time[RTC_TIMETYPE_HOUR],
								Local.time[RTC_TIMETYPE_MINUTE],
								Local.time[RTC_TIMETYPE_SECOND]);
						break;
					case GEN_NXT_START:
						timeinfo = localtime(&gen.genNextStart); //Cargo el valor agendado del prox start
						//Escaneo la hora de arranque del test
						mktime(timeinfo);
						Len = sprintf(&Buff[3], "%04d%02d%02d%02d%02d%02d",
								timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
								timeinfo->tm_mday, timeinfo->tm_hour,
								timeinfo->tm_min, timeinfo->tm_sec);
						break;
					case SP_id_tag1:
						leer_eeprom(&Buff[3], OFF_ID_TAG1, LEN_ID);
						Len = strlen(&Buff[3]);
						if (Len > LEN_ID) {
							Len = sprintf(&Buff[3], "ND");
						}
						break;
					case SP_id_tag2:
						leer_eeprom(&Buff[3], OFF_ID_TAG2, LEN_ID);
						Len = strlen(&Buff[3]);
						if (Len > LEN_ID) {
							Len = sprintf(&Buff[3], "ND");
						}
						Len = strlen(&Buff[3]);
						break;
					case SP_id_tag3:
						leer_eeprom(&Buff[3], OFF_ID_TAG3, LEN_ID);
						Len = strlen(&Buff[3]);
						if (Len > LEN_ID) {
							Len = sprintf(&Buff[3], "ND");
						}
						Len = strlen(&Buff[3]);
						break;
					case SP_PumpSN:
						if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
							eeprom_read(&Buff[3], OFF_SN, LEN_SN);
							xSemaphoreGive(mtxI2C1);
						}
						Len = strlen(&Buff[3]);
						if (Len > LEN_ID) {
							Len = sprintf(&Buff[3], "N SN");
						}
						Len = strlen(&Buff[3]);
						break;

					case SP_FirmVers:
						Len = sprintf(&Buff[3], FIRM_VERS);
						break;
					case SP_WHART_JKEY:
						Len = sprintf(&Buff[3], "%08x%08x%08x%08x",
								SP.WHART_JoinKey[0], SP.WHART_JoinKey[1],
								SP.WHART_JoinKey[2], SP.WHART_JoinKey[3]);
						break;
					case SP_WHART_NODE_ID:
						Len = sprintf(&Buff[3],
								"00.1B.1E.%.2X.%.2X.%.2X.%.2X.%.2X",
								IUserLongAddr[0], IUserLongAddr[1],
								IUserLongAddr[2], IUserLongAddr[3],
								IUserLongAddr[4]);
						break;
					case SP_WHART_STAG:
						Len =
								unpack_ascii(
										(char*) &gsHartSlaveInfo.sHartTagDesDate.aucHpsTag[0],
										&Buff[3], 6);
						Buff[3 + Len] = 0;
						break;
					case SP_WHART_LTAG:
						Len = sprintf(&Buff[3], "%s",
								gsHartSlaveInfo.aucLongTag);
						break;
					case Var_GSM_NBR:
						Len = sprintf(&Buff[3], gsm.pn);
						break;
					case Var_GSM_IMEI:
						Len = sprintf(&Buff[3], gsm.imei);
						break;
					case Var_GSM_NET:
						Len = sprintf(&Buff[3], gsm.oper);
						break;
					case SP_GPRS_APN:
						Len = sprintf(&Buff[3], gprs_apn);
						break;
					case SP_GSM_REG_NBR1:
						Len = sprintf(&Buff[3], gprs_rpn[0]);
						break;
					case SP_GSM_REG_NBR2:
						Len = sprintf(&Buff[3], gprs_rpn[1]);
						break;
					case SP_GSM_REG_NBR3:
						Len = sprintf(&Buff[3], gprs_rpn[2]);
						break;
					case SP_GSM_REG_NBR4:
						Len = sprintf(&Buff[3], gprs_rpn[3]);
						break;
					case SP_GSM_SMS_SC:
						Len = sprintf(&Buff[3], gsm.SMS_scenter);
						break;
					default:
						break;
					}
					Buff[2] = Len + 1;
					MBCrc16((uint8_t*) Buff, Len + 4);
					Len += 6;
					break;
				case FC6E_String_Write:
					switch ((Modb_RegAdd) Address) {
					//Analog Input  ========================
					case SP_AN_INTYP_1:
						CharTemp = Buff[5] - 48;
						if (CharTemp >= 0 && CharTemp < 2)
							SP.An[An_Idx].InTyp = CharTemp;
						else
							SP.An[An_Idx].InTyp = R4_20mA; //
						break;
					case SP_AN_SRC_1:
						CharTemp = Buff[5] - 48;
						if (CharTemp >= 0 && CharTemp < 2) {

							//Borro la asignación actual del canal
							for (i = 0; i < 6; i++) {
								if (SP.An[i].Src == CharTemp) {
									SP.An[i].Src = Not_Used;
									SP.AnChannelAssig[CharTemp] = Not_Used;
								}
							}
							SP.An[An_Idx].Src = CharTemp;

							for (i = 0; i < 2; i++) {
								if (SP.AnChannelAssig[i] == SP.An[An_Idx].Src) {
									SP.AnChannelAssig[CharTemp] = Not_Used;
								}
							}
							SP.AnChannelAssig[CharTemp] = An_Idx;

						} else {
							SP.An[An_Idx].Src = Not_Used;
						}

						break;
					case AN_IDX:
						sscanf(&Buff[5], "%d", &An_Idx);
						break;
					case SP_GPRS_APN:
						gprs_apn[0] = '\0';
						strncat(gprs_apn, &Buff[5], LEN_APN - 1);
						break;
					case SP_GSM_REG_NBR1:
						gprs_rpn[0][0] = '\0';
						strncat(gprs_rpn[0], &Buff[5], LEN_RPN - 1);
						break;
					case SP_GSM_REG_NBR2:
						gprs_rpn[1][0] = '\0';
						strncat(gprs_rpn[1], &Buff[5], LEN_RPN - 1);
						break;
					case SP_GSM_REG_NBR3:
						gprs_rpn[2][0] = '\0';
						strncat(gprs_rpn[2], &Buff[5], LEN_RPN - 1);
						break;
					case SP_GSM_REG_NBR4:
						gprs_rpn[3][0] = '\0';
						strncat(gprs_rpn[3], &Buff[5], LEN_RPN - 1);
						break;
					case SP_GSM_SMS_SC:
						strcpy(gsm.SMS_scenter, &Buff[5]);
						EvtToGSM.Src = SMS_ServiceCenter; //Enviar el comando para que actualice el SC Nº
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
						break;
					case SP_WHART_JKEY:
						sscanf(&Buff[5], "%x,%x,%x,%x", &SP.WHART_JoinKey[0],
								&SP.WHART_JoinKey[1], &SP.WHART_JoinKey[2],
								&SP.WHART_JoinKey[3]);
						EvtToHART.Src = write_JOIN_KEY;
						xQueueSend(queEvtToHART, &EvtToHART, 0);
						break;
					case SP_WHART_STAG:
						break;
					case SP_WHART_LTAG:
						break;

					case TimeDate:
						if (sscanf(&Buff[5], "%04d%02d%02d%02d%02d%02d",
								&Local.time[RTC_TIMETYPE_YEAR],
								&Local.time[RTC_TIMETYPE_MONTH],
								&Local.time[RTC_TIMETYPE_DAYOFMONTH],
								&Local.time[RTC_TIMETYPE_HOUR],
								&Local.time[RTC_TIMETYPE_MINUTE],
								&Local.time[RTC_TIMETYPE_SECOND]) == 6) {
							Chip_RTC_SetFullTime(LPC_RTC, &Local);
						}
						break;

					case GEN_NXT_START:
						timeinfo = localtime(&gen.genNextStart); //Cargo el valor agendado del prox start
						if (sscanf(&Buff[5], "%04d%02d%02d%02d%02d%02d",
								&timeinfo->tm_year, &timeinfo->tm_mon,
								&timeinfo->tm_mday, &timeinfo->tm_hour,
								&timeinfo->tm_min, &timeinfo->tm_sec) == 6) {

							timeinfo->tm_year -= 1900;
							timeinfo->tm_mon -= 1;
							gen.genNextStart = mktime(timeinfo);
						}
						break;
					case SP_id_tag1:
					case SP_id_tag2:
					case SP_id_tag3:
						strcpy(&id_Tag[Address - SP_id_tag1][0], &Buff[5]);
						guardar_eeprom((char*) &id_Tag[Address - SP_id_tag1][0],
						OFF_ID_TAG1 + LEN_ID * (Address - SP_id_tag1),
						LEN_ID);
						/*Aviso al Whart que modifique algun TAG*/
						EvtToHART.Src = write_LONG_TAG;
						xQueueSend(queEvtToHART, &EvtToHART, 0);

						/*Actualizar los valores de ID en la web*/
						mqtt_st = mqtt_ev_publish_id;
						xQueueSend(queMQTT_ev, &mqtt_st, 0);

						break;
					case SP_PumpSN:
						strcpy(id_SN, &Buff[5]);
						guardar_eeprom(id_SN, OFF_SN, LEN_SN);
						/*Reseteo el registro en la plataforma web*/
						SP.alreadyRegisteredOnLosant = false;
						/*Refrescar los valores de los controles via web*/
						mqtt_st = mqtt_ev_req_keys_from_broker;
						xQueueSend(queMQTT_ev, &mqtt_st, 0);

						/*Aviso al whart sobre el cambio*/
						EvtToHART.Src = write_TAG_DES_DATE;
						xQueueSend(queEvtToHART, &EvtToHART, 0);
						break;
					default:
						break;
					}
					break;

				case Diagnostic:
					break;
				default:
					xQueueReset(queMsgFromHMI);
					break;

				}

				vTaskDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS));
				if ((Oper != FC05_Write_Sing_Coil)
						&& (Oper != FC06_Write_Sing_Reg)
						&& (Oper != FC6E_String_Write)) {
					Send_UART(Len, (uint32_t*) Buff);
				}
				if ((Buff[0] == HMI_SL_SP_ID)
						&& ((Oper == FC05_Write_Sing_Coil)
								|| (Oper == FC06_Write_Sing_Reg)
								|| (Oper == FC10_Write_Mult_Reg)
								|| (Oper == FC6E_String_Write))) {
					//Comandos a ignorar o disparar una acción
					switch ((Modb_RegAdd) Address) {

					case SP_PAGE:
						break;

					case AN_IDX:
						break;

					case SP_GPRS_APN:
						guardar_eeprom(gprs_apn, OFF_GPRS_APN_PROF4,
								sizeof(gprs_apn));
						//todo: CUIDADO CON EL FORMATO DE GUARDADO EN EEPROM
						// DEBE INCLUIR EL USR Y PASS
						break;

					case SP_GSM_REG_NBR1:
					case SP_GSM_REG_NBR2:
					case SP_GSM_REG_NBR3:
					case SP_GSM_REG_NBR4:
						guardar_eeprom(
								(char*) &gprs_rpn[Address - SP_GSM_REG_NBR1],
								OFF_GSM_REG_NBR1
										+ LEN_RPN * (Address - SP_GSM_REG_NBR1),
								LEN_RPN);
						break;
					case SP_GSM_SEND_SMS:
						EvtToGSM.Src = cmd_Welcome;
						EvtToGSM.Val = shortTemp;
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
						break;
					case SP_WHART_NETID:
						EvtToHART.Src = write_NET_ID;
						xQueueSend(queEvtToHART, &EvtToHART, 0);
						break;
					case SP_WHART_RATE:
						EvtToHART.Src = write_UPD_RATE;
						xQueueSend(queEvtToHART, &EvtToHART, 0);
						break;
					case SP_FACT_RESET:
						read_factory_data(FactResetCode);
						break;

					case SP_SET_FACT_RESET_DATA:
						set_factory_data(FactResetCode);
						break;
					case SP_PIN_ADMIN:
					case SP_PIN_USR1:
					case SP_PIN_USR2:
						guardar_eeprom((char*) &PIN[0], OFF_PIN, LEN_PIN * 3);
						break;

					case GEN_PERIOD:
					case GEN_NXT_START:
					case GEN_TEST_TIMEOUT:
						upd_next_gen_start_data = true;
						break;
					case GEN_LOW_BATT_TIMEOUT:
					case GEN_LOW_BATT_LIM:
						if (gen.low_batt_limit > 30)
							gen.low_batt_limit = 30;
						else if (gen.low_batt_limit < 0)
							gen.low_batt_limit = 0;
						guardar_eeprom((char*) &gen, OFF_GEN, sizeof(gen));

						break;

						/*PARAMETROS para configuracion de bomba*/
					case SP_PumpRate: //Lo incluyo para q no grabe en EEPROM
					case SP_PLG_SIZE:
					case SP_PLG_STR:
					case SP_CF:
						snd_head_flow_sett(&newParameter, floatTemp,
								(Modb_RegAdd) Address, src_front_panel, 0);
						break;
					case SP_THRESH:
					case SP_CYCLE_T:
					case SP_HEADS:
					case SP_UNITS:
						snd_head_flow_sett(&newParameter, (float) shortTemp,
								(Modb_RegAdd) Address, src_front_panel, 0);
						break;

						/*PARAMETROS para configuracion de tanque*/
					case SP_tnk_Shp:
					case SP_AL_TankLevel:
					case SP_War_TankLevel:
						snd_tank_sett(&newParameter, (float) shortTemp,
								(Modb_RegAdd) Address, src_front_panel);
						break;
					case SP_tnk_sensor_hgt:
					case SP_tnk_dens:
					case SP_d1_tnk_VT_HT_diam_RT_width:
					case SP_d2_tnk_VT_height_HT_lenght_RT_height:
					case SP_d3_tnk_RT_length:
						snd_tank_sett(&newParameter, floatTemp,
								(Modb_RegAdd) Address, src_front_panel);
						break;
					default:
						guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
						guardar_eeprom((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
					}
				}
			} else {
				xQueueReset(queMsgFromHMI);
				//Descartar el paquete. No es para mi.
			}
		} else {
			//No paso el CRC
			xQueueReset(queMsgFromHMI);
		}
	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por el UART2 - Comunicacion HMI*/
void UART_HANDLER_NAME(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t Interr;
	static uint8_t ReadB, i;
	static uint8_t Buffer[4];

	Interr = Chip_UART_ReadIntIDReg(UART);

	if ((Interr & UART_IIR_INTID_RDA) || (Interr & UART_IIR_INTID_CTI)) {
		Chip_TIMER_Reset(TIMER); //Reseteo el tmr
		Chip_TIMER_ClearMatch(TIMER, 0); //
		Chip_TIMER_ClearMatch(TIMER, 1);
		Chip_TIMER_Enable(TIMER); //Habilito el tmr
		ReadB = Chip_UART_Read(UART, Buffer, 4);
		for (i = 0; i < ReadB; i++) {
			xQueueSendFromISR(queMsgFromHMI, &Buffer[i],
					&xHigherPriorityTaskWoken);
		}
	} else if (Interr & UART_IIR_INTID_THRE) {
		Chip_UART_IntDisable(UART, UART_IER_THREINT);
//		xSemaphoreGiveFromISR(Sem_DMA_TX2, &xHigherPriorityTaskWoken);
		vTaskNotifyGiveFromISR(tsk_modb_hmi_handler, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/
/* Atención de INT por el TMR0 */
void TMR_HANDLER_NAME(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if (Chip_TIMER_MatchPending(TIMER, 0)) { //Expiro T15??
		Chip_TIMER_ClearMatch(TIMER, 0); //
		xSemaphoreGiveFromISR(semExpT15_HMI, &xHigherPriorityTaskWoken);
	} else if (Chip_TIMER_MatchPending(TIMER, 1)) { //Expiro T35??
		Chip_TIMER_ClearMatch(TIMER, 1);
		Chip_TIMER_Disable(TIMER);
		xSemaphoreGiveFromISR(semExpT35_HMI, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_CommMB_HMI_Task(void) {
	xTaskCreate(vCommMB_HMI, (char*) "MB_HMI TSK", 290, NULL,
	Prio_CommMB_HMI_Task, &tsk_modb_hmi_handler); /* "Comunicación con el HMI*/
}
#endif
