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

#include "NxHMI.h"
//#include "rtc_17xx_40xx.h"
#if !USE_SIM231_TP_SCREEN
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
xTimerHandle timerUpdate,timerTout,timerLogOut;
typedef enum {
	tmr_upd_idx, tmr_tout_idx,tmr_logout_idx
} tmr_idx_type;


EventHMI ghmiev=getPag;
void vUpdTmr_Callback(xTimerHandle timerUpdate) {
	EventHMI hmiev;
	tmr_idx_type tmrIdx;
	tmrIdx = (tmr_idx_type) pvTimerGetTimerID(timerUpdate);
	switch(tmrIdx)
	{
		case tmr_upd_idx:

			ghmiev=getPag;
			xQueueSend(queHMI_ev, &ghmiev, 10);
			break;
		case tmr_tout_idx:
			ghmiev=getPag;
			xQueueSend(queHMI_ev, &ghmiev, 10);
			xTaskNotifyGive(tsk_nex_hmi_handler);
			break;
		case tmr_logout_idx:
			ghmiev=logOut;
			xQueueSend(queHMI_ev, &ghmiev, 10);
			ghmiev=getPag;
			xQueueSend(queHMI_ev, &ghmiev, 10);
			xTaskNotifyGive(tsk_nex_hmi_handler);
			break;
	}
}
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

static portTASK_FUNCTION(vCommNx_HMI_Rx, pvParameters)
{
	uint8_t data, i,page=0,comp=0;
	TickType_t delay;
	char Buf485_Rx[BUF485_SIZE]__attribute__ ((aligned (32)));
	char Buff[100];
	MB_Function_Code Oper;
	uint32_t rpmTemp=0;
	float tempfloat;
	uint16_t tempshort=0;
	EventHMI hmiev2=getPag;
	EvtToMOB_Type EvtToGSM;
	static uint16_t ReadB, ReadC;
	Parameter_upd_Type newParameter;
	portTickType xLastWakeTime;
	EvtToHART_Type EvtToHART;
	mqtt_event_type mqtt_st;
	xLastWakeTime = xTaskGetTickCount();
	strMsgFromHMI = xStreamBufferCreate(BUF485_SIZE, 1);
	while (1) {

		memset(Buf485_Rx, 0xff, BUF485_SIZE); //Preparo el buffer donde voy a alojar los datos.
		delay = portMAX_DELAY;
		ReadB = 0;
		do {
			ReadC = wait_Byte_Nex(Buf485_Rx, delay);
			ReadB += ReadC;
			delay = 20;
		} while (ReadC);
		memset(Buff, 0, 100);
		xTimerReset(timerTout,100);//reseteo timer d tout con la pantalla
		//nueva version del aloritmo de decodificacion se utilizara el tipo de
		//evento para disparar las diferente reacciones del sistema
		switch(Buf485_Rx[0])//Evento
		{
			case 0x66://La pantalla envio el valor actual de la pantalla
				page_nx=Buf485_Rx[1];
				break;
			case 0x65://se recivio un evento touch
				switch(Buf485_Rx[1])//N° pantalla
				{
					case 0x2:
						switch(Buf485_Rx[2])//ID componente
						{
							case 0x2:
								xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
								ghmiev=getPumpOn;
								xQueueSend(queHMI_ev, &ghmiev, 10);
								break;

							case 0x19:
								global_Override = 0xff & Buf485_Rx[3];
								break;
							case 0x15:
								tempfloat=sp_pump.PumpRate+10;
								snd_head_flow_sett(&newParameter, tempfloat,
										SP_PumpRate, src_front_panel, 0);
								break;
							case 0x16:
								tempfloat=sp_pump.PumpRate-10;
								snd_head_flow_sett(&newParameter, tempfloat,
										SP_PumpRate, src_front_panel, 0);
								break;

						}
						break;
					case 0x3:
					case 0x7:
					case 0x17:
						xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
						ghmiev=getPgAndCp;
						xQueueSend(queHMI_ev, &ghmiev, 10);
						break;
					case 0x8:
						xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
						ghmiev=getNewCoef;
						xQueueSend(queHMI_ev, &ghmiev, 10);
						break;
					case 0x9:
						if(Buf485_Rx[2]==3)
						{
							xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
							ghmiev=getPumpOn;
							xQueueSend(queHMI_ev, &ghmiev, 10);
						}
						break;
					case 0x12:
						switch(Buf485_Rx[2])
						{
							case 3:
								aCH=0;
								break;
							case 4:
								aCH=3;
								break;
							case 5:
								aCH=4;
								break;
							case 6:
								aCH=1;
								break;
							case 7:
								aCH=5;
								 break;
							case 8:
								aCH=2;
								break;

						}
						break;
					case 0x10:
						switch(Buf485_Rx[2])
						{
							case 3:
								SP.SP_GSM_REG_AUTH ^= 4;
								break;
							case 6:
								SP.SP_GSM_REG_AUTH ^= 1;
								break;
							case 8:
								SP.SP_GSM_REG_AUTH ^= 2;
								break;
							case 10:
								SP.SP_GSM_REG_AUTH ^= 8;
								break;
						}
						break;
					case 0x0F:
						if(Buf485_Rx[2]==0x3)
						{
							xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
							ghmiev=sendwSMS;
							xQueueSend(queHMI_ev, &ghmiev, 10);
						}
						break;
					case 0x15:
						switch(Buf485_Rx[2])//ID componente
						{
							case 0xc:
								xTimerStop(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
								ghmiev=getPumpOn;
								xQueueSend(queHMI_ev, &ghmiev, 10);
								break;
							case 0x4:
								tempfloat=sp_pump.PumpRate+10;
								snd_head_flow_sett(&newParameter, tempfloat,
										SP_PumpRate, src_front_panel, 0);
								break;
							case 0x8:
								tempfloat=sp_pump.PumpRate-10;
								snd_head_flow_sett(&newParameter, tempfloat,
										SP_PumpRate, src_front_panel, 0);
								break;

						}
						break;

				}
				break;
			case 0x71://se resivio un valor numerico
				if(ghmiev==updateVar)
				{
					ghmiev=getPag;
				}
				switch(ghmiev)
				{
				case getPag:
					page_nx=Buf485_Rx[1];
					xTimerStart(timerUpdate, 0);
					if(page_nx==19)
					{
						ghmiev=getACH;
					}
					else
					{
						ghmiev=getModVar;
					}
					xQueueSend(queHMI_ev, &ghmiev, 10);
					break;
				case getACH:
					aCH=Buf485_Rx[1]-1;
					ghmiev=getModVar;
					xQueueSend(queHMI_ev, &ghmiev, 10);
					break;
				case getModVar:
					if(Buf485_Rx[1]==1)
						ghmiev=NewValOK;
					else
					{
						ghmiev=updateVar;
						xQueueSend(queHMI_ev, &ghmiev, 10);

					}

					xTimerStart(timerUpdate, 0);
					break;
				case getPumpOn:
					page=(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					comp=Buf485_Rx[1];
					snd_head_flow_sett(&newParameter,
							(0xff & Buf485_Rx[1]) ? 1 : 0, PUMP_ON,
							src_front_panel, 0);
					ghmiev=NewValOK;

					xTimerStart(timerUpdate, 0);
					break;
				case getPgAndCp:
					modifVal=2;
					page=(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					comp=Buf485_Rx[1];
					ghmiev=(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					if(ghmiev!=volCalib)
						xQueueSend(queHMI_ev, &ghmiev, 10);
					else
						xTimerStart(timerUpdate, 0);
					break;

				case getSPNomSpd:
					ghmiev=NewValOK;
//							sp_pump.nomSpd=(Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1];
//							calcMaxSpeed(&sp_pump);
					break;
				case getSPPumpRt:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempfloat != sp_pump.PumpRate)
						snd_head_flow_sett(&newParameter, tempfloat,
								SP_PumpRate, src_front_panel, 0);
					break;
				case getTWidth:
					ghmiev=NewValOK;
					sp_pump.tnk_d1_VT_HT_diam_RT_width=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getTLen:
					ghmiev=NewValOK;
					sp_pump.tnk_d3_RT_length=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getTHgt:
					ghmiev=NewValOK;
					sp_pump.tnk_d2_VT_height_HT_lenght_RT_height=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getTDens:
					ghmiev=NewValOK;
					sp_pump.tnk_dens=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getTCap:
					ghmiev=NewValOK;
					sp_pump.TankCap=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getSeHgt:
					ghmiev=NewValOK;
					sp_pump.tnk_sensor_hgt=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100.0;
					break;
				case getSeSH:
					ghmiev=NewValOK;
					SP.An[0].HS=(uint16_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000;
					break;
				case getSeSL:
					ghmiev=NewValOK;
					SP.An[0].LS=(uint16_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000;
					break;
				case getSeEH:
					ghmiev=NewValOK;
					SP.An[0].HE=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000.0;
					break;
				case getSeEL:
					ghmiev=NewValOK;
					SP.An[0].LE=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000.0;
					break;
				case getWrLvl:
					ghmiev=NewValOK;
					tempshort=(uint16_t)((Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempshort != sp_pump.level_warning_limit_T1)
						snd_tank_sett(&newParameter, tempshort,
								SP_War_TankLevel, src_front_panel);
					break;
				case getAlLvl:
					ghmiev=NewValOK;
					tempshort=(uint16_t)((Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempshort != sp_pump.level_alarm_limit_T1)
						snd_tank_sett(&newParameter, tempshort, SP_AL_TankLevel,
													src_front_panel);
					break;
				case getPlSiz:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000.0;
					if (tempfloat != sp_pump.plg_size)
						snd_head_flow_sett(&newParameter, tempfloat,
								SP_PLG_SIZE, src_front_panel, 0);
					break;
				case getPlStr:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000.0;
					if (tempfloat != sp_pump.plg_str)
						snd_head_flow_sett(&newParameter, tempfloat,
								SP_PLG_STR, src_front_panel, 0);
					break;
				case getHeads:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempfloat != sp_pump.heads)
						snd_head_flow_sett(&newParameter, tempfloat,
								SP_HEADS, src_front_panel, 0);
					break;
				case getPwmTh:
					ghmiev=NewValOK;
					tempshort=((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempshort != sp_pump.Threshold)
						snd_head_flow_sett(&newParameter,(float) tempshort,
								SP_THRESH, src_front_panel, 0);
					break;
				case getCycle:
					ghmiev=NewValOK;
					tempshort=((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					if (tempshort != sp_pump.Cycle)
						snd_head_flow_sett(&newParameter,(float) tempshort,
								SP_CYCLE_T, src_front_panel, 0);
					break;
				case getNewCoef:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/1000;
					if(sp_pump.CF!=tempfloat)
					snd_head_flow_sett(&newParameter, tempfloat,
							SP_CF, src_front_panel, 0);
					break;
				case getaHe:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100;
					SP.An[aCH].HE=tempfloat;
					break;
				case getaLe:
					ghmiev=NewValOK;
					tempfloat=(float)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/100;
					SP.An[aCH].LE=tempfloat;
					break;
				case getaHs:
					ghmiev=NewValOK;
					tempshort=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.An[aCH].HS=tempshort;
					break;
				case getaLs:
					ghmiev=NewValOK;
					tempshort=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.An[aCH].LS=tempshort;
					break;
				case getaTy:
					ghmiev=NewValOK;
					tempshort=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.An[aCH].InTyp=tempshort;
					break;
				case getaSrs:
					ghmiev=NewValOK;
					tempshort=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.An[aCH].Src=tempshort;
					break;
				case getSLID:
					ghmiev=NewValOK;
					rpmTemp=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.RS485_SLID=rpmTemp;
					break;
				case getBaud:
					ghmiev=NewValOK;
					rpmTemp=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.RS485_BAUD=rpmTemp;
					break;
				case getLink:
					ghmiev=NewValOK;
					rpmTemp=(int32_t)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1]);
					SP.RS485_WL=rpmTemp;
					break;
				case getGR:
					ghmiev=NewValOK;
					sp_pump.GR=(Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					break;
				case getEPPR:
					ghmiev=NewValOK;
					sp_pump.EPPR=(Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					break;
//				case getMaxSpd:
//					ghmiev=NewValOK;
//					sp_pump.maxSpd=(Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1];
//					break;
				case getSPrpm:
					ghmiev=NewValOK;
					sp_pump.maxSpd=(Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1];
					break;
				case getPidP:
					ghmiev=NewValOK;
					sp_pump.PropGain=(double)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/10000.0;
					break;
				case getPidI:
					ghmiev=NewValOK;
					sp_pump.IntGain=(double)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/10000.0;
					break;
				case getPidD:
					ghmiev=NewValOK;
					sp_pump.DerGain=(double)((Buf485_Rx[4]<<24)+(Buf485_Rx[3]<<16)+(Buf485_Rx[2]<<8)+Buf485_Rx[1])/10000.0;
					break;
				case sendwSMS:
					if(Buf485_Rx[1]!=99)
					{
						EvtToGSM.Src = cmd_Welcome;
						EvtToGSM.Val = Buf485_Rx[1];
						xQueueSend(queEvtToMOBILE, &EvtToGSM, 0);
					}
					break;

				}
				if(ghmiev==NewValOK)
				{
					xQueueSend(queHMI_ev, &ghmiev, 10);
					ghmiev=0;
					guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
					guardar_eeprom((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
					xTimerStart(timerUpdate, 0);
					ghmiev=getPag;
					xQueueSend(queHMI_ev, &ghmiev, 10);
				}

				xTaskNotifyGive(tsk_nex_hmi_handler);
				break;
			case 0x70:
				i=0;
				while(Buf485_Rx[i]!=0xff)
				{
					i++;
				}
				Buf485_Rx[i]='\0';

				switch(ghmiev)
				{
					case getNum1:
						ghmiev=NewValOK;
						strcpy(gprs_rpn[0],&Buf485_Rx[1]);
						guardar_eeprom(
								(char*) &gprs_rpn[0],
								OFF_GSM_REG_NBR1,
								LEN_RPN);
						break;
					case getNum2:
						ghmiev=NewValOK;
						strcpy(gprs_rpn[1],&Buf485_Rx[1]);
						guardar_eeprom(
								(char*) &gprs_rpn[1],
								OFF_GSM_REG_NBR1
										+ LEN_RPN,
								LEN_RPN);
						break;
					case getNum3:
						ghmiev=NewValOK;
						strcpy(gprs_rpn[2],&Buf485_Rx[1]);
						guardar_eeprom(
								(char*) &gprs_rpn[2],
								OFF_GSM_REG_NBR1
										+ LEN_RPN * 2,
								LEN_RPN);
						break;
					case getNum4:
						ghmiev=NewValOK;
						strcpy(gprs_rpn[3],&Buf485_Rx[1]);
						guardar_eeprom(
								(char*) &gprs_rpn[3],
								OFF_GSM_REG_NBR1
										+ LEN_RPN * (3),
								LEN_RPN);
						break;

					case getClien:
						strcpy(&id_Tag[0][0], &Buf485_Rx[1]);
						guardar_eeprom((char*) &id_Tag[0][0],
						OFF_ID_TAG1 ,
						LEN_ID);
						/*Aviso al Whart que modifique algun TAG*/
						EvtToHART.Src = write_LONG_TAG;
						xQueueSend(queEvtToHART, &EvtToHART, 0);

						/*Actualizar los valores de ID en la web*/
						mqtt_st = mqtt_ev_publish_id;
						xQueueSend(queMQTT_ev, &mqtt_st, 0);
						break;
					case getField:
						strcpy(&id_Tag[1][0], &Buf485_Rx[1]);
						guardar_eeprom((char*) &id_Tag[1][0],
						OFF_ID_TAG1 + LEN_ID ,
						LEN_ID);
						/*Aviso al Whart que modifique algun TAG*/
						EvtToHART.Src = write_LONG_TAG;
						xQueueSend(queEvtToHART, &EvtToHART, 0);

						/*Actualizar los valores de ID en la web*/
						mqtt_st = mqtt_ev_publish_id;
						xQueueSend(queMQTT_ev, &mqtt_st, 0);
						break;
					case getWell:
						strcpy(&id_Tag[2][0], &Buf485_Rx[1]);
						guardar_eeprom((char*) &id_Tag[2][0],
						OFF_ID_TAG1 + LEN_ID * 2,
						LEN_ID);
						/*Aviso al Whart que modifique algun TAG*/
						EvtToHART.Src = write_LONG_TAG;
						xQueueSend(queEvtToHART, &EvtToHART, 0);

						/*Actualizar los valores de ID en la web*/
						mqtt_st = mqtt_ev_publish_id;
						xQueueSend(queMQTT_ev, &mqtt_st, 0);
						break;
					case getSN:
						ghmiev=NewValOK;
						strcpy(id_SN, &Buf485_Rx[1]);
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

//					case SP_id_tag1:
//						leer_eeprom(&Buff[3], OFF_ID_TAG1, LEN_ID);
//						Len = strlen(&Buff[3]);
//						if (Len > LEN_ID) {
//							Len = sprintf(&Buff[3], "ND");
//						}
//						break;
//					case SP_id_tag2:
//						leer_eeprom(&Buff[3], OFF_ID_TAG2, LEN_ID);
//						Len = strlen(&Buff[3]);
//						if (Len > LEN_ID) {
//							Len = sprintf(&Buff[3], "ND");
//						}
//						Len = strlen(&Buff[3]);
//						break;
//					case SP_id_tag3:
//						leer_eeprom(&Buff[3], OFF_ID_TAG3, LEN_ID);
//						Len = strlen(&Buff[3]);
//						if (Len > LEN_ID) {
//							Len = sprintf(&Buff[3], "ND");
//						}
//						Len = strlen(&Buff[3]);
//						break;
//					case SP_PumpSN:
//						if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
//							eeprom_read(&Buff[3], OFF_SN, LEN_SN);
//							xSemaphoreGive(mtxI2C1);
//						}
//						Len = strlen(&Buff[3]);
//						if (Len > LEN_ID) {
//							Len = sprintf(&Buff[3], "N SN");
//						}
//						Len = strlen(&Buff[3]);
//						break;
//
						break;
					case getPwd:
						ghmiev=NewValOK;
						if(strstr(&Buf485_Rx[1],"+741-"))
						{
							pwUsrLvl=3;
							xTimerStart(timerLogOut,100);
						}
						else if(strlen(&Buf485_Rx[1])>2)
						{
							sscanf(&Buf485_Rx[1], "%d", &tempshort);
							if((tempshort==PIN[0])||(tempshort==PIN[0])||(tempshort==PIN[0]))
							{
								pwUsrLvl=1;
								xTimerStart(timerLogOut,100);
							}
							else
							{
								pwUsrLvl=0;
							}

						}
						else
						{
							pwUsrLvl=0;
						}

				}
				if(ghmiev==NewValOK)
				{
					xQueueSend(queHMI_ev, &ghmiev, 10);
					ghmiev=0;
					guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
					guardar_eeprom((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
					xTimerStart(timerUpdate, 0);
					ghmiev=getPag;
					xQueueSend(queHMI_ev, &ghmiev, 10);
				}
				break;
			default:
				xTaskNotifyGive(tsk_nex_hmi_handler);
				break;

		}
		//fin de nueva version de algoritmo

	}
}

static portTASK_FUNCTION(vCommNx_HMI_Tx, pvParameters)
{
	uint8_t data, i=0;
	TickType_t delay;
	while (1) {
		/*Espero el primer dato*/
		delay = portMAX_DELAY;
		while (xQueueReceive(queMsgToHMINx, &data, delay) == pdPASS) {
			/*Hasta que se acabe la cola debo leer con cero delay*/
			delay = 0;
			while ((Chip_UART_ReadLineStatus(UART) & UART_LSR_THRE) == 0) {
				/*Espero que la interrupción por THRE me avise que puedo seguir*/
				ulTaskNotifyTake(TRUE, portMAX_DELAY);
			}

			/*Ahora si, envío un byte*/
			Chip_UART_SendByte(UART, data);
			/*Habilito la int por THRE*/
			i++;
			Chip_UART_IntEnable(UART, UART_IER_THREINT);
		}
	}
}
/*-----------------------------------------------------------
 * 	 */
static portTASK_FUNCTION(vCommNx_HMI, pvParameters) {
	char Buff[100];
	portTickType xLastWakeTime;
	static uint8_t An_Idx = 0, CharTemp, i;
	TickType_t delay;
	uint32_t shortTemp;
	int32_t rpmTemp=0;
	float floatTemp;
	double double_temp;
//	static S_Typ mem_SP_vol_unit;
//	static Flow_UNITS_Typ mem_SP_flow_unit;
	Parameter_upd_Type newParameter;
	xLastWakeTime = xTaskGetTickCount();
	EventHMI EventNx=0;
	char *Ptr_Char;

	// UART_2: COMUNICACION CON HMI //////////////////
	Chip_UART_Init(UART);
	Chip_UART_SetBaud(UART, BAUDRATE);
	Chip_UART_ConfigData(UART,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(UART,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV1);

	Chip_UART_TXEnable(UART);

	/* Enable UART2 interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 3);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	Chip_UART_IntEnable(UART, UART_IER_RBRINT);

	upd_next_gen_start_data = false;
	strMsgFromHMI = xStreamBufferCreate(BUF485_SIZE, 1);
	timerUpdate=xTimerCreate("Tmr_HMIUpdete", pdMS_TO_TICKS(800), pdTRUE,
			(void*) tmr_upd_idx, vUpdTmr_Callback);
	timerTout=xTimerCreate("timerTout", pdMS_TO_TICKS(1200), pdTRUE,
		(void*)tmr_tout_idx, vUpdTmr_Callback);

	timerLogOut=xTimerCreate("timerLogOut", pdMS_TO_TICKS(600*1000), pdTRUE,
		(void*)tmr_logout_idx, vUpdTmr_Callback);//timer para log out
	xTimerStart(timerUpdate, 0);
	xTimerStart(timerTout, 0);
	Create_CommNx_HMITx_Task();
	Create_CommNx_HMIRx_Task();
	while (1) {
		//RECEPTION
		//vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS))
		if (xQueueReceive(queHMI_ev, &EventNx, 30000) == pdPASS) {
			switch(EventNx)
			{
				case updateVar:
					rpmTemp=(int32_t)(pwUsrLvl);
					sprintf(Buff,"pwUsr=%d\0",rpmTemp);
					sendToNx(Buff);
					if (Var.local_remoto == TRUE)
					{
						xTimerReset(timerLogOut,100);
					}
					if(page_nx==0)
					{//pantalla principal
						rpmTemp=(int32_t)(Var.tank_vol);
						sprintf(Buff,"tankVol=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.TankCap);
						sprintf(Buff,"tankCap=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.PumpRate);
						sprintf(Buff,"setFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.Total_pumped);
						sprintf(Buff,"totPump=%d\0",rpmTemp);
						sendToNx(Buff);
						shortTemp=(int16_t)PumpOn;
						sprintf(Buff,"pumpOn=%d\0",shortTemp);
						sendToNx(Buff);

						rpmTemp=(int32_t)(Var.LinePress);
						sprintf(Buff,"linePres.val=%d\0",rpmTemp);
						sendToNx(Buff);

						rpmTemp=RegAlarmas;
						sprintf(Buff,"alarm.val=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.VBatt*10.0);
						sprintf(Buff,"vBatt.val=%d\0",rpmTemp);
						sendToNx(Buff);

						rpmTemp=(int32_t)(Var.CPanel*10.0);
						sprintf(Buff,"cPan.val=%d\0",rpmTemp);
						sendToNx(Buff);
						//Actualizacion de fecha y hora
						if((Local.time[RTC_TIMETYPE_HOUR]<10)&&(Local.time[RTC_TIMETYPE_MINUTE]<10))
						{
							sprintf(Buff,"hour.txt=\"0%d:0%d\"\0",Local.time[RTC_TIMETYPE_HOUR],Local.time[RTC_TIMETYPE_MINUTE]);
						}
						else
						{
							if(Local.time[RTC_TIMETYPE_HOUR]<10)
							{
								sprintf(Buff,"hour.txt=\"0%d:%d\"\0",Local.time[RTC_TIMETYPE_HOUR],Local.time[RTC_TIMETYPE_MINUTE]);
							}
							else{
								if(Local.time[RTC_TIMETYPE_MINUTE]<10)
								{
									sprintf(Buff,"hour.txt=\"%d:0%d\"\0",Local.time[RTC_TIMETYPE_HOUR],Local.time[RTC_TIMETYPE_MINUTE]);
								}
								else{
									sprintf(Buff,"hour.txt=\"%d:%d\"\0",Local.time[RTC_TIMETYPE_HOUR],Local.time[RTC_TIMETYPE_MINUTE]);
								}
							}
						}
						sendToNx(Buff);
						sprintf(Buff,"date.txt=\"%d/%d/%d\"\0",Local.time[RTC_TIMETYPE_DAYOFMONTH],Local.time[RTC_TIMETYPE_MONTH],Local.time[RTC_TIMETYPE_YEAR]);
						sendToNx(Buff);
					}
					if(page_nx==1)
					{//Tank
						rpmTemp=(int32_t)(Var.tank_vol);
						sprintf(Buff,"tankVol=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.tank_Lev*100.0);
						sprintf(Buff,"tankLvl=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.tnk_d1_VT_HT_diam_RT_width*100.0);
						sprintf(Buff,"tankW=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.tnk_d3_RT_length*100.0);
						sprintf(Buff,"tankL=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.tnk_d2_VT_height_HT_lenght_RT_height*100.0);
						sprintf(Buff,"tankH=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.tnk_dens*100.0);
						sprintf(Buff,"tankD=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.tnk_sensor_hgt*100.0);
						sprintf(Buff,"tankSh=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.TankCap*100.0);
						sprintf(Buff,"tankCap=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.level_warning_limit_T1);
						sprintf(Buff,"tankWr=%d",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.level_alarm_limit_T1);
						sprintf(Buff,"tankAl=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==2)
					{//Status bomba
						rpmTemp=(uint32_t)sp_pump.PumpRate;
						sprintf(Buff,"setFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=Var.Total_pumped;
						sprintf(Buff,"totPump=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(uint32_t)Var.PumpRate;
						sprintf(Buff,"curFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						shortTemp=(int16_t)PumpOn;
						sprintf(Buff,"pumpOn=%d\0",shortTemp);
						sendToNx(Buff);
					}
					if(page_nx==5)//pump settings
					{
						rpmTemp=(uint32_t)sp_pump.Threshold;
						sprintf(Buff,"treshH=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=sp_pump.Cycle;
						sprintf(Buff,"cycleT=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==6)
					{
						rpmTemp=(uint32_t)(sp_pump.plg_size*1000);
						sprintf(Buff,"plSize=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(uint32_t)(sp_pump.plg_str*1000);
						sprintf(Buff,"plStr=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(uint32_t)sp_pump.heads;
						sprintf(Buff,"heads=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(uint32_t)sp_pump.maxFlow ;
						sprintf(Buff,"maxFlow=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==8)
					{
						shortTemp=(int16_t)PumpOn;
						sprintf(Buff,"pumpOn=%d\0",shortTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.PumpRate);
						sprintf(Buff,"setFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.CF*1000);
						sprintf(Buff,"coefCor=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==9)
					{
						shortTemp=(int16_t)PumpOn;
						sprintf(Buff,"pumpOn=%d\0",shortTemp);
						sendToNx(Buff);
					}
					if(page_nx==13)
					{
						rpmTemp=(int32_t)(SP.RS485_SLID);
						sprintf(Buff,"cSlaveid=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.RS485_BAUD);
						sprintf(Buff,"cBaud=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.RS485_WL);
						sprintf(Buff,"cLink=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==14)
					{
						sprintf(Buff,"imei.txt=\"%s\"",gsm.imei);
						sendToNx(Buff);
						sprintf(Buff,"rssi.val=%d",gsm.rssi);
						sendToNx(Buff);
						sprintf(Buff,"net.txt=\"%s\"",gsm.oper);
						sendToNx(Buff);
					}
					if(page_nx==15)
					{
						sprintf(Buff,"Num1.txt=\"%s\"",gprs_rpn[0]);
						sendToNx(Buff);
						sprintf(Buff,"Num2.txt=\"%s\"",gprs_rpn[1]);
						sendToNx(Buff);
						sprintf(Buff,"Num3.txt=\"%s\"",gprs_rpn[2]);
						sendToNx(Buff);
						sprintf(Buff,"Num4.txt=\"%s\"",gprs_rpn[3]);
						sendToNx(Buff);
						sprintf(Buff,"smsServ.txt=\"%s\"",gsm.SMS_scenter);
						sendToNx(Buff);
					}
					if(page_nx==16)
					{
						sprintf(Buff,"Num1.txt=\"%s\"",gprs_rpn[0]);
						sendToNx(Buff);
						sprintf(Buff,"Num2.txt=\"%s\"",gprs_rpn[1]);
						sendToNx(Buff);
						sprintf(Buff,"Num3.txt=\"%s\"",gprs_rpn[2]);
						sendToNx(Buff);
						sprintf(Buff,"Num4.txt=\"%s\"",gprs_rpn[3]);
						sendToNx(Buff);
						sprintf(Buff,"smsA.val=%d",SP.SP_GSM_REG_AUTH);
						sendToNx(Buff);
					}
					if(page_nx==19)
					{
						rpmTemp=(int32_t)(Var.an[aCH].Signal*100.0);
						sprintf(Buff,"aVarSig=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.an[aCH].Eng_Un*100.0);
						sprintf(Buff,"aVarEng=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].HE*100.0);
						sprintf(Buff,"aHEng=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].LE*100.0);
						sprintf(Buff,"aLEng=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].HS);
						sprintf(Buff,"aHSig=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].LS);
						sprintf(Buff,"aLSig=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].InTyp);
						sprintf(Buff,"aTypeIn=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(SP.An[aCH].Src);
						sprintf(Buff,"aSource=%d\0",rpmTemp);
						sendToNx(Buff);

					}
					if(page_nx==20)
					{
						rpmTemp=(int32_t)(sp_pump.EPPR);
						sprintf(Buff,"encoPPR=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.GR);
						sprintf(Buff,"gearR=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.maxSpd);
						sprintf(Buff,"maxSpd=%d\0",rpmTemp);
						sendToNx(Buff);
					}
					if(page_nx==21)
					{
						rpmTemp=(int32_t)(sp_pump.PumpRate);
						sprintf(Buff,"setFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.PumpRate);
						sprintf(Buff,"curFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.MotorRPM);
						sprintf(Buff,"curRPM=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(Var.curr_error);
						sprintf(Buff,"errorFlow=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.PropGain*10000.0);
						sprintf(Buff,"pidP=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.IntGain*10000.0);
						sprintf(Buff,"pidI=%d\0",rpmTemp);
						sendToNx(Buff);
						rpmTemp=(int32_t)(sp_pump.DerGain*10000.0);
						sprintf(Buff,"pidD=%d\0",rpmTemp);
						sendToNx(Buff);
						shortTemp=(int16_t)PumpOn;
						sprintf(Buff,"pumpOn=%d\0",shortTemp);
						sendToNx(Buff);

					}
					if(page_nx==22)
					{
						sprintf(Buff,"Client.txt=\"%s\"",&id_Tag[0][0]);
						sendToNx(Buff);
						sprintf(Buff,"Field.txt=\"%s\"",&id_Tag[1][0]);
						sendToNx(Buff);
						sprintf(Buff,"Well.txt=\"%s\"",&id_Tag[2][0]);
						sendToNx(Buff);
						sprintf(Buff,"SN.txt=\"%s\"",id_SN);
						sendToNx(Buff);
						sprintf(Buff,"t2.txt=\"%s\"",FIRM_VERS);
						sendToNx(Buff);
					}
					xTimerReset(timerUpdate,100);//para pausar el refresco de variables hasta que procese el cambio
					break;
				case logOut:
					xTimerStop(timerLogOut,100);
					pwUsrLvl=0;

					sprintf(Buff,"pwUsr=0\0",rpmTemp);
					sendToNx(Buff);
					sprintf(Buff,"dp=0\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getPwd:
					sprintf(Buff,"get t7.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getNewCoef:
					sprintf(Buff,"get cMesCoef\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getACH:
					sprintf(Buff,"get aCh\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getPag:
					sprintf(Buff,"get dp\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getModVar:
					sprintf(Buff,"get modVal2\0",rpmTemp);
					sendToNx(Buff);
					break;
				case NewValOK:
					sprintf(Buff,"modVal2=0\0",rpmTemp);
					sendToNx(Buff);
					xTimerReset(timerUpdate,100);
					break;
				case getPgAndCp:
					strcpy(Buff,"get modVal");
					sendToNx(Buff);
					break;
				case getSPPumpRt:
					strcpy(Buff,"get n1.val");
					sendToNx(Buff);
					break;
				case getPumpOn:
					strcpy(Buff,"get bt0.val");
					sendToNx(Buff);
					break;
				case getSPNomSpd:
					strcpy(Buff,"get n2.val");
					sendToNx(Buff);
					break;
				case getTWidth:
					strcpy(Buff,"get x1.val");
					sendToNx(Buff);
					break;
				case getTLen:
					strcpy(Buff,"get x2.val");
					sendToNx(Buff);
					break;
				case getTHgt:
					strcpy(Buff,"get x3.val");
					sendToNx(Buff);
					break;
				case getTDens:
					strcpy(Buff,"get x4.val");
					sendToNx(Buff);
					break;
				case getTCap:
					strcpy(Buff,"get x6.val");
					sendToNx(Buff);
					break;
				case getSeHgt:
					strcpy(Buff,"get x5.val");
					sendToNx(Buff);
					break;
				case getSeSH:
					strcpy(Buff,"get x0.val");
					sendToNx(Buff);
					break;
				case getSeSL:
					strcpy(Buff,"get x1.val");
					sendToNx(Buff);
					break;
				case getSeEH:
					strcpy(Buff,"get x2.val");
					sendToNx(Buff);
					break;
				case getSeEL:
					strcpy(Buff,"get x3.val");
					sendToNx(Buff);
					break;
				case getWrLvl:
					strcpy(Buff,"get x7.val");
					sendToNx(Buff);
					break;
				case getAlLvl:
					strcpy(Buff,"get x8.val");
					sendToNx(Buff);
					break;
				case getPwmTh:
					strcpy(Buff,"get n0.val");
					sendToNx(Buff);
					break;
				case getCycle:
					strcpy(Buff,"get n1.val");
					sendToNx(Buff);
					break;
				case getPlSiz:
					strcpy(Buff,"get n0.val");
					sendToNx(Buff);
					break;
				case getPlStr:
					strcpy(Buff,"get n1.val");
					sendToNx(Buff);
					break;
				case getHeads:
					strcpy(Buff,"get n2.val");
					sendToNx(Buff);
					break;
				case getaHe:
					strcpy(Buff,"get x0.val");
					sendToNx(Buff);
					break;
				case getaLe:
					strcpy(Buff,"get x1.val");
					sendToNx(Buff);
					break;
				case getaHs:
					strcpy(Buff,"get n0.val");
					sendToNx(Buff);
					break;
				case getaLs:
					strcpy(Buff,"get n1.val");
					sendToNx(Buff);
					break;
				case getaTy:
					strcpy(Buff,"get n2.val");
					sendToNx(Buff);
					break;
				case getaSrs:
					strcpy(Buff,"get n3.val");
					sendToNx(Buff);
					break;
				case getSLID:
					sprintf(Buff,"get n0.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getBaud:
					sprintf(Buff,"get n1.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getLink:
					sprintf(Buff,"get n2.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getNum1:
					sprintf(Buff,"get t3.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getNum2:
					sprintf(Buff,"get t4.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getNum3:
					sprintf(Buff,"get t5.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getNum4:
					sprintf(Buff,"get t6.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case sendwSMS:
					sprintf(Buff,"get aux.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getGR:
					strcpy(Buff,"get n0.val");
					sendToNx(Buff);
					break;
				case getEPPR:
					strcpy(Buff,"get n1.val");
					sendToNx(Buff);
					break;
//				case getMaxSpd:
//					strcpy(Buff,"get n2.val");
//					sendToNx(Buff);
//					break;
				case getSPrpm:
					strcpy(Buff,"get n2.val");
					sendToNx(Buff);
					break;
				case getPidP:
					sprintf(Buff,"get x0.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getPidI:
					sprintf(Buff,"get x1.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getPidD:
					sprintf(Buff,"get x2.val\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getClien:
					sprintf(Buff,"get t3.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getField:
					sprintf(Buff,"get t4.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getWell:
					sprintf(Buff,"get t5.txt\0",rpmTemp);
					sendToNx(Buff);
					break;
				case getSN:
					sprintf(Buff,"get t6.txt\0",rpmTemp);
					sendToNx(Buff);
					break;

			}
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
		ReadB = Chip_UART_Read(UART, Buffer, 4);
		if (ReadB > 0) {
			xStreamBufferSendFromISR(strMsgFromHMI, Buffer, ReadB,
					&xHigherPriorityTaskWoken);
		}

	} else if (Interr & UART_IIR_INTID_THRE) {
		Chip_UART_IntDisable(UART, UART_IER_THREINT);
		vTaskNotifyGiveFromISR(tsk_nex_hmi_tx_handler, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/


void Create_CommNx_HMI_Task(void) {
	xTaskCreate(vCommNx_HMI, (char*) "NX_HMI TSK", 165, NULL,
	Prio_CommNx_HMI_Task, &tsk_nex_hmi_handler); /* "Comunicación con el HMI*/
}
void Create_CommNx_HMIRx_Task(void) {
	xTaskCreate(vCommNx_HMI_Rx, (char*) "NX_HMI Rx TSK", 300, NULL,
	Prio_CommNx_HMI_Rx_Task, &tsk_nex_hmi_rx_handler); /* "Comunicación con el HMI*/
}
void Create_CommNx_HMITx_Task(void) {
	xTaskCreate(vCommNx_HMI_Tx, (char*) "NX_HMI Tx TSK", 100, NULL,
	Prio_CommNx_HMI_Tx_Task, &tsk_nex_hmi_tx_handler); /* "Comunicación con el HMI*/
}

#endif
