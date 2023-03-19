/*
 * Codigo.c
 *
 *  Created on: 17/04/2012
 *      Author: admin
 */

#include "Definiciones.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* Last sector address */
#define START_ADDR_LAST_SECTOR  0x00078000

/* Size of each sector */
#define SECTOR_SIZE             1024

/* LAST SECTOR */
#define IAP_LAST_SECTOR         29

/* Number of bytes to be written to the last sector */
#define IAP_NUM_BYTES_TO_WRITE  256

/* Number of bytes to be written to the last sector */
#define IAP_NUM_BYTES_TO_READ  (4*sizeof(uint32_t))
//Cambiar el 4 por la cantidad de datos a almacenar en FLASH

/* Number elements in array */
#define ARRAY_ELEMENTS          (IAP_NUM_BYTES_TO_WRITE / sizeof(uint32_t))

//void vConfigureTimerForRunTimeStats(void) {
//	char temp = 0;
//
//	temp++;
//	if (temp > 100)
//		temp = 0;
//}

/*********************************************************************//**
 * @brief		Gestion de flags
 * @param[in]	Reg: puntero al registro donde buscar el bit o flag
 * @param[in]	Id: offset del bit
 * @param[in]	Stat: Valor al cual setear el bit o flag
 * @return		None
 **********************************************************************/
void SetFlag(/*uint32_t*/char *Reg, uint8_t Id, Bool Stat) {
	if (Stat)
		*Reg |= 0x01 << Id;
	else
		*Reg &= ~(0x01 << Id);
}

/*********************************************************************//**
 * @brief		Lee flags o bits de un registro
 * @param[in]	Reg: puntero al registro donde buscar el bit o flag
 * @param[in]	Id: offset del bit
 * @return		Estado del flag
 **********************************************************************/
Bool GetFlag(/*uint32_t*/char *Reg, uint8_t Id) {

	return (*Reg & (0x01 << Id));
}

/*********************************************************************//**
 * @brief		Gestiona el tratamiento de alarmas
 * @param[in]	Select			Indica que submenu es el seleccionado
 * 					0: no resalta nada
 * 					1: resalta Item Nº1
 * 					2: resalta Item Nº2
 * 					3: .
 * 					n: resalta Item Nºn
 * @param[in]	X				Coordenada X en pixeles
 * @param[in]	Y				Coordenada Y en pixeles
 * @return		None
 **********************************************************************/

Bool GetAlarm(AlarmPos_Type Id) {

	return (RegAlarmas & (0x01 << Id));
}
void SetAlarm(AlarmPos_Type Id) {
	mqtt_event_type mqtt_st;

	if (!GetAlarm(Id)) { //Verificar que la alarma este desactivada.
		/*Refrescar el actual de las alarmas via web*/
		mqtt_st = mqtt_ev_publish_alarms;
		xQueueSend(queMQTT_ev, &mqtt_st, 0);
	}

	RegAlarmas |= 0x01 << Id;
}

void ClearAlarm(AlarmPos_Type Id) {
	mqtt_event_type mqtt_st;

	if (GetAlarm(Id)) { //Verificar que la alarma este activa.
		/*Refrescar el actual de las alarmas via web*/
		mqtt_st = mqtt_ev_publish_alarms;
		xQueueSend(queMQTT_ev, &mqtt_st, 0);
	}

	RegAlarmas &= ~(0x01 << Id);
}

/*********************************************************************//**
 * @brief		Guardo en eeprom las modificaciones
 * @param[in]	buf: puntero de los registros a guardar
 * @param[in]	offset
 * @param[in]	len
 * @return		True: Pudo tomar el semaforo y guardar
 * 				False: No pudo tomar el semaforo
 **********************************************************************/

Bool guardar_eeprom(char *buf, uint16_t offset, uint16_t len) {

	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_write(buf, offset, len);
		xSemaphoreGive(mtxI2C1);
		return true;
	} else
		return false;
}

/*********************************************************************//**
 * @brief		Leo desde la eeprom
 * @param[in]	buf: puntero donde almacenar los registros leidos
 * @param[in]	offset
 * @param[in]	len
 * @return		True: Pudo tomar el semaforo y guardar
 * 				False: No pudo tomar el semaforo
 **********************************************************************/
Bool leer_eeprom(char *buf, uint16_t offset, uint16_t len) {
	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_read(buf, offset, len);
		xSemaphoreGive(mtxI2C1);
		return true;
	} else
		return false;
}

/*********************************************************************//**
 * @brief		Envia los parametros de configuracion del tanque.
 * @param[in]	newParameter	Direccion de la variable en la rutina de origen
 * @param[in]	value			Nuevo valor del parametro
 * @param[in]	parameter		Variable que identifica el parametro a cambiar
 * @param[in]	src				Fuente de donde proviene el cambio
 * @return		None
 **********************************************************************/
void snd_tank_sett(Parameter_upd_Type *newParameter, float value,
		Modb_RegAdd parameter, cmd_SP_Param_Src_Type src) {
	newParameter->value = value;
	newParameter->src = src;
	newParameter->parameter = parameter;
	xQueueSend(queTANK_SETT_upd, newParameter, 0);
}

/*********************************************************************//**
 * @brief		Envia los parametros de configuracion del cabezal de la bomba.
 * @param[in]	newParameter	Direccion de la variable en la rutina de origen
 * @param[in]	value			Nuevo valor del parametro
 * @param[in]	parameter		Variable que identifica el parametro a cambiar
 * @param[in]	src				Fuente de donde proviene el cambio
 * @param[in]	sender			Indice del celular que envio el comando
 * @return		None
 **********************************************************************/
void snd_head_flow_sett(Parameter_upd_Type *newParameter, float value,
		Modb_RegAdd parameter, cmd_SP_Param_Src_Type src, uint8_t sender) {
	newParameter->value = value;
	newParameter->src = src;
	newParameter->parameter = parameter;
	newParameter->sender = sender;
	xQueueSend(queHEAD_FLOW_SETT_upd, newParameter, 0);
}

/*********************************************************************//**
 * @brief		Lee las unidades de CAUDAL y VOLUMEN seteadas e imprime
 * 				los caracteres.
 * @param[in]	sp		Setpoint donde leer como estan configuradas las variables
 * @param[in]	var		Variable donde va a imprimir las unidades
 * @return		None
 **********************************************************************/
void read_flow_Vol_unit(SP_Type *sp, Var_Type *var) {
	switch (sp->flow_unit) {
	case Liters_hour:
		sprintf(var->flow_unit, "LPH");
		sprintf(var->vol_unit, "LIT");
		sp->vol_unit = Liters;
		break;
	case Liters_day:
		sprintf(var->flow_unit, "LPD");
		sprintf(var->vol_unit, "LIT");
		sp->vol_unit = Liters;
		break;
	case Galons_hour:
		sprintf(var->flow_unit, "GPH");
		sprintf(var->vol_unit, "GAL");
		sp->vol_unit = Galons;
		break;
	case Galons_day:
		sprintf(var->flow_unit, "GPD");
		sprintf(var->vol_unit, "GAL");
		sp->vol_unit = Galons;
		break;
	}
}

/*********************************************************************//**
 * @brief		Guarda los parámetros en flash.
 * @param[in]	Orig			Indica que submenu es el seleccionado
 * @param[in]	Valor			Indica que submenu es el seleccionado
 * @param[in]	X				Coordenada X en pixeles
 * @param[in]	Y				Coordenada Y en pixeles
 * @param[in]	ColorBG			Color del BackGround
 * @return		None
 **********************************************************************/
void flash_write(uint32_t *Vector) {
	uint8_t ret_code;

	/* Disable interrupt mode so it doesn't fire during FLASH updates */
	__disable_irq();
	/* IAP Flash programming */
	/* Prepare to write/erase the last sector */
	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT(
				"Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n",
				ret_code);
	}
	/* Erase the last sector */
	ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT(
				"Chip_IAP_EraseSector() failed to execute, return code is: %x\r\n",
				ret_code);
	}
	/* Prepare to write/erase the last sector */
	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT(
				"Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n",
				ret_code);
	}
	/* Write to the last sector */
	ret_code = Chip_IAP_CopyRamToFlash(START_ADDR_LAST_SECTOR, Vector,
	IAP_NUM_BYTES_TO_WRITE);
	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT(
				"Chip_IAP_CopyRamToFlash() failed to execute, return code is: %x\r\n",
				ret_code);
	}
	/* Re-enable interrupt mode */
	__enable_irq();
	Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

}

/*********************************************************************//**
 * @brief		Lee los parámetros en flash.
 * @param[in]	Orig			Indica que submenu es el seleccionado
 * @param[in]	Valor			Indica que submenu es el seleccionado
 * @param[in]	X				Coordenada X en pixeles
 * @param[in]	Y				Coordenada Y en pixeles
 * @param[in]	ColorBG			Color del BackGround
 * @return		None
 **********************************************************************/
void flash_read(uint32_t *Vector) {
	memcpy(Vector, (uint32_t*) START_ADDR_LAST_SECTOR, IAP_NUM_BYTES_TO_READ);

}
/*********************************************************************//**
 * @brief		Ordena de menor a mayo los datos de un arreglo.
 * @param[in]	ColorBG			Color del BackGround
 * @return		None
 **********************************************************************/
void MedianaFloat(uint8_t Size, float *Start, float *Valor) {

	float almacena;
	uint8_t pasada, j;
	float Vect[Size];

	memcpy(Vect, Start, 4 * Size);

	for (pasada = 1; pasada < Size; pasada++) {
		for (j = 0; j < Size - 1; j++) {
			if (*(Vect + j) > *(Vect + j + 1)) {
				almacena = *(Vect + j);
				*(Vect + j) = *(Vect + j + 1);
				*(Vect + j + 1) = almacena;
			}
		}
	}

	if (Size % 2 == 0) {
		//Par
		*Valor = (Vect[Size / 2] + Vect[1 + (Size / 2)]) / 2;
	} else {
		//Impar
		*Valor = Vect[1];
	}

}

/*********************************************************************//**
 * @brief		Ordena de menor a mayor los datos de un arreglo.
 * @param[in]
 * @return
 **********************************************************************/
void Mediana(uint8_t Size, uint32_t *Start, uint32_t *Valor) {

	uint32_t almacena;
	uint8_t pasada, j;
	uint32_t Vect[Size];

	memcpy(Vect, Start, 2 * Size);

	for (pasada = 1; pasada < Size; pasada++) {
		for (j = 0; j < Size - 1; j++) {
			if (*(Vect + j) > *(Vect + j + 1)) {
				almacena = *(Vect + j);
				*(Vect + j) = *(Vect + j + 1);
				*(Vect + j + 1) = almacena;
			}
		}
	}

	if (Size % 2 == 0) {
		//Par
		*Valor = (Vect[Size / 2] + Vect[1 + (Size / 2)]) / 2;
	} else {
		//Impar
		*Valor = Vect[1];
	}

}

uint16_t CrcCode;
Status Check_CRC(char *Dir, uint16_t Len) {
	if (Len < 3) {
		return ERROR; //Bad CRC
	} else {
		CrcCode = Crc16((uint8_t*) Dir, Len - 2);
		if (((CrcCode >> 8) & 0xff) != *(Dir + Len - 1)) {
			return ERROR; //Bad CRC
		} else if ((CrcCode & 0xff) != *(Dir + Len - 2)) {
			return ERROR; //Bad CRC
		} else {
			return SUCCESS; //Msg Ok
		}
	}
}

//todo: Desarrollar esta funcion para que sea adaptable a los diferentes productos
bool processSmsCommand(MOBILE_T *_mobile, char *ptr) {
	int32_t temp_long;
	float tempf;
	char *ptrm;
	Parameter_upd_Type newParameter;
	char tempchar[20],*temPhon;
	EvtToMOB_Type EvtToMOBILE;
	gen_ctrl_ev_Type EventoGen;
	ptrm = ptr;
	temPhon=strstr(_mobile->status.SMS_orig, gprs_rpn[0]);
	if ((strstr(temPhon, gprs_rpn[0]))
			&& (strlen(_mobile->status.SMS_orig) > 9)&&(strlen(gprs_rpn[0]) > 9)) {
		_mobile->status.Send_SMS_to_idx = 1;
	}
	else
	{
		temPhon=strstr(_mobile->status.SMS_orig, gprs_rpn[1]);
		if ((strstr(_mobile->status.SMS_orig, gprs_rpn[1]))
				&& (strlen(_mobile->status.SMS_orig) > 9)&&(strlen(gprs_rpn[1]) > 9)) {
				_mobile->status.Send_SMS_to_idx = 2;
		}
		else{
			temPhon=strstr(_mobile->status.SMS_orig, gprs_rpn[2]);
			if ((strstr(_mobile->status.SMS_orig, gprs_rpn[2]))
					&& (strlen(_mobile->status.SMS_orig) > 9)&&(strlen(gprs_rpn[2]) > 9)) {
				_mobile->status.Send_SMS_to_idx = 3;
			}
			else {
				temPhon=strstr(_mobile->status.SMS_orig, gprs_rpn[3]);
				if ((strstr(_mobile->status.SMS_orig, gprs_rpn[3]))
						&& (strlen(_mobile->status.SMS_orig) > 9)&&(strlen(gprs_rpn[3]) > 9)) {
					_mobile->status.Send_SMS_to_idx = 4;
				}
				else {
					//Sender is not recognized. Delete SMS
					//memset(buffer, 0, 30);
					//Limpio 30 posiciones de memoria para asegurarme que el comando
					//de "Borrar SMS" salga limpio
					return sendCheckReply(_mobile, "AT+CMGD=0,4", OK_CHAR, 6000);//Borro todos los sms leidos
				}
			}
		}
	}

	//Grabo el indice de quien envio el mensaje
	EvtToMOBILE.Val = _mobile->status.Send_SMS_to_idx;
	for(int i=0;i<=strlen(_mobile->data.bufferRX);i++)
		*(_mobile->data.bufferRX+i)=tolower(*(_mobile->data.bufferRX+i));
	ptrm =_mobile->data.bufferRX;

	//Inicializo en un valor de modo que por defecto no envie el EvtToMOBILE
	EvtToMOBILE.Src = cmd_NO_CMD;
	/*command: REPORT*/
	if (0!=strstr(_mobile->data.bufferRX, "report")) {
		EvtToMOBILE.Src = cmd_Report;
	}
	/*command: LOCATION*/
	else if (0!=strstr(_mobile->data.bufferRX, "location")) {
		EvtToMOBILE.Src = cmd_Location;
	}
	/*command: FLOW*/
	else if (0!=strstr(_mobile->data.bufferRX, "flow")) {

		if (sscanf(_mobile->data.bufferRX, "flow %f", &tempf) == 1) {
			snd_head_flow_sett(&newParameter, tempf, SP_PumpRate, src_sms,
					_mobile->status.Send_SMS_to_idx);
		}
	}
	/*command: START*/
	else if ((strstr(_mobile->data.bufferRX, "start")) != 0) {
		//START PUMP
		if ((strstr(_mobile->data.bufferRX, "pump")) != 0) {
			snd_head_flow_sett(&newParameter, 1, PUMP_ON, src_sms,
					_mobile->status.Send_SMS_to_idx);
		}
		//START NOTIF
		else if ((strstr(_mobile->data.bufferRX, "notif")) != 0) {
			EvtToMOBILE.Src = cmd_Start_Notif;
		}
		//START GEN
		else if ((strstr(_mobile->data.bufferRX, "gen")) != 0) {
			if (sscanf(_mobile->data.bufferRX, "start gen %d", &gen.gsm_timeout) != 1) {
				gen.gsm_timeout = 5 * 60;
			}
			EventoGen = genEv_Gsm_On;
			xQueueSend(queEvtGenStart, &EventoGen, 0);
		} else {	//Comando incompleto
			strncpy(EvtToMOBILE.Message, "Please use:\n"
					"Start pump\n"
					"Start gen TIMEOUT\n"
					"Start notif", 149);
			EvtToMOBILE.Src = sendSMS;
		}
	}
	/*command: STOP*/
	else if ((strstr(ptrm, "stop")) != 0) {

		//STOP PUMP
		if ((strstr(ptrm, "pump")) != 0) {
			snd_head_flow_sett(&newParameter, 0, PUMP_ON, src_sms,
					_mobile->status.Send_SMS_to_idx);
		}
		//STOP NOTIF
		else if ((strstr(ptrm, "notif")) != 0) {
			EvtToMOBILE.Src = cmd_Stop_Notif;
		}
		//STOP GEN
		else if ((strstr(ptrm, "gen")) != 0) {
			EventoGen = genEv_Gsm_Off;
			xQueueSend(queEvtGenStart, &EventoGen, 0);
		} else {	//Comando incompleto
			strncpy(EvtToMOBILE.Message, "Please use:\n"
					"Stop pump\n"
					"Stop gen\n"
					"Stop notif", 149);
			EvtToMOBILE.Src = sendSMS;
		}
	}
	/*command: SET BAUD RATE*/
	else if (strstr(ptrm, "baud")) {
		memset(EvtToMOBILE.Message, 0, MESS_BUFF_SZ);
		if (sscanf(ptrm, "baud %d", &temp_long) == 1) {

			if (temp_long == 9600 || temp_long == 19200
					|| temp_long == 115200) {
				SP.RS485_BAUD = temp_long;
				guardar_eeprom((char*) &SP,
				OFF_SP, sizeof(SP));
				sprintf(EvtToMOBILE.Message, "New Baudrate %d", SP.RS485_BAUD);
			} else {
				strncpy(EvtToMOBILE.Message,
						"Wrong value.\n Use 9600, 19200 or 115200", 149);
			}
		} else {
			strncpy(EvtToMOBILE.Message, "UNR.\n"
					"Use: baud [value]", 149);
		}
		EvtToMOBILE.Src = sendSMS;
	}
	/*command: READ EVENTS LOG*/
	else if (strstr(ptrm, "evlog")) {
		EvtToMOBILE.Src = cmd_read_events_log;
	}

	/*Unrecognized command*/
	else {
		strncpy(EvtToMOBILE.Message, "UNR.\n"
				"Use: Report, Location, Start notif, Stop notif, Start pump, Stop pump",
				149);
		EvtToMOBILE.Src = sendSMS;
	}

	if (EvtToMOBILE.Src != cmd_NO_CMD) {
		xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
	}
	sendCheckReply(_mobile, "AT+CMGD=0,4", OK_CHAR, 6000);
	return true;
}

bool verifyPumpOk(void)
{
	if(GetAlarm(Al_alarma_bajo_nivel_T1))
	{
		return false;
	}
	if(GetAlarm(Al_VeryLowBatt))
	{
		return false;
	}
	if(GetAlarm(Al_alarma_motor_atascado_B1))
	{
		return false;
	}
	return true;
}
