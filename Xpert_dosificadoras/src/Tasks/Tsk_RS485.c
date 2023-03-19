/*
 * Tsk_RS485.c
 *
 *  Created on: 08/11/2019
 *  Author: Walter Peresson
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "board.h"

/* Librerias de C includes. */
#include "Definiciones.h"
#include "uart.h"
#if ATV12
#include "ATV12H037.h"
#endif
#if V20
#include "SIN_V20.h"
#endif
#if ATV312
#include "ATV312HU11.h"
#endif
#define EXIT_TIME	(10 * 60 * 1000)

#define T15			750E-6	//750 uS
#define T35			1.75E-3//1.75 mS

#define UART				LPC_UART1
#define TIMER 				LPC_TIMER3
#define TMR_IRQ_SELEC 		TIMER3_IRQn
#define TMR_HANDLER_NAME 	TIMER3_IRQHandler

#define UART_IRQ_SELEC		UART1_IRQn
#define UART_HANDLER_NAME 	UART1_IRQHandler

#define INPUT_REG_ARR_SIZE					144	//from 30001 to 30072
#define HOLD_REG_ARR_SIZE_B1				146	//from 40001 to 40073
#define HOLD_REG_ARR_SIZE_B2				96	//from 41001 to 41048

#define START_ADD_B1	0
#define START_ADD_B2	1000

typedef struct {
	uint32_t ptr; /* Puntero a la variable apuntada */
	uint16_t idx;/* Indice dentro de la tabla de registros */
} MATRIX_REGISTER_T;

StreamBufferHandle_t strMsgFromRS485;

static char helloDebugMenu[] = ""
		"\r\n========================================================="
		"\r\n  Xpert SMART MONITORING & CONTROLLER \n"
		"\r\n========================================================="
		"\r\n";

static char helloSetupMenu[] = ""
		"\r\n========================================================="
		"\r\n  Xpert SMART SYSTEM SETUP \n"
		"\r\n========================================================="
		"\r\n";

static char select_option_title[] = "\rSelect an option:\n\n";

static char debugMainMenu[] = "\r1: Debug GSM/GPRS module\n"
		"\r2: Sniff GSM/GPRS\n"
		"\r3: Debug Wireless HART module\n\n";

static char setupMainMenu[] = "\r1: INPUT STATUS. Analog + digital Inputs\n"
		"\r2: SETTINGS\n\n";

static char exitMsg[] = "\r Type quit to exit\n\n";

static char tank_shapes_str[4][20] = { { "Two ref points" }, {
		"Vertical Cylinder" }, { "Horizontal Cylinder" }, { "Rectangular" } };

/*
 * origen: Variable de la cual quiero extraer el valor
 * dest: Registro de memoria que se publicará en MODBUS MAP
 * offset: Byte desde donde comenzar a leer
 * size: Cantidad de bytes a leer
 */
void store_value(char *origen, char *dest, uint8_t offset, uint8_t size) {
	dest += offset;
	*(dest + 1) = *origen++;
	*(dest) = *origen++;
	if (size == 4) {
		*(dest + 3) = *origen++;
		*(dest + 2) = *origen++;
	}
}

/*
 * dest: Variable
 * origen: Registro que mantiene las variables
 * offset: Byte desde donde comenzar a leer
 * size: Cantidad de bytes a leer
 */
void read_value(char *dest, char *origen, uint8_t offset, uint8_t size) {
	origen += offset;
	*(dest + 1) = *origen++;
	*(dest) = *origen++;
	if (size == 4) {
		*(dest + 3) = *origen++;
		*(dest + 2) = *origen++;
	}
}

void Send_RS485(uint32_t Len, /*uint32_t*/void *PtData) {
	char *data;

	data = PtData;

	while (Len > 0) {
		xQueueSend(queMsgToRS485, data++, 10);
		Len--;
	}
}

void init_uart_rs485(LPC_USART_T *uart) {

	if (SP.RS485_WL) { // WIRELESS
		RST_WPRO_OFF; //Comienzo con el XBee por defecto
		RXEN1_OFF;
		TXEN1_OFF;
	} else {	// WIRED
		RST_WPRO_ON;
		RXEN1_ON;
		TXEN1_OFF;
	}

	// UART_1: COMUNICACION RS485 //////////////////
	Chip_UART_Init(uart);
	Chip_UART_SetBaud(uart, SP.RS485_BAUD);
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

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TMR_IRQ_SELEC);
	NVIC_SetPriority(TMR_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 3);
	NVIC_EnableIRQ(TMR_IRQ_SELEC);
	/* Enable UART interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC, mainSOFTWARE_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	Chip_UART_IntEnable(uart, UART_IER_RBRINT);
}

uint32_t wait_key(char *data, TickType_t delay) {

	TickType_t _delay = delay;
	uint32_t ReadB, ReadC;

	ReadB = 0;
	ReadC = 0;
	do {
		ReadC = xStreamBufferReceive(strMsgFromRS485, data + ReadB,
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

bool cancel_key(char *Buf485_Rx) {
	if (strncmp(Buf485_Rx, "c\r", 2) == 0)
		/*ambos string son iguales*/
		return TRUE;
	else
		return FALSE;
}

/*-----------------------------------------------------------
 *
 */
static portTASK_FUNCTION(vCommMODB_RS485_Tx, pvParameters) {

	uint8_t data;
	TickType_t delay;

	while (1) {

		/*Espero el primer dato*/
		delay = portMAX_DELAY;

		while (xQueueReceive(queMsgToRS485, &data, delay) == pdPASS) {

			/*Hasta que se acabe la cola debo leer con cero delay*/
			delay = 0;

			while ((Chip_UART_ReadLineStatus(UART) & UART_LSR_THRE) == 0) {
				/*Espero que la interrupción por THRE me avise que puedo seguir*/
				ulTaskNotifyTake(TRUE, portMAX_DELAY);
			}

			/*Enciendo el TXEN*/
			if (!SP.RS485_WL) {
				TXEN1_ON;
				RXEN1_OFF;
			}
			/*Ahora si, envío un byte*/
			Chip_UART_SendByte(UART, data);

			/*Habilito la int por THRE*/
			Chip_UART_IntEnable(UART, UART_IER_THREINT);
		}
		ulTaskNotifyTake(TRUE, portMAX_DELAY);
		/*Apago el TXEN*/
		if (!SP.RS485_WL) {
//			ulTaskNotifyTake(TRUE, portMAX_DELAY);
			TXEN1_OFF;
			RXEN1_ON;
		}
	}
}

bool no_back_cmd(char *Buf485_Rx) {
	if (strncmp(Buf485_Rx, "quit\r", 5) == 0)
		return FALSE;
	else
		return TRUE;
}

/*-----------------------------------------------------------
 *
 */
static portTASK_FUNCTION(vCommMODB_RS485_Rx, pvParameters) {

	uint32_t Len;
	uint16_t tempshort;
	uint32_t tempint;
	uint16_t i, j;
	uint16_t Address;
	MB_Function_Code Oper;
	uint32_t ToutT15, ToutT32;
	EvtToMOB_Type EvtToSIM868;
	ev_state_bluet_type EvtToBlue;
	float tempfloat;
	int32_t mem_SP_BR;
	mqtt_event_type loc_mqtt_st;
	Parameter_upd_Type newParameter;
	char Buf485_Rx[BUF485_SIZE]__attribute__ ((aligned (32)));
	static uint16_t ReadB;
#if USAR_SNIFFER
	static uint16_t ReadC;
#endif
	TickType_t delay;
	uint16_t Index;

	uint32_t Coils_Registers __attribute__ ((aligned (32)));
	uint32_t DiscInp_Registers __attribute__ ((aligned (32)));
	char Hold_Registers_bloque1[HOLD_REG_ARR_SIZE_B1] __attribute__ ((aligned (32)));
	char Hold_Registers_bloque2[HOLD_REG_ARR_SIZE_B1] __attribute__ ((aligned (32)));

	char Input_Registers[INPUT_REG_ARR_SIZE] __attribute__ ((aligned (32)));

	char *Ptr_Char;

#if USAR_SNIFFER
	bool sniffer_stat = TRUE;
#endif

	//uint32_t hold_reg_idx[] = { &SP.An[0].HE, &SP.An[0].LE };

	/*Debo crear el stream buffer antes de configurar el puerto para evitar que un dato entrante
	 intente enviar al buffer str que no ha sido creado */
	strMsgFromRS485 = xStreamBufferCreate(BUF485_SIZE, 1);

	global_Scada_Control_cnt = 0;

	semDMA_TX1 = xSemaphoreCreateBinary();

	semExpT15_485 = xSemaphoreCreateBinary();
	semExpT35_485 = xSemaphoreCreateBinary();

	xSemaphoreTake(semDMA_TX1, 0);

	/* Enable timer 1 clock */
	Chip_TIMER_Init(TIMER);
	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(TIMER);

	mem_SP_BR = SP.RS485_BAUD;

	/*Inicializo el puerto */
	init_uart_rs485(UART);

	if (SP.RS485_BAUD >= 19200) {
		/*Si el valor es mayor que 19200 debemos usar un valor fijo de t1.5=750us y t3.5=1.75ms*/
		ToutT15 = 36000;
		ToutT32 = 84000;
	} else {
		/*9600 t1.5= 1.75ms y t3.5=4ms*/
		ToutT15 = 84000;
		ToutT32 = 196000;
	}
	Chip_TIMER_SetMatch(TIMER, 0, ToutT15); // t15
	Chip_TIMER_SetMatch(TIMER, 1, ToutT32); // t35
	Chip_TIMER_StopOnMatchDisable(TIMER, 1);

	xSemaphoreTake(semExpT15_485, 0);
	xSemaphoreTake(semExpT35_485, 0);

	Create_CommMODB_RS485_Tx_Task();

	while (1) {

		memset(Buf485_Rx, 0xff, BUF485_SIZE); //Preparo el buffer donde voy a alojar los datos.
		delay = portMAX_DELAY;
		ReadB = 0;

#if USAR_SNIFFER
		if (sniffer_stat) {
			sprintf(Buf485_Rx, "BT\r");
		} else {
			do {
				ReadC = wait_key(Buf485_Rx, delay);
				ReadB += ReadC;
				delay = 20;
			} while (ReadC);
		}
#else
//		do {
//			ReadC = wait_key(Buf485_Rx, delay);
//			ReadB += ReadC;
//			delay = 20;
//		} while (ReadC);

		ReadB = wait_key(Buf485_Rx, delay);
#endif

		/*Chequeo si el BR cambio.*/
		if (SP.RS485_BAUD != mem_SP_BR) {

			mem_SP_BR = SP.RS485_BAUD;

			/* Enable timer 1 clock */
			Chip_TIMER_Init(TIMER);

			/* Timer setup for match and interrupt at TICKRATE_HZ */
			Chip_TIMER_Reset(TIMER);

			init_uart_rs485(UART);

			if (SP.RS485_BAUD >= 19200) {
				// Si el valor es mayor que 19200 debemos usar un valor fijo de t1.5=750us y t3.5=1.75ms
				ToutT15 = 36000;
				ToutT32 = 84000;
			} else {
				// 9600 t1.5= 1.75ms y t3.5=4ms
				ToutT15 = 84000;
				ToutT32 = 196000;
			}
			Chip_TIMER_SetMatch(TIMER, 0, ToutT15); // t15
			Chip_TIMER_SetMatch(TIMER, 1, ToutT32); // t35
			Chip_TIMER_StopOnMatchDisable(TIMER, 1);
		}

		/*==========================================================================
		 * MODBUS SERVER
		 * ==========================================================================
		 */
		if (Check_CRC(Buf485_Rx, ReadB/*Index*/)) { //Check CRC
//++++++++++		SB <--- 485		+++++++++++++++++++++++++++++++++
			if (Buf485_Rx[0] == SP.RS485_SLID) {
				/*
				 * Seteo el valor de global_Scada_Control_cnt solo en caso de que el SCADA esté escribiendo
				 * el estado de PumpOn continuamente. Este estado es "BAJO CONTROL DE SCADA".
				 * En caso de que el SCADA no escriba mas datos el sistema decrementará el contador y al llegar a
				 * cero pasara al estado "SIN CONTROL DE SCADA".
				 *
				 * global_Scada_Control_cnt==10   =>  "BAJO CONTROL DE SCADA"
				 *
				 * global_Scada_Control_cnt==0    =>  "SIN CONTROL DE SCADA"
				 */
				if (global_Scada_Control_cnt < 10)
					global_Scada_Control_cnt = 10;

				/*Aseguro los valores para que no envie datos desde el SIM868*/
				debug_gsm = false;
				sniff_gsm = false;

				/*
				 * COILS REGISTERS
				 * En este registro se almacenan el estado de las bobinas desde la posición 1 a 16 (2000 sería el max).
				 * Para leer: FC-01
				 * Para escribir: FC-05
				 */

				//Limpio los registros
				Coils_Registers = 0;

				//Ahora actualizo los mismos con el valor de las variables
				Coils_Registers |= (PumpOn & 0x01)/*Marcha B1*/
				| ((0x00) << 1)/*Marcha B2*/
				| ((0x00) << 2)/*Reiniciar Totalizador*/
				| ((Var.DO & 0xff) << 3) /*Salidas digitales*/
				| ((SP.Activate_GPRS) << 9)/*Celular*/
				| ((SP.Activate_GPS) << 10)/*GPS*/
				| ((SP.RS485_WL) << 11)/*Link, wired or wireless*/
				| ((SP.auto_manual) << 12)/*0: manual, 1: Auto*/;

				/*
				 * DISCRETE INPUTS
				 * Almacena el estado de las entradas digitales desde la posición 1 a 2000.
				 * Para leer: FC-02
				 * Para escribir: NA
				 */

				//Limpio los registros
				DiscInp_Registers = 0;

				//Ahora actualizo los mismos con el valor de las variables
				DiscInp_Registers |= GetAlarm(Al_advertencia_bajo_nivel_T1)
						| (GetAlarm(Al_alarma_bajo_nivel_T1) << 1)
						| (GetAlarm(Al_advertencia_bajo_nivel_T2) << 2)
						| (GetAlarm(Al_alarma_bajo_nivel_T2) << 3)
						| (GetAlarm(Al_DoorOpen) << 4)
						| (GetAlarm(Al_alarma_motor_atascado_B1) << 5)
						| (GetAlarm(Al_alarma_motor_atascado_B2) << 6);
				DiscInp_Registers |= ((Var.local_remoto) << 7);
				DiscInp_Registers |= ((Var.DI & 0x1E) << 8);
				DiscInp_Registers |= ((Var.DI & (1 << OD_bp)) << 12);

				/*
				 * HOLDING REGISTERS
				 * Almacena los valores de los registros que se pueden modificar desde el RTU
				 * Para leer: FC-03
				 * Para escribir: FC-10
				 */
				/*HR Bloque 1*/

				/*Valores de configuración de los canales analógicos*/
				for (i = 0; i < 6; i++) {
					store_value((char*) &SP.An[i].HE, Hold_Registers_bloque1,
							12 * i, 4);
					store_value((char*) &SP.An[i].LE, Hold_Registers_bloque1,
							12 * i + 4, 4);
					store_value((char*) &SP.An[i].HS, Hold_Registers_bloque1,
							12 * i + 8, 2);
					store_value((char*) &SP.An[i].LS, Hold_Registers_bloque1,
							12 * i + 10, 2);
				} //termina en 70+2

				/*Coeficiente de conversión de altura a capacidad*/
				/*Ganancia (Densidad de químico)*/
				/*Offset (Nivel inicial de químico)*/
				for (i = 0; i < 6; i++) {
					store_value((char*) &SP.tnk[i].lit_by_m,
							Hold_Registers_bloque1, i * 12 + 72, 4);
					store_value((char*) &SP.tnk[i].dens, Hold_Registers_bloque1,
							i * 12 + 76, 4);
					store_value((char*) &SP.tnk[i].off, Hold_Registers_bloque1,
							i * 12 + 80, 4);
				} //termina en 140+4

				tempshort = SP.flow_unit;
				store_value((char*) &tempshort, Hold_Registers_bloque1, 144, 2);

				/*HR Bloque 2*/
				/*Parámetros Bomba 1*/
				store_value((char*) &sp_pump.PumpRate, Hold_Registers_bloque2,
						0, 4); //41001
				store_value((char*) &sp_pump.Threshold, Hold_Registers_bloque2,
						4, 2);
				tempshort = 0;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 6, 2); //41004: Reservado
				store_value((char*) &sp_pump.CF, Hold_Registers_bloque2, 8, 4);
				store_value((char*) &sp_pump.Cycle, Hold_Registers_bloque2, 12,
						2);
				tempshort = sp_pump.GR;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 14, 2);

				tempshort = sp_pump.level_warning_limit_T1;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 16, 2);
				tempshort = sp_pump.level_alarm_limit_T1;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 18, 2);

				/*Parámetros Bomba 2*/
				tempfloat = 0;
				tempshort = 0;

				store_value((char*) &tempfloat/*sp_pump.PumpRate*/,
						Hold_Registers_bloque2, 20, 4);
				store_value((char*) &tempshort/*sp_pump.Threshold*/,
						Hold_Registers_bloque2, 24, 2);

				tempshort = 0;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 26, 2); //41014: Reservado
				store_value((char*) &tempfloat/*sp_pump.CF*/,
						Hold_Registers_bloque2, 28, 4);
				store_value((char*) &tempshort/*sp_pump.Cycle*/,
						Hold_Registers_bloque2, 32, 2);
				tempshort = 0; //sp_pump.GR;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 34, 2);

				tempshort = 0; //sp_pump.level_warning_limit_T1;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 36, 2);
				tempshort = 0; //sp_pump.level_alarm_limit_T1;
				store_value((char*) &tempshort, Hold_Registers_bloque2, 38, 2);

				/*Parámetros proceso*/
				store_value((char*) &Var.caudal_produccion,
						Hold_Registers_bloque2, 40, 4);
				store_value((char*) &Var.presion_referencia,
						Hold_Registers_bloque2, 44, 4);

				/*
				 * INPUT REGISTERS
				 * Almacena el valor de registros de entrada que no se pueden modificar
				 * Para leer: FC-04
				 * Para escribir: NA
				 */

				tempfloat = 0;
				store_value((char*) &tempfloat, Input_Registers, 0, 4); //30001
				store_value((char*) &tempfloat, Input_Registers, 4, 4); //30003

				/*Valores de los canales analógicos*/
				for (i = 0; i < 6; i++) {
					tempfloat = Var.an[i].ADC_Un;
					store_value((char*) &tempfloat, Input_Registers, 12 * i + 8,
							4);
					store_value((char*) &Var.an[i].Signal, Input_Registers,
							12 * i + 12, 4);
					store_value((char*) &Var.an[i].Eng_Un, Input_Registers,
							12 * i + 16, 4);
				} //termina en 76+4

				/*Resultados de los cálculos de niveles de tanques*/
				for (i = 0; i < 6; i++) {
					if(i==0)
					{
						Var.tnk[i].nivel=Var.tank_Lev;
						Var.tnk[i].capacidad=Var.tank_vol;
					}
					store_value((char*) &Var.tnk[i].nivel, Input_Registers,
							i * 8 + 80, 4);
					store_value((char*) &Var.tnk[i].capacidad, Input_Registers,
							i * 8 + 84, 4);
				} //termina en 124+4
				i = 0;
				store_value((char*) &gps.Lat, Input_Registers, 128, 4);
				store_value((char*) &gps.Lon, Input_Registers, 132, 4);
				store_value((char*) &Var.Total_pumped, Input_Registers, 136, 4);
				tempshort = VSD_bomba1.FaultCode;
				store_value((char*) &tempshort, Input_Registers, 140, 2);
				tempshort = 0; //VSD_bomba2.FaultCode;
				store_value((char*) &tempshort, Input_Registers, 142, 2);

				//=========================================================================

				Address = 0;
				Address |= (Buf485_Rx[2] << 8) | Buf485_Rx[3];
				Oper = Buf485_Rx[1];
				switch (Oper) {
				case FC01_Read_Coils:
					Ptr_Char = (char*) &Coils_Registers; //Inicializo el puntero al inicio del registro
					Ptr_Char += Address / 8; //Desplazo el puntero a la dirección indicada
					Len = ((Buf485_Rx[4] << 8) + Buf485_Rx[5]); //Leo la cantidad de BITS/COILS a enviar.
					if (Address + Len > 16) { //Me aseguro que no pidieron datos mas alla de mi mapa de memoria para COILS
						Buf485_Rx[1] += 0x80;
						Buf485_Rx[2] = 0x02; //Illegal address
						Len = 3;
						MBCrc16((uint8_t*) Buf485_Rx, Len);
						Len += 2;
						break;
					}
					//Guardo todos los bytes que necesito en un byte auxiliar
					tempint = Coils_Registers;

					//Desplazo los bits necesarios para alinear al primer bit solicitado
					tempint >>= Address;

					//Calculo el "Byte Count". Si el reminder !=0 se debe poner N=N+1
					Buf485_Rx[2] = (Len / 8) + ((Len % 8) ? 1 : 0);

					for (i = 0; i < Buf485_Rx[2]; i++) {
						Buf485_Rx[3 + i] = 0x00ff & tempint;
						tempint >>= 8;
					}
					Len = 3 + i;
					MBCrc16((uint8_t*) Buf485_Rx, Len);
					Len += 2;
					break;
				case FC02_Read_Disc_Inp:
					Ptr_Char = (char*) &DiscInp_Registers;
					Ptr_Char += Address / 8;
					Len = ((Buf485_Rx[4] << 8) + Buf485_Rx[5]); //Cantidad de BITS/COIL a enviar.
					if (Address + Len > 16) {
						Buf485_Rx[1] += 0x80;
						Buf485_Rx[2] = 0x02; //Illegal address
						Len = 3;
						MBCrc16((uint8_t*) Buf485_Rx, Len);
						Len += 2;
						break;
					}
					//Guardo todos los bytes que necesito en un byte auxiliar
					tempint = DiscInp_Registers;

					//Desplazo los bits necesarios para alinear al primer bit solicitado
					tempint >>= Address;

					//Calculo el "Byte Count". Si el reminder !=0 se debe poner N=N+1
					Buf485_Rx[2] = (Len / 8) + ((Len % 8) ? 1 : 0);

					for (i = 0; i < Buf485_Rx[2]; i++) {
						Buf485_Rx[3 + i] = 0x00ff & tempint;
						tempint >>= 8;
					}
					Len = 3 + i;
					MBCrc16((uint8_t*) Buf485_Rx, Len);
					Len += 2;
					break;
				case FC03_Read_Hold_Reg:
					Len = 2 * ((Buf485_Rx[4] << 8) + Buf485_Rx[5]); //Cantidad de BYTES a enviar. Bytes =2*Registros
					if ((Address >= START_ADD_B1)
							&& (Address < (START_ADD_B1 + HOLD_REG_ARR_SIZE_B1))) {
						if ((Address * 2 + Len) > HOLD_REG_ARR_SIZE_B1) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque1;
						Ptr_Char += Address * 2;
					} else if ((Address >= START_ADD_B2)
							&& (Address < START_ADD_B2 + HOLD_REG_ARR_SIZE_B2)) {
						if (((Address - START_ADD_B2) * 2 + Len)
								> HOLD_REG_ARR_SIZE_B2) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque2;
						Ptr_Char += (Address - START_ADD_B2) * 2;
					} else {
						Buf485_Rx[1] += 0x80;
						Buf485_Rx[2] = 0x02; //Illegal address
						Len = 3;
						MBCrc16((uint8_t*) Buf485_Rx, Len);
						Len += 2;
						break;
					}
					memcpy(&Buf485_Rx[3], Ptr_Char, Len);
					Buf485_Rx[2] = Len;
					MBCrc16((uint8_t*) Buf485_Rx, Len + 3);
					Len += 5;
					break;
				case FC04_Read_Inp_Reg:
					Ptr_Char = (char*) &Input_Registers;
					Ptr_Char += Address * 2;
					Len = 2 * ((Buf485_Rx[4] << 8) + Buf485_Rx[5]); //Cantidad de BYTES a enviar. Bytes = 2*Registros
					if ((Address * 2 + Len) > INPUT_REG_ARR_SIZE) {
						Buf485_Rx[1] += 0x80;
						Buf485_Rx[2] = 0x02; //Illegal address
						Len = 3;
						MBCrc16((uint8_t*) Buf485_Rx, Len);
						Len += 2;
						break;
					}
					memcpy(&Buf485_Rx[3], Ptr_Char, Len);
					Buf485_Rx[2] = Len;
					MBCrc16((uint8_t*) Buf485_Rx, Len + 3);
					Len += 5;
					break;
				case FC05_Write_Sing_Coil:
					if (Address > 32) {
						Buf485_Rx[1] += 0x80;
						Buf485_Rx[2] = 0x02; //Illegal address
						Len = 3;
						MBCrc16((uint8_t*) Buf485_Rx, Len);
						Len += 2;
						break;
					}
					switch ((MB_Coils_Id) Address) {

					case Coil_1_MarchaB1:
						snd_head_flow_sett(&newParameter,
								(0xff & Buf485_Rx[4]) ? 1 : 0, PUMP_ON,
								src_modbus, 0);
						break;
					case Coil_2_MarchaB2:

						break;

					case Coil_3_Rst_Tot:
						snd_head_flow_sett(&newParameter,
								(0xff & Buf485_Rx[4]) ? 1 : 0, RST_TOTAL,
								src_modbus, 0);
						break;

					case Coil_4_DO1:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, DO1_bp, TRUE) :
								SetFlag((char*) &Var.DO, DO1_bp, FALSE);
						break;
					case Coil_5_DO2:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, DO2_bp, TRUE) :
								SetFlag((char*) &Var.DO, DO2_bp, FALSE);
						break;
					case Coil_6_DO3:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, DO3_bp, TRUE) :
								SetFlag((char*) &Var.DO, DO3_bp, FALSE);
						break;
					case Coil_7_DO4:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, DO4_bp, TRUE) :
								SetFlag((char*) &Var.DO, DO4_bp, FALSE);
						break;
					case Coil_8_RLY1:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, Rl1_bp, TRUE) :
								SetFlag((char*) &Var.DO, Rl1_bp, FALSE);
						break;
					case Coil_9_RLY2:
						(Buf485_Rx[4] == 0xff) ?
								SetFlag((char*) &Var.DO, Rl2_bp, TRUE) :
								SetFlag((char*) &Var.DO, Rl2_bp, FALSE);
						break;

					case Coil_10_Enable_GprsModule:
						SP.Activate_GPRS = (Buf485_Rx[4] == 0xff);
						//Guardo en eeprom las modificaciones
						guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
						break;

					case Coil_11_Enable_GPS:
						SP.Activate_GPS = (Buf485_Rx[4] == 0xff);
						//Guardo en eeprom las modificaciones
						guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
						break;

					case Coil_12_RS485_Wireless:
						SP.RS485_WL = (Buf485_Rx[4] == 0xff);

						if (SP.RS485_WL) { // WIRELESS
							RXEN1_OFF;
							TXEN1_OFF;
							RST_WPRO_OFF;
						} else { // WIRED
							RST_WPRO_ON;
							RXEN1_ON;
							TXEN1_OFF;
						}
						//Guardo en eeprom las modificaciones
						guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
						break;

					case Coil_13_AUTO_MANUAL:
						SP.auto_manual = (Buf485_Rx[4] == 0xff);
						//Guardo en eeprom las modificaciones
						guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));
						break;

					case Coil_14_Send_SMS_Welc_Usr1:
					case Coil_15_Send_SMS_Welc_Usr2:
					case Coil_16_Send_SMS_Welc_Usr3:
					case Coil_17_Send_SMS_Welc_Usr4:
						tempshort = ((MB_Coils_Id) Address);
//						if (Buf485_Rx[4] == 0xff) {
//							if (!GetFlag(Coils_Registers, tempshort)) {
//
//								SetFlag((char*) &Coils_Registers[0], tempshort,
//								TRUE);
//								EvtToSIM868.Src = cmd_Welcome;
//								EvtToSIM868.Val = tempshort - 3;
//								xQueueSend(queEvtToMOBILE, &EvtToSIM868, 0);
//							}
//						} else
//							SetFlag((char*) &Coils_Registers[0], tempshort,
//							FALSE);

						break;

					default:
						break;
					}
					Len = 8;
					break;
				case FC06_Write_Sing_Reg:
					if ((Address >= START_ADD_B1)
							&& (Address < START_ADD_B1 + HOLD_REG_ARR_SIZE_B1)) {
						if (((Address - START_ADD_B1) * 2 + 2)
								> HOLD_REG_ARR_SIZE_B1) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque1;
						Ptr_Char += Address * 2;
					} else if ((Address >= START_ADD_B2)
							&& (Address < START_ADD_B2 + HOLD_REG_ARR_SIZE_B2)) {
						if (((Address - START_ADD_B2) * 2 + 2)
								> HOLD_REG_ARR_SIZE_B2) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque2;
						Ptr_Char += (Address - START_ADD_B2) * 2;
					}
					*Ptr_Char++ = Buf485_Rx[4];
					*Ptr_Char++ = Buf485_Rx[5];

					MBCrc16((uint8_t*) Buf485_Rx, 6);
					Len = 8; //Uso Len para establecer los datos a enviar por el puerto
					break;
				case FC10_Write_Mult_Reg:
					Len = 2 * ((Buf485_Rx[4] << 8) + Buf485_Rx[5]); //Cantidad de BYTES a enviar. Bytes =2*Registros
					if ((Address >= START_ADD_B1)
							&& (Address < START_ADD_B1 + HOLD_REG_ARR_SIZE_B1)) {
						if ((Address * 2 + Len) > HOLD_REG_ARR_SIZE_B1) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque1;
						Ptr_Char += Address * 2;
					} else if ((Address >= START_ADD_B2)
							&& (Address < START_ADD_B2 + HOLD_REG_ARR_SIZE_B2)) {
						if (((Address - START_ADD_B2) * 2 + Len)
								> HOLD_REG_ARR_SIZE_B2) {
							Buf485_Rx[1] += 0x80;
							Buf485_Rx[2] = 0x02; //Illegal address
							Len = 3;
							MBCrc16((uint8_t*) Buf485_Rx, Len);
							Len += 2;
							break;
						}
						Ptr_Char = (char*) &Hold_Registers_bloque2;
						Ptr_Char += (Address - START_ADD_B2) * 2;
					}

					i = 0;
					while (i < Len) {
						*Ptr_Char++ = Buf485_Rx[7 + i];
						*Ptr_Char++ = Buf485_Rx[8 + i];
						i += 2;
					}
					Len /= 2; //Divido por 2 para pasar de Bytes a Registros
					Buf485_Rx[2] = 0xff & (Address >> 8);
					Buf485_Rx[3] = 0xff & Address;
					Buf485_Rx[4] = 0xff & (Len >> 8);
					Buf485_Rx[5] = 0xff & Len;
					MBCrc16((uint8_t*) Buf485_Rx, 6);
					Len = 8; //Uso Len para establecer los datos a enviar por el puerto
					break;

				case Diagnostic:
					break;
				default:
					break;
				}

				//Envio resuesta al comando
				if ( xSemaphoreTake(semExpT35_485,
						100) == pdTRUE) {
					if (Oper != FC6E_String_Write) {
						Send_RS485(Len, (uint32_t*) Buf485_Rx);
					}
				}

				//Si se escribieron datos en el Holding register debo actualizar las variables
				if (Oper == FC06_Write_Sing_Reg
						|| Oper == FC10_Write_Mult_Reg) {

					for (i = 0; i < 6; i++) {
						read_value((char*) &SP.An[i].HE, Hold_Registers_bloque1,
								12 * i, 4);
						read_value((char*) &SP.An[i].LE, Hold_Registers_bloque1,
								12 * i + 4, 4);
						read_value((char*) &SP.An[i].HS, Hold_Registers_bloque1,
								12 * i + 8, 2);
						read_value((char*) &SP.An[i].LS, Hold_Registers_bloque1,
								12 * i + 10, 2);
					} //termina en 70+2

					for (i = 0; i < 6; i++) {
						read_value((char*) &SP.tnk[i].lit_by_m,
								Hold_Registers_bloque1, i * 12 + 72, 4);
						read_value((char*) &SP.tnk[i].dens,
								Hold_Registers_bloque1, i * 12 + 76, 4);
						read_value((char*) &SP.tnk[i].off,
								Hold_Registers_bloque1, i * 12 + 80, 4);
					} //termina en 100+4

					read_value((char*) &tempshort, Hold_Registers_bloque1, 144,
							2);
					if (tempshort != SP.flow_unit)
						snd_head_flow_sett(&newParameter, tempshort, SP_UNITS,
								src_modbus, 0);

					/*HR Bloque 2*/
					/*Parámetros Bomba 1*/
					read_value((char*) &tempfloat, Hold_Registers_bloque2, 0,
							4);
					if (tempfloat != sp_pump.PumpRate)
						snd_head_flow_sett(&newParameter, tempfloat,
								SP_PumpRate, src_modbus, 0);

					read_value((char*) &tempshort, Hold_Registers_bloque2, 4,
							4);
					if (tempshort != sp_pump.Threshold)
						snd_head_flow_sett(&newParameter, tempshort, SP_THRESH,
								src_modbus, 0);

					read_value((char*) &tempfloat, Hold_Registers_bloque2, 8,
							4);
					if (tempfloat != sp_pump.CF)
						snd_head_flow_sett(&newParameter, tempfloat, SP_CF,
								src_modbus, 0);

					read_value((char*) &tempshort, Hold_Registers_bloque2, 12,
							2);
					if (tempshort != sp_pump.Cycle)
						snd_head_flow_sett(&newParameter, tempshort, SP_CYCLE_T,
								src_modbus, 0);

					read_value((char*) &sp_pump.GR, Hold_Registers_bloque2, 14,
							2);

					read_value((char*) &tempshort, Hold_Registers_bloque2, 16,
							2);
					if (tempshort != sp_pump.level_warning_limit_T1)
						snd_tank_sett(&newParameter, tempshort,
								SP_War_TankLevel, src_modbus);

					read_value((char*) &tempshort, Hold_Registers_bloque2, 18,
							2);
					if (tempshort != sp_pump.level_alarm_limit_T1)
						snd_tank_sett(&newParameter, tempshort, SP_AL_TankLevel,
								src_modbus);

					/*Parámetros Bomba 2*/
//					read_value((char*) &sp_pump.PumpRate,
//							Hold_Registers_bloque2, 20, 4);
//					read_value((char*) &sp_pump.Threshold,
//							Hold_Registers_bloque2, 24, 4);
//					read_value((char*) &sp_pump.CF, Hold_Registers_bloque2, 28,
//							4);
//					read_value((char*) &sp_pump.Cycle, Hold_Registers_bloque2,
//							32, 2);
//					tempshort = sp_pump.GR;
//					read_value((char*) &tempshort, Hold_Registers_bloque2, 34,
//							2);
//					tempshort = 0;
//					read_value((char*) &tempshort, Hold_Registers_bloque2, 36,
//							2); //41019: Reservado
//					read_value((char*) &tempshort, Hold_Registers_bloque2, 38,
//							2); //41020: Reservado
//
					/*Parámetros proceso*/
					read_value((char*) &Var.caudal_produccion,
							Hold_Registers_bloque2, 40, 4);
					read_value((char*) &Var.presion_referencia,
							Hold_Registers_bloque2, 44, 4);

				}
			}
		}

		/*==========================================================================
		 * Sistema de CONFIGURACION Y DIAGNOSTICO por RS485
		 * ==========================================================================
		 */
		else if (Buf485_Rx[0] == 'S') { //Chequeo que se trate del comando para abrir el sistema de setup
			vTaskDelay(1500);
			if (!strncmp("ST\r", Buf485_Rx, 3)) {

				do {
					Send_RS485(strlen(helloSetupMenu), helloSetupMenu);
					vTaskDelay(1);
					Send_RS485(strlen(select_option_title),
							select_option_title);
					vTaskDelay(1);
					Send_RS485(strlen(setupMainMenu), setupMainMenu);
					vTaskDelay(1);
					Send_RS485(strlen(exitMsg), exitMsg);
					ReadB = 0;

					/*Aseguro los valores para que no envie datos desde el SIM868*/
					debug_gsm = false;
					sniff_gsm = false;

					if (wait_key(Buf485_Rx, EXIT_TIME) == 0) {
						goto EXIT_SETUP;
					}

					//===========================================================================================================
					// OPCIÓN 1. STATUS. Monitor Analog + digital Inputs
					if (!strncmp(Buf485_Rx, "1\r", 2)) {

						sprintf(Buf485_Rx,
								"1. STATUS: Analog inputs + digital inputs\r\n"
										"\r\n"
										"Press any key to stop\r\n\r\n");
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);

						Index = 0;
						do {
//							CLR_SCR;
							i = sprintf(Buf485_Rx,
									"An In [Ch1 Ch2]: %.2f %.2f\t"
											"Dig In [Ch1 to Ch4] %d %d %d %d\r",
									Var.an[0].Signal, Var.an[1].Signal,
									(Var.DI & 0x01), ((Var.DI >> 1) & 0x01),
									((Var.DI >> 2) & 0x01),
									((Var.DI >> 3) & 0x01));
							Send_RS485(i, Buf485_Rx);

							/* Pongo un límite de 1000  lecturas. Luego el sistema volverá
							 * a ejecutar el MODBUS server */
							Index++;

							/*Mientras no entre algun dato, con 500ms de espera*/
						} while (!wait_key(Buf485_Rx, 500) && (Index < 1000));

						sprintf(Buf485_Rx,
								" Quit ==== END INPUT STATUS MONITORING ====\r\n");

						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
					}
					//===========================================================================================================
					// OPCIÓN 2.  SETTINGS
					else if (!strncmp(Buf485_Rx, "2\r", 2)) {

						do {
							sprintf(Buf485_Rx,
									"2. SETTINGS. Select an option <<<<\r\n"
											"\r\n"
											"1: ID\r\n"
											"2: Tank settings\r\n"
											"3: Analog channels\r\n"
											"4: GPRS and SMS\r\n"
											"5: Bluetooth\r\n"
											"6: Modbus\r\n"
											"7: Alarms\r\n"
											"\r\n%s", exitMsg);
							Send_RS485(strlen(Buf485_Rx), Buf485_Rx);

							if (wait_key(Buf485_Rx, EXIT_TIME) == 0) {
								goto EXIT_SETUP;
							}

							if (sscanf(Buf485_Rx, "%d", &tempshort) == 1) {
								if (tempshort > 0 && tempshort <= 7) {

									switch (tempshort) {

									//=========================================================================
									case 1:				//2.1 ID
										do {
											sprintf(Buf485_Rx,
													"2.1 Device ID. Select an option to update...\r\n"
															"\r\n"
															"1: Tag 1: %s\r\n"
															"2: Tag 2: %s\r\n"
															"3: Tag 3: %s\r\n"
															"4: SN: %s\r\n"
															"\r\n"
															"Firmware Version: %s"
															"\r\n%s",
													&id_Tag[0], &id_Tag[1],
													&id_Tag[2], id_SN,
													FIRM_VERS, exitMsg);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												if (tempshort > 0
														&& tempshort <= 4) {

													switch (tempshort) {
													case 1:	//Tag 1
													case 2:	//Tag 2
													case 3:	//Tag 3
														sprintf(Buf485_Rx,
																"2.1.[1..3]Enter new tag (c to cancel): ");
														Send_RS485(
																strlen(
																		Buf485_Rx),
																Buf485_Rx);

//														memset(Buf485_Rx, 0xff,
//														LEN_ID * 3);//Limpio el buffer para esperar la entrada

														tempshort--;//Resto 1 para llevarlo al rango [0..2]
														if (wait_key(Buf485_Rx,
														EXIT_TIME) == 0) {
															goto EXIT_SETUP;
														}

														if (!cancel_key(
																Buf485_Rx)) {
															if (sscanf(
																	Buf485_Rx,
																	"%[^\r]s",
																	&Buf485_Rx[BUF485_SIZE
																			/ 2])
																	== 1) {
																strncpy(
																		&id_Tag[tempshort][0],//todo: chequear q esto funciona ok
																		&Buf485_Rx[BUF485_SIZE
																				/ 2],
																		LEN_ID);
																guardar_eeprom(
																		id_Tag[tempshort],
																		OFF_ID_TAG1
																				+ tempshort
																						* LEN_ID,
																		LEN_ID);

																loc_mqtt_st =
																		mqtt_ev_publish_id;
																//Enviar a la cola para su ejecución
																xQueueSend(
																		queMQTT_ev,
																		&loc_mqtt_st,
																		0);
															}
														}
														break;
													case 4: {	// Set SN
														sprintf(Buf485_Rx,
																"2.1.4 Enter PASS[space]SN (Ej: 1234 xpm20xxx). (c to cancel): ");

														Send_RS485(
																strlen(
																		Buf485_Rx),
																Buf485_Rx);
														if (wait_key(Buf485_Rx,
														EXIT_TIME) == 0) {
															goto EXIT_SETUP;
														}

														if (!cancel_key(
																Buf485_Rx)) {
															if (sscanf(
																	Buf485_Rx,
																	"%s %s",
																	&Buf485_Rx[BUF485_SIZE
																			/ 6],
																	&Buf485_Rx[BUF485_SIZE
																			/ 3])) {
																if (!strcmp(
																		&Buf485_Rx[BUF485_SIZE
																				/ 6],
																		"5013")) {
																	sprintf(
																			id_SN,
																			&Buf485_Rx[BUF485_SIZE
																					/ 3]);
																	guardar_eeprom(
																			id_SN,
																			OFF_SN,
																			LEN_SN);
//																	/*Crear tarea de soporte MQTT*/
//																	if (!tsk_mqtt_handler)
//																		Create_COMM_MQTT_task();
																	/*Crear tarea de soporte MQTT*/
																	//if (!tsk_mqtt_handler)
#if USE_SIM868
																		Create_COMM_MQTT_SIM868_task();
#elif USE_BG96
																		Create_COMM_MQTT_BG96_task();
#elif USE_SIM7070G
																		//Create_COMM_MQTT_task();
#endif
																}
															}
														}
													}
													default:
														break;
													}

												}
											}
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 2:	//2.2 Tank settings
										do {
//											sprintf(Buf485_Rx,
//													"2.2 Tank settings. Select an option to update...\r\n"
//															"\r\n"
//															"1: Tank shape: %s\r\n"
//															"2: Vert cyl hght[m]: %.2f\r\n"
//															"3: Vert cyl diam[m]: %.2f\r\n"
//															"4: Hor cyl lgth[m]: %.2f\r\n"
//															"5: Hor cyl diam[m]: %.2f\r\n"
//															"6: Rect hght[m]: %.2f\r\n"
//															"7: Rect lgth[m]: %.2f\r\n"
//															"8: Rect wide[m]: %.2f\r\n"
//															"9: Dens [Kg/m3]: %.2f\r\n"
//															"10: Sensor hght [m]: %.2f\r\n"
//															"11: Volume units: %s\r\n"
//															"\r\n%s",
//													tank_shapes_str[SP.tnk_shp
//															- 1],
//													SP.tnk_VertCyl_hgt,
//													SP.tnk_VertCyl_diam,
//													SP.tnk_HorCyl_lgt,
//													SP.tnk_HorCyl_diam,
//													SP.tnk_Rect_hgt,
//													SP.tnk_Rect_lgt,
//													SP.tnk_Rect_wid,
//													SP.tnk_dens,
//													SP.tnk_sensor_hgt, VUN,
//													exitMsg);
											sprintf(Buf485_Rx,
													"2.2 Tank settings. Select an option to update...\r\n"
															"\r\n"
															"1: Tank shape: %s\r\n"
															"2: VT or HT diam, RT width[m]: %.2f\r\n"
															"3: VT height, HT length, RT height[m]: %.2f\r\n"
															"4: RT length[m]: %.2f\r\n"
															"5: Dens [Kg/m3]: %.2f\r\n"
															"6: Sensor hght [m]: %.2f\r\n"
															"7: Volume units: %s\r\n"
															"\r\n%s",
													tank_shapes_str[sp_pump.tnk_shp
															- 1],
													sp_pump.tnk_d1_VT_HT_diam_RT_width,
													sp_pump.tnk_d2_VT_height_HT_lenght_RT_height,
													sp_pump.tnk_d3_RT_length,
													sp_pump.tnk_dens,
													sp_pump.tnk_sensor_hgt,
													Var.vol_unit, exitMsg);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												if (tempshort > 0
														&& tempshort <= 7) {
													//2.1.1
													if (tempshort == 1) {
														sprintf(Buf485_Rx,
																"Select an option:\r\n"
																		"1: %s\r\n"
																		"2: %s\r\n"
																		"3: %s\r\n"
																		"4: %s\r\n",
																tank_shapes_str[0],
																tank_shapes_str[1],
																tank_shapes_str[2],
																tank_shapes_str[3]);
														Send_RS485(
																strlen(
																		Buf485_Rx),
																Buf485_Rx);
														if (wait_key(Buf485_Rx,
														EXIT_TIME) == 0) {
															goto EXIT_SETUP;
														}
														if (sscanf(Buf485_Rx,
																"%d",
																&tempshort)
																== 1) {
															if (tempshort > 0
																	&& tempshort
																			<= 4)
																sp_pump.tnk_shp =//todoÑusar la cola para actualizar
																		tempshort;
															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
														}

														//2.1.11
													} else if (tempshort == 7) {
														sprintf(Buf485_Rx,
																"Select an option:\r\n"
																		"1: Galons\r\n"
																		"2: Liters\r\n");
														Send_RS485(
																strlen(
																		Buf485_Rx),
																Buf485_Rx);
														if (wait_key(Buf485_Rx,
														EXIT_TIME) == 0) {
															goto EXIT_SETUP;
														}
														if (sscanf(Buf485_Rx,
																"%d",
																&tempshort)
																== 1) {
															if (tempshort > 0
																	&& tempshort
																			<= 2)
																SP.vol_unit =
																		tempshort;
															//Imprimo las unidades del tanque
															if (SP.vol_unit
																	== Liters)
																sprintf(
																		Var.vol_unit,
																		"Lit");
															else
																sprintf(
																		Var.vol_unit,
																		"Gal");

															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
														}
													}

													//2.1.2 to 2.1.6
													else {
														sprintf(Buf485_Rx,
																" Enter new value (c to cancel)  = ");

														Send_RS485(
																strlen(
																		Buf485_Rx),
																Buf485_Rx);
														if (wait_key(Buf485_Rx,
														EXIT_TIME) == 0) {
															goto EXIT_SETUP;
														}

														if (!cancel_key(
																Buf485_Rx)) {
															if (sscanf(
																	Buf485_Rx,
																	"%f",
																	&tempfloat)
																	== 1) {
																switch (tempshort) {
																case 2:
																	sp_pump.tnk_d1_VT_HT_diam_RT_width =
																			tempfloat;
																	break;
																case 3:
																	sp_pump.tnk_d2_VT_height_HT_lenght_RT_height =
																			tempfloat;
																	break;
																case 4:
																	sp_pump.tnk_d3_RT_length =
																			tempfloat;
																	break;
																case 5:
																	sp_pump.tnk_dens =
																			tempfloat;
																	break;
																case 6:
																	sp_pump.tnk_sensor_hgt =
																			tempfloat;
																	break;
																}

																guardar_eeprom(
																		(char*) &SP,
																		OFF_SP,
																		sizeof(SP));
															}
														}
													}
												}
											}

											//===========================================================================================================
											// OPCIÓN 12.  Exit tank settings menu
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 3:	//2.3 Analog channels
										do {
											sprintf(Buf485_Rx,
													"2.3 Analog channels settings. Select an option to update...\r\n"
															"\r\n"
															"1: An1-High Eng [psi]: %.2f\r\n"
															"2: An1-Low Eng  [psi]: %.2f\r\n"
															"3: An1-High Sign [mA]: %u\r\n"
															"4: An1-Low Sign  [mA]: %u\r\n"
															"\r\n"
															"5: An2-High Eng [psi]: %.2f\r\n"
															"6: An2-Low Eng  [psi]: %.2f\r\n"
															"7: An2-High Sign [mA]: %u\r\n"
															"8: An2-Low Sign  [mA]: %u\r\n",

													SP.An[0].HE, SP.An[0].LE,
													SP.An[0].HS, SP.An[0].LS,
													SP.An[1].HE, SP.An[1].LE,
													SP.An[1].HS, SP.An[1].LS);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												if (tempshort > 0
														&& tempshort <= 8) {

													sprintf(Buf485_Rx,
															" Enter new value (c to cancel)  = ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}

													if (!cancel_key(
															Buf485_Rx)) {
														if (sscanf(Buf485_Rx,
																"%f",
																&tempfloat)
																== 1) {
															switch (tempshort) {
															case 1: //2.3.1
																SP.An[0].HE =
																		tempfloat;
																break;
															case 2: //2.3.2
																SP.An[0].LE =
																		tempfloat;
																break;
															case 3: //2.3.3
																SP.An[0].HS =
																		tempfloat;
																break;
															case 4: //2.3.4
																SP.An[0].LS =
																		tempfloat;
																break;
															case 5: //2.3.5
																SP.An[1].HE =
																		tempfloat;
																break;
															case 6: //2.3.6
																SP.An[1].LE =
																		tempfloat;
																break;
															case 7: //2.3.7
																SP.An[1].HS =
																		tempfloat;
																break;
															case 8: //2.3.8
																SP.An[1].LS =
																		tempfloat;
																break;
															}
															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
														}
													}
												}
											}

											//===========================================================================================================
											// 2.3.9  Exit
//										} while ((strncmp(Buf485_Rx, "9\r", 2)));
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 4:	//2.4 GPRS and SMS
										do {
											sprintf(Buf485_Rx,
													"2.4 GPRS, SMS users. Select an option to update...\r\n"
															"\r\n"
															"1: Cell Nbr U1: %u\r\n"
															"2: Cell Nbr U2: %u\r\n"
															"3: Cell Nbr U3: %u\r\n"
															"4: Cell Nbr U4: %u\r\n"
															"5: Send welcome SMS to User\r\n"
															"6: GPRS APN: %s\r\n"
															"7: GPRS USER: %s\r\n"
															"8: GPRS PASSWORD: %s\r\n"
															"9: SMS service center: %s\r\n"
															"10: GPRS Status: %s\r\n"
															"\r\n"
															"Operator: %s\r\n"
															"Signal strength: %d\r\n"
															"Registered: %s\r\n"
															"Coord: Lat(%f), Lon(%f)\r\n"
															"\r\n"
															"%s or any value to refresh...\r\n",

													gsm.RPN[0], gsm.RPN[1],
													gsm.RPN[2], gsm.RPN[3],
													gprs_apn, gprs_usr,
													gprs_pass, gsm.SMS_scenter,
													SP.Activate_GPRS ?
															"On" : "Off",
													gsm.oper, gsm.rssi,
													gsm.reg_on_netw ?
															"Yes" :
															"Not regist",
													gps.Lat, gps.Lon, exitMsg);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												//2.4.1 to .4 ---Set Mobile Nbr---
												if (tempshort > 0
														&& tempshort <= 4) {

													sprintf(Buf485_Rx,
															" Enter new value (c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);

													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}

													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														if (sscanf(Buf485_Rx,
																"%u",
																&gsm.RPN[tempshort
																		- 1])
																== 1) {
															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
															for (i = 0; i < 4;
																	i++) {
																sprintf(
																		gprs_rpn[i],
																		"%u",
																		gsm.RPN[i]);
															}
														} else
															leer_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
													}
												}
												//2.4.5 --- Send Welcome SMS to User ---
												else if (tempshort == 5) {
													sprintf(Buf485_Rx,
															" Enter usr number (1 to 4) to send SMS(c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}

													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														sscanf(Buf485_Rx, "%u",
																&tempshort);
														if (tempshort >= 1
																&& tempshort
																		< 5) {
															EvtToSIM868.Src =
																	cmd_Welcome;
															EvtToSIM868.Val =
																	tempshort;
															xQueueSend(
																	queEvtToMOBILE,
																	&EvtToSIM868,
																	0);
														}
													}
												}
												//2.4.6 ---Set APN---
												else if (tempshort == 6) {
													sprintf(Buf485_Rx,
															"2.4.6 Enter new APN (c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														if (sscanf(Buf485_Rx,
																"%s",
																&Buf485_Rx[BUF485_SIZE
																		/ 3])
																== 1) {
															strcpy(gprs_apn,
																	&Buf485_Rx[BUF485_SIZE
																			/ 3]);
//															gsm.new_apn =
//															true;
														}
//						      sscanf (Buf485_Rx, "%s", gprs_apn);
//						      sprintf (&Buf485_Rx[BUF485_SIZE / 3],
//							       "%s;%s;%s;%s", gsm.oper, gprs_apn,
//							       gprs_usr, gprs_pass);
//
//						      guardar_eeprom (
//							  &Buf485_Rx[BUF485_SIZE / 3],
//							  OFF_GPRS_APN_PROF1
//							      + Var.apn_prof * LEN_APN_PROF,
//							  LEN_APN_PROF);
													}
												}
												//2.4.7 ---Set USR---
												else if (tempshort == 7) {
													sprintf(Buf485_Rx,
															"2.4.7 Enter new USER (c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														sscanf(Buf485_Rx, "%s",
																gprs_usr);
														sprintf(
																&Buf485_Rx[BUF485_SIZE
																		/ 3],
																"%s;%s;%s;%s",
																gsm.oper,
																gprs_apn,
																gprs_usr,
																gprs_pass);

														guardar_eeprom(
																&Buf485_Rx[BUF485_SIZE
																		/ 3],
																OFF_GPRS_APN_PROF1
																		+ Var.apn_prof
																				* LEN_APN_PROF,
																LEN_APN_PROF);
													}
												}
												//2.4.8 ---Set PASS---
												else if (tempshort == 8) {
													sprintf(Buf485_Rx,
															"2.4.8 Enter new PASS (c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														sscanf(Buf485_Rx, "%s",
																gprs_pass);
														sprintf(
																&Buf485_Rx[BUF485_SIZE
																		/ 3],
																"%s;%s;%s;%s",
																gsm.oper,
																gprs_apn,
																gprs_usr,
																gprs_pass);

														guardar_eeprom(
																&Buf485_Rx[BUF485_SIZE
																		/ 3],
																OFF_GPRS_APN_PROF1
																		+ Var.apn_prof
																				* LEN_APN_PROF,
																LEN_APN_PROF);
													}
												}
												//2.4.9 --- Set SMS Service Center ---
												else if (tempshort == 9) {
													sprintf(Buf485_Rx,
															"2.4.9 Enter new Service Center Nbr (c to cancel): ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}

													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														sscanf(Buf485_Rx, "%s",
																gsm.SMS_scenter);
														EvtToSIM868.Src =
																SMS_ServiceCenter; //Enviar el comando para que actualice el SC Nº
														xQueueSend(
																queEvtToMOBILE,
																&EvtToSIM868,
																0);
													}
												}
												//2.4.10 ---Turn On/Off GPRS---
												else if (tempshort == 10) {
//						  sprintf (Buf485_Rx,
//							   "2.4.10 Operation Not allowed.\r\n"
//							   "\r\n");
//						  Send_RS485 (strlen (Buf485_Rx), Buf485_Rx);
													sprintf(Buf485_Rx,
															"Select an option:\r\n"
																	"1: Off\r\n"
																	"2: On\r\n");
													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (sscanf(Buf485_Rx, "%d",
															&tempshort) == 1) {
														if (tempshort > 0
																&& tempshort
																		<= 2)
															SP.Activate_GPRS =
																	tempshort
																			- 1;
//														guardar_eeprom(
//																(char*) &SP,
//																OFF_SP,
//																sizeof(SP));
													}

												}
											}

											//===========================================================================================================
											//quit  Exit
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 5:	//2.5 Bluetooth
										do {
											sprintf(Buf485_Rx,
													"2.5 Bluetooth. Select an option to update...\r\n"
															"\r\n"
															"1: Status: %s\r\n"
															"2: Host name: %s\r\n"
															"3: PIN code: %u\r\n"
															"\r\n%s",

													SP.Activate_Bluetooth ?
															"On" : "Off",
													BT_HOST_NAME, SP.BT_pinCode,
													exitMsg);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {

												//2.5.1
												if (tempshort == 1) {
													sprintf(Buf485_Rx,
															"2.5.1 >>>>> USE FRONT PANEL PUSH BUTTON TO ACTIVATE.\r\n"
																	"\r\n");
													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
//													sprintf(Buf485_Rx,
//															"Select an option:\r\n"
//																	"1: Off\r\n"
//																	"2: On\r\n");
//													Send_RS485(
//															strlen(Buf485_Rx),
//															Buf485_Rx);
//													if (wait_key(Buf485_Rx,
//													EXIT_TIME) == 0) {
//														goto EXIT_SETUP;
//													}
//													if (sscanf(Buf485_Rx, "%d",
//															&tempshort) == 1) {
//														if (tempshort > 0
//																&& tempshort
//																		<= 2)
//															SP.Activate_Bluetooth =
//																	tempshort
//																			- 1;
//														guardar_eeprom(
//																(char*) &SP,
//																OFF_SP,
//																sizeof(SP));
//													}
												}

												//2.5.2
												else if (tempshort == 2) {

													sprintf(Buf485_Rx,
															"2.5.2 Operation Not allowed.\r\n"
																	"\r\n");
													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
												}

												//2.5.3
												if (tempshort == 3) {

													sprintf(Buf485_Rx,
															"2.5.3 Enter new PIN Code: ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (sscanf(Buf485_Rx, "%u",
															&tempshort) == 1) {
														if (tempshort >= 0
																&& tempshort
																		<= 9999) {
															SP.BT_pinCode =
																	tempshort;
															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));

															EvtToBlue.sta =
																	blue_init;
															EvtToBlue.ev =
																	blue_ev_NE;

															//Enviar a la cola para su ejecución
															xQueueSend(
																	queBLUE_SM,
																	&EvtToBlue,
																	0);
														}
													}
												}
											}

											//===========================================================================================================
											// 2.5.4  Exit
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 6:	//2.6 Modbus
										do {
											sprintf(Buf485_Rx,
													"2.6 Modbus. Select an option to update...\r\n"
															"\r\n"
															"WARNING!: After changing any of the following values, "
															"you will need to setup accordingly the comm settings from the host "
															"\r\n\r\n"
															"1: Slave ID: %u\r\n"
															"2: Baudrate [9600, 19200, 115200]: %u\r\n"
															"3: Link: %s\r\n",

													SP.RS485_SLID,
													SP.RS485_BAUD,
													SP.RS485_WL ?
															"wireless" :
															"wired");

											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												if (tempshort > 0
														&& tempshort <= 2) {

													sprintf(Buf485_Rx,
															" Enter new value (c to cancel)  = ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														if (sscanf(Buf485_Rx,
																"%d", &Len)
																== 1) {
															switch (tempshort) {
															case 1: //2.6.1 Slave ID
																if ((Len > 0)
																		&& (Len
																				< 255)) {
																	SP.RS485_SLID =
																			Len;
																}
																break;
															case 2: //2.6.2 BaudRate
																if ((Len == 9600)
																		| (Len
																				== 19200)
																		| (Len
																				== 115200)) {
																	SP.RS485_BAUD =
																			Len;
																}
																break;
															}
															guardar_eeprom(
																	(char*) &SP,
																	OFF_SP,
																	sizeof(SP));
														}
													}
												}

												/*2.6.3 Wired or wireless*/
												else if (tempshort == 3) {
													sprintf(Buf485_Rx,
															"2.6.3 Select an option:\r\n"
																	"1: Wired\r\n"
																	"2: Wireless\r\n");
													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (sscanf(Buf485_Rx, "%d",
															&tempshort) == 1) {
														if (tempshort > 0
																&& tempshort
																		<= 2)
															SP.RS485_WL =
																	tempshort
																			- 1;
														if (SP.RS485_WL) { // WIRELESS
															RST_WPRO_OFF; //Comienzo con el XBee por defecto
															RXEN1_OFF;
															TXEN1_OFF;
														} else { // WIRED
															RST_WPRO_ON;
															RXEN1_ON;
															TXEN1_OFF;
														}
														guardar_eeprom(
																(char*) &SP,
																OFF_SP,
																sizeof(SP));
													}
												}
											}

											//===========================================================================================================
											// 2.6.4  Exit
										} while (no_back_cmd(Buf485_Rx));
										break;
									case 7:	//2.7 Alarms
										do {
											sprintf(
													&Buf485_Rx[BUF485_SIZE - 10],
													(SP.vol_unit == Galons) ?
															"Gal" : "Lit");
											sprintf(Buf485_Rx,
													"2.7 Alarms. Select an option to update...\r\n"
															"\r\n"
															"1: Warning. Tank Level [%s]: %u\r\n"
															"2: Alarm. Tank Level [%s]: %u\r\n"
															"\r\n%s",

													&Buf485_Rx[BUF485_SIZE - 10],
													sp_pump.level_warning_limit_T1,
													&Buf485_Rx[BUF485_SIZE - 10],
													sp_pump.level_alarm_limit_T1,
													exitMsg);
											Send_RS485(strlen(Buf485_Rx),
													Buf485_Rx);

											if (wait_key(Buf485_Rx,
											EXIT_TIME) == 0) {
												goto EXIT_SETUP;
											}

											if (sscanf(Buf485_Rx, "%d",
													&tempshort) == 1) {
												if (tempshort > 0
														&& tempshort <= 2) {

													sprintf(Buf485_Rx,
															" Enter new value (c to cancel)  = ");

													Send_RS485(
															strlen(Buf485_Rx),
															Buf485_Rx);
													if (wait_key(Buf485_Rx,
													EXIT_TIME) == 0) {
														goto EXIT_SETUP;
													}
													if (strncmp(Buf485_Rx,
															"c\r", 2) != 0) {
														if (sscanf(Buf485_Rx,
																"%d", &Len)
																== 1) {
															switch (tempshort) {
															case 1: //2.5.1
//																SP.War_TankLevel =
//																		Len;
																snd_tank_sett(
																		&newParameter,
																		Len,
																		SP_War_TankLevel,
																		src_modbus);
																break;
															case 2: //2.5.2
//																SP.AL_TankLevel =
//																		Len;
																snd_tank_sett(
																		&newParameter,
																		Len,
																		SP_AL_TankLevel,
																		src_modbus);
																break;

															}
														}
													}
												}
											}

											//===========================================================================================================
											// 2.5.3  Exit
										} while (no_back_cmd(Buf485_Rx));
										break;
									default:
										break;
									}

									guardar_eeprom((char*) &SP, OFF_SP,
											sizeof(SP));
								}
								/*Para evitar que salga al menú siguiente*/
								Buf485_Rx[0] = 'A';
							}
							//===========================================================================================================
							// quit.  Exit settings menu
						} while (strncmp(Buf485_Rx, "quit\r", 5));
						sprintf(Buf485_Rx,
								" Quit ==== END SETTINGS MENU ====\r\n");

						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
					}
				}
				//===========================================================================================================
				// quit.  Exit setup System
				while (no_back_cmd(Buf485_Rx));
				EXIT_SETUP:

				sprintf(Buf485_Rx, " Quit ==== END SETUP MENU ====\r\n"
						"Starting MODBUS slave server...");
				Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
				debug_gsm = false;
			}
		}

		/*==========================================================================
		 * Sistema de DEBUGGER por RS485
		 * ==========================================================================
		 */
		else if (Buf485_Rx[0] == 'B') { //Chequeo que se trate del comando para abrir el debugger
			vTaskDelay(1500);
			if (!strncmp("BT\r", Buf485_Rx, 3)) {

				do {
					Send_RS485(strlen(helloDebugMenu), helloDebugMenu);
					vTaskDelay(1);
					Send_RS485(strlen(select_option_title),
							select_option_title);
					vTaskDelay(1);
					Send_RS485(strlen(debugMainMenu), debugMainMenu);
					vTaskDelay(1);
					Send_RS485(strlen(exitMsg), exitMsg);
					ReadB = 0;

#if USAR_SNIFFER
					if (sniffer_stat) {
						sprintf(Buf485_Rx, "2\r");
						sniffer_stat = FALSE;
					} else {
						if (wait_key(Buf485_Rx, EXIT_TIME) == 0) {
							goto EXIT_DEBUG;
						}
					}

#else
					if (wait_key(Buf485_Rx, EXIT_TIME) == 0) {
						goto EXIT_DEBUG;
					}

#endif

//===========================================================================================================
					// OPCIÓN 1. Debug GSM / GPRS
					if (!strncmp(Buf485_Rx, "1\r", 2)) {

						sprintf(Buf485_Rx,
								"<<<< GSM/GPRS Debug tool <<<<\r\n%s", exitMsg);
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
						debug_gsm = true;

						do {
							Index = wait_key(Buf485_Rx, EXIT_TIME);
							if (!Index) {
								goto EXIT_DEBUG;
							}

							// Consola ---> GSM Task
							i = 0;
							j = 0;
							EvtToSIM868.Src = sendToMOBILE;
							while (Index > 0) {

								EvtToSIM868.Message[i] = Buf485_Rx[(j
										* MESS_BUFF_SZ) + i];
								i++;
								Index--;

								if (i == MESS_BUFF_SZ) {
									i = 0;
									j++;
									EvtToSIM868.Val = MESS_BUFF_SZ;
									xQueueSend(queEvtToMOBILE, &EvtToSIM868,
											2000);
								}
							}
							if (i > 0) {
								EvtToSIM868.Val = i;
								xQueueSend(queEvtToMOBILE, &EvtToSIM868, 2000);
							}

						} while (no_back_cmd(Buf485_Rx));
						debug_gsm = false;
						sprintf(Buf485_Rx,
								" Quit ==== END DEBUGGING GSM MODULE ====\r\n");
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
					}
//===========================================================================================================
					// OPCIÓN 2. SNIFF GSM / GPRS
					else if (!strncmp(Buf485_Rx, "2\r", 2)) {

						sprintf(Buf485_Rx,
								"<<<< SNIFFER GSM/GPRS tool <<<<\r\n%s",
								exitMsg);
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
						sniff_gsm = true;

						do {
							if (wait_key(Buf485_Rx, EXIT_TIME) == 0) {
								goto EXIT_DEBUG;
							}
						} while (no_back_cmd(Buf485_Rx));
						sniff_gsm = false;
						sprintf(Buf485_Rx,
								" Quit ==== END SNIFFER GSM MODULE ====\r\n");

						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
					}
//===========================================================================================================
					// quit.  Debug Wireless HART module
					else if (!strncmp(Buf485_Rx, "3\r", 2)) {

						sprintf(Buf485_Rx, "<<<< WIRELESHART tool <<<<\r\n%s",
								exitMsg);
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);

						sprintf(Buf485_Rx,
								"\t<<<< TOOL NOT AVAILABLE <<<<\r\n\r\n\r\n");
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
//						debug_whart = true;
//						ReadB = 0;
//						Lst_Index = 0;
//						Index = 0;
//
//						do {
//							Index = xStreamBufferReceive(strMsgFromRS485,
//									Buf485_Rx,
//									BUF485_SIZE,
//									EXIT_TIME);
//							if (!Index) {
//								goto EXIT_DEBUG;
//							}
////-----------------------------------------------------------------------------------------------------------
//							//Consola  <--- WHART Task
//							if (data_RX_avail || data_TX_avail) {
//
//								tempshort = Buf485_Rx[50];
//								Len = 0;
//
//								if (data_RX_avail)
//									Send_RS485(6, "Sent: ");
//								else
//									Send_RS485(6, "Rcvd: ");
//
//								for (i = 0; i < tempshort; i++) {
//									Len = sprintf(&Buf485_Rx[200], "%.2x ",
//											Buf485_Rx[51 + i]);
//									Send_RS485(Len, &Buf485_Rx[200]);
//								}
//								data_RX_avail = false;
//								data_TX_avail = false;
//								CARR_RET;
//							}
//
////-----------------------------------------------------------------------------------------------------------
//							// Consola ---> WHART Task
//							if (ReadB != 0) {
//								i = 0;
//								if (Buf485_Rx[Index - 1] == 0xd) {
//
//									if (!strncmp(Buf485_Rx, "quit", 4)) {
//										break;
//									}
//									Buf485_Rx[Index - 1] = 0x00;
//									EvtToHART.Src = send_cmd;
//									EvtToHART.Val =
//											sscanf(Buf485_Rx,
//													"cmd %d,"
//															"%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,"
//															"%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,"
//															"%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,"
//															"%2x,%2x",
//
//													&Buf485_Rx[200],
//													&Buf485_Rx[202],
//													&Buf485_Rx[203],
//													&Buf485_Rx[204],
//													&Buf485_Rx[205],
//													&Buf485_Rx[206],
//													&Buf485_Rx[207],
//													&Buf485_Rx[208],
//													&Buf485_Rx[209],
//													&Buf485_Rx[210],
//													&Buf485_Rx[211],
//													&Buf485_Rx[212],
//													&Buf485_Rx[213],
//													&Buf485_Rx[214],
//													&Buf485_Rx[215],
//													&Buf485_Rx[216],
//													&Buf485_Rx[217],
//													&Buf485_Rx[218],
//													&Buf485_Rx[219],
//													&Buf485_Rx[220],
//													&Buf485_Rx[221],
//													&Buf485_Rx[222],
//													&Buf485_Rx[223],
//													&Buf485_Rx[224],
//													&Buf485_Rx[225],
//													&Buf485_Rx[226],
//													&Buf485_Rx[227],
//													&Buf485_Rx[228],
//													&Buf485_Rx[229],
//													&Buf485_Rx[230],
//													&Buf485_Rx[231],
//													&Buf485_Rx[232],
//													&Buf485_Rx[233]);
//									if (EvtToHART.Val > 0) {
//										xQueueSend(queEvtToHART, &EvtToHART, 0);
//									} else {
//										Send_RS485(27,
//												"Use cmd to send command\r\n");
//									}
//									Index = 0;
//									Lst_Index = 0;
//									CARR_RET;
//								}
//								if (Index > Lst_Index) {
//									Send_RS485(Index - Lst_Index,
//											&Buf485_Rx[Lst_Index]);
//									Lst_Index = Index;
//								}
//
//							}
//
////							xTimerReset(Timer_Seg485, 0); //RESETEO EL TIMER DE SEGURIDAD
//						} while (!Flag_Seg);
//						debug_whart = false;
						sprintf(Buf485_Rx,
								" Quit ==== END DEBUGGING WHART MODULE ====\r\n");
						Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
					}
				}

//===========================================================================================================
				// OPCIÓN 4.  Exit Debugging System
				while (no_back_cmd(Buf485_Rx));
				EXIT_DEBUG:

				sprintf(Buf485_Rx, " Quit ==== END DEBUGGING MENU ====\r\n"
						"Starting MODBUS slave server...");
				Send_RS485(strlen(Buf485_Rx), Buf485_Rx);
				debug_gsm = false;
				sniff_gsm = false;
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
		Chip_TIMER_Reset(TIMER); //Reseteo el tmr
		Chip_TIMER_ClearMatch(TIMER, 0); //
		Chip_TIMER_ClearMatch(TIMER, 1);
		Chip_TIMER_MatchEnableInt(TIMER, 0);
		Chip_TIMER_MatchEnableInt(TIMER, 1);
		Chip_TIMER_Enable(TIMER); //Habilito el tmr

		ReadB = Chip_UART_Read(UART, Buffer, LOC_BUFF_SZ);

		if (ReadB > 0) {
			xStreamBufferSendFromISR(strMsgFromRS485, Buffer, ReadB,
					&xHigherPriorityTaskWoken);
		}

	} else if (Interr & UART_IIR_INTID_THRE) {
		Chip_UART_IntDisable(UART, UART_IER_THREINT);
		vTaskNotifyGiveFromISR(tsk_modb_rs485_tx_handler,
				&xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/
/* Atención de INT por el TMR3 */
void TMR_HANDLER_NAME(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken =
	pdFALSE;
	if (Chip_TIMER_MatchPending(TIMER, 0)) { //Expiro T15??
		Chip_TIMER_ClearMatch(TIMER, 0); //
		Chip_TIMER_MatchDisableInt(TIMER, 0);

	} else if (Chip_TIMER_MatchPending(TIMER, 1)) { //Expiro T35??
		Chip_TIMER_ClearMatch(TIMER, 1);
		Chip_TIMER_MatchDisableInt(TIMER, 1);
		xSemaphoreGiveFromISR(semExpT35_485, &xHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_CommMODB_RS485_Tx_Task(void) {
	xTaskCreate(vCommMODB_RS485_Tx, (char*) "MB RS485 Tx TSK", 130,
	NULL,
	Prio_CommMODB_RS485_Tx_Task, &tsk_modb_rs485_tx_handler);
}

void Create_CommMODB_RS485_Rx_Task(void) {
	xTaskCreate(vCommMODB_RS485_Rx, (char*) "MB RS485 Rx TSK", 512,
	NULL,
	Prio_CommMODB_RS485_Rx_Task, &tsk_modb_rs485_rx_handler);
}

