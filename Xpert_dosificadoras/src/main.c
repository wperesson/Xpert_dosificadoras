/*

 */
#include "board.h"
#include "chip.h"

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "stream_buffer.h"

/* Librerias de C includes. */

#include "Definiciones.h"

//#include "trcUser.h"
//#include "trcConfig.h"
//#include "trcHardwarePort.h"

#define NEW_DEVICE	0
//0: Update de dispositivos con SN asignado
//1: Para q grabe tbn el SN genérico para nuevos disp.

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

/* EEPROM SLAVE data */
#define I2C_SLAVE_EEPROM_SIZE       64
#define I2C_SLAVE_EEPROM_ADDR       0x5A
#define I2C_SLAVE_IOX_ADDR          0x5B

#define CLAVE_FIRST_FLASHING	0xa0a1

/*****************************************************************************
 * Declaración de variables globales
 ****************************************************************************/

RTC_TIME_T Local;

time_t RawTime, utcTime;
Bool uart0_tx_busy;

volatile Bool debug_gsm = false;
volatile Bool sniff_gsm = false;
volatile Bool debug_whart /*, data_RX_avail ,data_TX_avail*/;
Bool PumpOn;
Bool global_Override;
Bool global_Very_low_battery;
uint8_t global_Scada_Control_cnt;

Bool sendingSMS;

Var_Type Var __attribute__ ((aligned (32)))__attribute__ ((section(".bss.$RamAHB32*"))); // Estructura de VARIABLES DEL SISTEMA
SP_Type SP __attribute__ ((aligned (32)))__attribute__ ((section(".bss.$RamAHB32*"))); // Estructura de SET POINTS del sistema
SP_Pump_Type sp_pump __attribute__ ((aligned (32)))__attribute__ ((section(".bss.$RamAHB32*"))); // Estructura de SET POINTS del sistema
uint8_t Buf485[BUF485_SIZE]__attribute__ ((aligned (32)));
Typ_VSP VSD_bomba1;
PHONE_DATA_T gsm __attribute__ ((section(".bss.$RamAHB32*")));
GPS_DATA_T gps __attribute__ ((section(".bss.$RamAHB32*")));

GENERATOR_CTRL_T gen;
Bool upd_next_gen_start_data; //Cuando se debe hacer el siguiente test de generador

uint16_t mem_Net_Id, mem_Ref_Rate, Act_Page;
uint8_t hart_WR_NET_ID_ret_cnt, hart_WR_JOIN_KEY_ret_cnt,
		hart_WR_UPD_RATE_ret_cnt;

char gprs_rpn[4][LEN_RPN]; //Registered Phone Number 1: 2984558529H
char gprs_apn[LEN_APN]; //String con el apn
char gprs_usr[LEN_GPRS_USR_PASS]; //Usuario para loguearse al gprs
char gprs_pass[LEN_GPRS_USR_PASS]; //Password para loquearse al gprs

char id_SN[LEN_SN]; 	//Serial Number del dispositivo
char id_Tag[3][LEN_ID];	//Tags de identificación del activo

char clientID[LEN_CLIENT_ID];
char access_key[LEN_ACCESS_KEY];
char access_secret[LEN_ACCESS_SECRET];

char endpoint_url[LEN_ENDPOINT_URL];

char BT_HOST_NAME[LEN_BLUE_HOST_NM];

uint16_t PIN[3]; //Vector de pines de acceso de usuarios

//const char ok_char[] = "OK";

uint16_t RegAlarmas; //Cada bit me dice que alarma esta activa

//int16_t Ubi_Server_Interactions_ctr = 0;

/* TAREAS */
TaskHandle_t tsk_error_handler;
TaskHandle_t tsk_alarm_handler;
TaskHandle_t tsk_di_handler;
TaskHandle_t tsk_do_handler;
TaskHandle_t tsk_an_in_handler;
TaskHandle_t tsk_motor_control_handler;
TaskHandle_t tsk_tank_level_handler;
TaskHandle_t tsk_energy_mon_handler;
TaskHandle_t tsk_power_gen_handler;
TaskHandle_t tsk_modb_hmi_handler;
TaskHandle_t tsk_nex_hmi_handler;
TaskHandle_t tsk_nex_hmi_rx_handler;
TaskHandle_t tsk_nex_hmi_tx_handler;
TaskHandle_t tsk_whart_handler;
TaskHandle_t tsk_modb_rs485_rx_handler;
TaskHandle_t tsk_modb_rs485_tx_handler;
TaskHandle_t tsk_events_log_handler;
TaskHandle_t tsk_mobile_rx_handler;
TaskHandle_t tsk_mobile_tx_handler;
TaskHandle_t tsk_mobile_ev_handler;
TaskHandle_t tsk_mqtt_handler;
TaskHandle_t tsk_motor_elec_tx_handler;
TaskHandle_t tsk_motor_elec_rx_handler;
TaskHandle_t tsk_motor_elec_Ctrl_handler;

/* COLAS */
xQueueHandle queMsgFromHMI;		//Dato a enviar al MB
xQueueHandle queHMI_ev;
xQueueHandle queMsgToHMINx;
xQueueHandle queMsgFromHMINx;
xQueueHandle queMsgToMOBILE;		//Datos de UART a enviar a SIM868
xQueueHandle queMsgFromMOBILE;		//Datos de UART proveniente de SIM868
xQueueHandle queMsgToRS485;
QueueHandle_t queMsgToATV12;//Datos de UART a enviar a RS485
xQueueHandle queEvtGenStart;
xQueueHandle queEvtToHART;			//Cola de eventos HART
xQueueHandle queEvtToMOBILE;		//Cola de eventos a enviar al módulo GSM
xQueueHandle queHEAD_FLOW_SETT_upd;
xQueueHandle queTANK_SETT_upd;
xQueueHandle queMQTT_ev;
xQueueHandle queBLUE_SM;
xQueueHandle queEV_LOG;
xQueueHandle queSentByTx;				//Cola de datos enviados a recibir.

/* SEMAFOROS */
xSemaphoreHandle semExpT15_HMI = NULL; //
xSemaphoreHandle semExpT35_HMI = NULL; //
xSemaphoreHandle semExpT15_485 = NULL; //
xSemaphoreHandle semExpT35_485 = NULL; //
xSemaphoreHandle mtxI2C1 = NULL; // Semaforo para controlar el I2C1
xSemaphoreHandle mtxMOBILE = NULL; // Semaforo para controlar el Módulo GSM
xSemaphoreHandle semDMA_TX1 = NULL;

/* STREAM BUFFERS */
StreamBufferHandle_t stream_Sniffer;

/*****************************************************************************
 * Prototipos de funciones
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*-----------------------------------------------------------*/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id) {
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

static void init_i2c(void) {
#define SPEED_100KHZ         100000

	/* Puerto I2C1:
	 * 	LTC2946: Energy Monitor
	 * 	24LC16BT: EEPROM 16Kb  */
	SystemCoreClockUpdate();
	Board_I2C_Init(I2C1);

	Chip_I2C_Init(I2C1);
	Chip_I2C_SetClockRate(I2C1, SPEED_100KHZ);

	/* Set default mode to INTERRUPT */
	NVIC_DisableIRQ(I2C1_IRQn);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
//	NVIC_SetPriority(I2C1_IRQn, mainSOFTWARE_INTERRUPT_PRIORITY);
	NVIC_SetPriority(I2C1_IRQn, mainSOFTWARE_INTERRUPT_PRIORITY + 1);
}

/*-----------------------------------------------------------*/

int main(void) {
	uint8_t i;
	struct tm *timeinfo;

	/* Generic Initialization */

	SystemCoreClockUpdate();

	Board_Init();

	if (LPC_RTC->RTC_AUX & RTC_AUXEN_RTC_OSCFEN) {
		LPC_RTC->RTC_AUX |= RTC_AUXEN_RTC_OSCFEN;
		LPC_RTC->RTC_AUX |= (0x1 << 4);
		Chip_RTC_Init(LPC_RTC);
		Chip_RTC_Enable(LPC_RTC, ENABLE);
		//Inicializo por primera vez el RTC de modo que contenga datos válidos.
		Local.time[RTC_TIMETYPE_SECOND] = 0;
		Local.time[RTC_TIMETYPE_MINUTE] = 30;
		Local.time[RTC_TIMETYPE_HOUR] = 9;
		Local.time[RTC_TIMETYPE_DAYOFMONTH] = 1;
		Local.time[RTC_TIMETYPE_DAYOFWEEK] = 7;
		Local.time[RTC_TIMETYPE_DAYOFYEAR] = 1;
		Local.time[RTC_TIMETYPE_MONTH] = 1;
		Local.time[RTC_TIMETYPE_YEAR] = 2017;

		Chip_RTC_SetFullTime(LPC_RTC, &Local);
	}

	init_i2c();

	/*-----------------------------------------------------------
	 * 	CREACIÓN DE SEMAFOROS
	 */
	mtxI2C1 = xSemaphoreCreateMutex();
	mtxMOBILE = xSemaphoreCreateMutex();
#if USE_BLUETOOTH
	semBLUE_pb = xSemaphoreCreateBinary();
#endif
	xSemaphoreGive(mtxI2C1);
	xSemaphoreGive(mtxMOBILE);

	/*-----------------------------------------------------------
	 * 	CREACIÓN DE COLAS
	 */
#if USE_SIM231_TP_SCREEN
	queMsgFromHMI = xQueueCreate(100, sizeof(uint8_t));
#else
	queMsgFromHMINx = xQueueCreate(100, sizeof(uint8_t));
	queMsgToHMINx = xQueueCreate(100, sizeof(uint8_t));
	queHMI_ev = xQueueCreate(10, sizeof(EventHMI));
#endif

	queEvtToHART = xQueueCreate(4, sizeof(EvtToHART_Type));
	queHEAD_FLOW_SETT_upd = xQueueCreate(2, sizeof(Parameter_upd_Type));
	queTANK_SETT_upd = xQueueCreate(7, sizeof(Parameter_upd_Type));
	queEV_LOG = xQueueCreate(4, sizeof(Events_to_log_Type));

	queMsgFromMOBILE = xQueueCreate(MOBILE_BUFF_SIZE, sizeof(uint8_t));
	queMsgToMOBILE = xQueueCreate(1, sizeof(UART_TX_TRANSFER_T));
	queEvtToMOBILE = xQueueCreate(2, sizeof(EvtToMOB_Type));
	queSentByTx = xQueueCreate(1, sizeof(uint16_t));

	queMsgToRS485 = xQueueCreate(300, sizeof(uint8_t));
	queMsgToATV12 = xQueueCreate(300, sizeof(uint8_t));
#if USE_MOD1100S
	queEvtToHART = xQueueCreate(4, sizeof(EvtToHART_Type));
#endif

	queMQTT_ev = xQueueCreate(5, sizeof(mqtt_event_type));
#if USE_BLUETOOTH
	queBLUE_SM = xQueueCreate(1, sizeof(ev_state_bluet_type));
#endif

	queEvtGenStart = xQueueCreate(5, sizeof(gen_ctrl_ev_Type));

	/*-----------------------------------------------------------
	 * 	CREACIÓN DE STREAM BUFFERS
	 */
	stream_Sniffer = xStreamBufferCreate(BUF485_SIZE, 1);

	/*Recuperar valor de SP en EEPROM*/
	eeprom_read((char*) &SP, OFF_SP, sizeof(SP));
	eeprom_read((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
	/*Dispositivo nuevo? Inicio valores por defecto*/
	if (SP.key != CLAVE_FIRST_FLASHING) {

		//No leyó bien el key, por lo tanto asumo q los datos de SP estan mal.
		//Cargo datos por defecto grabados en flash
		for (i = 0; i < 6; i++) {
			SP.An[i].HE = 3000;
			SP.An[i].LE = 0;
			SP.An[i].HS = 20;
			SP.An[i].LS = 4;
			SP.An[i].InTyp = R4_20mA;
			SP.An[i].Src = Not_Used;
		}

		SP.An[0].Src = Tank_Level;
		SP.An[0].HE = 2.03;		//SPAN PRESSURE DEL SENSOR DE NIVEL
		SP.An[1].Src = Line_Press;
		SP.AnChannelAssig[Tank_Level] = Tank_Level;
		SP.AnChannelAssig[Line_Press] = Line_Press;
		sp_pump.CF = 1;
		sp_pump.Cycle = 3;
		sp_pump.DerGain = 0;
		sp_pump.IntGain = 0.5;
		sp_pump.PropGain = 1.5;
		sp_pump.EPPR = 12;
		sp_pump.GR = 44;
		sp_pump.MotorRPM = 2000;
		sp_pump.plg_str = 1;
		sp_pump.plg_size = 0.5;
		sp_pump.heads = 2;

		SP.WHART_NetId = 0;
		SP.WHART_RefRate = 4;
		SP.WHART_JoinKey[0] = 0;
		SP.WHART_JoinKey[1] = 0;
		SP.WHART_JoinKey[2] = 0;
		SP.WHART_JoinKey[3] = 0;

		sp_pump.PumpRate = 25;
		SP.RS485_BAUD = 115200;
		SP.RS485_SLID = 10;
		SP.RS485_WL = true;
#if MOTOR_ELEC
		sp_pump.nomSpd=1000;
#endif
		sp_pump.maxSpd = 2400;
		sp_pump.TankCap = 250;//Capacidad maxima del tanque. Sirve para cuando usamos 2 Ref points
		sp_pump.ZeroTankEng = 0;
		sp_pump.ZeroTankCap = 10;
		sp_pump.SpanTankEng = 5;
		sp_pump.SpanTankCap = 300;
		sp_pump.level_warning_limit_T1 = 40;		//Warning Level
		sp_pump.level_alarm_limit_T1 = 20;

		SP.SP_GSM_REG_AUTH = 0x0000;
		SP.UseGpsTime = true;
		SP.TimeZone = -3; //ARGENTINA
		sp_pump.Threshold = 30;

		SP.vol_unit = Liters;
		SP.flow_unit = Liters_day;

		sp_pump.tnk_shp = Rect;
		sp_pump.tnk_d1_VT_HT_diam_RT_width = 0.96;
		sp_pump.tnk_d2_VT_height_HT_lenght_RT_height = 1;
		sp_pump.tnk_d3_RT_length = 1.15;
		sp_pump.tnk_sensor_hgt = 0.03;
		sp_pump.tnk_dens = 1000;

		SP.Activate_Bluetooth = false;
		SP.Activate_GPRS = false;
		SP.alreadyRegisteredOnLosant = false;
		SP.mqtt_ref_rate = 10;

		SP.key = CLAVE_FIRST_FLASHING;

		//Guardo los datos en EEPROM y en EEPROM_FD
		eeprom_write((char*) &SP, OFF_SP, sizeof(SP));
		eeprom_write((char*) &sp_pump, OFF_sp_pump, sizeof(sp_pump));
		eeprom_write((char*) &SP, OFF_FACT_DATA, sizeof(SP));

		//Reseteo los numeros del phonebook
		for (i = 0; i < 4; i++) {
			sprintf(gprs_rpn[i], "0");
		}
		eeprom_write((char*) &gprs_rpn, OFF_GSM_REG_NBR1, sizeof(gprs_rpn));

		/*blanqueo los campos*/
		eeprom_write("--", OFF_ID_TAG1, LEN_ID);
		eeprom_write("--", OFF_ID_TAG2, LEN_ID);
		eeprom_write("--", OFF_ID_TAG3, LEN_ID);

#if NEW_DEVICE
//			eeprom_write(NO_SN, OFF_SN, LEN_SN);
			eeprom_write("xp20-10-1000", OFF_SN, LEN_SN);
		#endif

#if SET_CTRY_ARGENTINA //Usar siempre minusculas
		eeprom_write("PERSONAL;datos.personal.com;datos;datos",
		OFF_GPRS_APN_PROF1, LEN_APN_PROF);
		eeprom_write("CTI Movil;igprs.claro.com.ar;;", OFF_GPRS_APN_PROF2,
		LEN_APN_PROF);
#if USE_SIM868
		eeprom_write("UNIFON;internet.gprs.unifon.com.ar;internet;internet",
		OFF_GPRS_APN_PROF3,
		LEN_APN_PROF);
#elif USE_BG96
			eeprom_write("Movistar Movistar;wap.gprs.unifon.com.ar;wap;wap",
			OFF_GPRS_APN_PROF3,
			LEN_APN_PROF);
	#endif

		eeprom_write("nop;internet.movil;internet;internet",
		OFF_GPRS_APN_PROF4, LEN_APN_PROF);
#elif SET_CTRY_KUWAIT
		eeprom_write("VIVA;VIVA;;", OFF_GPRS_APN_PROF1, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF2, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF3, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF4, LEN_APN_PROF);
	#elif SET_CTRY_COLOMBIA
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF1, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF2, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF3, LEN_APN_PROF);
		eeprom_write("Oper;APN;User;Password", OFF_GPRS_APN_PROF4, LEN_APN_PROF);
	#endif

		//Inicializo por primera vez el RTC de modo que contenga datos válidos.
		Local.time[RTC_TIMETYPE_SECOND] = 0;
		Local.time[RTC_TIMETYPE_MINUTE] = 0;
		Local.time[RTC_TIMETYPE_HOUR] = 0;
		Local.time[RTC_TIMETYPE_DAYOFMONTH] = 1;
		Local.time[RTC_TIMETYPE_DAYOFWEEK] = 7;
		Local.time[RTC_TIMETYPE_DAYOFYEAR] = 1;
		Local.time[RTC_TIMETYPE_MONTH] = 1;
		Local.time[RTC_TIMETYPE_YEAR] = 2017;

		Chip_RTC_SetFullTime(LPC_RTC, &Local);

		gen.low_batt_timeout = 0;
		gen.test_timeout = 0;
		gen.low_batt_limit = 11.5;
		gen.period = 7;

		time(&gen.genNextStart);
		timeinfo = localtime(&gen.genNextStart);

		timeinfo->tm_hour = Local.time[RTC_TIMETYPE_HOUR];
		timeinfo->tm_min = Local.time[RTC_TIMETYPE_MINUTE];
		timeinfo->tm_sec = Local.time[RTC_TIMETYPE_SECOND];
		timeinfo->tm_mday = Local.time[RTC_TIMETYPE_DAYOFMONTH];
		timeinfo->tm_mon = Local.time[RTC_TIMETYPE_MONTH];
		timeinfo->tm_year = Local.time[RTC_TIMETYPE_YEAR] - 1900;
		gen.genNextStart = mktime(timeinfo);
		eeprom_write((char*) &gen, OFF_GEN, sizeof(gen));

		//Blanqueo las coordenadas en EEPROM
		gps.Lat = 0;
		gps.Lon = 0;
		eeprom_write((char*) &gps.Lat, OFF_LAT, LEN_COORD);
		eeprom_write((char*) &gps.Lon, OFF_LON, LEN_COORD);

		//Reseteo el totalizador y guardo en eeprom
		Var.Total_gearmot_turns = 0; //Inicializo la variable
		eeprom_write((char*) &Var.Total_gearmot_turns, OFF_TOTALIZER,
		LEN_TOTALIZER);

		PIN[0] = 1122; //PIN ADMIN
		PIN[1] = 1111; //PIN USER 1
		PIN[2] = 2222; //PIN USER 2

		eeprom_write((char*) &PIN[0], OFF_PIN, LEN_PIN * 3);

		/*Escribo el url del endpoint por defecto*/
		eeprom_write(endpoint_url, OFF_ENDPOINT_URL,
		LEN_ENDPOINT_URL);

	}

	/*Recuperar valores en EEPROM*/
	eeprom_read((char*) &gprs_rpn, OFF_GSM_REG_NBR1, sizeof(gprs_rpn));
	eeprom_read((char*) &gen, OFF_GEN, sizeof(gen));
	eeprom_read((char*) &PIN[0], OFF_PIN, LEN_PIN * 3);
	eeprom_read(&id_Tag[0][0], OFF_ID_TAG1, sizeof(id_Tag));

	eeprom_read((char*) &id_SN, OFF_SN, LEN_SN);
	eeprom_read((char*) &Var.Total_gearmot_turns, OFF_TOTALIZER, LEN_TOTALIZER); //Leo el estado del Totalizer

#if USE_BLUETOOTH
		/*Conformo el Bluetooth HOST NAME usando el SN del dispositivo*/
		sprintf((char*) BT_HOST_NAME, "%s", id_SN);
	#endif

	/*Imprimo en formato numérico los celulares*/
	for (i = 0; i < 4; i++) {
		sscanf(gprs_rpn[i], "%u", &gsm.RPN[i]);
	}

	Chip_RTC_GetFullTime(LPC_RTC, &Local);

	/*Leo e imprimo las unidades seteadas*/
	read_flow_Vol_unit(&SP, &Var);

	/*ASEGURO ALGUNOS VALORES*/
	gen.contact = false;
	PumpOn = false;
	Var.rst_totalizer = false;

	/*-----------------------------------------------------------
	 * 	CREACIÓN DE TAREAS
	 */
#if USE_SIM231_TP_SCREEN
	Create_CommMB_HMI_Task();	//Comunicación con HMI
#else
	Create_CommNx_HMI_Task();
#endif
	Create_DIG_IN_task();
//	Create_DIG_OUT_task();
	Create_AN_IN_task();
#if !MOTOR_ELEC
	Create_MOTOR_CTRL_task();
#else
	Create_CommMOTOR_ELEC_Tx_Task();;
#endif
	Create_TANK_LEVEL_task();
	Create_ENERGY_MON_task();
	//Create_PWR_GENtask();
	Create_CommMODB_RS485_Rx_Task();
	Create_ALARM_Task();
#if USE_MOD1100S
	Create_CommWHART_Task();
#endif
	Create_COMM_MOBILE_Rx_task();

//	extern uint8_t FreeRTOSDebugConfig[];
//
//	int main(void) {
//		if (FreeRTOSDebugConfig[0] == 0) { /* just use it, so the linker cannot remove FreeRTOSDebugConfig[] */
//			for (;;)
//				; /* FreeRTOSDebugConfig[0] should always be non-zero, so this should never happen */
//		}
//		/* other code in main */
//	}

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	while (1) {
	};
	return 0;
}
/*-----------------------------------------------------------*/

///// INTERRUPCIONES   //////////////////////////////////////
/*-----------------------------------------------------------*/

void RTC_IRQHandler(void) {

	/* Check for alarm match */
	if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM)) {
		/* Clear pending interrupt */
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);

	}

	if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE)) {
		/* Clear pending interrupt */
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);

	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por las entradas digitales. */
void EINT3_IRQHandler(void) {

	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint32_t Interr;

	Interr = Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, 0);
	if (Interr & (1 << DI_1)) { // Int por DI-1
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, (1 << DI_1));
		Chip_GPIOINT_SetIntRising(LPC_GPIOINT, GPIOINT_PORT0, 0 << DI_1); //Deshabilito la interr
	}

	Interr = Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, 2);

	Interr = Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, 0); //Int por teclado frontal
	if (Interr & (0xf << 15)) {
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, (0xf << 15));
	}
	Interr = Chip_GPIOINT_GetStatusRising(LPC_GPIOINT, 2);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/
/* Atención de INT por DMA. */
void DMA_IRQHandler(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaUART1_RX) == SUCCESS) {
//		xSemaphoreGiveFromISR(Sem_UART1_RX, &xHigherPriorityTaskWoken);
	}

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaUART1_TX) == SUCCESS) {
	}

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaUART2_RX) == SUCCESS) {
	}

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaUART2_TX) == SUCCESS) {
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/
/* Atención de INT por WATCHDOG. */
void WDT_IRQHandler(void) {
	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	Chip_WWDT_GetCurrentCount(LPC_WWDT);

	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

//	On = (Bool) !On;

	/* The chip will reset before this happens, but if the WDT doesn't
	 have WWDT_MOD_WDRESET enabled, this will hit once */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);

//		if (wdtFeedState == NO_FEED) {
		Chip_WWDT_Start(LPC_WWDT); /* Needs restart */
//		}
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/
/* Atención de INT por I2C1. */
void I2C1_IRQHandler(void) {
	i2c_state_handling(I2C1);
}

/*-----------------------------------------------------------*/
/* Atención de INT por I2C0. */
void I2C0_IRQHandler(void) {
	i2c_state_handling(I2C0);
}

////////////////////////////////////////////////////////////////////////////////
void vApplicationMallocFailedHook(void)
/*-----------------------------------------------------------*/
{
	/* This function will only be called if an API call to create a task, queue
	 or semaphore fails because there is too little heap RAM remaining. */
	for (;;)
		;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
	/* This function will only be called if a task overflows its stack.  Note
	 that stack overflow checking does slow down the context switch
	 implementation. */
	char Error, TaskName[20];
	strcpy(TaskName, (char*) pcTaskName);
	for (;;) {
		Error++;
	}

}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {

}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void) {
//pca9532_setLeds(0x0100,0xffff);
	/* This example does not use the tick hook to perform any processing. */
}
