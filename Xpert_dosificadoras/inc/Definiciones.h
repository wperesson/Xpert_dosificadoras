/*
 * Definiciones.h
 *
 *	EDGE S.A.
 *	www.edgeinst.com.ar
 *	wperesson@edgeinst.com.ar
 *  Author: Walter Peressón
 *
 */

#ifndef Definiciones_H_
#define Definiciones_H_

/*
 * DEFINICIONES DE CONFIGURACION DEL EQUIPO
 *
 * Estas definiciones deben setearse de acuerdo al hardware usado.
 * Con 1 se activa la opción deseada
 */

/*===============================================
 * 		CONTROLLER_TYPE
 * Seleccionar el que corresponde al dispositivo actual
 */
#define		CONTROLLER_TYPE		"Xpert"
//#define		CONTROLLER_TYPE		"Xpert-m"
//#define		CONTROLLER_TYPE		"Xpert-lp"

/*===============================================
 *		DEVICE_TYPE
 * Distingue entre diferentes aplicaciones de dispositivos
 */
#define		DEVICE_TYPE		"CHIP"		//Chemical Injection Pump
//#define		DEVICE_TYPE		"POC"		//Pump Off Controller
//#define		DEVICE_TYPE		"SSV"		//Surface Safety Valve
//#define		DEVICE_TYPE		"TANKL"		//Tank Level
//#define		DEVICE_TYPE		"PRESS"		//Pressure sensor

/*===============================================
 *		POWER_TYPE
 * Selecciona la forma de alimentación del dispositivo
 */
#define		POWER_TYPE		"Solar"		//Paneles solares
//#define		POWER_TYPE		"Battery"		//Batería interna
//#define		POWER_TYPE		"External"		//Alimentación provista por sistema externo
//#define		POWER_TYPE		"Electric"		//Red eléctrica

/*===============================================
 *		MODULOS DE COMUNICACIÓN Y GPS
 */

//Posicionamiento por GPS / GLONASS
//#define USE_GPS					1

////Red Celular
//#define USE_SIM868				0 //Módulo de comunicaciones GPRS/GSM
//#define USE_BG96				1 //Módulo de comunicaciones LTE-CatM1

//WirelessHART
#define USE_MOD1100S			0 //Módulo de comunicaciones WirelessHART

//Bluetooth
#define USE_BLUETOOTH			0 //Módulo Bluetooth.

#if USE_BLUETOOTH
	#if USE_SIM868
	#define USE_BLUETOOTH_SIM868
	#else
	#define USE_BLUETOOTH_XXX
	#endif
#endif

/*===============================================
 *		PANTALLA TOUCH
 */
#define USE_SIM231_TP_SCREEN	0

/*===============================================
 *		SISTEMA DE GENERADOR DE BACKUP
 */
#define USE_BACKUP_GENERATOR	0

#define USE_GPRS_PERSONAL		0
#define USE_GPRS_CLARO			0
#define USE_GPRS_MOVISTAR		1
#define USE_GPRS_VIVA			0

#define USE_24LC16				1 //EEPROM de 16Kb
#define USE_AT24CM02			0 //EEPROM de 2Mb

#define SET_CTRY_ARGENTINA		1
#define SET_CTRY_KUWAIT			0
#define SET_CTRY_COLOMBIA		0

////Definir la empresa me permite escribir el endpoint url para la misma
//#define SET_COMPANY_EDGE		0
//#define SET_COMPANY_VISTA		1
//#define SET_COMPANY_REMINGTON	1
//#define SET_COMPANY_YPF			0
//#define SET_COMPANY_ALTOS		0

//Comentar según estemos desplegando en producción o en desarrollo
#define	UNDER_DEVELOPMENT		0

/*Definiciones útiles para debug*/
#define USAR_SNIFFER 0 //0 para desactivar,
#define MOTOR_ELEC 1//=0 para bomba solar
#if MOTOR_ELEC
#define V20 	0
#define ATV12 	0
#define ATV312	1
#endif
//=============================================================================

#include "board.h"
#include "MobileConfig.h"

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "stream_buffer.h"

#include "rtc_17xx_40xx.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "time.h"
#include "math.h"
#include "CRC_Calc.h"

#if USE_24LC16
#include "24LC16.h"
#elif USE_AT24CM02
#endif

#include "factReset.h"
//#include "ATV12H037.h"
#include "NxHMI.h"

#include "mobile.h"
#include "SMS.h"
#include "TcpIp.h"
#include "gps.h"
#include "mqtt.h"
#if ATV12
#include "ATV12H037.h"
#elif ATV312
#include "ATV312HU11.h"
#endif

#define FIRM_VERS	"x3.10"

#if USE_GPRS_PERSONAL
#define USER "datos"  // if your apn doesnt have username just leave it ""
#define PASS "datos"  // if your apn doesnt have password just leave it ""
#else
#define USER ""  // if your apn doesnt have username just leave it ""
#define PASS ""  // if your apn doesnt have password just leave it ""
#endif

#define MESS_BUFF_SZ		160

///*================================================================
// * 		LOSANT
// * ===============================================================
// */
//
//#define BROKER "broker.losant.com"
////#define PORT "8883"
//#define PORT "1883"

#define NO_SN	"No SN"

#define SEND_OK  		"\r\nSEND OK\r\n"
#define SEND_OK_LEN 	11 //Longitud del string anterior

#define MQTT_BUFF_SIZE		400//300
#define MESS_BUFF_SZ		160
#define BLUE_BUFF_SIZE		300
#define BUF485_SIZE			200

#define MOBILE_BUFF_SIZE	200

#define MC24LC08_ADDR 0xa0		/* I2C address of the 24LC08 EEPROM */

#define DEBUG_RS485_SEL						(0x31)
#define DEBUG_GSM_SEL                       (0x32)
#define DEBUG_WHART_SEL						(0x33)

#define M_PI 3.14159265359

#if	USE_24LC16
//EEPROM 24LC16 BLOCK MEMORIES
#define BLOCK_0		0*EEPROM_BLOCK_SIZE
#define BLOCK_1		1*EEPROM_BLOCK_SIZE
#define BLOCK_2		2*EEPROM_BLOCK_SIZE
#define BLOCK_3		3*EEPROM_BLOCK_SIZE
#define BLOCK_4		4*EEPROM_BLOCK_SIZE
#define BLOCK_5		5*EEPROM_BLOCK_SIZE
#define BLOCK_6		6*EEPROM_BLOCK_SIZE
#define BLOCK_7		7*EEPROM_BLOCK_SIZE
#elif  USE_AT24CM02
//EEPROM AT24CM02 BLOCK MEMORIES
#define BLOCK_0		0*EEPROM_PAGE_SIZE
#define BLOCK_1		1*EEPROM_PAGE_SIZE
#define BLOCK_2		2*EEPROM_PAGE_SIZE
#define BLOCK_3		3*EEPROM_PAGE_SIZE
#define BLOCK_4		4*EEPROM_PAGE_SIZE
#define BLOCK_5		5*EEPROM_PAGE_SIZE
#define BLOCK_6		6*EEPROM_PAGE_SIZE
#define BLOCK_7		7*EEPROM_PAGE_SIZE
#endif

// Offsets + Lengths of EEPROM fields
#define LEN_SN				16
#define LEN_ID				20
#define LEN_PIN				2
#define LEN_RPN				16
#define LEN_GEN				sizeof(gen)
#define LEN_COORD			4
#define LEN_TOTALIZER		4
#define LEN_APN				32
#define LEN_APN_PROF		64
#define LEN_BLUE_HOST_NM	16
#define LEN_GPRS_USR_PASS	10

#define LEN_CLIENT_ID		25
#define LEN_ACCESS_KEY		37
#define LEN_ACCESS_SECRET	65

#define LEN_ENDPOINT_URL	50

#define LEN_WHART_JK		8*4
#define LEN_WHART_NID		8*4

//Pagina #0
#define OFF_SN 				BLOCK_0						//0, 15
#define OFF_ID_TAG1			OFF_SN + LEN_SN				//16, 35
#define OFF_ID_TAG2			OFF_ID_TAG1+LEN_ID			//36, 55
#define OFF_ID_TAG3 		OFF_ID_TAG2+LEN_ID			//56, 75
#define OFF_GSM_REG_NBR1	OFF_ID_TAG3+LEN_ID			//76, 91
#define OFF_GSM_REG_NBR2	OFF_GSM_REG_NBR1+LEN_RPN	//92, 107
#define OFF_GSM_REG_NBR3	OFF_GSM_REG_NBR2+LEN_RPN	//108, 123
#define OFF_GSM_REG_NBR4	OFF_GSM_REG_NBR3+LEN_RPN	//124, 139
#define OFF_LAT 			OFF_GSM_REG_NBR4+LEN_RPN	//140, 155
#define OFF_LON 			OFF_LAT+LEN_COORD			//156, 159
#define OFF_TOTALIZER		OFF_LON+LEN_COORD			//160, 163
#define OFF_GEN				OFF_TOTALIZER+LEN_TOTALIZER	//164, 182
#define OFF_PIN				OFF_GEN+LEN_GEN				//183, 188	Para acceder a todas las claves
#define OFF_PIN_ADMIN		OFF_PIN						//183, 184	Para acceder solo a la clave del ADMIN
#define OFF_PIN_USER_1		OFF_PIN_ADMIN+LEN_PIN		//185, 186
#define OFF_PIN_USER_2		OFF_PIN_USER_1 + LEN_PIN	//187, 188

//Pagina #1
#define OFF_SP				BLOCK_1		//256,

//Pagina #2
#define OFF_FACT_DATA		BLOCK_2		//512,

//Pagina #3
#define OFF_sp_pump			BLOCK_3		//256,

//Pagina #4
#define OFF_WHART			BLOCK_4		//768,

//Pagina #5
#define OFF_CLIENT_ID		BLOCK_5								//1024, 1048; 	25
#define OFF_ACCESS_KEY		OFF_CLIENT_ID + LEN_CLIENT_ID 		//1049, 1085;	25+37=62
#define OFF_ACCESS_SECRET 	OFF_ACCESS_KEY+LEN_ACCESS_KEY 		//1086, 1150;	62+65=127
#define OFF_ENDPOINT_URL 	OFF_ACCESS_SECRET+LEN_ACCESS_SECRET //1151,	1200;	127+50=177

//Pagina #6
#define OFF_GPRS_APN_PROF1	BLOCK_6 				//64
#define OFF_GPRS_APN_PROF2	OFF_GPRS_APN_PROF1+LEN_APN_PROF 	//64+64=128
#define OFF_GPRS_APN_PROF3	OFF_GPRS_APN_PROF2+LEN_APN_PROF 	//128+64=192
#define OFF_GPRS_APN_PROF4	OFF_GPRS_APN_PROF3+LEN_APN_PROF 	//192+64=256

#define DIn_OD_MASK		0x01
#define DIn_1_MASK		0x02
#define DIn_2_MASK		0x04
#define DIn_3_MASK		0x08
#define DIn_4_MASK		0x10

#define DOut_1_MASK		0x01
#define DOut_2_MASK		0x02
#define DOut_3_MASK		0x04
#define DOut_4_MASK		0x08
#define DOut_rl1_MASK	0x10
#define DOut_rl2_MASK	0x20

//Notifications FLAGS
#define FROM_3G_BIT    	0x01
#define FROM_DEBUG_BIT  0x02

#define HMI_SL_VAR_ID	0xAA
#define HMI_SL_SP_ID	0xAB
#define HMI_SL_GPS_ID	0xAC
#define HMI_SL_TIME_ID	0xAD
#define HMI_SL_ID_ID	0xAE

#define DIn_OD	0x01
#define DIn_1	0x02
#define DIn_2	0x04
#define DIn_3	0x08
#define DIn_4	0x10

#define DOut_1	0x01
#define DOut_2	0x02
#define DOut_3	0x04
#define DOut_4	0x08
#define DOut_REL1	0x10
#define DOut_REL2	0x20

/*================================================================
 * 		TIPOS DE DATOS
 * ===============================================================
 */

typedef enum {
	Error = 0, /**<  */
	Ok, /**<  */
} Error_Type;

/*----------------------------------------------------------
 * GENERADOR EXTERNO
 */
typedef enum {
	genEv_none = 0,
	genEv_Manual_On,
	genEv_Manual_Off,
	genEv_Sch_On,
	genEv_Sch_Off,
	genEv_Low_Batt_On,
	genEv_Gsm_On,
	genEv_Gsm_Off,
	genEv_Modbus_On,
	genEv_Modbus_Off,
	genEv_fail_start_manual,
	genEv_fail_start_gsm,
	genEv_fail_start_lowBatt,
	genEv_fail_start_sched,
} gen_ctrl_ev_Type;

typedef struct {
	Bool contact; /*Estado de la variable de comando del generador*/
	Bool running;/*Estado del generador.
	 True: el generador esta corriendo
	 False: el generador no arranca*/
	uint16_t period; /* Cada cuantos dias se enciende */
	uint16_t test_timeout;/* Cuanto tiempo se enciende en los test periódicos*/
	uint16_t low_batt_timeout;/* Cuanto tiempo se enciende por bajo nivel de bateria */
	uint16_t gsm_timeout;/* Tiempo de trabajo cuando lo enciendo por SMS*/
	float low_batt_limit; /* Limite inferior en el cual se enciende el generador */
	time_t genNextStart;
	uint8_t sender;/*Nro de quien envio el SMS de control*/
} GENERATOR_CTRL_T;

/*----------------------------------------------------------
 * Origen del evento de comando de Start/stop de la bomba
 */
typedef enum {
	OnOff_front_panel = 0,
	OnOff_modbus,
	OnOff_sms,
	OnOff_web,
	OnOff_very_low_batt,
} cmd_PumpOn_Type;

/*----------------------------------------------------------
 * Origen de modificacion del parámetro de la bomba
 */
typedef enum {
	src_front_panel = 0,
	src_modbus,
	src_sms,
	src_web,
	src_dig_input,
	src_self_generated
} cmd_SP_Param_Src_Type;

typedef enum {
	scr_Home = 0, scr_whart = 20,
} screen_idx_Type;

/*----------------------------------------------------------
 * CANALES ANALÓGICOS
 */
typedef struct {
	uint16_t ADC_Un; /* Valor digital del ADC */
	float Signal;/* Valor de señal mA o mV */
	float Eng_Un;/* Valor Ingenieria PSI*/
} VAR_ADC_CH_SETUP_T;

/*----------------------------------------------------------
 * GPS
 */
typedef struct {
	float Lat;
	float Lon;
	float Speed_kph;
	float Altitude;
	Bool gps_fix;
	struct tm GPSTime;
} GPS_INFO_T;

/*----------------------------------------------------------
 * NIVEL DE TANQUE Y CAUDAL
 */
typedef enum {
	ref_points = 1,
	Vert_Cyl,
	Horiz_Cyl,
	Rect,
	Horiz_Oval,
	Vert_Oval,
	Horiz_Caps,
	Vert_Caps
} Tank_Shp_Typ;

/*----------------------------------------------------------
 * NIVELES DE TANQUES
 */
typedef struct {
	float nivel; /* Nivel actual en m*/
	float capacidad; /* Contenido actual en litros*/
} VAR_TNK_LEV_T;

//typedef enum {
//	Galons = 1, Liters
//} Vol_UNITS_Typ;

typedef enum {
	Liters = 1, Galons
} Vol_UNITS_Typ;

typedef enum {
	Liters_hour = 1, Liters_day, Galons_hour, Galons_day
} Flow_UNITS_Typ;

///*UNITS, fow*/
//typedef enum {
//	Galons_day = 1, Galons_hour, Liters_day, Liters_hour
//} Flow_UNITS_Typ;

/*DO, posicion de cada bit*/
typedef enum {
	DO1_bp, DO2_bp, DO3_bp, DO4_bp, Rl1_bp, Rl2_bp
} DO_position_Typ;

/*DI, posicion de cada bit*/
typedef enum {
	OD_bp, DI1_bp, DI2_bp, DI3_bp, DI4_bp
} DI_position_Typ;

/*Apn, profiles*/
typedef enum {
	Profile_1 = 0, Profile_2, Profile_3, Profile_4
} APN_profile_Type;

/*----------------------------------------------------------
 * VAR
 */
typedef struct {

	VAR_ADC_CH_SETUP_T an[6];
	VAR_TNK_LEV_T tnk[6];

	uint16_t whart_sstr;
	uint32_t whart_stat;
	uint16_t gsm_sstr;

	/*Selecciona el perfil del APN a aplicar*/
	APN_profile_Type apn_prof;

	/*DI almacena el estado de cada una de las ENTRADAS digitales incluido el sensor de puerta
	 *Bit 0: DI_OD
	 *Bit 1: DI_1
	 *Bit 2: DI_2
	 *Bit 3: DI_3
	 *Bit 4: DI_4
	 *Bit 5:
	 *Bit 6:
	 *Bit 7:
	 */
	uint8_t DI;

	/*DO almacena el estado de cada una de las SALIDAS digitales.
	 * Usar el enum DOx_bp para identificar el valor de la salida requerido
	 *Bit 0: DO1_bp
	 *Bit 1: DO2_bp
	 *Bit 2:
	 *Bit 3:
	 *Bit 4:
	 *Bit 5:
	 *Bit 6:
	 *Bit 7:
	 */
	uint8_t DO;

	float PumpRate; //Variable que mide el caudal según el encoder.
	float MotorRPM; //Revoluciones Por Minuto del motor
	float tank_Lev;	//Nivel en altura de columna
	float tank_vol; 		//Volumen actual. Depende de la unidad elegida
	float tank_vol_liters; 	//Volumen del tanque en litros
	uint16_t tank_vol_perc;	//Volumen del tanque en porcentaje
	uint16_t LinePress;
	float VPanel;
	float VBatt;
	float CPanel;
	float VExtGen;
	float PPanel;
	float EPannel;
	float ChPannel; //Carga en uC del panel en corriente
	float ZeroConc;
	float M;		// Pendiente de la recta
	float B;		// Ordenada al origen

	double prev_error;
	double int_error;
	double control;
	double curr_error;

	uint32_t Total_gearmot_turns; //Cantidad de vueltas totales de la salida del motorreductor
	float Total_pumped; //Cantidad de GAL bombeados

	/*
	 *  local_remoto = TRUE 	->	El sistema esta en LOCAL, por lo tanto obedecerá unicamente a la pantalla.
	 *  local_remoto = FALSE 	->	El sistema esta en REMOTO, por lo tanto seguirá los comandos por ej de int, sms, o modbus
	 */
	Bool local_remoto;
	char flow_unit[4];	//String de unidades de caudal: LPH, LPD, GPH, GPD
	char vol_unit[4];	//String de unidades de volumen: LIT or GAL

	/*Resetea el totalizador desde el modbus
	 * 	1: Total_mot_turns=0; Total_pumped=0; además resetea en eeprom
	 * 	0: Cuanta Total_mot_turns y Total_mot_turns;
	 */
	Bool rst_totalizer;

	float caudal_produccion;	//Variable proceso.
	float presion_referencia;	//Variable proceso.

} Var_Type;

/*----------------------------------------------------------
 * HMI - MODBUS MAP
 */
typedef enum {

	Var_PumpRate = 1,
	Var_TankLevel,
	Var_TankVolume,
	Var_TankLevel_P,
	Var_TankLevel_Eng,
	Var_LinePress_Eng,
	Var_VPannel,
	Var_CPannel,
	Var_VExtGen,
	Var_PPannel,
	Var_EPannel,
	Var_ChPannel,
	Var_VBatt,
	Var_Motor_Err,
	Var_Motor_RPM,
	GEN_RUNNING,
	Var_ALARM,
	Var_Total_PUMPED,

	Var_AN1 = 20,
	Var_AN2,
	Var_AN3,
	Var_AN4,
	Var_AN5,
	Var_AN6,
	Var_DI,

	PUMP_ON = 30,
	OVERRIDE,
	GEN_START,
	RST_TOTAL,

	SP_PumpRate = 50,
	SP_TankCap,
	SP_SpanTankEng,
	SP_SpanTankCap,
	SP_ZeroTankEng,
	SP_ZeroTankCap,
	SP_War_TankLevel,
	SP_AL_TankLevel,

	SP_CYCLE_T = 60,
	SP_THRESH,
	SP_CF,
	SP_VPR,
	SP_PLG_SIZE,
	SP_PLG_STR,
	SP_HEADS,
	SP_GR,
	SP_MAXSPD,
	SP_EPPR,
	SP_PROP,
	SP_INT,
	SP_DER,
	SP_Motor_RPM,		//Solo lectura
	SP_PAGE,		//Página actual en la pantalla
	SP_UNITS, /*Bits 0..1: Unidades de Volumen (Galon, Liters)
	 Bits 2..3: Unidades de caudal (Galon_day, Galon_hour, Liter_day, Liter_hour)*/

	SP_tnk_Shp = 80,
//	SP_d3_tnk_RT_length,
//	SP_d1_tnk_VT_HT_diam_RT_width,
//	SP_d2_tnk_VT_height_HT_lenght_RT_height,

	SP_d2_tnk_VT_height_HT_lenght_RT_height,
	SP_d1_tnk_VT_HT_diam_RT_width,
	SP_d3_tnk_RT_length,
	SP_tnk_sensor_hgt,
	SP_tnk_dens,

	Lat = 90,
	Lon,
	Alt,
	Speed_kph,
	gps_fix,
	UtcTime,

	TimeDate = 100,
	SP_TimeZone,
	SP_UseGpsTime,

//	SP_Client = 110,
//	SP_Field,
//	SP_Well,

	SP_id_tag1 = 110,
	SP_id_tag2,
	SP_id_tag3,
	SP_PumpSN,
	SP_FirmVers,

	SP_RS485_WL = 120,
	SP_RS485_SLID,
	SP_RS485_BAUD,
	Var_GSM_IMEI,
	Var_GSM_NBR,
	Var_GSM_SSTRG,
	Var_GSM_NET,
	SP_GSM_REG_NBR1,
	SP_GSM_REG_NBR2,
	SP_GSM_REG_NBR3,
	SP_GSM_REG_NBR4,
	SP_GSM_REG_AUTH,
	SP_GSM_SMS_SC,
	SP_GSM_SEND_SMS,

	SP_WHART_NETID = 140,
	SP_WHART_JKEY,
	SP_WHART_RATE,
	Var_WHART_SSTR,
	Var_WHART_STAT,
	SP_WHART_STAG,
	SP_WHART_LTAG,
	SP_WHART_NODE_ID,

	SP_GPRS_APN = 150,

	SP_AN_SRC_1 = 160,
	SP_AN_INTYP_1,
	SP_AN_HS,
	SP_AN_LS,
	SP_AN_HE,
	SP_AN_LE,
	Var_AN_SIG,
	Var_AN_ENG,
	AN_IDX,

	GEN_PERIOD = 180,
	GEN_TEST_TIMEOUT,
	GEN_LOW_BATT_TIMEOUT,
	GEN_NXT_START,
	GEN_LOW_BATT_LIM,

	SP_FACT_RESET = 240,
	SP_SET_FACT_RESET_DATA,
	SP_PIN_ADMIN,
	SP_PIN_USR1,
	SP_PIN_USR2

} Modb_RegAdd; //Mantener actualizado con el SHipTide

/*----------------------------------------------------------
 * ENTRADAS ANALOGICAS
 */
typedef enum {
	Tank_Level = 0, Line_Press, Not_Used,
} AnIn_SourceType;

typedef enum {
	R0_5V = 0, R0_10V, R4_20mA,
} AnIn_RangeType;

typedef struct {
	AnIn_RangeType InTyp;	//Rango de entrada del canal, 0-5V, 4-20mA
	AnIn_SourceType Src;	//A que sensor esta asignado el canal
	uint16_t HS; 	//Valor de la señal: 5V o 20mA o 10V
	uint16_t LS;	//
	float HE; //Valor de la magnitud en unidades de ingenieria: 3000psi, 5psi
	float LE;
} SP_ADC_CH_SETUP_T;//size: 14 bytes

typedef struct {
	float lit_by_m;	//Coeficiente de conversión de altura a capacidad. En Lit/m
	float dens;		//Ganancia o Densidad relativa del químico
	float off;		//Offset o nivel inicial del tanque
} TANK_CH_SETUP_T;//size: 14 bytes

/*----------------------------------------------------------
 * SP
 */
typedef struct {

	Bool RS485_WL; //1: Wireless; 0: Wired
	Bool UseGpsTime;
	Bool Activate_GPS; //Controla si activo o no el módulo GPS
	Bool Activate_GPRS; //Controla si activo o no el módulo celular
	Bool Activate_Bluetooth; //Controla si activo o no el Bluetooth
	Bool auto_manual; //Modo de control de la bomba. 0: manual, 1: Auto

	Bool alreadyRegisteredOnLosant;

	int8_t TimeZone;

	uint8_t AnChannelAssig[2]; //Indica en que canal esta conectada cada sensor de LINE_PRESS y TANK_LEVEL

	Vol_UNITS_Typ vol_unit; 	//Unidad de Volumen
	Flow_UNITS_Typ flow_unit;	//Unidad de Caudal

	int16_t RS485_SLID;

	uint16_t WHART_RefRate;
	uint16_t WHART_NetId;
	uint32_t WHART_JoinKey[4];
	uint16_t SP_GSM_REG_AUTH; //Bits para indicar quien esta autorizado a recibir SMS

	uint16_t BT_pinCode;

	uint16_t mqtt_ref_rate; /*Refresh rate en minutos*/

	SP_ADC_CH_SETUP_T An[6];

	TANK_CH_SETUP_T tnk[6];

	int32_t RS485_BAUD;

	uint16_t key; 	//Clave para saber q leyó bien la EEPROM

} SP_Type;


/*----------------------------------------------------------
 * sp_pump
 */
typedef struct {

	uint8_t heads; //Nro de cabezales
	uint8_t GR; //Gear Ratio
	uint8_t EPPR; //Encoder Pulsos por Revolución

	Tank_Shp_Typ tnk_shp;
	Vol_UNITS_Typ vol_unit; 	//Unidad de Volumen
	Flow_UNITS_Typ flow_unit;	//Unidad de Caudal

	int16_t RS485_SLID;
	int16_t level_warning_limit_T1;
	int16_t level_alarm_limit_T1;
	uint16_t Cycle; //Ciclo en MIN para bajos caudales
	uint16_t maxSpd; //Límite Max del motor en RPM
	float maxFlow;//pump rate maximo
	uint16_t Threshold; //Valor límite bajo del cual se aplica PWM con un T = Cycle
#if MOTOR_ELEC
	uint16_t nomSpd; //velocidad nominal del motor en RPM(Motor electrico)
	uint16_t poles; //numero de polos en el motor
	uint16_t minSpeed;//velocidad minima a 15Hz
#endif
	float TankCap;
	float SpanTankCap;
	float ZeroTankCap;
	float SpanTankEng;
	float ZeroTankEng;

	float plg_size;
	float plg_str;
	float CF; //Correction factor. Valor calculado como el coeficiente para corregir en base a la calibración
	float VPR; //Valor de Volumen Por Revolución en Lit o Gal
	float PumpRate; //Valor seteado de Caudal
	float MotorRPM; //RPM del motor.

	float tnk_d1_VT_HT_diam_RT_width;		//Actualizada 29-09-2020
	float tnk_d2_VT_height_HT_lenght_RT_height;		//Actualizada 29-09-2020
	float tnk_d3_RT_length;					//Actualizada 29-09-2020
	float tnk_sensor_hgt;
	float tnk_dens;

	uint16_t key; 	//Clave para saber q leyó bien la EEPROM
	//PID
	double DerGain;
	double IntGain;
	double PropGain;

} SP_Pump_Type;
/*----------------------------------------------------------
 * MOBILE
 */
#if V20
#include "SIN_V20.h"
#endif
/*MOBILE, event source*/
typedef enum {
	MOBILE_Init,
	sendToMOBILE,
	Alarm,
	Location,
	Tank,
	SMS_ServiceCenter,
	sendSMS,
	read_process_sms,
	cmd_Location,
	cmd_Report,

	cmd_Start_Notif,
	cmd_Stop_Notif,
	cmd_Welcome,
	cmd_reset_totalizer,
	cmd_msg_received,
	ref_gps,
	ref_SMS,
//	cmd_GPRS_set_apn,
	cmd_read_events_log,

	cmd_NO_CMD //Para indicar un comando vacio

} EvtToMOBILE_src_Type;

/*SIM868, event source, value, message*/
typedef struct {
	EvtToMOBILE_src_Type Src;
	uint32_t Val;
	char Message[MESS_BUFF_SZ]; //Array de caracteres del mensaje
} EvtToMOB_Type;

typedef struct {
	cmd_SP_Param_Src_Type src;
	Modb_RegAdd parameter;
	uint8_t sender;
	float value;
} Parameter_upd_Type;

///*UART. Data to transfer*/
//typedef struct Data {
//	char *ptr;	//Puntero
//	uint16_t Len;	//Contexto de la variable
//} uart_tx_Type;

/*----------------------------------------------------------
 * WirelessHART
 */

/*WHART connection status*/
typedef enum {
	reset_whart_module,
	send_cmd,
	comm0,
	comm1,
	write_TAG_DES_DATE,
	write_DEV_VAR_UNIT,
	write_LONG_TAG,
	write_NET_ID,
	read_NET_ID,
	write_JOIN_KEY,
	read_JOIN_STATUS,
	write_FORCE_JOIN_MODE,
	write_UPD_RATE,
	read_BURST_MODE,
	request_ACT_ADV,
	config_changed
} EvtToHARTSrc_Type;

/*Eventos hacia el Hart*/
typedef struct {
	EvtToHARTSrc_Type Src;
	uint16_t Val;
} EvtToHART_Type;

///*----------------------------------------------------------
// * GSM
// */
//typedef struct {
//	char imei[17]; 			//El imei tiene 15 cifras
//	char oper[20]; 			//Operator
//	char pn[15]; 			//Phone Number
//	char SMS_orig[15]; 		//Nro del que envía el SMS
//	char SMS_scenter[15];	//SMS service Center
//	uint32_t RPN[4]; 		//Numeros de celulares en formato numérico
//	uint8_t SMS_idx;		//Indice del mensaje
//	uint8_t rssi; 			//Received Signal Strength Indication
//	uint8_t ber; 			//Bit Error Rate
//	uint8_t act;		//Access technology 0:GSM; 8:LTE CatM1; 9:LTE Cat NB1
//	uint16_t Send_SMS_to_idx; //Phonebook Index al cual enviar un mensaje desde el panel.
//	Bool reg_on_netw;		//Registrado en la red??
//
//	/*Activa el forwarding de los SMS
//	 * 	1: fwd on
//	 * 	0: fwd off
//	 */
//	Bool sms_fwd;
//	char SMS_fwd_orig[15]; 		//Nro donde hacer fwd del SMS
//
//	/*Indica si el APN es nuevo
//	 * 	1: New APN
//	 * 	0: Actual APN
//	 */
//	Bool new_apn;
//} GSM_Type;

///*----------------------------------------------------------
// * GPS
// */
//typedef struct {
//	float Lat;
//	float Lon;
//	float Speed_kph;
//	float Altitude;
//	Bool gps_fix;
//	struct tm GPSTime;
//} GPS_Type;

/*----------------------------------------------------------
 * GPRS
 */

/*Estado de la conexion GPRS*/
typedef enum {
	gprs_IP_INITIAL = 0,
	gprs_IP_START,
	gprs_IP_CONFIG,
	gprs_IP_GPRSACT,
	gprs_IP_STATUS,
	gprs_TCP_CONNECTING,
	gprs_CONNECT_OK,
	gprs_TCP_CLOSING,
	gprs_TCP_CLOSED,
	gprs_PDP_DEACT,
	gprs_ERROR
} gprs_connection_stat_Type;

/*Gprs conn status*/
typedef enum {
	lte_IP_INITIAL = 0,
	lte_IP_START,
	lte_IP_CONFIG,
	lte_IP_LTEACT,
	lte_IP_STATUS,
	lte_TCP_CONNECTING,
	lte_CONNECT_OK,
	lte_TCP_CLOSING,
	lte_TCP_CLOSED,
	lte_PDP_DEACT,
	lte_ERROR
} lte_connection_stat_Type;

/*TCP_send. Return codes*/
typedef enum {
	tcpSend_SEND_FAIL = -1, //SIM868 devolvio SEND_FAIL
	tcpSend_NONE_SEND_OK = -2, //En sistema no devolvio SEND_OK
	tcpSend_CME_ERROR = -3,
	tcpSend_NOT_READY = -4, //No se encontr'o el > para enviar
	tcpSend_NOT_BROKER_RESPONSE = -5 //El broker no devolvio respuesta
// Si es >0 indica cuantos bytes se enviaron
} tcp_send_Type;

/*Indice de operadoras*/
typedef enum {
	Movistar, Personal, Claro, Viva, Ooredo, custom
} Operadora_Red_celular_Id;

/*----------------------------------------------------------
 * MQTT
 */

/*Mqtt task states*/
typedef enum {
	mqtt_ev_NO_EV,
	mqtt_ev_connect_to_broker,
	mqtt_ev_publish_variables,
	mqtt_ev_publish_events,
	mqtt_ev_publish_id,
	mqtt_ev_publish_commands,
	mqtt_ev_publish_sett,
	mqtt_ev_publish_alarms,
	mqtt_ev_subscribe,
	mqtt_ev_disconnect_from_broker,
	mqtt_ev_req_keys_from_broker,
	mqtt_ev_gprs_on,
} mqtt_event_type;

/*----------------------------------------------------------
 * BLUETOOTH
 */

/*BLUETOOTH task states*/
typedef enum {
	blue_idle,
	blue_init,
	blue_incoming_rq,
	blue_scanning,
	blue_pairing,
	blue_connected,
	blue_publish_id,
	blue_deinit,
	blue_unchanged
} blue_state_type;

/*BLUETOOTH Events*/
typedef enum {
	blue_ev_NE,
	blue_ev_pb,
	blue_ev_tmr,
	blue_ev_device,
	blue_ev_hb1,
	blue_ev_hb2,
	blue_ev_hb3,
	blue_ev_hb4
} blue_ev_type;

/*BLUETOOTH New state and event*/
typedef struct {
	blue_state_type sta; //Estado siguiente
	blue_ev_type ev;	// Evento: pulsador, timer
} ev_state_bluet_type;

/*----------------------------------------------------------
 * MODBUS SERVER
 */

/*MODBUS COILS IDENTIFICATION*/
typedef enum {
	Coil_1_MarchaB1,
	Coil_2_MarchaB2,
	Coil_3_Rst_Tot,
	Coil_4_DO1,
	Coil_5_DO2,
	Coil_6_DO3,
	Coil_7_DO4,
	Coil_8_RLY1,
	Coil_9_RLY2,
	Coil_10_Enable_GprsModule,
	Coil_11_Enable_GPS,
	Coil_12_RS485_Wireless,
	Coil_13_AUTO_MANUAL,
	Coil_14_Send_SMS_Welc_Usr1,
	Coil_15_Send_SMS_Welc_Usr2,
	Coil_16_Send_SMS_Welc_Usr3,
	Coil_17_Send_SMS_Welc_Usr4,
} MB_Coils_Id;

/*
 * MODBUS. Coils ID // Vienen de Xpert-m
 typedef enum {
 Coil_1_DO1,
 Coil_2_DO2,
 Coil_3_Enable_GprsModule,
 Coil_4_RS485_Wireless,
 Coil_5_Send_SMS_Welc_Usr1,
 Coil_6_Send_SMS_Welc_Usr2,
 Coil_7_Send_SMS_Welc_Usr3,
 Coil_8_Send_SMS_Welc_Usr4,
 } MB_Coils_Id;
 */

/*MODBUS Function codes*/
typedef enum {
	FC01_Read_Coils = 1,
	FC02_Read_Disc_Inp,
	FC03_Read_Hold_Reg,
	FC04_Read_Inp_Reg,
	FC05_Write_Sing_Coil,
	FC06_Write_Sing_Reg,
	Read_Excep_Stat,
	Diagnostic,
	FC10_Write_Mult_Reg = 16,
	FC6D_String_Read = 109,
	FC6E_String_Write = 110,
} MB_Function_Code;

/*MODBUS SERVER MEMORY ADDRESS*/
typedef enum {
	MOD_PumpRate = 0,	//4Bytes
	MOD_TankLevel = 4,	//4Bytes
	MOD_LinePress = 8,	//4Bytes
	MOD_VPannel = 10,	//4Bytes
	MOD_CPanel = 14,	//4Bytes
	MOD_VBatt = 18,		//4Bytes
	MOD_VExtGen = 22,	//4Bytes

} MOD_SERVER_Add_Type;

/*----------------------------------------------------------
 * ALARMAS
 */

/*Posición de cada alarma en el Reg de ALARMAS*/
typedef enum {
	Al_LowBatt,
	Al_advertencia_bajo_nivel_T1,
	Al_alarma_bajo_nivel_T1,
	Al_DoorOpen,
	Al_GenFail,
	Al_alarma_motor_atascado_B1,
	/*Mantener los códigos anteriores en orden para retrocompatibilidad con las pantallas*/
	Al_VeryLowBatt,
	Al_advertencia_bajo_nivel_T2,
	Al_alarma_bajo_nivel_T2,
	Al_alarma_motor_atascado_B2,
	Al_LAST
} AlarmPos_Type;

typedef enum {
	al_st_NO_ALARM,
	al_st_ALARM_SHOOTED,
	al_st_ALARM_ON,
	al_st_ALARM_RESETED,
} AlarmStat_Type;

/*----------------------------------------------------------
 * EVENTOS
 */

/*Código de eventos a almacenar en el log*/
typedef enum {
	Sart_pump_FP = 10,
	Stop_pump_FP,
	Sart_pump_Web,
	Stop_pump_Web,
	Sart_pump_MODB,
	Stop_pump_MODB,
	Sart_pump_SMS_usr1 = 20,
	Stop_pump_SMS_usr1,
	Sart_pump_SMS_usr2,
	Stop_pump_SMS_usr2,
	Sart_pump_SMS_usr3,
	Stop_pump_SMS_usr3,
	Sart_pump_SMS_usr4,
	Stop_pump_SMS_usr4,
	Sart_pump_SMS_usr5,
	Stop_pump_SMS_usr5,

	NewFlow_FP = 40,
	NewFlow_SMS_usr1,
	NewFlow_SMS_usr2,
	NewFlow_SMS_usr3,
	NewFlow_SMS_usr4,
	NewFlow_SMS_usr5,

	logged_in_Usr_Admin = 50,
	logged_in_Usr_1,
	logged_in_Usr_2,
	logged_in_Usr_Dev,

} Events_to_log_Type;

#define mainSOFTWARE_INTERRUPT_PRIORITY 		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY

/* Task priorities  - higher the number higher the priority */
#define Prio_ERROR_Task 			6
#define Prio_ALARM_Task 			4
#define Prio_CommMB_HMI_Task		5
#define Prio_CommNx_HMI_Task		5
#define Prio_CommNx_HMI_Rx_Task		5
#define Prio_CommNx_HMI_Tx_Task		5

#define Prio_AN_IN_Task				4
#define Prio_DI_Task				3
#define Prio_DO_Task				3
#define Prio_EN_MON_Task			5

#define Prio_MOTOR_CTRL_Task		7
#define Prio_PWR_GEN_Task			4
#define Prio_TANK_LEVEL_Task		4
#define Prio_EV_LOG_Task			4

#define Prio_CommMODB_RS485_Rx_Task	6
#define Prio_CommMODB_RS485_Tx_Task	7

#define Prio_COMM_MOBILE_Rx_Task	6
#define Prio_COMM_MOBILE_Tx_Task	4
#define Prio_COMM_MOBILE_Ev_Task	4
#define Prio_COMM_MQTT_Task			5

#define Prio_CommWHART_Task			4

#define Prio_CommMOTOR_ELEC_Tx_Task	6
#define Prio_CommMOTOR_ELEC_Rx_Task	5
#define Prio_CommMOTOR_ELEC_Ctrl_Task 7
/* DMA channel assignment */
#define dmaUART1_TX		0
#define dmaUART1_RX		1
#define dmaUART0_TX		2
#define dmaUART0_RX		3
#define dmaUART2_TX		4
#define dmaUART2_RX		5
#define dmaUART3_TX		6
#define dmaUART3_RX		7

/* TIMERS */
extern xTimerHandle Timers[1];

/* TAREAS */
extern TaskHandle_t tsk_error_handler;
extern TaskHandle_t tsk_alarm_handler;
extern TaskHandle_t tsk_di_handler;
extern TaskHandle_t tsk_do_handler;
extern TaskHandle_t tsk_an_in_handler;
extern TaskHandle_t tsk_motor_control_handler;
extern TaskHandle_t tsk_tank_level_handler;
extern TaskHandle_t tsk_energy_mon_handler;
extern TaskHandle_t tsk_power_gen_handler;
extern TaskHandle_t tsk_modb_hmi_handler;
extern TaskHandle_t tsk_nex_hmi_handler;
extern TaskHandle_t tsk_nex_hmi_rx_handler;
extern TaskHandle_t tsk_nex_hmi_tx_handler;
extern TaskHandle_t tsk_whart_handler;
extern TaskHandle_t tsk_modb_rs485_rx_handler;
extern TaskHandle_t tsk_modb_rs485_tx_handler;
extern TaskHandle_t tsk_events_log_handler;
extern TaskHandle_t tsk_mobile_rx_handler;
extern TaskHandle_t tsk_mobile_tx_handler;
extern TaskHandle_t tsk_mobile_ev_handler;
extern TaskHandle_t tsk_mqtt_handler;
extern TaskHandle_t tsk_motor_elec_tx_handler;
extern TaskHandle_t tsk_motor_elec_rx_handler;
extern TaskHandle_t tsk_motor_elec_Ctrl_handler;
/* COLAS */
extern QueueHandle_t queMsgFromHMI;
extern QueueHandle_t queHMI_ev;
extern QueueHandle_t queMsgToHMINx;
extern QueueHandle_t queMsgFromHMINx;
extern QueueHandle_t queMsgToMOBILE;		//Datos de UART a enviar a SIM868
extern QueueHandle_t queMsgFromMOBILE;	//Datos de UART proveniente de SIM868
extern QueueHandle_t queMsgToRS485;		//Datos de UART a enviar a RS485
extern QueueHandle_t queMsgToATV12;
extern QueueHandle_t queEvtGenStart;
extern QueueHandle_t queEvtToMOBILE;
extern QueueHandle_t queEvtToHART;
extern QueueHandle_t queHEAD_FLOW_SETT_upd;
extern QueueHandle_t queTANK_SETT_upd;
extern QueueHandle_t queMQTT_ev;
extern QueueHandle_t queBLUE_SM;
extern QueueHandle_t queEV_LOG;
extern QueueHandle_t queSentByTx;			//Cola de datos enviados a recibir.

/* SEMAFOROS */
extern SemaphoreHandle_t semExpT15_HMI; //
extern SemaphoreHandle_t semExpT35_HMI; //
extern SemaphoreHandle_t semExpT15_485; //
extern SemaphoreHandle_t semExpT35_485; //
extern SemaphoreHandle_t mtxI2C1;
extern SemaphoreHandle_t mtxMOBILE;
extern SemaphoreHandle_t semBLUE_pb;
extern SemaphoreHandle_t semDMA_TX1;

/* STREAM BUFFERS */
extern StreamBufferHandle_t stream_Sniffer;

extern uint32_t PrevT, ActT, ElapT;

extern Var_Type Var; 	// Estructura de VARIABLES DEL SISTEMA
extern SP_Type SP; 	// Estructura de SET POINTS del sistema
extern SP_Pump_Type sp_pump;
//extern volatile REG_Type REG;	// Estructura de REGISTROS del sistema
extern uint8_t Buf485[BUF485_SIZE];

extern PHONE_DATA_T gsm;
extern GPS_DATA_T gps;

extern GENERATOR_CTRL_T gen;
extern Bool upd_next_gen_start_data;

extern Bool config_JoinKey, config_JoinNow, upt_rate;
extern uint16_t mem_Net_Id, mem_Ref_Rate, Act_Page;
extern uint8_t hart_WR_NET_ID_ret_cnt, hart_WR_JOIN_KEY_ret_cnt,
		hart_WR_UPD_RATE_ret_cnt;

extern char gprs_rpn[4][LEN_RPN];
extern char gprs_apn[LEN_APN];
extern char gprs_usr[LEN_GPRS_USR_PASS];
extern char gprs_pass[LEN_GPRS_USR_PASS];

extern char BT_HOST_NAME[LEN_BLUE_HOST_NM];

extern char id_Client[LEN_ID];
extern char id_Field[LEN_ID];
extern char id_well[LEN_ID];

extern volatile char SMS_SC[LEN_ID];

extern char id_SN[LEN_SN]; //Serial Number del dispositivo
extern char id_Tag[3][LEN_ID];

extern char clientID[LEN_CLIENT_ID];
extern char access_key[LEN_ACCESS_KEY];
extern char access_secret[LEN_ACCESS_SECRET];

extern char endpoint_url[LEN_ENDPOINT_URL];

extern const char ok_char[];

extern SP_Type SPT;
extern Var_Type VarT;
extern float PtArr[20];
extern uint16_t NroPuntos;

extern RTC_TIME_T Local, UTC;
extern time_t RawTime, utcTime;
extern Bool uart0_tx_busy;
//extern struct tm GPSTime;
extern struct tm curTime;

extern uint16_t PIN[3];
extern uint16_t RegAlarmas;
volatile extern Bool debug_gsm, sniff_gsm;
//extern int16_t Ubi_Server_Interactions_ctr;
volatile extern Bool debug_whart/*, data_RX_avail ,data_TX_avail*/;
extern Bool PumpOn;
extern Bool global_Override;
extern Bool global_Very_low_battery;
extern uint8_t global_Scada_Control_cnt;
extern Bool sendingSMS;

/*Encabezados de funciones*/
void Create_DIG_IN_task(void);
void Create_CommMB_HMI_Task(void);
void Create_DIG_OUT_task(void);
void Create_MOTOR_CTRL_task(void);
void Create_TANK_LEVEL_task(void);
void Create_AN_IN_task(void);
void Create_ENERGY_MON_task(void);
void Create_PWR_GENtask(void);
void Create_CommMODB_RS485_Rx_Task(void);
void Create_CommMODB_RS485_Tx_Task(void);
void Create_COMM_SIM868_task(void);
void Create_ALARM_Task(void);
void Create_CommWHART_Task(void);
void Create_COMM_MOBILE_Rx_task(void);
void Create_COMM_MOBILE_Tx_task(void);
void Create_COMM_MOBILE_Ev_task(void);
void Create_COMM_MQTT_SIM868_task(void);
void Create_COMM_MQTT_BG96_task(void);
void Create_COMM_MQTT_task(void);
void Create_CommMOTOR_ELEC_Rx_Task(void);
void Create_CommMOTOR_ELEC_Tx_Task(void);
void Create_CommMOTOR_ELEC_Ctrl_Task(void);

extern void SetFlag(char*, uint8_t, Bool);
extern Bool GetFlag(char*, uint8_t);

extern void SetAlarm(AlarmPos_Type Id);
extern void ClearAlarm(AlarmPos_Type Id);
extern Bool GetAlarm(AlarmPos_Type Id);

extern Bool guardar_eeprom(char *buf, uint16_t offset, uint16_t len);
extern Bool leer_eeprom(char *buf, uint16_t offset, uint16_t len);
extern void snd_tank_sett(Parameter_upd_Type*, float, Modb_RegAdd,
		cmd_SP_Param_Src_Type);
extern void snd_head_flow_sett(Parameter_upd_Type*, float, Modb_RegAdd,
		cmd_SP_Param_Src_Type, uint8_t);
extern void read_flow_Vol_unit(SP_Type*, Var_Type*);
extern Status Check_CRC(char*, uint16_t);


extern Typ_VSP VSD_bomba1;
#endif /* Definiciones_H_ */
