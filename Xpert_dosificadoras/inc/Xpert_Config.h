/*
 * 	Xpert_Config.h
 *
 *	EDGE S.A.
 *	www.edgeinst.com.ar
 *	wperesson@edgeinst.com.ar
 *  Author: Walter Peressón
 *  Mayo 2022
 *
 */

#ifndef Xpert_Config_H_
#define Xpert_Config_H_

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
//#define		CONTROLLER_TYPE		"Xpert"
//#define		CONTROLLER_TYPE		"Xpert-m"
#define		CONTROLLER_TYPE		"Xpert-lp"

/*===============================================
 *		DEVICE_TYPE
 * Distingue entre diferentes aplicaciones de dispositivos
 */
//#define		DEVICE_TYPE		"CHIP"		//Chemical Injection Pump
//#define		DEVICE_TYPE		"POC"		//Pump Off Controller
//#define		DEVICE_TYPE		"SSV"		//Surface Safety Valve
//#define		DEVICE_TYPE		"TANKL"		//Tank Level
#define			DEVICE_TYPE		"PRESS"		//Pressure sensor

/*===============================================
 *		POWER_TYPE
 * Selecciona la forma de alimentación del dispositivo
 */
//#define			POWER_TYPE		"Solar"		//Paneles solares
//#define		POWER_TYPE		"Battery"		//Batería interna
//#define		POWER_TYPE		"External"		//Alimentación provista por sistema externo
#define		POWER_TYPE		"Electric"		//Red eléctrica

/*===============================================
 *		MODULOS DE COMUNICACIÓN Y GPS
 */

//Posicionamiento por GPS / GLONASS
#define USE_GPS					1

//Red Celular
//#define USE_SIM868				0 //Módulo de comunicaciones GPRS/GSM
//#define USE_BG96				0 //Módulo de comunicaciones LTE-CatM1
//#define USE_SIM7070G			0 //Módulo de comunicaciones LTE-CatM1

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

#define LEN_SN				16
#define LEN_ID				20
#define LEN_MAC				17

/*----------------------------------------------------------
 * MODOS DE FUNCIONAMIENTO DEL EQUIPO
 */
typedef enum {
	alone_device, concentrator_device, slave_device, upgrade_firm
} DEVICE_MODE_T;

/*----------------------------------------------------------
 * IDENTIFICACIÓN DEL EQUIPO
 */
typedef struct {
	char serialNumber[LEN_SN]; 	//Serial Number del dispositivo
	char deviceTag[3][LEN_ID];	//Tags de identificación del activo
	char xbeeMac[LEN_MAC];	//MAC de XBee
	char xbeeESP[LEN_MAC];	//MAC de ESP32
} DEVICE_ID_T;

/*----------------------------------------------------------
 * TIPO DE DISPOSITIVO
 */
typedef enum {
	Xpert, Xpert_M, Xpert_LP, Xpert_VLP
} DEVICE_TYPE_T;

typedef struct {
	DEVICE_ID_T id;
	DEVICE_MODE_T mode;
	DEVICE_TYPE_T type;
	char owner[20];	//Cliente o dueño del equipo
} DEVICE_T;

#endif /* Xpert_Config_H_ */
