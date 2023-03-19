/*
 * Mobile_types.h
 *
 *  Created on: 4 ene. 2022
 *      Author: Walter Peresson
 */

#ifndef MODULOS_MOBILE_TYPES_H_
#define MODULOS_MOBILE_TYPES_H_

#include "MobileConfig.h"
#include "Xpert_Config.h"
#include "board.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "string.h"
#include "time.h"

#if USE_K32L2B3
#include "stdint.h"
#include "stdbool.h"
#include "peripherals.h"

#elif USE_LPC1769
#include "lpc_types.h"
#endif

//Constantes usadas en el módulo mobile
#define OK_CHAR	"OK"

//Funciones externas usadas en el módulo
//#define DISABLE_UART_INTERRUPT UART_DisableInterrupts(base, mask);

//Definiciones globales
#define LEN_CLIENT_ID		25
#define LEN_ACCESS_KEY		37
#define LEN_ACCESS_SECRET	65
#define LEN_BROKER_URL		20
#define LEN_ENDPOINT_URL	100
#define LEN_ENDPOINT_PATH	28
#define LEN_APN				32
#define LEN_GPRS_USR_PASS	10
#define LEN_RPN				16

/* ON , OFF*/
typedef enum {
	OFF = 0, ON
} ON_OFF_ENUM_T;

/*----------------------------------------------------------
 * Dato a solicitar información
 */
typedef enum {
	IMEI = 0, 	//IMEI. International Mobile station Equipment Identity
	CIMI,		//(IMSI.Request International Mobile Subscriber Identity
	CSQ,		//Signal Quality Report
	CREG,		//Network Registration
	CGATT,		//Attach or Detach from GPRS Service
	COPS,		//Operator Selection
	CNACT,		//APP Network Active
	CFUN,
	IPR,		//Baud rate del comm
	CACID,
	CLTS,		//Sincronismo de hora local con la red
	CCLK,		//Tiempo dentro del módulo
	GMR, 		//Versión del firmware
	AT,			//Comando para saber si el módulo esta Powered ON
	CMGF,		//Comando para saber en si es ta en formato 0:PDU o 1:Text
} MOBILE_INFO_ENUM;

/*----------------------------------------------------------
 * Estado de registro en la red
 */
typedef enum {
	NOT_REGISTERED = 0, //Not registered, MT is not currently searching a new operator to register to
	REGISTERED_HN,	//Registered, home network
	NOT_REGISTERED_SEARCHING,//Not registered, but MT is currently searching a new operator to register to
	REGISTRATION_DENIED,	//Registration denied
	UNKNOWN,				//
	REGISTERED_ROAM,		//Registered, roaming
} NETWORK_REG_STATUS_ENUM;

/*----------------------------------------------------------
 * Funcionalidad del módulo
 */
typedef enum {
	FUNCT_MINIMUM = 0, 	//Minimum functionality
	FUNCT_FULL,	//Full functionality (Default)
	FUNCT_DIS_TX_RX = 4,//Disable phone both transmit and receive RF circuits.
	FUNCT_FACT_TEST,	//Factory Test Mode
	FUNCT_RESET,		//Reset
	FUNCT_OFFLINE,		//Offline Mode
} PHONE_FUNCTIONALITY_ENUM;

/*----------------------------------------------------------
 * Código de operación al conectarnos al broker MQTT
 */
typedef enum {
	CONNECTED = 0, 		//
	CME_NOT_ALLOWED,	//
	ERROR_FROM_BROKER,				//
	NO_IP,				//
} BROKER_CONN_REPLY_ENUM;

/*----------------------------------------------------------
 * UART. Data to transfer
 */
typedef struct Data {
	char *ptr;	//Puntero
	uint16_t Len;	//Contexto de la variable
} UART_TX_TRANSFER_T;

/*----------------------------------------------------------
 * APN USR y PASS para la red
 */
typedef struct {
	char gprs_apn[LEN_APN]; //String con el apn
	char gprs_usr[LEN_GPRS_USR_PASS]; //Usuario para loguearse al gprs
	char gprs_pass[LEN_GPRS_USR_PASS]; //Password para loquearse al gprs
	char gprs_rpn[4][LEN_RPN]; //Registered Phone Number 1: 2984558529H
} NETWORK_DATA_T;

/*----------------------------------------------------------
 * INTERCAMBIO DE DATOS CON MOBILE
 */
typedef struct {
	QueueHandle_t to_mobile;	//
	QueueHandle_t fr_mobile;	//
	char *bufferRX;
	char *bufferTX;
	uint16_t bufferRX_size;
	uint16_t bufferTX_size;	//todo: pasar todos los envios para que usen este buffer
	uint32_t timeout;
} MOBILE_DATA_T;

/*----------------------------------------------------------
 * ESTADO DEL MOBILE
 */
typedef struct {
	char imei[17]; 			//El imei tiene 15 cifras
	char oper[30]; 			//Operator
	char pn[15]; 			//Phone Number
	char SMS_orig[15]; 		//Nro del que envía el SMS
	char SMS_scenter[15];	//SMS service Center
	uint32_t RPN[4]; 		//Numeros de celulares en formato numérico
	uint8_t SMS_idx;		//Indice del mensaje
	uint16_t Send_SMS_to_idx; //Phonebook Index al cual enviar un mensaje desde el panel.
	uint8_t rssi; 			//Received Signal Strength Indication
	uint8_t ber; 			//Bit Error Rate


	//BG96
	/* Access technology
	 * 0: GSM
	 * 8: LTE Cat M1
	 * 9: LTE Cat NB1 */
	// SIM7070G
	/* Access technology
	 * 	0: User-specified GSM access technology
	 * 	1: GSM compact
	 * 	3: GSM EGPRS
	 * 	7: User-specified LTE M1 A GB access technology
	 * 	9: User-specified LTE NB S1 access technology */
	uint8_t accessTechnology;


	/* NETWORK REGISTRATION
	 * 0 Not registered, MT is not currently searching a new operator to register to
	 * 1 Registered, home network
	 * 2 Not registered, but MT is currently searching a new operator to register to
	 * 3 Registration denied
	 * 4 Unknown
	 * 5 Registered, roaming*/
	NETWORK_REG_STATUS_ENUM reg_on_netw;		//Registrado en la red??

	/*Activa el forwarding de los SMS
	 * 	1: fwd on
	 * 	0: fwd off
	 */
	uint8_t sms_fwd;
	char SMS_fwd_orig[15]; 		//Nro donde hacer fwd del SMS

	/*Indicates the state of GPRS attachment
	 * 0 Detached
	 * 1 Attached*/
	uint8_t isAttachedToGPRS;

	/*Indicates the state of Network
	 * 0 Deactive
	 * 1 Active*/
	uint8_t isNetworkActive;

//	uint8_t module_running;
} PHONE_DATA_T;

/*----------------------------------------------------------
 * CREDENCIALES Y ENDPOINT
 */
typedef struct {
	char ClientID[LEN_CLIENT_ID];
	char access_key[LEN_ACCESS_KEY];
	char access_secret[LEN_ACCESS_SECRET];
//	char slavesClientID[10][LEN_CLIENT_ID]; 		//Slave 1 will have index 0
	char endpoint_url[LEN_ENDPOINT_URL];
	char endpoint_path[LEN_ENDPOINT_PATH];
	char broker_url[LEN_BROKER_URL];
} CREDENTIAL_DATA_TYPE;

/*----------------------------------------------------------
 * MOBILE: Este tipo de datos contiene todos los tipos necesarios
 * para identificar la red, el device, mantener el estado del mismo y
 * almacenar las credenciales de registro en Losant
 */
typedef struct {
	MOBILE_DATA_T data;
	PHONE_DATA_T status;
	NETWORK_DATA_T network;
	CREDENTIAL_DATA_TYPE credential;
} MOBILE_T;

/*----------------------------------------------------------
 * GPS
 */
typedef struct {
	float Lat;
	float Lon;
	float Speed_kph;
	float Altitude;
	float course_over_gnd;
	uint8_t gps_run;
	uint8_t gps_fix;
	time_t UTC_time;
	struct tm GPS_time;
	int8_t TimeZone;
	uint16_t intentosToFix;
	uint16_t secondsToFix;
} GPS_DATA_T;

#endif /* MODULOS_MOBILE_TYPES_H_ */
