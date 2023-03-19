/*
 * MobileConfig.h
 *
 *  Created on: 4 ene. 2022
 *      Author: Walter Peresson
 */

#ifndef MODULOS_MOBILECONFIG_H_
#define MODULOS_MOBILECONFIG_H_

//#include "FreeRTOS.h"
//#include "queue.h"

//#include "string.h"

//Define el módulo celular
#define USE_SIM868				0 //Módulo de comunicaciones GPRS/GSM
#define USE_BG96				0 //Módulo de comunicaciones LTE-CatM1
#define USE_SIM7070G			1 //Módulo de comunicaciones LTE-CatM1/Nb-IoT/GPRS

//Define el SIM o la proveedora que vamos a usar
#define USE_KITE_MOVISTAR				1
#define USE_KITE_MOVISTAR_GLOBAL		0
#define USE_OLIVIA						0
/*================================================================
 * 		LOSANT
 * ===============================================================
 */

#define BROKER 	"broker.losant.com"
#define PORT_NUMB 	"1883"

#if SET_COMPANY_EDGE
	#define ENDPOINT_URL	"https://edge.onlosant.com"
#elif SET_COMPANY_VISTA
#define ENDPOINT_URL	"https://vista.onlosant.com"
#elif SET_COMPANY_YPF
#define ENDPOINT_URL	"https://ypfsa.onlosant.com"
#elif SET_COMPANY_ALTOS
	#define ENDPOINT_URL	"https://altos.onlosant.com"
#endif

#if SET_COMPANY_EDGE
	#define TEST_ENDPOINT_URL	"https://test-edge.onlosant.com"
#elif SET_COMPANY_VISTA
#define TEST_ENDPOINT_URL	"https://test-vista.onlosant.com"
#elif SET_COMPANY_YPF
#define TEST_ENDPOINT_URL	"https://test-ypfsa.onlosant.com"
#elif SET_COMPANY_ALTOS
	#define TEST_ENDPOINT_URL	"https://test-altos.onlosant.com"
#endif

#if USE_OLIVIA
	#define GPRS_APN "simbase"
	#define GPRS_USER ""
	#define GPRS_PASS ""
#elif USE_KITE_MOVISTAR
	#define GPRS_APN  "gm2m.movistar"
	#define GPRS_USER "gm2m"
	#define GPRS_PASS "gm2m"
#elif USE_KITE_MOVISTAR_GLOBAL
	#define GPRS_APN  "simglobalar.movistar"
	#define GPRS_USER "simglobal"
	#define GPRS_PASS "simglobal"
#endif

#endif /* MODULOS_MOBILECONFIG_H_ */
