/*
 * ATV12H037.h
 *
 *  Created on: 26 mar. 2021
 *      Author: german
 */
#include "board.h"
//#include "Definiciones.h"
#if V20
#ifndef DRIVERS_INC_ATV12H037_H_
#define DRIVERS_INC_ATV12H037_H_
#define	WDOG_TIME		1	//40001| |Tiempo de vigilancia| |R/W| |ms| |1| |0 - 65535| |-| |-
#define	WDOG_ACTION		2	//40002| |Acción de vigilancia| |R/W| |-| |1| |-| |-| |-
#define	FREQ_REF		3	//40003| |Consigna de frecuencia| |R/W| |%| |100| |0,00 - 100,00| |HSW| |HSW
#define	RUN_ENABLE		4	//40004| |Habilitación de funcionamiento| |R/W| |-| |1| |0 - 1| |STW :3| |STW :3
#define	CMD_FWD_REV		5	//40005| |Comando adelante/atrás| |R/W| |-| |1| |0 - 1| |STW : 11| |STW : 11
#define	CMD_START		6	//40006| |Orden de arranque| |R/W| |-| |1| |0 - 1| |STW :0| |STW :0
#define	FAULT_ACK		7	//40007| |Confirmación de fallo| |R/W| |-| |1| |0 - 1| |STW :7| |STW :7
#define	PID_SETP_REF	8	//40008| |Referencia de consigna PID| |R/W| |%| |100| |-200,0 - 200,0| |P2240| |P2240
#define	ENABLE_PID		9	//40009| |Habilitación de PID| |R/W| |-| |1| |0 - 1| |r0055.8| |(BICO) P2200
#define	CURRENT_LMT		10	//40010| |Límite de corriente| |R/W| |%| |10| |10,0 - 400,0| |P0640| |P0640
#define	ACCEL_TIME		11	//40011| |Tiempo de aceleración| |R/W| |s| |100| |0,00 - 650,0| |P1120| |P1120
#define	DECEL_TIME		12	//40012| |Tiempo de deceleración| |R/W| |s| |100| |0,00 - 650,0| |P1121| |P1121
#define	DIGITAL_OUT_1	14	//40014| |Salida digital 1| |R/W| |-| |1| |HIGH LOW| |r0747.0| |(BICO) P0731
#define	DIGITAL_OUT_2	15	//40015| |Salida digital 2| |R/W| |-| |1| |HIGH LOW| |r0747.1| |(BICO) P0732
#define	REF_FREQ		16	//40016| |Frecuencia de referencia| |R/W| |Hz| |100| |1,00 - 550,00| |P2000| |P2000
#define	PID_UP_LMT		17	//40017| |Límite superior de PID| |R/W| |%| |100| |-200,0 - 200,0| |P2291| |P2291
#define	PID_LO_LMT		18	//40018| |Límite inferior de PID| |R/W| |%| |100| |-200,0 - 200,0| |P2292| |P2292
#define	P_GAIN			19	//40019| |Ganancia proporcional| |R/W| |-| |1000| |0,000 - 65,000| |P2280| |P2280
#define	I_GAIN			20	//40020| |Ganancia integral| |R/W| |s| |1| |0 - 60| |P2285| |P2285
#define	D_GAIN			21	//40021| |Ganancia diferencial| |R/W| |-| |1| |0 - 60| |P2274| |P2274
#define	FEEDBK_GAIN		22	//40022| |Ganancia de realimentación| |R/W| |%| |100| |0,00 - 500,00| |P2269| |P2269
#define	LOW_PASS		23	//40023| |Paso bajo| |R/W| |-| |100| |0,00 - 60,00| |P2265| |P2265
#define	FREQ_OUTPUT		24	//40024| |Salida de frecuencia| |R| |Hz| |100| |-327,68 - 327,67| |r0024| |r0024
#define	SPEED			24	//40025| |Velocidad| |R| |RPM| |1| |-16250 - 16250| |r0022| |r0022
#define	CURRENT			26	//40026| |Corriente filtrada| |R| |A| |100| |0 - 163,83| |r0027| |r0027
#define	TORQUE			27	//40027| |Par| |R| |Nm| |100| |-325,00 - 325,00| |r0031| |r0031
#define	ACTUAL_PWR		28	//40028| |Potencia real| |R| |kW| |100| |0 - 327,67| |r0032| |r0032
#define	TOTAL_KWH		29	//40029| |Total kWh| |R| |kWh| |1| |0 - 32767| |r0039| |r0039
#define	DC_BUS_VOLTS	30	//40030| |Tensión del embarrado DC| |R| |V| |1| |0 - 32767| |r0026| |r0026
#define	REFERENCE		31	//40031| |Referencia| |R| |Hz| |100| |-327,68 - 327,67| |r0020| |r0020
#define	RATED_PWR		32	//40032| |Potencia nominal| |R| |kW| |100| |0 - 327,67| |r0206| |r0206
#define	OUTPUT_VOLTS	33	//40033| |Salida de tensión| |R| |V| |1| |0 - 32767| |r0025| |r0025
#define	FWD_REV			34	//40034| |Adelante/atrás| |R| |-| |1| |FWD REV| |ZSW : 14| |ZSW : 14
#define	STOP_RUN		35	//40035| |Parada/marcha| |R| |-| |1| |STOP RUN| |ZSW :2| |ZSW :2
#define	AT_MAX_FREQ		36	//40036| |Funcionamiento a frecuencia máxima| |R| |-| |1| |MAX NO| |ZSW : 10| |ZSW : 10
#define	CONTROL_MODE	37	//40037| |Modo de regulación| |R| |-| |1| |SERIAL LOCA L| |ZSW :9| |ZSW :9
#define	ENABLED			38	//40038| |Habilitado| |R| |-| |1| |ON OFF| |ZSW :0| |ZSW :0
#define	READY_TO_RUN	39	//40039| |Listo para funcionar| |R| |-| |1| |READY OFF| |ZSW :1| |ZSW :1
#define	ANALOG_IN_1		40	//40040| |Entrada analógica 1| |R| |%| |100| |-300,0 - 300,0| |r0754[0]| |r0754[0]
#define	ANALOG_IN_2		41	//40041| |Entrada analógica 2| |R| |%| |100| |-300,0 - 300,0| |r0754[1]| |r0754[1]
#define	ANALOG_OUT_1	42	//40042| |Salida analógica 1| |R| |%| |100| |-100,0 - 100,0| |r0774[0]| |r0774[0]
#define	FREQ_ACTUAL		44	//40044| |Frecuencia real| |R| |%| |100| |-100,0 - 100,0| |HIW| |HIW
#define	PID_SETP_OUT	45	//40045| |Salida de consigna PID| |R| |%| |100| |-100,0 - 100,0| |r2250| |r2250
#define	PID_OUTPUT		46	//40046| |Salida PID| |R| |%| |100| |-100,0 - 100,0| |r2294| |r2294
#define	PID_FEEDBACK	47	//40047| |Realimentación PID| |R| |%| |100| |-100,0 - 100,0| |r2266| |r2266
#define	DIGITAL_IN_1	48	//40048| |Entrada digital 1| |R| |-| |1| |HIGH LOW| |r0722.0| |r0722.0
#define	DIGITAL_IN_2	49	//40049| |Entrada digital 2| |R| |-| |1| |HIGH LOW| |r0722.1| |r0722.1
#define	DIGITAL_IN_3	50	//40050| |Entrada digital 3| |R| |-| |1| |HIGH LOW| |r0722.2| |r0722.2
#define	DIGITAL_IN_4	51	//40051| |Entrada digital 4| |R| |-| |1| |HIGH LOW| |r0722.3| |r0722.3
#define	FAULT			54	//40054| |Fallo| |R| |-| |1| |FAULT OFF| |ZSW :7| |ZSW :7
#define	LAST_FAULT		55	//40055| |Último fallo| |R| |-| |1| |0 - 32767| |r0947[0]| |r0947[0]
#define	FAULT1			56	//40056| |Fallo 1| |R| |-| |1| |0 - 32767| |r0947[1]| |r0947[1]
#define	FAULT2			57	//40057| |Fallo 2| |R| |-| |1| |0 - 32767| |r0947[2]| |r0947[2]
#define	FAULT3			58	//40058| |Fallo 3| |R| |-| |1| |0 - 32767| |r0947[3]| |r0947[3]
#define	WARNING			59	//40059| |Aviso| |R| |-| |1| |WARN OK| |ZSW :| |ZSW :
#define	LAST_WARNING	60	//40060| |Último aviso| |R| |-| |1| |0 - 32767| |r2110| |r2110
#define	INVERTER_VER	61	//40061| |Versión de convertidor| |R| |-| |100| |0,00 - 327,67| |r0018| |r0018
#define	DRIVE_MODEL		62	//40062| |Modelo de convertidor| |R| |-| |1| |0 - 32767| |r0201| |r0201
#define	STW				99	//40100| |STW| |R/W| |-| |1| || |PZD 1| |PZD 1
#define	HSW				100	//40101| |HSW| |R/W| |-| |1| || |PZD 2| |PZD 2
#define	ZSW				110	//40110| |ZSW| |R| |-| |1| || |PZD 1| |PZD 1
#define	HIW				111	//40111| |HIW| |R| |-| |1| || |PZD 2| |PZD 2
#define	HAND_AUTO		349	//40349| |HAND/AUTO| |R| |-| |1| |HAND AUTO| |r0807| |r0807
#define	FAULT_1			400	//40400| |Fallo 1| |R| |-| |1| |0 - 32767| |r0947[0]| |r0947[0]
#define	FAULT_2			401	//40401| |Fallo 2| |R| |-| |1| |0 - 32767| |r0947[1]| |r0947[1]
#define	FAULT_3			402	//40402| |Fallo 3| |R| |-| |1| |0 - 32767| |r0947[2]| |r0947[2]
#define	FAULT_4			403	//40403| |Fallo 4| |R| |-| |1| |0 - 32767| |r0947[3]| |r0947[3]
#define	FAULT_5			404	//40404| |Fallo 5| |R| |-| |1| |0 - 32767| |r0947[4]| |r0947[4]
#define	FAULT_6			405	//40405| |Fallo 6| |R| |-| |1| |0 - 32767| |r0947[5]| |r0947[5]
#define	FAULT_7			406	//40406| |Fallo 7| |R| |-| |1| |0 - 32767| |r0947[6]| |r0947[6]
#define	FAULT_8			407	//40407| |Fallo 8| |R| |-| |1| |0 - 32767| |r0947[7]| |r0947[7]
#define	PRM_ERROR_CODE	499	//40499| |Código de error de parámetro| |R| |-| |1| |0 - 254| |-| |-
#define	PI_FEEDBACK		521	//40521| |Realimentación PID| |R| |%| |100| |-100,0 - 100,0| |r2266| |r2266




typedef enum {
	nOF=0,
	InF,
	CFF=3,
	CFI,
	SLF1,
	EPF1=8,
	OCF,
	CrF1,
	OHF=16,
	OLF,
	ObF,
	OSF,
	OPF1,
	PHF,
	USF,
	SCF1,
	SOF,
	tnF,
	InF1,
	InF2,
	InF3,
	InF4,
	SCF3=32,
	OPF2,
	SLF2=42,
	SLF3=45,
	InF9=51,
	InFb=53,
	tJF,
	SCF4,
	SCF5,
	InFE=69,
	CFI2=77,
	ULF=100,
	OLC,
	SPIF,
	LFF1=106,
	XXXX=253,
}TyATHfault;

typedef struct{
	uint16_t RPMset;//indica el setpoit RMP
	uint16_t RPMOut;//indica las RPM de salida del variador
	uint8_t SLID;// indica el esclavo
	uint16_t LastAddTX; //Guarda la ultima direccion enviada
	bool Rx_OK;//Indicador de que la utima encuesta fue exitosa
	uint8_t LastOper;
	TyATHfault FaultCode;
	uint8_t Status;

}Typ_VSP;
//extern Typ_VSP VSD;
//uint16_t calcMaxSpeed(SP_Pump_Type *sp);
#define FAULT_MASK 0x8
#endif
#endif /* DRIVERS_INC_SIM20_H_ */
