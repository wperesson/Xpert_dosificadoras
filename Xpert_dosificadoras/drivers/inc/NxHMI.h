/*
 * NxHMI.h
 *
 *  Created on: 12 oct. 2022
 *      Author: German Ghigi
 */

#ifndef DRIVERS_INC_NXHMI_H_
#define DRIVERS_INC_NXHMI_H_
#include "board.h"

typedef enum
{
	instInvalid=0x00,
	instSuccess=0x01,
	compInvalid=0x02,
	pageInvalid=0x03,
	pictInvalid=0x04,
	fontInvalid=0x05,
	crcInvalid=0x09,
	baudInvalid=0x11,
	wavChInvalid=0x12,
	varibNamInvalid=0x1A,
	varibOpInvalid=0x1B,
	asignaFail=0x1C,
	QuantParamInvalid=0x1E,
	iOOpFail=0x1F,
	EscCharInvalid=0x20,
	varNameToLong=0x23,
	serBuffOvrflow=0x24,
	touchEvent=0x65,
	currentPageNum=0x66,
	touchCordAwake=0x67,
	touchCordSleep=0x68,
	strDataEnc=0x70,
	numDataEnc=0x71,
	AutoSleep=0x86,
	AutoWake=0x87,
	nextionRdy=0x88,
	microSDUpgd=0x89,
	transDataFinish=0xFD,
	transDataRdy=0xFE,
}nxReturnType;

typedef enum
{
	updateVar,
	getPag,
	getModVar,
	NewValOK,
	getPgAndCp,
	getNewCoef,
	getACH,
	sendwSMS,
	getPumpOn,
	logOut,
	getPwd=0x1F,
	getSPNomSpd=0x206,
	getSPPumpRt=0x213,
	getTWidth=0x10F,
	getTLen=0x110,
	getTHgt=0x111,
	getTDens=0x112,
	getTCap=0x114,
	getSeHgt=0x113,
	getWrLvl=0x115,
	getAlLvl=0x116,
	getPwmTh=0x504,//todos los que se usan en botones hay que poner el id del boton y no de la casilla number que le corresponda al boton
	getCycle=0x503,
	getPlSiz=0x601,
	getPlStr=0x602,
	getHeads=0x603,
	getSeSH=0x680,
	getSeSL=0x681,
	getSeEH=0x682,
	getSeEL=0x683,
	volCalib=0x907,
	getaTy=0x130F,
	getaSrs=0x1310,
	getaHe=0x130C,
	getaLe=0x130D,
	getaHs=0x130E,
	getaLs=0x1312,
	getSLID=0xd07,
	getBaud=0xd04,
	getLink=0xd05,
	getNum1=0x100E,
	getNum2=0x100F,
	getNum3=0x1010,
	getNum4=0x1011,
	getGR=0x1409,
	getEPPR=0x140a,
	//getMaxSpd=0x140b,
	getSPrpm=0x140b,
	getPidP=0x150d,
	getPidI=0x150e,
	getPidD=0x150f,
	getClien=0x1608,
	getField=0x1609,
	getWell=0x160a,
	getSN=0x160b,
}EventHMI;
Status sendToNx(char *buff);
uint32_t wait_Byte_Nex(char *data, TickType_t delay);
extern uint16_t page_nx,pwUsrLvl;
extern uint8_t modifVal,aCH;
extern StreamBufferHandle_t strMsgFromHMI;
#endif /* DRIVERS_INC_NXHMI_H_ */
