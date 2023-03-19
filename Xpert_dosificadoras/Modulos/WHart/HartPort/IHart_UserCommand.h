#ifndef _IHART_USERCOMMAND_H
#define _IHART_USERCOMMAND_H

#include "IHart_glb_defs.h"
#include "IHart_DataAcquisition.h"
#include "IHart_DataSave.h"

/*Device-Specific Status :gucDeviceStatus[1]-HART_DEV_MALF*/
#define DEVICE_MALFUNCTION_ACTIVE               0x01

extern USIGN8  gucResetFlag;
extern USIGN16 nResetDelayTime;

extern void SetFieldDeviceStatus(USIGN8 ucStatus); 
extern void ClearFieldDeviceStatus(USIGN8 ucStatus); 

/*************************************************
 接口功能 :设备自检
****************************************************/
void IHartSelfTest(void);

/*******************************************************************
接口功能  : 设备复位
********************************************************************/
void IHartDeviceReset(void);


USIGN16 RespondCommand_130(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);
USIGN16 RespondCommand_35(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);

USIGN16 RespondCommand_170(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);
USIGN16 RespondCommand_171(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);
USIGN16 RespondCommand_512(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);
#endif