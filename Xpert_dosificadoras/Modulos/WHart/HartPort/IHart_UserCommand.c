#include "IHart_UserCommand.h"
#include "IHart_Uart.h"

USIGN8  gucResetFlag    = 0;
USIGN16 nResetDelayTime = 0;

/*-------------------------------------------------------------------------*/
/*************************************************
 �ӿڹ��� : �豸�Լ�,���û�ʵ��
 ����     ����
 ����ֵ   ����
 ����     �� 
****************************************************/
void IHartSelfTest(void)
{
    USIGN8 SelfState;

    SelfState=0; //�Լ�״̬
    
    if(SelfState)//�Լ�-����
    {
        SetFieldDeviceStatus(HART_DEV_MALF);//��λ�豸״̬-���ϱ�־λ
        gsHartSlaveInfo.gucDeviceStatus[1] |= DEVICE_MALFUNCTION_ACTIVE;//����������Ϣ
    }
    else         //�Լ�-�޹���
    {
        ClearFieldDeviceStatus(HART_DEV_MALF);//����״̬
        gsHartSlaveInfo.gucDeviceStatus[1] &= ~DEVICE_MALFUNCTION_ACTIVE;//���������Ϣ
    }
}

/*******************************************************************
�ӿڹ���  : �豸��λ�����û�ʵ��
�������: ��
�������: ��
����ֵ: ��
********************************************************************/
void IHartDeviceReset(void)
{
    gucResetFlag = 1;
    gsHartSlaveInfo.ucNoticeResetFlag = 1;//�յ���λ����#42���� ֪ͨģ�鸴λ��־λ
}

/*************************************************
    Command35: Write Primary Variable Range Values 
****************************************************/
USIGN16 RespondCommand_35(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth)
{
    USIGN8  ucUnit;
    float   fNewUp;
    float   fNewLow;
    float   fSensorUpLimit;
    float   fSensorLowLimit;
    
    USIGN8  ucDataLen = 9;//���ݳ���

    if (nReqLength < ucDataLen)
    {
        return TOO_FEW_DATA_BYTES_RECEIVED;
    }
    else if ( gsHartSlaveInfo.ucWriteProtect )
    {
        return IN_WRITE_PROTECT_MODE;
    }
    else 
    {
        memcpy(pRspData, pReqData, 9);
        *nRspLenth = 9;
        
        ucUnit = pReqData[0];
        SetData(&pReqData[1], 4, (USIGN8*)&fNewUp, 0);
        SetData(&pReqData[5], 4, (USIGN8*)&fNewLow, 0);
		
        //Ŀǰ��λֻ֧��1-14. �����ķ��ش���
		if (NewUnitRight(gucDefaultClassification[PVVar], pReqData[0]))
		{
			return INVALID_SELECTION;
		}
        
        fNewUp  = DevUnitChange(gucDefaultClassification[PVVar], ucUnit, fNewUp, gucDefaultUnit[PVVar]);
        fNewLow = DevUnitChange(gucDefaultClassification[PVVar], ucUnit, fNewLow, gucDefaultUnit[PVVar]);
        
        fSensorUpLimit  = gsDeviceVars[PVVar].unUpperLimit.f;
        fSensorLowLimit = gsDeviceVars[PVVar].unLowerLimit.f;

        if ( (fNewUp>fSensorUpLimit && fNewLow<fSensorLowLimit) ||
             (fNewUp<fSensorLowLimit && fNewLow>fSensorUpLimit)   )
        {
            return UPPER_AND_LOWER_RANGEVALUES_OUT_OF_LIMITS;
        }
        if (fNewLow >  fSensorUpLimit)
        {
            return LOWER_RANGE_VALUE_TOO_HIGH;
        }
        if (fNewUp > fSensorUpLimit)
        {
            return UPPER_RANGE_VALUE_TOO_HIGH;
        }
        if (fNewUp < fSensorLowLimit)
        {
            return UPPER_RANGE_VALUE_TOO_LOW;
        }
        if (fNewLow < fSensorLowLimit)
        {
            return LOWER_RANGE_VALUE_TOO_LOW;
        }
        if (fNewUp == fNewLow)
        {
            return INVALID_SPAN;
        }
      

		gsDeviceVars[PVVar].unUpper.f = fNewUp;
		gsDeviceVars[PVVar].unLower.f = fNewLow;
		gsDeviceVars[0].ucPvRangeUnit = ucUnit;  
        
        SetFieldDeviceStatus(HART_CFG_CHANGE);
        /*���������̵�λ*/
        IHartWriteEeprom((USIGN8*)&gsDeviceVars[0].ucPvRangeUnit, EE_PV_RANGE_UNIT, 1);  

        IHartWriteEeprom((USIGN8*)&gsDeviceVars[PVVar].unUpper.f, EE_PV_UPPER_RANGE, 8);
    }
    return NO_COMMAND_SPECIFIC_ERRORS;
}


/*************************************************
    ����130:��λ/���� д���� �Զ�������
****************************************************/
USIGN16 RespondCommand_130(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth)
{
    
    if(nReqLength < 1)
    {
      return TOO_FEW_DATA_BYTES_RECEIVED;
    }
    
    if(*pReqData == 1)
        gsHartSlaveInfo.ucWriteProtect=1;   
    else
        gsHartSlaveInfo.ucWriteProtect=0;
    
    *pRspData  = gsHartSlaveInfo.ucWriteProtect;
    *nRspLenth = 1;
    
    return NO_COMMAND_SPECIFIC_ERRORS;
}

/*************************************************
    ����170��Write self_stack information
****************************************************/
USIGN16 RespondCommand_170(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth)
{   
//    char ucSize,ucIndex;
	USIGN8 ucSize,ucIndex;
    USIGN8 ucDataLen = 30;//���ݳ���
	USIGN16 uResponse = NO_COMMAND_SPECIFIC_ERRORS;
    int i=0;
    int j;
    
    if (nReqLength < ucDataLen)
    {
        uResponse = TOO_FEW_DATA_BYTES_RECEIVED;
    }
    else if ( gsHartSlaveInfo.ucWriteProtect )
    {
        uResponse = IN_WRITE_PROTECT_MODE;
    }
    else
    {
        for(j=0;j<4;j++)
        {
            ucSize=4;
            ucIndex=0;
            while(ucSize > 0)
            {
                ucSize--;
                ((char *)(&gsDeviceVars[j].unValue.f))[ucIndex++] = ((char *)pReqData+i)[ucSize];
            }       
            //memcpy(&gsDeviceVars[j].unValue.f, pucData+i, 4);
            i+=4;
            gsDeviceVars[j].ucForcedUnit = pReqData[i++];
            gsDeviceVars[j].ucStatus     = pReqData[i++];
        }
        
        ucSize=4;
        ucIndex=0;
        while(ucSize > 0)
        {
            ucSize--;
            ((char *)(&gsHartSlaveInfo.gsHartPowerInfo.fBatV))[ucIndex++] = ((char *)pReqData+i)[ucSize];
        } 
        i+=4;
        gsHartSlaveInfo.gsHartPowerInfo.ucForcedUnit = pReqData[i++];
        switch(pReqData[i++])
        {
            case 0:
              gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Nominal;
              break; 
            case 1:
              gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Low;
              break;              
            case 2:
              gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Critically_Low;
              break;              
            case 3:
              gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Recharging_Low;
              break;                           
            default:
              gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Recharging_High;
              break;
        }
        
        memcpy(pRspData, pReqData, i);
        *nRspLenth = i;  
    }
    return uResponse;
}


/*************************************************
    ����171��Read Self_stack information (power information)
****************************************************/
USIGN16 RespondCommand_171(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth)
{   
	USIGN16 uResponse  = NO_COMMAND_SPECIFIC_ERRORS;

    SetData((void *)&gsHartSlaveInfo.fPV, 4, pRspData, 0);
    *(pRspData+4) = gsDeviceVars[0].ucForcedUnit;
    *(pRspData+5) = gsDeviceVars[0].ucStatus;

    SetData((void *)&gsHartSlaveInfo.fSV, 4, pRspData, 6);
    *(pRspData+10) = gsDeviceVars[1].ucForcedUnit;
    *(pRspData+11) = gsDeviceVars[1].ucStatus;

    SetData((void *)&gsHartSlaveInfo.fTV, 4, pRspData, 12);
    *(pRspData+16) = gsDeviceVars[2].ucForcedUnit;
    *(pRspData+17) = gsDeviceVars[2].ucStatus;

    SetData((void *)&gsHartSlaveInfo.fQV, 4, pRspData, 18);
    *(pRspData+22) = gsDeviceVars[3].ucForcedUnit;
    *(pRspData+23) = gsDeviceVars[3].ucStatus;
    
    SetData((void *)&gsHartSlaveInfo.gsHartPowerInfo.fBatV, 4, pRspData, 24);
    *(pRspData+28) = gsHartSlaveInfo.gsHartPowerInfo.ucForcedUnit;
    *(pRspData+29) = gsHartSlaveInfo.gsHartPowerInfo.PowerStatus;                               

    SetData((void *)&gsHartSlaveInfo.ulTime, 4, pRspData, 30);

    *nRspLenth = 34; 
    
    return uResponse;
}

/*************************************************
    ����512��Read Country Code
****************************************************/
USIGN16 RespondCommand_512(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth)
{   
	USIGN16 uResponse  = NO_COMMAND_SPECIFIC_ERRORS;

    *(pRspData)   = 0x00;
    *(pRspData+1) = 0x9C;
    *(pRspData+2) = 0x00;
    *nRspLenth    = 3; 

    return uResponse;
}
