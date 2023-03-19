#include "IHart_DataAcquisition.h"
#include "IHart_DataSave.h"
#include "IHart_glb_defs.h"
#include <math.h>

DeviceVariable gsDeviceVars[VARIABLE_COUNT];     //Device variable information
SlaveInfo gsHartSlaveInfo;				    //Slave Device information
ParityInfo gsParityInfo;                     //Parity information
T_STATUS_DATA gsStatusData;                     //Synchronization information

#ifdef FUN_MONITOR_RTC
USIGN16 MyNewRtc;
#endif

/********************************************************************
 * Name: IHartUpdateDynamic
 * Description: Update dynamic variables              
 * Parameters: -
 * Return: -
 *******************************************************************/
void IHartUpdateDynamic(void) {
	//��̬��������
	if (IUserDynamicsCount >= 1) {
		gsHartSlaveInfo.fPV = GetDevValue(PVVar);
	}
	if(IUserDynamicsCount >= 2)
	{
		gsHartSlaveInfo.fSV = GetDevValue(SVVar);
	}
	if(IUserDynamicsCount >= 3)
	{
		gsHartSlaveInfo.fTV = GetDevValue(TVVar);
	}
	if(IUserDynamicsCount >= 4)
	{
		gsHartSlaveInfo.fQV = GetDevValue(QVVar);
	}

	//����PV�����İٷֱ�
	gsHartSlaveInfo.fPercent = GetPerRange(PVVar);
}

/********************************************************************
 * Name: InitHartSlaveInfo
 * Description: Initialize gsHartSlaveInfo              
 * Parameters: -
 * Return: -
 *******************************************************************/
void InitHartSlaveInfo(void) {
	USIGN8 i = 0;

	/*1.Init gsHartSlaveInfo*/
	gsHartSlaveInfo.ucPollingAddr = DEVICE_POLLINGADDR;

	WriteBuffer((USIGN8*) gsHartSlaveInfo.aucHartMessage, IUserMessage, 24);

	for (i = 0; i < 3; i++) {
		gsHartSlaveInfo.aucAssemblyNumber[i] = IUserAssemblyNumber[i];
	}
	WriteBuffer((USIGN8*) &gsHartSlaveInfo.sHartTagDesDate.aucHpsTag,
			IUserHpsTag, 6);

	WriteBuffer((USIGN8*) &gsHartSlaveInfo.sHartTagDesDate.aucHpsDes,
			IUserHpsDes, 12);
	for (i = 0; i < 3; i++) {
		*(&gsHartSlaveInfo.sHartTagDesDate.ucHpsDay + i) = IUserDate[i];
	}
	for (i = 0; i < 32; i++) {
		gsHartSlaveInfo.aucLongTag[i] = 0;
	}
//	memcpy(gsHartSlaveInfo.aucLongTag, id_PumpSN, LEN_SN);

	gsHartSlaveInfo.ucSenComm38 = 0;
	gsHartSlaveInfo.ucLock_Divice = 0;

	gsHartSlaveInfo.ucNumOfRequestPreambles = PREAMBLE_NUM;
	gsHartSlaveInfo.ucDevFlag = 0x08;
	gsHartSlaveInfo.ucNeedMoreStatus = 0;
	gsHartSlaveInfo.ucPrimaryMaster = 1;
	gsHartSlaveInfo.ucMasterChange = 0;
	gsHartSlaveInfo.ucMasterType = 0;
	gsHartSlaveInfo.ucFirst = 0;
	gsHartSlaveInfo.ucFirstPrimaryMaster = 0;

	gsHartSlaveInfo.ucLoopMode = 0;
	gsHartSlaveInfo.fCurrent = 4.0;  //PV loop current

	/* Init Battery information */
	//fBatV - Module operating voltage.  when fBatV < 3V, power is becoming critically low
	gsHartSlaveInfo.gsHartPowerInfo.PowerSource = BATTER_POWER;
	gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Recharging_High;
	gsHartSlaveInfo.gsHartPowerInfo.DurationTime = 0;
	gsHartSlaveInfo.gsHartPowerInfo.RecoverTime = 0;
	gsHartSlaveInfo.gsHartPowerInfo.BatteryLife = 0;
	gsHartSlaveInfo.gsHartPowerInfo.fBatV = 3.3;
	gsHartSlaveInfo.gsHartPowerInfo.ucForcedUnit = Volts;

	gsHartSlaveInfo.fPV = 0.0;
	gsHartSlaveInfo.fSV = 0.0;
	gsHartSlaveInfo.fTV = 0.0;
	gsHartSlaveInfo.fQV = 0.0;
	gsHartSlaveInfo.ulTime = 0;

	for (i = 0; i < 25; i++) {
		gsHartSlaveInfo.gucDeviceStatus[i] = 0x00;
	}
	gsHartSlaveInfo.ucWriteProtect = 0;
	gsHartSlaveInfo.ucDeviceProfile = 0x81;
	gsHartSlaveInfo.ucAnalogChannelFlags = 0x00;
	gsHartSlaveInfo.gucWirelessOperationMode = 0x00;

	/* Init Diagnostic information */
	gsHartSlaveInfo.gsDiagnosticInfo.DiagnosticID = 5; //(default)
	gsHartSlaveInfo.gsDiagnosticInfo.AvgRSL = 0;

	gsHartSlaveInfo.gsUpdateDynamicFlag = 0;
	gsHartSlaveInfo.ucNoticeResetFlag = 0;  //֪ͨģ�鸴λ��־λ
	gsHartSlaveInfo.gsAD_Execute_Flag = 0;

	/*2.Init Synchronization information */
	gsStatusData.ucLocalDeviceStatusCommon = 0;
	gsStatusData.ucLocalExtendedStatus = 0;
	gsStatusData.ucLocalStandardizedStatus3 = 0;
	gsStatusData.ucLocalDeviceStatus = 0;
	gsStatusData.ucLocalConfigChangedCounter = 0;

	gsStatusData.ucRemoteDeviceStatusCommon = 0;
	gsStatusData.ucRemoteExtendedStatus = 0;
	gsStatusData.ucRemoteStandardizedStatus3 = 0;
	gsStatusData.ucRemoteConfigChangedCounter = 0;
	gsStatusData.ColdStart_CC_Flag.Data = 0;
	gsStatusData.ucRemoteStandardizedStatus1 = 0;
	gsStatusData.ucRemoteStandardizedStatus2 = 0;
}

/********************************************************************
 * Name: InitDeviceVars
 * Description: Initialize gsDeviceVars              
 * Parameters: -
 * Return: -
 *******************************************************************/
void InitDeviceVars(void) {
	USIGN8 i;

	for (i = 0; i < IUserVariableCount; i++) {
		gsDeviceVars[i].unValue.f = 0.0;
		gsDeviceVars[i].ucProperty = 0;
		gsDeviceVars[i].ucForcedUnit = gucDefaultUnit[i];
		gsDeviceVars[i].ucPvRangeUnit = 0;
		gsDeviceVars[i].ucFunct = FUCTION_LINEAR;
		gsDeviceVars[i].ucAlarm = ALARM_NONE;
		gsDeviceVars[i].aucSensorSN[0] = 0xFF;
		gsDeviceVars[i].aucSensorSN[1] = 0xFF;
		gsDeviceVars[i].aucSensorSN[2] = 0xFF;
		gsDeviceVars[i].fDamp = 0;
		gsDeviceVars[i].unUpperLimit.f = IUserDefaultUpperLimit[i];
		gsDeviceVars[i].unLowerLimit.f = IUserDefaultLowerLimit[i];
		gsDeviceVars[i].unUpper.f = gsDeviceVars[i].unUpperLimit.f;
		gsDeviceVars[i].unLower.f = gsDeviceVars[i].unLowerLimit.f;
		gsDeviceVars[i].fZero = 0;
		gsDeviceVars[i].fMinSpan = (gsDeviceVars[i].unUpperLimit.f
				- gsDeviceVars[i].unLowerLimit.f) / 100;
		gsDeviceVars[i].ucClassification = gucDefaultClassification[i];
		gsDeviceVars[i].ucFamily = IUserDefaultFamily[i];
		gsDeviceVars[i].ucStatus = 0xC0;
		gsDeviceVars[i].aucDynamicVar = i;
	}
	gsDeviceVars[0].ucPvRangeUnit = gsDeviceVars[PVVar].ucForcedUnit;
}

/********************************************************************
 * Name: HartVarRestoreRam
 * Description: Recovery gsHartSlaveInfo stored in NVM           
 * Parameters: -
 * Return: -
 *******************************************************************/
void HartVarRestoreRam(void) {
	//�ָ���һҳ��Ϣ
	IHartReadEeprom((USIGN8*) &gsHartSlaveInfo.ucPollingAddr, EE_POLLING_ADDR,
			83);

	IHartReadEeprom((USIGN8*) &gsStatusData.ucLocalDeviceStatus, EE_CFG_CHANGE,
			3);
	gsStatusData.ucLocalDeviceStatus = (gsStatusData.ucLocalDeviceStatus
			& (HART_CFG_CHANGE | HART_COLD_START));

	IHartReadEeprom((USIGN8*) &gsHartSlaveInfo.ucNumOfRequestPreambles,
	EE_PREAMBLES_NUM, 1);

	IHartReadEeprom((USIGN8*) &gsStatusData.ucRemoteConfigChangedCounter,
	EE_RMOTE_CON_CHA_COUNTER, 2);
}

/********************************************************************
 * Name: DeviceVarRestoreRam
 * Description: Recovery gsDeviceVars stored in NVM           
 * Parameters: -
 * Return: -
 *******************************************************************/
void DeviceVarRestoreRam(void) {
	USIGN8 i;

	//�ָ�PV��������
	IHartReadEeprom((USIGN8*) &gsDeviceVars[PVVar].ucProperty, EE_PV_PROPERTY, DEVICE_VARIABLE_LENGTH);

	for (i = 0; i < IUserVariableCount; i++) {
		IHartReadEeprom((USIGN8*) &gsDeviceVars[i].aucDynamicVar,
				(EE_DYNAMIC_MAP + i * EE_DYNAMIC_VAR_LENGTH),
				EE_DYNAMIC_VAR_LENGTH);
	}
}

/********************************************************************
 * Name: HartVarBurnEE
 * Description: Save gsHartSlaveInfo to NVM           
 * Parameters: -
 * Return: -
 *******************************************************************/
void HartVarBurnEE(void) {
	//�����һҳ��Ϣ
	IHartWriteEeprom((USIGN8*) &gsHartSlaveInfo.ucPollingAddr, EE_POLLING_ADDR,
			83);

	IHartWriteEeprom((USIGN8*) &gsStatusData.ucLocalDeviceStatus, EE_CFG_CHANGE,
			3);

	IHartWriteEeprom((USIGN8*) &gsHartSlaveInfo.ucNumOfRequestPreambles,
	EE_PREAMBLES_NUM, 1);
}

/********************************************************************
 * Name: HartVarBurnEE
 * Description: Save gsDeviceVars to NVM           
 * Parameters: -
 * Return: -
 *******************************************************************/
void DeviceVarBurnEE(void) {
	USIGN8 i;

	//����PV��������
	IHartWriteEeprom((USIGN8*) &gsDeviceVars[PVVar].ucProperty, EE_PV_PROPERTY, DEVICE_VARIABLE_LENGTH);

	for (i = 0; i < IUserVariableCount; i++) {
		IHartWriteEeprom((USIGN8*) &gsDeviceVars[i].aucDynamicVar,
				(EE_DYNAMIC_MAP + i * EE_DYNAMIC_VAR_LENGTH),
				EE_DYNAMIC_VAR_LENGTH);
	}
}

/******************************************************************************
 * Name: IHartDeviceInit
 * Description: Initialize the Device Variables ( gsDeviceVars,gsHartSlaveInfo )
 Recover data from NVM if IUserFactorySet = 1;
 * Parameters: -
 * Return: -
 *****************************************************************************/
void IHartDeviceInit(void) {
	USIGN8 statueEE;

	/*Init Uart parity info*/
	memset((USIGN8*) &gsParityInfo, 0, sizeof(ParityInfo));

	// Init gsDeviceVars
	InitDeviceVars();

	// Init gsHartSlaveInfo and gsStatusData
	InitHartSlaveInfo();

	IHartReadEeprom(&statueEE, EE_DATA_SIGN, 1);
	//Recover data from EEPROM
	if ((statueEE == EE_HAS_DATA) && (IUserFactorySet == 0)) {
		DeviceVarRestoreRam();
		HartVarRestoreRam();
	} else //Save the default data to EEPROM
	{
		DeviceVarBurnEE();
		HartVarBurnEE();

		statueEE = EE_HAS_DATA;
		IHartWriteEeprom((USIGN8*) &statueEE, EE_DATA_SIGN, 1);
	}
}

/*==============================================================*/
/******************************************************************************
 * Name: IsBatteryAlarm
 * Description: Battery level warning.              
 * Parameters: -
 * Return: -
 *****************************************************************************/
void IsBatteryAlarm(void) {
	//12.08ϵͳ������  ��Դ����
	/*Extended Device Status��Critical Power Failure
	 when fBatV < BATTERY_ALARM_VALUE, Battery alarm.*/
//    if(gsHartSlaveInfo.gsHartPowerInfo.fBatV < BATTERY_ALARM_VALUE)
//    {
//        gsStatusData.ucLocalExtendedStatus |= (0x04); 
//        gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Low;
//    }
//    else
//    {
	gsStatusData.ucLocalExtendedStatus &= (~0x04);
	gsHartSlaveInfo.gsHartPowerInfo.PowerStatus = Power_Status_Recharging_High;
//    }
}

/******************************************************************************
 * Name: IsDevVarsOutofLimit
 * Description: Device variables overflow alarm.              
 * Parameters: -
 * Return: -
 *****************************************************************************/
void IsDevVarsOutofLimit(void) {
	USIGN8 i;
	float fUpRange;
	float fLowRange;
	float fValue;

	for (i = 0; i < IUserVariableCount; i++) {
		fValue = DevUnitChange(gucDefaultClassification[i],
				gsDeviceVars[i].ucForcedUnit, gsDeviceVars[i].unValue.f,
				gucDefaultUnit[i]);

		fUpRange = gsDeviceVars[i].unUpper.f;
		fLowRange = gsDeviceVars[i].unLower.f;

		if ((fValue < fLowRange && fValue < fUpRange)
				|| (fValue > fLowRange && fValue > fUpRange)) {
			//Extended Device Status��Device Variable Alert
			gsStatusData.ucLocalExtendedStatus |= (0x02);
			//This bit is set if any Device Variable is in an Alarm of Waring State.
			break;
		}
	}
	gsStatusData.ucLocalExtendedStatus &= (~0x02);
}

/******************************************************************************
 * Name: NewUnitRight
 * Description: Analyzing the new units is reasonable.              
 * Parameters: -
 * Return: Reasonable 0     Unreasonable  1
 *****************************************************************************/
USIGN8 NewUnitRight(USIGN8 ucClassification, USIGN8 ucUnit) {
	USIGN8 ucStatus = 1;

	switch (ucClassification) {
#ifdef CLASSIFICATION_64
	case CLASSIFICATION_TEMPERATURE:
		if ((ucUnit >= 32 && ucUnit <= 37) || (ucUnit == 163)) {
			ucStatus = 0;
		} else {
			ucStatus = 1;
		}
		break;
#endif
#ifdef CLASSIFICATION_70
	case CLASSIFICATION_TIME:
		if (ucUnit >= 50 && ucUnit <= 53) {
			ucStatus = 0;
		} else {
			ucStatus = 1;
		}
		break;
#endif
#ifdef CLASSIFICATION_81
	case CLASSIFICATION_ANALYTICAL:
		if ((ucUnit == 57) || (ucUnit == 59) || (ucUnit == 150)
				|| (ucUnit == 160) || (ucUnit == 161)) {
			ucStatus = 0;
		} else {
			ucStatus = 1;
		}
		break;
#endif
#ifdef CLASSIFICATION_83
	case CLASSIFICATION_EMF:
		if ((ucUnit == 36) || (ucUnit == 58)) {
			ucStatus = 0;
		} else {
			ucStatus = 1;
		}
		break;
#endif 
#ifdef CLASSIFICATION_84
	case CLASSIFICATION_CURRENT:
		if (ucUnit == 39) {
			ucStatus = 0;
		} else {
			ucStatus = 1;
		}
		break;
#endif 
	}
	return ucStatus;
}

/******************************************************************************
 * Name: DevUnitChange
 * Description: Conversions acquisition value, according to the unit.              
 * Parameters:  USIGN8 ucSrcUnit,  Source Unit
 float  fValue,     Source acquisition value
 USIGN8 unDestUnit, Destination Unit
 * Return: the converted floating value
 *****************************************************************************/
FLOAT DevUnitChange(USIGN8 ucClassification, USIGN8 ucSrcUnit, FLOAT fValue,
		USIGN8 ucDestUnit) {

	if (ucSrcUnit == ucDestUnit) {
		return fValue;
	}

	switch (ucClassification) {
#ifdef CLASSIFICATION_64
	case CLASSIFICATION_TEMPERATURE:
		if ((ucSrcUnit >= 32 && ucSrcUnit <= 37) || (ucSrcUnit == 163)) {
			switch (ucSrcUnit) {
			case DC:
//				fValue = fValue;
				break;
			case DF:
				fValue = (fValue - 32) * 5 / 9;
				break;
			case DR:
				fValue = (fValue - 32 - 495.69) * 5 / 9;
				break;
			case DK:
				fValue = fValue - 273.15;
				break;
			default:
				break;
			}

			switch (ucDestUnit) {
			case DC:
//				fValue = fValue;
				break;
			case DF:
				fValue = fValue * 9 / 5 + 32;
				break;
			case DR:
				fValue = fValue * 9 / 5 + 32 + 495.69;
				break;
			case DK:
				fValue = fValue + 273.15;
				break;
			default:
				break;
			}
		}
		break;
#endif 

#ifdef CLASSIFICATION_70
	case CLASSIFICATION_TIME:
		if (ucSrcUnit >= 50 && ucSrcUnit <= 53) {
			//ȫ��ת���� Seconds
			switch (ucSrcUnit) {
			case Minutes:
				fValue = fValue * 60.0;
				break;
			case Seconds:
//				fValue = fValue;
				break;
			case Hours:
				fValue = fValue * 3600.0;
				break;
			case Days:
				fValue = fValue * 86400.0;
				break;
			default:
				break;
			}

			//����Secondsת����Ŀ�ĵ�λ
			switch (ucDestUnit) {
			case Minutes:
				fValue = fValue / 60.0;
				break;
			case Seconds:
//				fValue = fValue;
				break;
			case Hours:
				fValue = fValue / 3600.0;
				break;
			case Days:
				fValue = fValue / 86400.0;
				break;
			default:
				break;
			}
		}
		break;
#endif 

#ifdef CLASSIFICATION_81
	case CLASSIFICATION_ANALYTICAL:
		if ((ucSrcUnit == 57) || (ucSrcUnit == 59) || (ucSrcUnit == 150)
				|| (ucSrcUnit == 160) || (ucSrcUnit == 161)) {
//			fValue = fValue;
		}
		break;
#endif 

#ifdef CLASSIFICATION_83
	case CLASSIFICATION_EMF:
		if ((ucSrcUnit == 36) || (ucSrcUnit == 58)) {
			switch (ucSrcUnit) {
			case Volts:
//				fValue = fValue;
				break;
			case Millivolts:
				fValue = fValue * 0.001;
				break;
			default:
//				fValue = fValue;
				break;
			}

			switch (ucDestUnit) {
			case Volts:
//				fValue = fValue;
				break;
			case Millivolts:
				fValue = fValue * 1000.0;
				break;
			default:
//				fValue = fValue;
				break;
			}
		}
		break;
#endif 
#ifdef CLASSIFICATION_84
	case CLASSIFICATION_CURRENT:
		if (ucSrcUnit == 39) {
//			fValue = fValue;
		}
		break;
#endif 

	default:
//		fValue = fValue;
		break;
	}

	return fValue;
}

/*************************************************
 Ŀ�ģ�  ���豸��ǰ�ٷֱ�
 ������  ��
 ����ֵ����
 ������  ��
 ****************************************************/
float GetPerRange(USIGN8 ucIndex) {
	float fPercOfRange;
	float fPvValue = DevUnitChange(gucDefaultClassification[ucIndex],
			gsDeviceVars[ucIndex].ucForcedUnit, gsDeviceVars[ucIndex].unValue.f,
			gucDefaultUnit[ucIndex]);
	float fPvLower = gsDeviceVars[ucIndex].unLower.f;
	float fPvUpper = gsDeviceVars[ucIndex].unUpper.f;

	if (fabs(fPvUpper - fPvLower) < 0.000001) {
		fPercOfRange = 0.0;
	} else {
		fPercOfRange = (fPvValue - fPvLower) / (fPvUpper - fPvLower) * 100;
	}
	return fPercOfRange;
}

/*************************************************
 Ŀ�ģ�  ȡ���豸����
 ������  USIGN8 ucIndex ������������
 0����һ������1���ڶ���2 ��������3������
 ����ֵ��float, ����ֵ 
 ������  ��
 ****************************************************/
float GetDevValue(USIGN8 ucIndex) {
	union MyFloat unDeviceValue;

	if (ucIndex < IUserVariableCount) {
        if(ucIndex == 0)
        {
            /*��һ������ = �ɼ�ֵ - �����׼ֵ*/
            unDeviceValue.f = gsDeviceVars[0].unValue.f - gsDeviceVars[0].fZero;
        }
        else
		unDeviceValue.f = gsDeviceVars[ucIndex].unValue.f;
	} else {
		unDeviceValue.b[0] = 0x7F; //Slot 0: Device Variable Value
		unDeviceValue.b[1] = 0xA0; //Slot 0: Device Variable Value
		unDeviceValue.b[2] = 0x00; //Slot 0: Device Variable Value
		unDeviceValue.b[3] = 0x00; //Slot 0: Device Variable Value
	}
	return unDeviceValue.f;
}
