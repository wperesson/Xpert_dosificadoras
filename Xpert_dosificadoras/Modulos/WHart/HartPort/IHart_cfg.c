#include "IHart_cfg.h"
#include "IHart_DataAcquisition.h"
#include "IHart_UserCommand.h"

#include "Definiciones.h"

const USIGN8 IUserRevision[4] = { 7,      //HART�汾
		10,     //�豸�汾
		13,      //����汾-SVN
		20 }; //Ӳ���汾

//const USIGN8 IUserLongAddr[7] = { 0xE2, 0xF6,       //Device Type �豸���� : M1100
//		0x00, 0x09, 0x88,              //Series Number:ʡ(����) ID(2 bytes)
//		0x60, 0x1E };                   //Mfr ID ���̺� : Microcyber Inc.

USIGN8 IUserLongAddr[7] = { 0xE2, 0xF6,       //Device Type �豸���� : M1100
		0x00, 0x09, 0x88,              //Series Number:ʡ(����) ID(2 bytes)
		0x60, 0x1E };                   //Mfr ID ���̺� : Microcyber Inc.


char IUserHpsTag[] = "CHIP2017";     //Short Tag. Device tag (max. 8 characters)

const char IUserHpsDes[] = "Smart Inj Pump";  //Device descriptor (max. 16 characters)

const char IUserMessage[] = "Manufactured by GOFSCO";  //Device message (max. 32 characters)

const USIGN8 IUserAssemblyNumber[3] = { 0x00, 0x00, 0x00 }; //Final Assembly Number

const USIGN8 IUserDate[3] = { 1, 10, 117 };//Device ex-factory date //Date��Day/Month/Year(Starting from 1900 count)

const USIGN8 IUserDynamicsCount = DYNAMICS_COUNT;               //��̬��������

const USIGN8 IUserVariableCount = VARIABLE_COUNT;               //�豸��������

const USIGN8 IUserSTOTime = 200;                                //�豸���Ӧʱ��

const USIGN8 IUserRTSLowTimer = 4;                  //��������ʱ��ǰ�����Ϳ�����ʱ��

const USIGN8 IUserVariableUpdateRate = 100; //;                     //������������,ÿ�� n x 1ms

//�ָ���������,1�ô�λ��ָ�Ĭ�ϳ�������,Ϊ0ʱ���EEPROM�ж�ȡ����ʱ��������
const USIGN8 IUserFactorySet = 0;

//Hart�˷���ʧ�Դ�����ʼ��ַ
const USIGN16 IUserHartDataSaveStartAddr = OFF_WHART;                       //0;

//�豸����Ĭ�ϵ�λ,������λ��IHart_DataAcquisition.h
const USIGN8 gucDefaultUnit[VARIABLE_COUNT] = { psi, LITERS, GALLONS_PER_DAY,
		Percent };

//�豸����Ĭ�Ϸ���,��IHart_cfg.h
const USIGN8 gucDefaultClassification[VARIABLE_COUNT] = {
		CLASSIFICATION_PRESSURE,
		CLASSIFICATION_VOLUME, CLASSIFICATION_VOLUMETRIC_FLOW,
		CLASSIFICATION_NONE };
//�豸�����룬��IHart_cfg.h
const USIGN8 IUserDefaultFamily[VARIABLE_COUNT] = { FAMILY_PRESSURE,
		FAMILY_PRESSURE,
		FAMILY_LEVEL,
		FAMILY_NOT_USED };
//�豸����Ĭ�ϵ�������
const FLOAT IUserDefaultUpperLimit[VARIABLE_COUNT] = { 3300.00, 1000.00, 200.00,
		65535.00 };
const FLOAT IUserDefaultLowerLimit[VARIABLE_COUNT] = { 0.00, 0.00, 0.00, 0.00 };

//User-defined Common Practice Commands range 32~121 (Except mandatory command)
const UserDefineCommand IHartDefineCommands[] = { { NULL, NULL }
//    {35, RespondCommand_35}
//    {512, RespondCommand_512}
		};

//User-defined Device Specific Commands range (128~253 || 1024~33791) 
const UserDefineCommand IUserDefineCommands[] = { { 130, RespondCommand_130 }
//    {170 , RespondCommand_170},
//    {171 , RespondCommand_171}
		};

//Number of User-defined Commands
const USIGN16 IHartCommandCount = 0;
const USIGN16 IUserCommandCount = 1;

