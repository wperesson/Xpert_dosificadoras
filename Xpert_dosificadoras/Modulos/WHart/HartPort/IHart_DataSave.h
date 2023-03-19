#ifndef _IHART_DATASAVE_H
#define _IHART_DATASAVE_H

#include "IHart_glb_defs.h"

/**************  EE memory control ***************/

//---------��Ҫ���浽EEPROM��---------------------------------------//
#define CMD_185 				    0xB9                            // ����185�ָ�Ĭ��У׼���ݱ�־λ
#define EE_DEFAULT                  0x79                            // Default EE Data Flag
#define EE_HAS_DATA                 0x69                            // Have EE Data Flag
#define EE_RESTOR_DATA              0x68                            // Restor EE Data Flag
#define EE_DATA_NUM_PER_PAGE        50
#define EE_BEGIN_STATIC_ADDR        IUserHartDataSaveStartAddr      // ���澲̬���ݵ���ʼ��ַ

#define EE_DATA_SIGN                EE_BEGIN_STATIC_ADDR            //1 byte �о�̬���ݱ�־

/**********************  ��һҳ �豸��Ϣ                 ****************************/
#define EE_POLLING_ADDR             (EE_DATA_SIGN+1)                //1  byte  ��ѵ��ַ
#define EE_MESSAGE                  (EE_POLLING_ADDR+1)             //24 bytes ��Ϣ
#define EE_ASSEMBLE                 (EE_MESSAGE+24)                 //3  bytes ���װ���
#define EE_TAGDESDATE               (EE_ASSEMBLE+3)                 //21 bytes ��ǩ����������
#define EE_LOG_TAG                  (EE_TAGDESDATE+21)              //32 bytes ��λ��
#define EE_SEN_COMM38               (EE_LOG_TAG+32)                 //1  bytes ����38�ı��־
#define EE_LOCK_DIVICE              (EE_SEN_COMM38+1)               //1  byte  ���ز���ʹ��

#define EE_CFG_CHANGE               (EE_LOCK_DIVICE+1)              //1  byte  ��̬�ı��־
#define EE_CON_CHA_COUNTER          (EE_CFG_CHANGE+1)               //2  bytes ��������Counter
#define EE_PREAMBLES_NUM            (EE_CON_CHA_COUNTER+2)          //1  bytes ǰ��������

/************************** PV�������� ***********************************************************/        
#define DEVICE_VARIABLE_LENGTH      32                              //32 bytes �豸�����洢����
#define EE_PV_PROPERTY              (EE_PREAMBLES_NUM+1)            //1  bytes Property
#define EE_PV_UNIT                  (EE_PV_PROPERTY+1)              //1  bytes ������λ
#define EE_PV_RANGE_UNIT            (EE_PV_UNIT+1)                  //1  bytes PV_Range��λ
#define EE_PV_TRANS_FUN             (EE_PV_RANGE_UNIT+1)            //1  bytes �������ݺ�������
#define EE_PV_ALARM_CODE            (EE_PV_TRANS_FUN+1)             //1  bytes ��������ѡ�����
#define EE_PV_SENSOR_SN             (EE_PV_ALARM_CODE+1)            //3  bytes �������������к�
#define EE_PV_DAMP                  (EE_PV_SENSOR_SN+3)             //4  bytes ����������ֵ
#define EE_PV_UPPER_RANGE           (EE_PV_DAMP+4)                  //4  bytes ������������
#define EE_PV_LOWER_RANGE           (EE_PV_UPPER_RANGE+4)           //4  bytes ������������
#define EE_PV_UPPERLIMIT            (EE_PV_LOWER_RANGE+4)           //4  bytes ��������������
#define EE_PV_LOWERLIMIT            (EE_PV_UPPERLIMIT+4)            //4  bytes ��������������
#define EE_PV_ZERO                  (EE_PV_LOWERLIMIT+4)            //4  bytes �������

//��̬�������豸����ӳ���ϵ  4����̬����
#define EE_DYNAMIC_VAR_LENGTH       1                               //��̬����ӳ���ַ����
#define EE_DYNAMIC_MAP              (EE_PV_ZERO+4)                  //1 bytes ��̬�������豸������ӳ���ϵ

//DLL024C��Ϣ
#define EE_DLL024_BUF               (EE_DYNAMIC_MAP+10)             //30 buffer
#define EE_DLL024_STATE             (EE_DLL024_BUF+30)              //1  buffer
#define EE_DLL024_PARITY            (EE_DLL024_STATE+1)             //2  buffer
#define EE_DLL024_GAPERROR          (EE_DLL024_PARITY+2)            //50 buffer

#define EE_RMOTE_CON_CHA_COUNTER     1024
/****************EEPROM��������*************************/

/****************************************
�ӿڹ���  : ��EEPROMָ����ַ��ָ�����ȵ�����
�������: USIGN8 *RamAddress���������ݴ�ŵ�ַ�� 
          USIGN16 RomAddress��Eeprom��ȡ��ַ��
          USIGN8 Number     ����ȡ���� 
�������: ��
����ֵ  : ��  
******************************************************************************/
extern void IHartReadEeprom(USIGN8 *RamAddress,USIGN16 RomAddress,USIGN8 Number);

/****************************************
�ӿڹ���: ��EEPROMָ����ַд��ָ�����ȵ�����
�������: USIGN8 *RamAddress����д�����ݵĴ�ŵ�ַ�� 
          USIGN16 RomAddress��д��Eeprom�ĵ�ַ��
          USIGN8 Number     ��д�볤�� 
�������: ��
����ֵ  : ��  
******************************************************************************/
extern void IHartWriteEeprom(USIGN8  *RamAddress,USIGN16 RomAddress,USIGN8 Number);


#endif