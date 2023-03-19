#ifndef _IHART_DATASAVE_H
#define _IHART_DATASAVE_H

#include "IHart_glb_defs.h"

/**************  EE memory control ***************/

//---------需要保存到EEPROM中---------------------------------------//
#define CMD_185 				    0xB9                            // 命令185恢复默认校准数据标志位
#define EE_DEFAULT                  0x79                            // Default EE Data Flag
#define EE_HAS_DATA                 0x69                            // Have EE Data Flag
#define EE_RESTOR_DATA              0x68                            // Restor EE Data Flag
#define EE_DATA_NUM_PER_PAGE        50
#define EE_BEGIN_STATIC_ADDR        IUserHartDataSaveStartAddr      // 保存静态数据的起始地址

#define EE_DATA_SIGN                EE_BEGIN_STATIC_ADDR            //1 byte 有静态数据标志

/**********************  第一页 设备信息                 ****************************/
#define EE_POLLING_ADDR             (EE_DATA_SIGN+1)                //1  byte  轮训地址
#define EE_MESSAGE                  (EE_POLLING_ADDR+1)             //24 bytes 消息
#define EE_ASSEMBLE                 (EE_MESSAGE+24)                 //3  bytes 最后装配号
#define EE_TAGDESDATE               (EE_ASSEMBLE+3)                 //21 bytes 标签描述和日期
#define EE_LOG_TAG                  (EE_TAGDESDATE+21)              //32 bytes 长位号
#define EE_SEN_COMM38               (EE_LOG_TAG+32)                 //1  bytes 命令38改变标志
#define EE_LOCK_DIVICE              (EE_SEN_COMM38+1)               //1  byte  本地操作使能

#define EE_CFG_CHANGE               (EE_LOCK_DIVICE+1)              //1  byte  组态改变标志
#define EE_CON_CHA_COUNTER          (EE_CFG_CHANGE+1)               //2  bytes 本地配置Counter
#define EE_PREAMBLES_NUM            (EE_CON_CHA_COUNTER+2)          //1  bytes 前导符数量

/************************** PV变量参数 ***********************************************************/        
#define DEVICE_VARIABLE_LENGTH      32                              //32 bytes 设备变量存储长度
#define EE_PV_PROPERTY              (EE_PREAMBLES_NUM+1)            //1  bytes Property
#define EE_PV_UNIT                  (EE_PV_PROPERTY+1)              //1  bytes 变量单位
#define EE_PV_RANGE_UNIT            (EE_PV_UNIT+1)                  //1  bytes PV_Range单位
#define EE_PV_TRANS_FUN             (EE_PV_RANGE_UNIT+1)            //1  bytes 变量传递函数代码
#define EE_PV_ALARM_CODE            (EE_PV_TRANS_FUN+1)             //1  bytes 变量报警选择代码
#define EE_PV_SENSOR_SN             (EE_PV_ALARM_CODE+1)            //3  bytes 变量传感器序列号
#define EE_PV_DAMP                  (EE_PV_SENSOR_SN+3)             //4  bytes 主变量阻尼值
#define EE_PV_UPPER_RANGE           (EE_PV_DAMP+4)                  //4  bytes 变量量程上限
#define EE_PV_LOWER_RANGE           (EE_PV_UPPER_RANGE+4)           //4  bytes 变量量程下限
#define EE_PV_UPPERLIMIT            (EE_PV_LOWER_RANGE+4)           //4  bytes 变量传感器上限
#define EE_PV_LOWERLIMIT            (EE_PV_UPPERLIMIT+4)            //4  bytes 变量传感器下限
#define EE_PV_ZERO                  (EE_PV_LOWERLIMIT+4)            //4  bytes 变量零点

//动态变量与设备变量映射关系  4个动态变量
#define EE_DYNAMIC_VAR_LENGTH       1                               //动态变量映射地址长度
#define EE_DYNAMIC_MAP              (EE_PV_ZERO+4)                  //1 bytes 动态变量与设备变量的映射关系

//DLL024C信息
#define EE_DLL024_BUF               (EE_DYNAMIC_MAP+10)             //30 buffer
#define EE_DLL024_STATE             (EE_DLL024_BUF+30)              //1  buffer
#define EE_DLL024_PARITY            (EE_DLL024_STATE+1)             //2  buffer
#define EE_DLL024_GAPERROR          (EE_DLL024_PARITY+2)            //50 buffer

#define EE_RMOTE_CON_CHA_COUNTER     1024
/****************EEPROM操作函数*************************/

/****************************************
接口功能  : 从EEPROM指定地址读指定长度的数据
输入参数: USIGN8 *RamAddress：读出数据存放地址， 
          USIGN16 RomAddress：Eeprom读取地址，
          USIGN8 Number     ：读取长度 
输出参数: 无
返回值  : 无  
******************************************************************************/
extern void IHartReadEeprom(USIGN8 *RamAddress,USIGN16 RomAddress,USIGN8 Number);

/****************************************
接口功能: 向EEPROM指定地址写入指定长度的数据
输入参数: USIGN8 *RamAddress：带写入数据的存放地址， 
          USIGN16 RomAddress：写入Eeprom的地址，
          USIGN8 Number     ：写入长度 
输出参数: 无
返回值  : 无  
******************************************************************************/
extern void IHartWriteEeprom(USIGN8  *RamAddress,USIGN16 RomAddress,USIGN8 Number);


#endif