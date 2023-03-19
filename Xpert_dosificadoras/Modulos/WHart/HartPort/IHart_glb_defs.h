#ifndef _IHART_GLB_DEFS_H_
#define _IHART_GLB_DEFS_H_

//#include <msp430x14x.h>
#include <board.h>
#include "chip.h"
#include <string.h>
#include "Definiciones.h"

#define CPU_F                            ((double)3686400) 
#define delay_us(x)                      __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x)                      __delay_cycles((long)(CPU_F*(double)x/1000.0)) 

typedef signed char SIGN8;
typedef unsigned char USIGN8;
typedef unsigned short USIGN16;
typedef unsigned long USIGN32;
typedef unsigned char BOOL;
typedef float FLOAT;

#define uchar       unsigned char
#define uint        unsigned int
#define ulong       unsigned long

union MyFloat {
	float f;
	USIGN8 b[4];
};

/*����汾*/
//#define VERSION_HCF
#define VERSION_RED
//#define VERSION_BULE

/*����*/
#define CLOCK_32_768KHz
//#define CLOCK_3_6864MHz

/*�Ǳ�����ģ��RTC����*/
//#define FUN_MONITOR_RTC
/*****************  ȫ�ֱ�������    ****************/
#define	NOP1	                    _NOP();
#define	NOP2	                    _NOP();_NOP()
#define	NOP3	                    _NOP();_NOP();_NOP()
#define	NOP4	                    _NOP();_NOP();_NOP();_NOP()
#define	NOP5	                    _NOP();_NOP();_NOP();_NOP();_NOP()

#define TRUE                        1
#define FALSE                       0

#define DYNAMICS_COUNT              4              //��̬��������
#define VARIABLE_COUNT              4              //�豸��������

#define PREAMBLE_MAX_NUM	        20	           // ���ǰ��������
#define PREAMBLE_NUM		        5			   // Num Of Request Preambles

#define RX_PAYLOAD_MAX_NUM          80			   // �ο�#9
#define TX_PAYLOAD_MAX_NUM          89			   // �ο�#78

#define HPS_DATA_FIELD_SIZE         (TX_PAYLOAD_MAX_NUM + 11)	

#define USER_MAX_UART_RX_BUFFER     (PREAMBLE_MAX_NUM + RX_PAYLOAD_MAX_NUM + 11)  //���ڽ�����󻺳���
#define USER_MAX_UART_TX_BUFFER     (PREAMBLE_MAX_NUM + HPS_DATA_FIELD_SIZE)      //���ڷ�����󻺳���

#define DEVICE_POLLINGADDR	        0	           //Polling Address
#define BATTERY_ALARM_VALUE         2.7            //��ر���ֵ

#define CLASSIFICATION_64
#define CLASSIFICATION_70
#define CLASSIFICATION_81
#define CLASSIFICATION_83
#define CLASSIFICATION_84

///**************** Ӳ���ӿں궨��  ****************/
///*25LC320*/
//#define EEPROM_CS_H         P1OUT |= BIT6
//#define EEPROM_CS_L         P1OUT &= (~BIT6)
//#define EEPROM_WP_H         P2OUT |= BIT0
//#define EEPROM_WP_L         P2OUT &= (~BIT0)
//#define EEPROM_SCK_H        P2OUT |= BIT5
//#define EEPROM_SCK_L        P2OUT &= (~BIT5)
//#define EEPROM_SDI_H        P2OUT |= BIT4
//#define EEPROM_SDI_L        P2OUT &= (~BIT4)
//#define EEPROM_SDO_IN       (P1IN & BIT7)
//
///*AT24C16*/
//#define CLR_I2C_SDA         P4OUT &= ~(0x02)                    //P4.1 =0
//#define SET_I2C_SDA         P4OUT |=  (0x02)                    //P4.1 =1
//#define I2C_SDA_IN          P4DIR &=~(0x02)
//#define I2C_SDA_OUT         P4DIR |=0x02
//#define RE_I2C_SDA          (P4IN & BIT1)                       //P4.1Ϊ����
//#define CLR_I2C_SCL         (P4OUT &= ~(0x04))                  //P4.2 =0
//#define SET_I2C_SCL         (P4OUT |=  (0x04))                  //P4.2 =1

//#define HART_TX_Enable      (P3OUT |= BIT2);(P4OUT &= (~BIT0))  //HART���Ϳ���ʹ��
//#define HART_RX_Enable      (P3OUT &= (~BIT2));(P4OUT |= BIT0)  //HART���͹ر� ����ʹ��
//
//#define	AD421_DATA_H        P4OUT |= BIT3
//#define	AD421_DATA_L        P4OUT &= (~BIT3)
//#define	AD421_SCLK_H        P4OUT |= BIT4
//#define	AD421_SCLK_L        P4OUT &= (~BIT4)
//#define	AD421_LATCH_H       P4OUT |= BIT5
//#define	AD421_LATCH_L       P4OUT &= (~BIT5)
//
#if !MOTOR_ELEC
#define WHM_CD              LPC_WHM_CD//whar(P1IN & BIT3)
#define WHM_RTS_H           WHART_RT_OFF//(P1OUT |= BIT2)                     //����ģ�� ���ͽ��� ����
#define WHM_RTS_L           WHART_RT_ON//(P1OUT &= (~BIT2))                  //����ģ�� ����4ms���� ��������
#else
// En caso de usar motor electrico se remplaza el WHART con un adaptador 485
#define WHM_CD 0//����ģ��
#define WHM_RTS_H          0//(P1OUT |= BIT2)                     //����ģ�� ���ͽ��� ����
#define WHM_RTS_L          0//(P1OUT &= (~BIT2))
#endif
//-- COMMAND RESPONSE ERROR CODES -------------------------
#define     NO_COMMAND_SPECIFIC_ERRORS           (0<<8)
#define     UNDEFINED                            (1<<8)
#define     INVALID_SELECTION                    (2<<8)
#define     PASSED_PARAMETER_TOO_LARGE           (3<<8)
#define     PASSED_PARAMETER_TOO_SMALL           (4<<8)
#define     TOO_FEW_DATA_BYTES_RECEIVED          (5<<8)
#define     DEVICE_SPECIFIC_COMMAND_ERROR        (6<<8)
#define     IN_WRITE_PROTECT_MODE                (7<<8)
#define     RC8                                  (8<<8)
#define     RC9                                  (9<<8)
#define     RC10                                 (10<<8)
#define     RC11                                 (11<<8)
#define     RC12                                 (12<<8)
#define     RC13                                 (13<<8)
#define     RC14                                 (14<<8)
#define     RC15                                 (15<<8)
#define     ACCESS_RESTRICTED                    (16<<8)
#define     INVALID_DEVICE_VARIABLE_INDEX        (17<<8)
#define     INVALID_UNITS_CODE                   (18<<8)
#define     DEVICE_VARIABLE_INDEX_NOT_ALLOWED    (19<<8)
#define     INVALID_EXTENDED_COMMAND_NUMBER      (20<<8)
#define     RC28                                 (28<<8)
#define     COMMAND_RESPONSE_TRUNCATED           (30<<8)
#define     HART_BUSY                            (32<<8)
#define     DELAYED_RESPONSE_INITIATED           (33<<8)
#define     DELAYED_RESPONSE_RUNNING             (34<<8)
#define     DELAYED_RESPONSE_DEAD                (35<<8)
#define     DELAYED_RESPONSE_CONFLICT            (36<<8)
#define     COMMAND_NOT_IMPLEMENTED              (64<<8)
#define     INVALID_SPAN                         (29<<8)

#define     UPDATE_FAILURE                              RC8
#define     SET_TO_NEAREST_POSSIBLE_VALUE               RC8
#define     ALL_BUT_RUNNING_DELAYED_RESPONSES_FLUSHED   RC8

//#define   UPPER_RANGE_VALUE_TOO_HIGH                  RC9
#define     LOWER_RANGE_VALUE_TOO_HIGH                  RC9
#define     ZERO_RANGE_VALUE_TOO_HIGH                   RC9
#define     APPLIED_PROCESS_TOO_HIGH                    RC9
#define     NOT_IN_PROPER_CURRENT_MODE                  RC9
#define     UNABLE_TO_SQUAWK                            RC9
#define     CONFIGURATION_CHANGE_COUNTER_MISMATCH       RC9
#define     INVALID_COMMAND_REQUESTED                   RC9

//#define   UPPER_RANGE_VALUE_TOO_LOW                   RC10
#define     LOWER_RANGE_VALUE_TOO_LOW                   RC10
#define     ZERO_RANGE_VALUE_TOO_LOW                    RC10
#define     APPLIED_PROCESS_TOO_LOW                     RC10
#define     INVALID_LOCAL_PANEL_LOCK_CODE               RC10
#define     INVALID_LOCK_CODE                           RC10
#define     INVALID_WRITE_DEVICE_VAR_COM_CODE           RC10

#define     UPPER_RANGE_VALUE_TOO_HIGH                  RC11
#define     IN_MULTI_DROP_MODE                          RC11
#define     INVALID_DEVICE_VARIABLE_CODE                RC11
#define     TRIM_ERROR_EXCESS_CORRECTION_ATTEMPTED      RC11
#define     CANNOT_LOCK_PANEL                           RC11
#define     LOOP_CURRENT_NOT_ACTIVE                     RC11

#define     UPPER_RANGE_VALUE_TOO_LOW                   RC12
//#define   INVALID_UNITS_CODE                          RC12
#define     INVALID_SLOT_NUMBER                         RC12
#define     INVALID_MODE_SELECTION                      RC12

#define     INVALID_TRANSFER_FUNCTION_CODE              RC13
#define     UPPER_AND_LOWER_RANGEVALUES_OUT_OF_LIMITS   RC13
#define     COMPUTATION_ERROR                           RC13
#define     COMMAND_NUMBER_NOT_SUPPORTED                RC13

#define     SPAN_TOO_SMALL_THE_SPAN                     RC14
#define     NEW_LOWER_RANGE_VALUE                       RC14
#define     PUSHED_UPPER_RANGE                          RC14
#define     VALUE_OVER_TRANSDUCER_LIMIT                 RC14

#define     INVALID_ANALOG_CHANNEL                      RC15
#define     CODE_NUMBER                                 RC15

#define     INVALID_RANGE_UNITS_CODE                    RC28

/* ��Ӧ����Ϣ */
#define HART_ERR				0x80	//ͨѶ����
#define HART_PARITY_VTC_ERR		0x40	//��ֱ��ż����
#define HART_OVERRUN_ERR		0x20	//���Ǵ���UART�Ľ��ջ��������ٳ���һ���ֽڱ�����
#define HART_FRAME_ERR			0x10	//�������һ�������ֽڵ�ֹͣλδ��UART��⵽
#define HART_PARITY_LGT_ERR		0x08	//������ż����
//#define RESERVED				0x00	//����
#define HART_BUFFER_ERR			0x02	//���������
#define HART_UNDEFINED			0x00	//û�ж���

/* �豸״̬ */
#define HART_DEV_MALF			0x80	//�ֳ�����������
#define HART_CFG_CHANGE			0x40	//��̬�仯
#define HART_COLD_START			0x20	//������
#define HART_MORE				0x10	//�и���״̬��Ϣ����
#define HART_FIXED				0x08	//��������ģ��������̶�
#define HART_SATURATED			0x04	//��������ģ������ѱ���
#define HART_NP_LIMIT			0x02	//������������ Non-Primary Variable Out of Limits��
#define HART_P_LIMIT			0x01	//����������

#endif
