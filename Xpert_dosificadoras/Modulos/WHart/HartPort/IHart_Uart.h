#ifndef _IHART_UART_H
#define _IHART_UART_H

/*UART�����뷢�ͷ���ֵö������*/
typedef enum {
	IHART_PROCESS_NO_ERR = 0,    //�޴���
	IHART_DATA_TOO_LONG = 1,    //���ݳ���������
	IHART_INVALIDE_UART_NUM = 2,    //�Ƿ���UART�ţ���0��1��
	IHART_UART_BUSY = 3,    //�������ݴ��������
	IHART_BYTECOUNT_ERR = 4,    //�����е�byte count����
	IHART_PREAMNUM_TOOSHORT = 5,    //�����е�ǰ��������̫��
	IHART_PARITY_ERR = 6,    //У��ʧ��
	IHART_INVALID_DELIMITER = 7,    //delimiter����
	IHART_INVALID_ADDR = 8,    //�����е�ַ��������
	IHART_INVALID_CMDID = 9,    //�����зǷ��������
	IHART_RSPDATA_TOOLONG = 10,   //�ظ����Ĺ���
	IHART_ADDR_NOT_MATCH = 11,   //��ַ��ƥ��
	IHART_RSPONSE_TIMEOUT = 12,   //�����ʱ�����ظ�
	IHART_BURST_CMD3 = 13,   //�յ�Burst�����׼
	IHART_INVALID_SADDRES = 14,   //�̵�ַʹ�ô����׼��ֻ��CMD0��ʹ�ö̵�ַ
	IHART_PROCESS_CONFLIT = 99,   //���ڳ�ͻ���ȴ�����0������
	IHART_PROCESS_ERR = 100
} IHART_UART_FUNTION_ENUM;

typedef enum {

	write_long_tag   = 22,				//0x16
	write_burts_period = 103,			//0x67
	read_busrt_mode_config = 105,		//0x69
	write_join_key = 768,				//0x300
	read_join_status = 769,				//0x301
	request_active_Advertising = 770,	//0x302
	force_join_mode = 771,				//0x303
	read_join_mode_config = 772,		//0x304 //NOT IMPLEMENTED
	write_net_id = 773,					//0x305,
	read_net_id = 774 					//0x306,
} user_command_type;

/*��������*/
extern USIGN8 UART1_Receive_finish;
extern USIGN8 UART0_Receive_finish;

extern volatile USIGN8 nDelayRTSTime;

/*�û����ýӿں���*/
/*******************************************************************
 �ӿڹ���  : �û�����һ֡���ݴ���
 �������: USIGN8 nComNum�����յ����ݵ�UART�ţ�0��������ģ�������Ķ˿�; 1��ά���˿ڡ�
 USIGN8* pData�����յ������ݵ���ʼ��ַ��
 USIGN8 nDataLength�����յ������ݳ��ȡ�
 �������: ��
 ����ֵ: ��I_UART_FUNTION_ENUMö�ٽṹ�塣
 ********************************************************************/
IHART_UART_FUNTION_ENUM IHartDealUartRecData(USIGN8 nComNum, USIGN8* pData,
		USIGN8 nDataLength);

/*******************************************************************
 �ӿڹ���  : �û����ͻ������е����ݡ����û�ʵ��
 �������: USIGN8 nComNum���跢�����ݵ�UART�ţ�0��������ģ�������Ķ˿�; 1��ά���˿ڡ�
 USIGN8* pData���������ݵ���ʼ��ַ��
 USIGN8 nDataLength���������ݵĳ��ȡ�
 �������: ��
 ����ֵ: I_UART_FUNTION_ENUM����I_UART_FUNTION_ENUMö�ٽṹ�塣
 ********************************************************************/
IHART_UART_FUNTION_ENUM IHartDealUartSendData(USIGN8 nComNum, USIGN8* pData,
		USIGN8 nDataLength);

void IHartUserActiveRequest(void);
void IHartSendEnd(USIGN8 nComNum);
void Deal_USRT1_Receive_Fun(void);
//void Deal_USRT0_Receive_Fun(void);
IHART_UART_FUNTION_ENUM Deal_USRT0_Receive_Fun(void);
void Deal_USRT0_Read(void);

#endif
