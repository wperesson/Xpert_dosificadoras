#include "IHart_DataSave.h"
#include "IHart_DataAcquisition.h"

/*******************  ��EEPROMָ����ַ��ָ�����ȵ�����  *********************
 * ��ڲ������޷����ַ���ָ�루�������ݴ�ŵ�ַ�� �޷��Ŷ����ͣ�ָ����ַ�� *
 *            �޷����ַ��ͣ����������ֽ�����                                 *
 *   ����ֵ�� ��                                                              *
 ******************************************************************************/

void IHartReadEeprom(USIGN8 *RamAddress, USIGN16 RomAddress, USIGN8 Number) {
//#ifdef VERSION_HCF
//	Read_AT24CXX(RamAddress,RomAddress,Number);
//#else
//
//	USIGN8 i ;
//
//	RomAddress &= EEPROM_FULL ;
//	EEPROM_CS_L ;
//    write_spi_byte ( EEPROM_READ ) ;
//  	write_spi_word ( RomAddress ) ;
//	for ( i = 0 ; i < Number ; i ++ )
//	{
//		* RamAddress = read_spi_byte ( ) ;
//		RamAddress ++ ;
//	}
//	EEPROM_CS_H ;
//#endif

	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_read((char*)RamAddress, RomAddress, Number);
		xSemaphoreGive(mtxI2C1);
	}

}

/***************  ��EEPROMдָ����ַָ�����ȵ�����  *********************
 * ��ڲ������޷����ַ���ָ�루ָ���д���ݣ� �޷��Ŷ����ͣ�ָ����ַ�� *
 *            �޷����ַ��ͣ���д�����ֽ�����                             *
 *   ����ֵ�� ��                                                          *
 **************************************************************************/
void IHartWriteEeprom(USIGN8 *RamAddress, USIGN16 RomAddress, USIGN8 Number) {
//#ifdef VERSION_HCF
//	Write_AT24CXX(RamAddress,RomAddress,Number);
//#else
//	USIGN8 i, ucInPageLength;
//	USIGN16 uiaccountlength;
//
//	while (Number != 0) {
//		eeprom_WREN();       // CS High
//		RomAddress &= EEPROM_FULL;
//		uiaccountlength = (RomAddress & (EEPROM_PAGE - 1));
//		ucInPageLength = EEPROM_PAGE - uiaccountlength;        //����ҳ��ʣ�೤��
//		if (Number > ucInPageLength) {
//			Number -= ucInPageLength;	//ҳ��ʣ�೤��С�����賤�ȣ������´�д��������
//		} else {
//			ucInPageLength = Number;
//			Number = 0;			  //ҳ��ʣ�೤�ȴ��ڵ������賤�ȣ�ִ��һ��д����
//		}
//		EEPROM_CS_L;
//		write_spi_byte(EEPROM_WRITE);	//��EEPROM��дָ��
//		write_spi_word(RomAddress);	    //��EEPROMдʮ��λ��ַ
//		for (i = 0; i < ucInPageLength; i++) {
//			write_spi_byte(*RamAddress);
//			RamAddress++;
//		}
//		EEPROM_CS_H;
//		RomAddress += ucInPageLength;
//		if (RomAddress > EEPROM_FULL) {
//			RomAddress = 0;
//		}
//		do {
//			i = eeprom_RDSR();
//		} while (i & 0x01);
//	}
//	eeprom_WRDI();
//#endif

	if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {
		eeprom_write((char*)RamAddress, RomAddress, Number);
		xSemaphoreGive(mtxI2C1);
	}
}
