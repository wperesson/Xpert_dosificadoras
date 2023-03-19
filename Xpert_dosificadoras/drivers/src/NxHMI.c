/*
 * NxHMI.c
 *
 *  Created on: 12 oct. 2022
 *      Author: German Ghigi
 */
#include "board.h"
#include "Definiciones.h"
#include "NxHMI.h"
StreamBufferHandle_t strMsgFromHMI;
uint16_t page_nx=0,pwUsrLvl=0;//tener en cuenta poner la pagina inicial al prender la pantalla en este caso page=0
uint8_t modifVal=0,aCH=0;
void decodeNxFrame()
{

}

Status sendToNx(char *buff)
{
	uint32_t Len;
	Status Stat;

	Len=strlen(buff);
	*(buff+Len)=0xff;
	*(buff+Len+1)=0xff;
	*(buff+Len+2)=0xff;
	Len+=2;//para que no envie un dato de mas
	while ((xQueueSend(queMsgToHMINx, buff++, 10) != errQUEUE_FULL)
			&& (Len > 0)) {
		Len--;
	}
	//Chip_UART_IntEnable(LPC_UART2, UART_IER_THREINT);
//
//	if (Len == Chip_UART_SendBlocking(LPC_UART2, (uint32_t*) buff, Len)) {
//		Stat = SUCCESS;
//	} else {
//		Stat = ERROR;
//	}
	//ulTaskNotifyTake(TRUE, portMAX_DELAY);
	vTaskDelay(20);
	return Stat;

}


uint32_t wait_Byte_Nex(char *data, TickType_t delay) {

	TickType_t _delay = delay;
	uint32_t ReadB, ReadC;

	ReadB = 0;
	ReadC = 0;
	do {
		ReadC = xStreamBufferReceive(strMsgFromHMI, data + ReadB,
		4, _delay);
		ReadB += ReadC;

		/* Delay para esperar por si el UART recibe una interrupción por
		 * una cadena de bytes que alcanzaron el trigger level u otro byte
		 * que llego y se activo el Character Timeout Indicator que es 3.5 a 4.5
		 * veces el tiempo de cada caracter.
		 *
		 * delay = (10 bits /9600 bps) x TRIGGER LEVEL*/
		_delay = 15;

	} while (ReadC && (ReadB < BUF485_SIZE - 16));
	return ReadB;

}
