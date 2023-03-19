/*
 * Tsk_MOBILE_Rx_Tx.c
 *
 *  Created on: 08/11/2019
 *  Author: Walter Peresson
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Definiciones.h"

#define UART				LPC_UART3
#define UART_IRQ_SELEC		UART3_IRQn
#define UART_HANDLER_NAME 	UART3_IRQHandler
#define BAUDRATE			115200
#define REF_RATE			20000

StreamBufferHandle_t strMsgFromMOBILE;

static portTASK_FUNCTION(vCOMM_MOBILE_Tx_Task,pvParameters) {
	UART_TX_TRANSFER_T data;

	while (1) {

		/* En esta operación recibimos el valor inicial de la cola. Luego en la Rx
		 * vamos a recibir el valor de la cola para quitarlo de la misma.
		 * De esta manera sabemos cuantos bytes fueron enviados para esperar los
		 * mismos que retornan del MOBILE*/
		while (xQueueReceive(queMsgToMOBILE, &data, portMAX_DELAY) != pdFAIL) {
			Chip_UART_SendBlocking(UART, data.ptr, data.Len);
			Chip_UART_SendBlocking(UART, "\r\n", 2);
			//todo: usar a Non Blocking funcion DMA para enviar los datos

			/*Envio la cantidad de bytes despachados*/
			xQueueSend(queSentByTx, &data.Len, 0);
		}
	}
}

static portTASK_FUNCTION(vCOMM_MOBILE_Rx_Task,pvParameters) {
	float temp_float;
	uint8_t temp_byte;
	char temp_char[50];
	EvtToMOB_Type EvtToMOBILE;
	char Buf_MOBILE[MOBILE_BUFF_SIZE];
	char *ptr, *ptr2;
	static uint16_t ReadB, ReadC, i, SentByTx, msg_len;
	bool rutear_msg;
	Parameter_upd_Type newParameter;
	mqtt_event_type mqtt_st;
//	ev_state_bluet_type evblue_st;
//	Bool refrescar_MQTT;
//	uart_tx_Type data;
	TickType_t delay;

	strMsgFromMOBILE = xStreamBufferCreate(MOBILE_BUFF_SIZE+4, 2);
//	strMsgFromMOBILE = xStreamBufferCreateStatic(MOBILE_BUFF_SIZE, 2);

//Debo crear el stream buffer antes de configurar el puerto para evitar que un dato entrante
//intente enviar al buffer str que no ha sido creado

#if USE_BG96
	BG96_RST_OFF;
#endif

// UART_3: COMUNICACION CON GSM MODULE //////////////////
	Chip_UART_Init(UART);
	Chip_UART_SetBaud(UART, BAUDRATE);
	Chip_UART_ConfigData(UART,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(UART,
	UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3);

	/* Enable UART3 interrupt */
	NVIC_DisableIRQ(UART_IRQ_SELEC);
	NVIC_SetPriority(UART_IRQ_SELEC,
	mainSOFTWARE_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(UART_IRQ_SELEC);

	Chip_UART_TXEnable(UART);
	Chip_UART_ReadIntIDReg(UART);

	Create_COMM_MOBILE_Tx_task();
	Create_COMM_MOBILE_Ev_task();


	while (1) {

		memset(Buf_MOBILE, 0, MOBILE_BUFF_SIZE);

		delay = portMAX_DELAY;
		ReadB = 0;
		do {
			ReadC = xStreamBufferReceive(strMsgFromMOBILE, &Buf_MOBILE[ReadB],
			MOBILE_BUFF_SIZE, delay);
			ReadB += ReadC;
			delay = 20;
		} while (ReadC && (ReadB < MOBILE_BUFF_SIZE - 16));

		/* Solo en caso de que este activado el debugger o el sistema de sniffer
		 * copio los datos entrantes al RS485 y asi puedo visualizar los datos que
		 * se estan intercambiando con el MOBILE*/
		if (sniff_gsm || debug_gsm) {
			if (ReadB != 0) {
				i = 0;
				while ((xQueueSend(queMsgToRS485, &Buf_MOBILE[i++], 10)
						!= errQUEUE_FULL) && (i < ReadB)) {
				}
			}
		}
		/*Recibo el valor de la cantidad de bytes que despaché al MOBILE
		 * y asi puedo comparar con los que retornan.*/
		if (xQueueReceive(queSentByTx, &SentByTx, 0) == pdFAIL) {
			/*Si no tengo mensajes enviados, entonces este mensaje se originó en el MOBILE*/
			SentByTx = 0;
		}

		/*Por defecto, el dato es no reconocido. Asi el sistema lo enviará a
		 * quien lo este esperando*/
		rutear_msg = true;

		/*Por defecto, el dato viene solo. En caso de que venga otro en el mismo stream
		 * debo setear este flag*/
		if (SentByTx == 0) {
			/*Mensaje recibido sin ninguno enviado, llegado desde MOBILE*/

			/* Por defecto mando el stream entero que llego*/
			msg_len = ReadB;

			/**/
			if ((ptr = strstr(Buf_MOBILE, SEND_OK)) != NULL) {
				rutear_msg = true;
			}

			else if ((ptr = strstr(Buf_MOBILE, "CONNECT OK")) != NULL) {
				rutear_msg = true;
				/*Reporto la longitud del mensaje*/
				msg_len = strlen(ptr) + 2;
			}

			/*Dato proveniente de SMS???*/
			/* En caso de recibir un SMS, el SIM me enviará una notificación con el formato:
			 * +CMTI: "SM",1
			 * */
			else if ((ptr = strstr(Buf_MOBILE, "+CMTI:")) != NULL) {
				rutear_msg = false;

				/*Reporto la longitud del mensaje*/
				msg_len = strlen(ptr) + 2;

				//Dato proviene desde SMS.
				if (!debug_gsm) {
					EvtToMOBILE.Src = read_process_sms;
					xQueueSend(queEvtToMOBILE, &EvtToMOBILE, 0);
				}
			}

			else if ((ptr = strstr(Buf_MOBILE, "+SMSTATE: 0")) != NULL) {
				/*Reporto la longitud del mensaje*/
				mqtt_st = mqtt_ev_connect_to_broker;
				xQueueSend(queMQTT_ev, &mqtt_st, 0);
			}

			/*Dato proveniente de SUBSCRIPTION?
			 * En caso de que venga un comando desde el broker, puede llegar junto a otra respuesta
			 * de una operacion en curso. Por lo tanto debo cerciorarme que el comando no tenga
			 * otra información antes ni después del comando.
			 *
			 * Alternativa 1: información antes del comando
			 * xxxxx/losant/5xxxxxxx38/command
			 *
			 * Alternativa 2: información después del comando
			 * xxxxx/losant/5xxxxxxx38/command
			 * */
			else if ((ptr = strstr(Buf_MOBILE + 10, clientID)) != NULL) {
				if ((ptr = strstr(ptr, "/command")) != NULL) {
					if ((ptr = strstr(Buf_MOBILE + 5, "name\"")) != NULL) {
						/*Si tiene name es reconocido*/
						rutear_msg = false;

						/*Todos la info recibida por aca son comandos enviados desde el dashboard.
						 * Por lo tanto, por defecto debo reenviar el estado de los controles. En
						 * caso que algún parámetro necesite actualizar otro campo debo especificarlo
						 * dentro de la llave*/
						ptr = strtok(ptr, ":");	//name\"

						ptr = strtok(NULL, ",");	//\"idpmp\" u otro comando

						/*===== COMANDOS ======*/
						//---------Comando: crun -------------------------------
						if (!strcmp(ptr, "\"crun\"")) {
							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"value":1

							if (strstr(ptr, "\"val\"")) {
								if (sscanf(ptr, "\"val\":%d", &temp_byte)
										== 1) {
									snd_head_flow_sett(&newParameter,
											(float) temp_byte, PUMP_ON, src_web,
											0);
								}
							}
						}
						//---------Comando: cdo -------------------------------
						if (!strcmp(ptr, "\"cdo\"")) {

							/*Actualizar los valores de CTRLS*/
							mqtt_st = mqtt_ev_publish_commands;
							xQueueSend(queMQTT_ev, &mqtt_st, 0);

							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"do1":"false"

							if (strstr(ptr, "\"do1\"")) {
								if (sscanf(ptr, "\"do1\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, DO1_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, DO1_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}

							if (strstr(ptr, "\"do2\"")) {
								if (sscanf(ptr, "\"do2\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, DO2_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, DO2_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}

							if (strstr(ptr, "\"do3\"")) {
								if (sscanf(ptr, "\"do3\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, DO3_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, DO3_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}

							if (strstr(ptr, "\"do4\"")) {
								if (sscanf(ptr, "\"do4\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, DO4_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, DO4_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}

							if (strstr(ptr, "\"rl1\"")) {
								if (sscanf(ptr, "\"rl1\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, Rl1_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, Rl1_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}

							if (strstr(ptr, "\"rl2\"")) {
								if (sscanf(ptr, "\"rl2\":\"%[^\"]s\"",
										&temp_char) == 1) {
									if (!strcmp(temp_char, "true"))
										SetFlag((char*) &Var.DO, Rl2_bp, TRUE);
									else
										SetFlag((char*) &Var.DO, Rl2_bp, FALSE);
								}
								ptr = strtok(NULL, ",");	//"do2":"false"
							}
						}
						//---------Comando: cclr -------------------------------
						if (!strcmp(ptr, "\"cclr\"")) {

							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"value":1

							if (strstr(ptr, "\"val\"")) {
								if (sscanf(ptr, "\"val\":%d", &temp_byte)
										== 1) {
									snd_head_flow_sett(&newParameter,
											(float) temp_byte, RST_TOTAL,
											src_web, 0);
								}
							}
						}
						//---------Comando: cflo -------------------------------
						else if (!strcmp(ptr, "\"cflo\"")) {

							/*Mando a actualizar los controles via mqtt luego de confirmar el valor
							 en la rutina de control del motor*/

							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"un":"1"

							if (sscanf(ptr, "\"un\":\"%[^\"]s\"", temp_char)
									== 1) {
								if (!strncmp(temp_char, "LPH", 3))
									temp_byte = Liters_hour;
								else if (!strncmp(temp_char, "LPD", 3))
									temp_byte = Liters_day;
								else if (!strncmp(temp_char, "GPH", 3))
									temp_byte = Galons_hour;
								else if (!strncmp(temp_char, "GPD", 3))
									temp_byte = Galons_day;
								else
									temp_byte = 0;
								if (temp_byte)
									snd_head_flow_sett(&newParameter,
											(float) temp_byte, SP_UNITS,
											src_web, 0);
							}

							ptr = strtok(NULL, ",");	//"val":"18"
							if (sscanf(ptr, "\"val\":\"%f\"", &temp_float)
									== 1) {
								snd_head_flow_sett(&newParameter, temp_float,
										SP_PumpRate, src_web, 0);
							}
						}
						//---------Comando: fupgr -------------------------------
						else if (!strcmp(ptr, "\"fupgr\"")) {
							if (sscanf(ptr,
									"fupgr\",\"payload\":{\"value\":\"%d\"",
									temp_char) == 1) {
								temp_char[0] ?
										(newParameter.value = 1) :
										(newParameter.value = 0);

							}
						}

						/*===== PARAMETROS ======*/
						//---------Parámetro: sal -------------------------------
						else if (!strcmp(ptr, "\"sal\"")) {
							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"wr":"10"
							if (sscanf(ptr, "\"wr\":\"%f\"", &temp_float)
									== 1) {
								snd_tank_sett(&newParameter, temp_float,
										SP_War_TankLevel, src_web);
							}

							ptr = strtok(NULL, ",");	//"al":"5"
							if (sscanf(ptr, "\"al\":\"%f\"", &temp_float)
									== 1) {
								snd_tank_sett(&newParameter, temp_float,
										SP_AL_TankLevel, src_web);
							}

							ptr = strtok(NULL, "}");	//"al":"5"
							if (sscanf(ptr, "\"lb\":\"%f\"", &temp_float)
									== 1) {
								gen.low_batt_limit = temp_float;
								//Almacenar en EEPROM los cambios en gen
								guardar_eeprom((char*) &gen, OFF_GEN,
										sizeof(gen));

							}
						}
						//---------Parámetro: stk -------------------------------
						else if (!strcmp(ptr, "\"stk\"")) {
							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, "}");	//

							if ((ptr2 = strstr(ptr, "\"shgt\"")) != NULL) {
								if (sscanf(ptr2, "\"shgt\":\"%f\"", &temp_float)
										== 1) {
									snd_tank_sett(&newParameter, temp_float,
											SP_tnk_sensor_hgt, src_web);
								}
							}

							if ((ptr2 = strstr(ptr, "\"dens\"")) != NULL) {
								if (sscanf(ptr2, "\"dens\":\"%f\"", &temp_float)
										== 1) {
									snd_tank_sett(&newParameter, temp_float,
											SP_tnk_dens, src_web);
								}
							}

							if ((ptr2 = strstr(ptr, "\"shap\"")) != NULL) {
								if (sscanf(ptr2, "\"shap\":\"%d\"", &temp_byte)
										== 1) {
									if ((temp_byte >= ref_points)
											&& (temp_byte <= Rect)) {
										snd_tank_sett(&newParameter, temp_byte,
												SP_tnk_Shp, src_web);
									}
								}
							}

							if ((ptr2 = strstr(ptr, "\"dim1\"")) != NULL) {
								if (sscanf(ptr2, "\"dim1\":\"%f\"", &temp_float)
										== 1) {
									snd_tank_sett(&newParameter, temp_float,
											SP_d1_tnk_VT_HT_diam_RT_width,
											src_web);
								}
							}

							if ((ptr2 = strstr(ptr, "\"dim2\"")) != NULL) {
								if (sscanf(ptr2, "\"dim2\":\"%f\"", &temp_float)
										== 1) {
									snd_tank_sett(&newParameter, temp_float,
											SP_d2_tnk_VT_height_HT_lenght_RT_height,
											src_web);
								}
							}
							if ((ptr2 = strstr(ptr, "\"dim3\"")) != NULL) {
								if (sscanf(ptr2, "\"dim3\":\"%f\"", &temp_float)
										== 1) {
									snd_tank_sett(&newParameter, temp_float,
											SP_d3_tnk_RT_length, src_web);
								}
							}
						}
						//---------Parámetro: scf -------------------------------
						else if (!strcmp(ptr, "\"scf\"")) {
							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"wr":"10"
							if (sscanf(ptr, "\"cf\":\"%f\"", &temp_float)
									== 1) {
								snd_head_flow_sett(&newParameter, temp_float,
										SP_CF, src_web, 0);
							}
						}
						//---------Parámetro: srp -------------------------------
						else if (!strcmp(ptr, "\"srp\"")) {
							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//"wr":"10"
							if (sscanf(ptr, "\"rp\":\"%f\"", &temp_float)
									== 1) {

								if (temp_float < 1)
									temp_float = 1;
								else if (temp_float > 120)
									temp_float = 120;

								SP.mqtt_ref_rate = temp_float;//El parámetro viene en segundos

								//Almacenar en EEPROM los cambios en SP
								guardar_eeprom((char*) &SP, OFF_SP, sizeof(SP));

								/*Actualizar los valores de ID*/
								mqtt_st = mqtt_ev_publish_sett;
								xQueueSend(queMQTT_ev, &mqtt_st, 0);
							}
						}

						//---------Comando: idpmp -------------------------------
						else if (!strcmp(ptr, "\"idpmp\"")) {
							/*Actualizar los valores de ID*/
							mqtt_st = mqtt_ev_publish_id;
							xQueueSend(queMQTT_ev, &mqtt_st, 0);

							ptr = strtok(NULL, "{");	//\"payload\":

							ptr = strtok(NULL, ",");	//{\"tg1\":\"EDGE SA\"
							sscanf(ptr, "\"tg1\":\"%[^\"]s\"", &id_Tag[0]);

							ptr = strtok(NULL, ",");	//\"tg2\":\"--\"
							sscanf(ptr, "\"tg2\":\"%[^\"]s\"", &id_Tag[1]);

							ptr = strtok(NULL, ",");	//\"tg3\":\"--\"
							sscanf(ptr, "\"tg3\":\"%[^\"]s\"", &id_Tag[2]);

							for (i = 0; i < 3; i++) {
								guardar_eeprom(&id_Tag[i][0],
								OFF_ID_TAG1 + i * LEN_ID,
								LEN_ID);
							}
						}
					}
				}
			}

			if (rutear_msg) {

				/*Enviar el contenido del mensaje, alguien lo esta esperando*/
				for (i = 0; i < msg_len; i++) {
					if (xQueueSend(queMsgFromMOBILE, &Buf_MOBILE[i],
							0) != errQUEUE_FULL) {

					}
				}
			}
		} else {
			if (ReadB < SentByTx) {
				/*En este caso recibimos menos datos de los que
				 * mandamos. Por lo tanto envío todos*/
				for (i = 0; i < ReadB; i++) {
					if (xQueueSend(queMsgFromMOBILE, &Buf_MOBILE[i],
							0) != errQUEUE_FULL) {

					}
				}
			} else {
				/*Envio solo la parte que agregó el MOBILE*/
				for (i = SentByTx; i < ReadB; i++) {
					if (xQueueSend(queMsgFromMOBILE, &Buf_MOBILE[i],
							0) != errQUEUE_FULL) {

					}
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/
/* Atención de INT por el UART - Comunicacion c módulo GPRS/GSM+GPS*/
void UART_HANDLER_NAME(void) {
#define LOC_BUFF_SZ	30

	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint32_t Interr;
	static uint8_t ReadB;
	static uint8_t Buffer[LOC_BUFF_SZ];

	Interr = Chip_UART_ReadIntIDReg(UART) & 0xff;
	if ((Interr & UART_IIR_INTID_RDA) || (Interr & UART_IIR_INTID_CTI)) {

		ReadB = Chip_UART_Read(UART, Buffer, LOC_BUFF_SZ);

		if (ReadB > 0) {
			xStreamBufferSendFromISR(strMsgFromMOBILE, Buffer, ReadB,
					&xHigherPriorityTaskWoken);
		} else if (ReadB == 0) {
			Chip_UART_IntDisable(UART, UART_IER_RBRINT);
			Chip_UART_ReadIntIDReg(UART);
			Chip_UART_ReadLineStatus(UART);
			Chip_UART_IntEnable(UART, UART_IER_RBRINT);
		}

	} else if (Interr & UART_IIR_INTID_THRE) {

	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void Create_COMM_MOBILE_Rx_task(void) {
	xTaskCreate(vCOMM_MOBILE_Rx_Task, (char*) "MOBILE Rx TSK", 270,	//todo: guarda que el stack tiene poco espacio
			NULL, Prio_COMM_MOBILE_Rx_Task, &tsk_mobile_rx_handler);
}

void Create_COMM_MOBILE_Tx_task(void) {
	xTaskCreate(vCOMM_MOBILE_Tx_Task, (char*) "MOBILE Tx TSK", 110,
	NULL,
	Prio_COMM_MOBILE_Tx_Task, &tsk_mobile_tx_handler);
}

