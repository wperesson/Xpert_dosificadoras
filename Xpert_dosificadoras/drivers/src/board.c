/*
 * @brief NXP LPC1769 LPCXpresso board file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "string.h"
#include "retarget.h"
#include "Definiciones.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* System oscillator rate and RTC oscillator rate */
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize UART pins */
void Board_UART_Init(LPC_USART_T *pUART) {
	/* Pin Muxing has already been done during SystemInit */
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void) {
#if defined(DEBUG_ENABLE)
	Board_UART_Init(DEBUG_UART);

	Chip_UART_Init(DEBUG_UART);
	Chip_UART_SetBaud(DEBUG_UART, 115200);
	Chip_UART_ConfigData(DEBUG_UART,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);

	/* Enable UART Transmit */
	Chip_UART_TXEnable(DEBUG_UART);
#endif
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch) {
#if defined(DEBUG_ENABLE)
	while ((Chip_UART_ReadLineStatus(DEBUG_UART) & UART_LSR_THRE) == 0) {
	}
	Chip_UART_SendByte(DEBUG_UART, (uint8_t) ch);
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void) {
#if defined(DEBUG_ENABLE)
	if (Chip_UART_ReadLineStatus(DEBUG_UART) & UART_LSR_RDR) {
		return (int) Chip_UART_ReadByte(DEBUG_UART);
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str) {
#if defined(DEBUG_ENABLE)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Set up and initialize all required blocks and functions related to the
 board hardware */
void Board_Init(void) {
	/* Sets up DEBUG UART */
//	DEBUGINIT();
	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);
	Board_IO_Init(); // Inicializa los pines como entrada o salida
	Chip_IOCON_Init(LPC_IOCON);

}

/* Returns the MAC address assigned to this board */
void Board_ENET_GetMacADDR(uint8_t *mcaddr) {
	const uint8_t boardmac[] = { 0x00, 0x60, 0x37, 0x12, 0x34, 0x56 };

	memcpy(mcaddr, boardmac, 6);
}

/* Initialize pin muxing for SSP interface */
void Board_SSP_Init(LPC_SSP_T *pSSP) {
	if (pSSP == LPC_SSP1) {
		/* Set up clock and muxing for SSP1 interface */
		/*
		 * Initialize SSP0 pins connect
		 * P0.7: SCK
		 * P0.6: SSEL
		 * P0.8: MISO
		 * P0.9: MOSI
		 */
		Chip_IOCON_PinMux(LPC_IOCON, 0, 7, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 6, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 8, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 9, IOCON_MODE_INACT, IOCON_FUNC2);
	} else {
		/* Set up clock and muxing for SSP0 interface */
		/*
		 * Initialize SSP0 pins connect
		 * P0.15: SCK
		 * P0.16: SSEL - NO USADO
		 * P0.17: MISO
		 * P0.18: MOSI
		 */
		Chip_IOCON_PinMux(LPC_IOCON, 0, 15, IOCON_MODE_INACT, IOCON_FUNC2);
//		Chip_IOCON_PinMux(LPC_IOCON, 0, 16, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 17, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 18, IOCON_MODE_INACT, IOCON_FUNC2);
	}
}

/* Initialize pin muxing for SPI interface */
void Board_SPI_Init(bool isMaster) {
	/* Set up clock and muxing for SSP0 interface */
	/*
	 * Initialize SSP0 pins connect
	 * P0.15: SCK
	 * P0.16: SSEL
	 * P0.17: MISO
	 * P0.18: MOSI
	 */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 15, IOCON_MODE_PULLDOWN, IOCON_FUNC3);
	if (isMaster) {
		Chip_IOCON_PinMux(LPC_IOCON, 0, 16, IOCON_MODE_PULLUP, IOCON_FUNC0);
		Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 16, true);
		Board_SPI_DeassertSSEL();

	} else {
		Chip_IOCON_PinMux(LPC_IOCON, 0, 16, IOCON_MODE_PULLUP, IOCON_FUNC3);
	}
	Chip_IOCON_PinMux(LPC_IOCON, 0, 17, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 18, IOCON_MODE_INACT, IOCON_FUNC3);
}

/* Assert SSEL pin */
void Board_SPI_AssertSSEL(void) {
	Chip_GPIO_WritePortBit(LPC_GPIO, 0, 16, false);
}

/* De-Assert SSEL pin */
void Board_SPI_DeassertSSEL(void) {
	Chip_GPIO_WritePortBit(LPC_GPIO, 0, 16, true);
}

void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn) {
	I2S_AUDIO_FORMAT_T I2S_Config;

	/* Chip_Clock_EnablePeripheralClock(SYSCTL_CLOCK_I2S); */

	I2S_Config.SampleRate = 48000;
	I2S_Config.ChannelNumber = 2; /* 1 is mono, 2 is stereo */
	I2S_Config.WordWidth = 16; /* 8, 16 or 32 bits */
	Chip_I2S_Init(pI2S);
	Chip_I2S_TxConfig(pI2S, &I2S_Config);
}

/* Sets up board specific I2C interface */
void Board_I2C_Init(I2C_ID_T id) {
	switch (id) {
	case I2C0:
		Chip_IOCON_PinMux(LPC_IOCON, 0, 27, IOCON_MODE_INACT, IOCON_FUNC1);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 28, IOCON_MODE_INACT, IOCON_FUNC1);
		Chip_IOCON_SetI2CPad(LPC_IOCON, I2CPADCFG_STD_MODE);
		break;

	case I2C1:
		Chip_IOCON_PinMux(LPC_IOCON, 0, 19, IOCON_MODE_INACT, IOCON_FUNC3);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 20, IOCON_MODE_INACT, IOCON_FUNC3);
		Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);
		Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
		break;

	case I2C2:
		Chip_IOCON_PinMux(LPC_IOCON, 0, 10, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_PinMux(LPC_IOCON, 0, 11, IOCON_MODE_INACT, IOCON_FUNC2);
		Chip_IOCON_EnableOD(LPC_IOCON, 0, 10);
		Chip_IOCON_EnableOD(LPC_IOCON, 0, 11);
		break;
	default:
		break;
	}
}

void Board_IO_Init(void) {
//	/* INTERRUPTOR PANEL FRONTAL ************************************************
//	 * P1.15  - LED_SWITCH
//	 * P0.10  - ON_OFF
//	 */
//	LED_SWITCH_OFF;
//	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, LED_SWITCH, TRUE);
//	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_1, FALSE);

	/* UART1 TXEN, RXEN, RST ************************************************
	 * P2.05 - TXEN1
	 * P2.07 - RXEN1
	 * P2.03 - RST_WPRO
	 * P0.06 - GSM_RI
	 * P1.16 - GSM_RST
	 * P1.17 - GSM_KEY
	 * P0.07 - WHART_CD
	 * P0.08 - WHART_RT
	 */
	TXEN1_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 2, TXEN1, TRUE);
	RXEN1_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 2, RXEN1, TRUE);
	RST_WPRO_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 2, RST_WPRO, TRUE);
//	GSM_KEY_OFF;
//	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, GSM_KEY, TRUE);

#if USE_SIM868
	MOBILE_GPS_EN_OFF;
#elif USE_BG96
	BG96_RST_OFF;
	Chip_IOCON_EnableOD(LPC_GPIO, 1, PWR_KEY);
	Chip_IOCON_EnableOD(LPC_GPIO, 1, GPS_EN);
#endif
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, GPS_EN, TRUE);

	PWRKEY_4G_OFF;
	Chip_IOCON_EnableOD(LPC_GPIO, 1, PWR_KEY);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, PWR_KEY, TRUE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, GSM_RI, FALSE);

//	GSM_RST_OFF;
//	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, GSM_RST, TRUE);
//	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, GSM_RI, FALSE);

	WHART_RT_OFF;
#if !MOTOR_ELEC
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, WHART_CD, FALSE);
#else
	WHART_CD_ON;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, WHART_CD, TRUE);
#endif
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, WHART_RT, TRUE);

	WHART_RST_ON;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, WHART_RST, TRUE);

	/* SEÑALES DE ENTRADA (AN Y DIG) + TACHO MOTORES *********************************
	 * P0.18 - DI_1
	 * P0.17 - DI_2
	 * P0.16 - DI_3
	 * P0.15 - DI_4
	 * P0.29 - DI_5 Open Door
	 * P0.23 - AN_1
	 * P0.24 - AN_2
	 * P0.25 - AN_3
	 * P0.26 - AN_4
	 * P1.30 - AN_5
	 * P1.31 - AN_6
	 * P1.26 - TACHO_MOT_1
	 * P1.27 - TACHO_MOT_2
	 */
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_1, FALSE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_2, FALSE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_3, FALSE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_4, FALSE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, DI_0, FALSE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, TACHO_1, TRUE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, TACHO_2, TRUE);

	/* SEÑALES DE SALIDA + PWR MOTORES + PWM_MOTOR + EXTRA RELAY *********************************
	 * P1.0 - SGN_O1
	 * P1.1 - SGN_O2
	 * P1.4 - SGN_O3
	 * P1.8 - SGN_O4
	 * P1.9 - PWR_REL_1.2
	 * P1.10 - PWR_REL_1.1
	 * P1.14 - PWR_MOTOR_2
	 * P1.15 - PWR_MOTOR_1
	 * P1.16 - POWER KEY GSM MODULE // WHRT_RST
	 * P0.22 - HMI_PWDN
	 * P3.25 - MOT1_PWM
	 * P3.26 - MOT2_PWM
	 */
	SGN_O1_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, SGN_O1, TRUE);
	SGN_O2_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, SGN_O2, TRUE);
	SGN_O3_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, SGN_O3, TRUE);
	SGN_O4_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, SGN_O4, TRUE);
	PWR_MOTOR_1_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, PWR_MOT1, TRUE);
	PWR_MOTOR_2_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, PWR_MOT2, TRUE);
	PWR_RELAY_1_1_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, PWR_REL_1_1, TRUE);
	PWR_RELAY_1_2_OFF;
	Chip_GPIO_WriteDirBit(LPC_GPIO, 1, PWR_REL_1_2, TRUE);

}

void Serial_CreateStream(void *Stream) {
}
