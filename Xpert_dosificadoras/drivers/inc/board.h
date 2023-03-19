#ifndef __BOARD_H_
#define __BOARD_H_

#include "chip.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Define DEBUG_ENABLE to enable IO via the DEBUGSTR, DEBUGOUT, and
 DEBUGIN macros. If not defined, DEBUG* functions will be optimized
 out of the code at build time.
 */
#define DEBUG_ENABLE

/** Define DEBUG_SEMIHOSTING along with DEBUG_ENABLE to enable IO support
 via semihosting. You may need to use a C library that supports
 semihosting with this option.
 */
// #define DEBUG_SEMIHOSTING
/** Board UART used for debug output and input using the DEBUG* macros. This
 is also the port used for Board_UARTPutChar, Board_UARTGetChar, and
 Board_UARTPutSTR functions.
 */
#define DEBUG_UART LPC_UART1

/**
 * @}
 */

/* Board name */
#define BOARD_NXP_LPCXPRESSO_1769
#define USE_RMII

#define LED_SWITCH		15
#define DI_1			16
#define DI_2			15
#define DI_3			17
#define DI_4			18
#define DI_0			29
#define TACHO_1			26
#define TACHO_2			27
#define CS_TSYS			31
#define CS_DAC			16
#define CS_COMM			17
#define RC_FID1			0
#define PWR_MOT1		15
#define PWR_MOT2		14
#define PWR_REL_1_1		10
#define PWR_REL_1_2		9
#define SGN_O4			8
#define SGN_O3			4
#define SGN_O2			1
#define SGN_O1			0

#define SPI1_CLK		7
#define SCK0			20
#define MISO0			23
#define MOSI0			24
#define SSEL0
#define SCK1			7
#define MISO1			8
#define MOSI1			9
#define TXD1			0
#define RXD1			1
#define TXD2			10
#define RXD2			11
#define TXD3			0
#define RXD3			1
#define TXEN1			5
#define RXEN1			7
#define RST_WPRO		3
//#define SIM868_PWR_KEY		16		//SIM868 PWRKEY
//#define BG96_PWR_KEY		16		//BG96 PWR ON OFF
#define PWR_KEY				16		//BG96 or SIM868 PWR_KEY ON OFF
#define BG96_RST			17		//BG96 nRESET
#define GSM_RI			6
#define GPS_EN			17		//SIM868 GPS ENABLE
#define WHART_CD		7
#define WHART_RT		8
#define WHART_RST		9

#define PWR_MOTOR_1_ON			Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_MOT1,TRUE)
#define PWR_MOTOR_1_OFF			Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_MOT1,FALSE)

#define PWR_MOTOR_2_ON			Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_MOT2,TRUE)
#define PWR_MOTOR_2_OFF			Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_MOT2,FALSE)

#define PWR_RELAY_1_1_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_1,TRUE)//Enciende el Rele K3_1
#define PWR_RELAY_1_1_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_1,FALSE)//Apaga el Rele K3_1

#define PWR_RELAY_1_2_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_2,TRUE)//Enciende el Rele K3_2
#define PWR_RELAY_1_2_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_2,FALSE)//Apaga el Rele K3_2

#define SGN_O1_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O1,TRUE)//Enciende la salida DO1
#define SGN_O1_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O1,FALSE)//Apaga la salida DO1

#define SGN_O2_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O2,TRUE)//Enciende la salida DO2
#define SGN_O2_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O2,FALSE)//Apaga la salida DO2

#define SGN_O3_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O3,TRUE)//Enciende la salida DO3
#define SGN_O3_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O3,FALSE)//Apaga la salida DO3

#define SGN_O4_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O4,TRUE)//Enciende la salida DO4
#define SGN_O4_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O4,FALSE)//Apaga la salida DO4

#define DO_O1_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O1,TRUE)//Enciende la salida DO1
#define DO_O1_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O1,FALSE)//Apaga la salida DO1

#define DO_O2_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O2,TRUE)//Enciende la salida DO2
#define DO_O2_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O2,FALSE)//Apaga la salida DO2

#define DO_O3_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O3,TRUE)//Enciende la salida DO3
#define DO_O3_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O3,FALSE)//Apaga la salida DO3

#define DO_O4_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O4,TRUE)//Enciende la salida DO4
#define DO_O4_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, SGN_O4,FALSE)//Apaga la salida DO4

#define DO_RL1_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_1,TRUE)//Enciende la salida Rl1
#define DO_RL1_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_1,FALSE)//Apaga la salida Rl1

#define DO_RL2_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_2,TRUE)//Enciende la salida Rl2
#define DO_RL2_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_REL_1_2,FALSE)//Apaga la salida Rl2

#define TXEN1_ON				Chip_GPIO_SetPinState(LPC_GPIO,2, TXEN1,TRUE)
#define TXEN1_OFF				Chip_GPIO_SetPinState(LPC_GPIO,2, TXEN1,FALSE)

#define RXEN1_OFF				Chip_GPIO_SetPinState(LPC_GPIO,2, RXEN1,TRUE)
#define RXEN1_ON				Chip_GPIO_SetPinState(LPC_GPIO,2, RXEN1,FALSE)

#define RST_WPRO_OFF			Chip_GPIO_SetPinState(LPC_GPIO,2, RST_WPRO,TRUE)
#define RST_WPRO_ON				Chip_GPIO_SetPinState(LPC_GPIO,2, RST_WPRO,FALSE)

//#define GSM_RST_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_RST,FALSE)
//#define GSM_RST_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_RST,TRUE)

//#define SIM868_PWRKEY_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, SIM868_PWR_KEY,TRUE)
//#define SIM868_PWRKEY_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, SIM868_PWR_KEY,FALSE)
//
//#define BG96_PWRKEY_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, BG96_PWR_KEY,FALSE)
//#define BG96_PWRKEY_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, BG96_PWR_KEY,TRUE)

#define PWRKEY_4G_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_KEY,FALSE)
#define PWRKEY_4G_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, PWR_KEY,TRUE)

#define BG96_RST_ON			Chip_GPIO_SetPinState(LPC_GPIO,1, BG96_RST,FALSE)
#define BG96_RST_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, BG96_RST,TRUE)

#define MOBILE_GPS_EN_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, GPS_EN,TRUE)
#define MOBILE_GPS_EN_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, GPS_EN,FALSE)

#define WHART_RT_ON				Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_RT,FALSE)
#define WHART_RT_OFF			Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_RT,TRUE)

#define WHART_RST_ON			Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_RST,FALSE)
#define WHART_RST_OFF			Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_RST,TRUE)

//#define GSM_KEY_ON				Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_KEY,FALSE)
//#define GSM_KEY_OFF				Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_KEY,TRUE)

//#define SIM868_GPS_EN_ON		Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_KEY,TRUE)
//#define SIM868_GPS_EN_OFF		Chip_GPIO_SetPinState(LPC_GPIO,1, GSM_KEY,FALSE)

#define LPC_WHM_CD					Chip_GPIO_GetPinState(LPC_GPIO,0, WHART_CD)
#define WHART_CD_OFF					Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_CD,FALSE)
#define WHART_CD_ON						Chip_GPIO_SetPinState(LPC_GPIO,0, WHART_CD,TRUE)
/**
 * @brief	Initialize pin muxing for a UART
 * @param	pUART	: Pointer to UART register block for UART pins to init
 * @return	Nothing
 */
void Board_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Returns the MAC address assigned to this board
 * @param	mcaddr : Pointer to 6-byte character array to populate with MAC address
 * @return	Nothing
 * @note    Returns the MAC address used by Ethernet
 */
void Board_ENET_GetMacADDR(uint8_t *mcaddr);

/**
 * @brief	Initialize pin muxing for SSP interface
 * @param	pSSP	: Pointer to SSP interface to initialize
 * @return	Nothing
 */
void Board_SSP_Init(LPC_SSP_T *pSSP);

/**
 * @brief	Initialize pin muxing for SPI interface
 * @param	isMaster	: true for master mode, false for slave mode
 * @return	Nothing
 */
void Board_SPI_Init(bool isMaster);

/**
 * @brief	Assert SSEL pin
 * @return	Nothing
 */
void Board_SPI_AssertSSEL(void);

/**
 * @brief	De-assert SSEL pin
 * @return	Nothing
 */
void Board_SPI_DeassertSSEL(void);

/**
 * @brief	Sets up board specific I2C interface
 * @param	id	: ID of I2C peripheral
 * @return	Nothing
 */
void Board_I2C_Init(I2C_ID_T id);

/**
 * @brief	Sets up I2C Fast Plus mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @note	This function must be called before calling
 *          Chip_I2C_SetClockRate() to set clock rates above
 *          normal range 100KHz to 400KHz. Only I2C0 supports
 *          this mode.
 */
STATIC INLINE void Board_I2C_EnableFastPlus(I2C_ID_T id) {
	Chip_IOCON_SetI2CPad(LPC_IOCON, I2CPADCFG_FAST_MODE_PLUS);
}

/**
 * @brief	Disables I2C Fast plus mode and enable normal mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 */
STATIC INLINE void Board_I2C_DisableFastPlus(I2C_ID_T id) {
	Chip_IOCON_SetI2CPad(LPC_IOCON, I2CPADCFG_STD_MODE);
}

/**
 * @brief	Initialize input and outputs on the board
 * @return	Nothing
 */
void Board_IO_Init(void);

/**
 * @brief	Create Serial Stream
 * @param	Stream	: Pointer to stream
 * @return	Nothing
 */
void Serial_CreateStream(void *Stream);

/**
 * @}
 */

#include "board_api.h"
#include "lpc_phy.h"

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H_ */
