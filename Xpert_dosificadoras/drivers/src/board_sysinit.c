/*
 * @brief NXP LPC1769 LPCXpresso Sysinit file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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

/* The System initialization code is called prior to the application and
 initializes the board for run-time operation. Board initialization
 includes clock setup and default pin muxing configuration. */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Pin muxing configuration */
STATIC const PINMUX_GRP_T pinmuxing[] = {
		{ 0, 0, IOCON_MODE_INACT | IOCON_FUNC0 }, /* NC */
		{ 0, 1, IOCON_MODE_INACT | IOCON_FUNC0 }, /* NC */
		{ 0, 2, IOCON_MODE_INACT | IOCON_FUNC1 }, /* TXD0 */
		{ 0, 3, IOCON_MODE_INACT | IOCON_FUNC1 }, /* RXD0 */
		{ 0, 4, IOCON_MODE_INACT | IOCON_FUNC3 }, /* CAN2_RX */
		{ 0, 5, IOCON_MODE_INACT | IOCON_FUNC3 }, /* CAN2_TX */
		{ 0, 6, IOCON_MODE_INACT | IOCON_FUNC0 }, /* GSM_RI*/
		{ 0, 7, IOCON_MODE_INACT | IOCON_FUNC0 }, /* WHART_CD */
		{ 0, 8, IOCON_MODE_INACT | IOCON_FUNC0 }, /* WHART_RT */
		{ 0, 9, IOCON_MODE_INACT | IOCON_FUNC0 }, /* WHRT_RST */
		{ 0, 10, IOCON_MODE_INACT | IOCON_FUNC1 }, /* TXD2 */
		{ 0, 11, IOCON_MODE_PULLUP | IOCON_FUNC1 }, /* RXD2 */
		{ 0, 15, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DI_2 */
		{ 0, 16, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DI_1 */
		{ 0, 17, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DI_3 */
		{ 0, 18, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DI_4 */
		{ 0, 19, IOCON_MODE_INACT | IOCON_FUNC3 }, /* SDA1 */
		{ 0, 20, IOCON_MODE_INACT | IOCON_FUNC3 }, /* SCL1 */
		{ 0, 21, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 0, 22, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 0, 23, IOCON_MODE_INACT | IOCON_FUNC1 }, /* AN_4 */
		{ 0, 24, IOCON_MODE_PULLDOWN | IOCON_FUNC1 }, /* AN_3 */
		{ 0, 25, IOCON_MODE_INACT | IOCON_FUNC1 }, /* AN_2 */
		{ 0, 26, IOCON_MODE_INACT | IOCON_FUNC1 }, /* AN_1*/
		{ 0, 27, IOCON_MODE_PULLUP | IOCON_FUNC1 }, /* SDA0*/
		{ 0, 28, IOCON_MODE_PULLUP | IOCON_FUNC1 }, /* SCL0*/
		{ 0, 29, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DI_5 */
		{ 0, 30, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 1, 0, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DO_1*/
		{ 1, 1, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DO_2*/
		{ 1, 4, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DO_3*/
		{ 1, 8, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* DO_4*/
		{ 1, 9, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* PWR_REL_1.2*/
		{ 1, 10, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* PWR_REL_1.1*/
		{ 1, 14, IOCON_MODE_PULLDOWN | IOCON_FUNC0 }, /* PWR_MOTOR_2*/
		{ 1, 15, IOCON_MODE_PULLDOWN | IOCON_FUNC0 }, /* PWR_MOTOR_1*/
		{ 1, 16, IOCON_MODE_INACT | IOCON_FUNC0 }, /* PWR_KEY */
		{ 1, 17, IOCON_MODE_INACT | IOCON_FUNC0 }, /* GPS_EN or BG96_RST*/
		{ 1, 18, IOCON_MODE_PULLDOWN | IOCON_FUNC0 }, /* NC */
		{ 1, 19, IOCON_MODE_PULLDOWN | IOCON_FUNC0 }, /* NC */
		{ 1, 20, IOCON_MODE_INACT | IOCON_FUNC3 }, /* SCK0 */
		{ 1, 21, IOCON_MODE_INACT | IOCON_FUNC3 }, /* SSEL0 */
		{ 1, 22, IOCON_MODE_INACT | IOCON_FUNC0 }, /* CS_HMI*/
		{ 1, 23, IOCON_MODE_INACT | IOCON_FUNC0 }, /* MISO0 */
		{ 1, 24, IOCON_MODE_INACT | IOCON_FUNC0 }, /* MOSI0 */
		{ 1, 25, IOCON_MODE_INACT | IOCON_FUNC0 }, /* NC */
		{ 1, 26, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* TACHO_MOT_1*/
		{ 1, 27, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* TACHO_MOT_2*/
		{ 1, 28, IOCON_MODE_INACT | IOCON_FUNC0 }, /* NC */
		{ 1, 29, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 1, 30, IOCON_MODE_INACT | IOCON_FUNC3 }, /* AN_6*/
		{ 1, 31, IOCON_MODE_INACT | IOCON_FUNC3 }, /* AN_5*/
		{ 2, 0, IOCON_MODE_INACT | IOCON_FUNC2 }, /*  TXD1 */
		{ 2, 1, IOCON_MODE_PULLUP | IOCON_FUNC2 }, /*  RXD1 */
		{ 2, 2, IOCON_MODE_INACT | IOCON_FUNC2 }, /*  CTS1 */
		{ 2, 3, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /*  WPRO_RST */
		{ 2, 4, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 2, 5, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* gpio *//* TXEN1*/
		{ 2, 6, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC*/
		{ 2, 7, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* gpio *//* RTS1 */
		{ 2, 8, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC*/
		{ 2, 9, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC*/
		{ 2, 10, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 2, 11, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 2, 12, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 2, 13, IOCON_MODE_PULLUP | IOCON_FUNC0 }, /* NC */
		{ 3, 25, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* MOT1_PWM */
		{ 3, 26, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* MOT2_PWM */
		{ 4, 28, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* TXD3 */
		{ 4, 29, IOCON_MODE_PULLUP | IOCON_FUNC3 }, /* RXD3 */

};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Sets up system pin muxing */
void Board_SetupMuxing(void) {
	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing,
			sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));
}

/* Setup system clocking */
void Board_SetupClocking(void) {
	Chip_SetupXtalClocking();

	/* Setup FLASH access to 5 clocks (120MHz clock) */
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_120MHZ_CPU);
}

/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void) {
	Board_SetupMuxing();
	Board_SetupClocking();
}
