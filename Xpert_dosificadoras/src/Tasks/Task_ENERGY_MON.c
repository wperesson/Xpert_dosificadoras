/*
 * TaskINPUT.c
 *
 *  Created on: 09/03/2014
 *      Author: admin
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LTC2946.h"

#include "Definiciones.h"

#define CONFIG_LTC2946_REG_A()	LTC2946_write(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLA_REG, LTC2946_CHANNEL_CONFIG_A_V_C_3 | LTC2946_VDD | LTC2946_OFFSET_CAL_EVERY | LTC2946_ADIN_GND) //! Sets the LTC2946 to continuous mode

#define CONFIG_LTC2946_REG_B()	LTC2946_write(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLB_REG, (LTC2946_DISABLE_ALERT_CLEAR & LTC2946_DISABLE_SHUTDOWN & LTC2946_DISABLE_CLEARED_ON_READ & LTC2946_DISABLE_STUCK_BUS_RECOVER & LTC2946_DISABLE_AUTO_RESET) | LTC2946_ENABLE_ACC) //! Sets the LTC2946 to continuous mode

#define RST_ACC_LTC2946_REG_B()	LTC2946_write(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLB_REG, (LTC2946_DISABLE_ALERT_CLEAR & LTC2946_DISABLE_SHUTDOWN & LTC2946_DISABLE_CLEARED_ON_READ & LTC2946_DISABLE_STUCK_BUS_RECOVER) | LTC2946_RESET_ACC | LTC2946_ENABLE_ACC) //! Sets the LTC2946 to continuous mode

// LSB Weights
const float LTC2946_DELTA_SENSE_lsb = 2.5006105E-05; //!< Typical Delta lsb weight in volts
const float LTC2946_ADIN_lsb = 5.001221E-04; //!< Typical ADIN lsb weight in volts
const float LTC2946_VIN_lsb = 2.5006105E-02; //!< Typical VIN lsb weight in volts
const float LTC2946_Power_lsb = 6.25305E-07; //!< Typical POWER lsb weight in V^2 VIN_lsb * DELTA_SENSE_lsb
const float LTC2946_ADIN_DELTA_SENSE_lsb = 1.25061E-08; //!< Typical sense lsb weight in V^2  *ADIN_lsb * DELTA_SENSE_lsb
const float LTC2946_INTERNAL_TIME_lsb = 4101.00 / 250000.00; //!< Internal TimeBase lsb. Use LTC2946_TIME_lsb if an external CLK is used. See Settings menu for how to calculate Time LSB.

static float LTC2946_TIME_lsb = 16.39543E-3; //!< Static variable which is based off of the default clk frequency of 250KHz.

static portTASK_FUNCTION(vENERGY_MON_Task,pvParameters) {
	portTickType xLastWakeTime;

	uint8_t Temp_code;
	uint16_t Adin_code, current_code, VIN_code;
	uint32_t powercode, charge_code, energy_code;
//	float voltage, energy, energy_lsb, coulomb_lsb, coulombs, Adin;
	float resistor;
	uint32_t sec_counter;
	RTC_TIME_T now_time;

	xLastWakeTime = xTaskGetTickCount();

	resistor = 0.005;
	sec_counter = 0;
	vTaskDelay(2);
	while (xSemaphoreTake(mtxI2C1, 10) != pdTRUE) {
	}
	LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLA_REG, &Temp_code);
	LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLB_REG, &Temp_code);

//	LTC2946_write(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLA_REG, //Write CTRLA Reg
//			LTC2946_CHANNEL_CONFIG_A_V_C_3 |
//			LTC2946_VDD |
//			LTC2946_OFFSET_CAL_EVERY |
//			LTC2946_ADIN_GND); //! Sets the LTC2946 to continuous mode
//
//	LTC2946_write(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLB_REG, //Write CTRLB Reg
//			LTC2946_DISABLE_ALERT_CLEAR &
//			LTC2946_DISABLE_SHUTDOWN &
//			LTC2946_DISABLE_CLEARED_ON_READ &
//			LTC2946_DISABLE_STUCK_BUS_RECOVER &
//			LTC2946_DISABLE_AUTO_RESET |
//			LTC2946_ENABLE_ACC); //! Sets the LTC2946 to continuous mode

	CONFIG_LTC2946_REG_A();
	CONFIG_LTC2946_REG_B();
	xSemaphoreGive(mtxI2C1);

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));

		if (xSemaphoreTake(mtxI2C1, 10) == pdTRUE) {

			//Leo los registros para controlar
//			LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLA_REG,
//					&Temp_code);
//			LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CTRLB_REG,
//					&Temp_code);
//			LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_ADIN_MSB_REG,
//					&Temp_code);
//			LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_ADIN_LSB_REG,
//					&Temp_code);
//			//mido la frecuencia de conversion (Clock Divider Command)
//			LTC2946_read(I2C1, LTC2946_I2C_ADDRESS, LTC2946_CLK_DIV_REG,
//					&Temp_code);

			//Read ADIN connected to EXT GENERATOR
			if (LTC2946_read_12_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_ADIN_MSB_REG, &Adin_code) == 2) { // Reads the voltage code across sense resistor
				Var.VExtGen = (float) Adin_code * 14.7 * LTC2946_ADIN_lsb; //! 1) Calculate voltage from ADC code and delta sense lsb
			}

			//Read CURRENT with a sense resistor
			if (LTC2946_read_12_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_DELTA_SENSE_MSB_REG, &current_code) == 2) { // Reads the voltage code across sense resistor
				Var.CPanel = (25e-6 / resistor) * (float) current_code;
				//Calculate VOLTAGE at BATTERY
				Var.VBatt = Var.VPanel - Var.CPanel * resistor;
			}

			//Read VOLTAGE at Pannel
			if (LTC2946_read_12_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_VIN_MSB_REG, &VIN_code) == 2) { // Reads VIN voltage code
				Var.VPanel = 25e-3 * (float) VIN_code;
			}

			//Read POWER at Pannel
			if (LTC2946_read_24_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_POWER_MSB2_REG, &powercode) == 3) {
				Var.PPanel = (25e-6 / resistor) * 25e-3 * powercode;
			}

			//Read ENERGY delivered by Pannel
			if (LTC2946_read_32_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_ENERGY_MSB3_REG, &energy_code) == 4) { // Reads energy code
				Var.EPannel = (LTC2946_TIME_lsb * 65536 * (25e-6 / resistor)
						* 25e-3 * (float) energy_code) / 1000; //Energy in KiloJoules
			}
			/*
			 * Resetear la energia entregada por el panel a la medianoche
			 *
			 */
			if (sec_counter == 0) {
				Chip_RTC_GetFullTime(LPC_RTC, &now_time);
				if ((now_time.time[RTC_TIMETYPE_HOUR]
						+ now_time.time[RTC_TIMETYPE_MINUTE]
						+ now_time.time[RTC_TIMETYPE_SECOND]) < 2) {
					//Estamos a medianoche
					RST_ACC_LTC2946_REG_B();

					//Vuelvo a dejar los valores originales
					CONFIG_LTC2946_REG_A();
					CONFIG_LTC2946_REG_B();
					sec_counter = 3600 * 24; //Cargo de nuevo el contador con los segundos de un dia entero
				} else {
					//El sec_counter no estaba inicializado, por lo tanto lo cargo con el valor de segundos actuales
					sec_counter = now_time.time[RTC_TIMETYPE_HOUR] * 3600
							+ now_time.time[RTC_TIMETYPE_MINUTE] * 60
							+ now_time.time[RTC_TIMETYPE_SECOND];
				}
			} else if (sec_counter > 0) {
				sec_counter--;
			}

			//Read CHARGE delivered by Pannel
			if (LTC2946_read_32_bits(I2C1, LTC2946_I2C_ADDRESS,
			LTC2946_CHARGE_MSB3_REG, &charge_code) == 4) { // Reads charge code
				Var.ChPannel = (25e-6 / resistor) * 16 * LTC2946_TIME_lsb
						* (float) charge_code;
			}

			xSemaphoreGive(mtxI2C1);
		}
	}
}

void Create_ENERGY_MON_task(void) {
	xTaskCreate(vENERGY_MON_Task, (char *) "ENERGY MON TSK", 128,
	NULL, Prio_EN_MON_Task, &tsk_energy_mon_handler);
}

