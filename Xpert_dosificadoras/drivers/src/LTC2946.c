/*!
 LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor

 @verbatim

 The LTC®2946 is a rail-to-rail system monitor that measures
 current, voltage, power, charge and energy. It features an
 operating range of 2.7V to 100V and includes a shunt regulator
 for supplies above 100V. The current measurement common mode
 range of 0V to 100V is independent of the input supply.
 A 12-bit ADC measures load current, input voltage and an
 auxiliary external voltage. Load current and internally
 calculated power are integrated over an external clock or
 crystal or internal oscillator time base for charge and energy.
 An accurate time base allows the LTC2946 to provide measurement
 accuracy of better than ±0.6% for charge and ±1% for power and
 energy. Minimum and maximum values are stored and an overrange
 alert with programmable thresholds minimizes the need for software
 polling. Data is reported via a standard I2C interface.
 Shutdown mode reduces power consumption to 15uA.

 @endverbatim

 http://www.linear.com/product/LTC2946

 http://www.linear.com/product/ltc2946#demoboards

 REVISION HISTORY
 $Revision: 2041 $
 $Date: 2013-10-15 16:22:28 -0700 (Tue, 15 Oct 2013) $

 Copyright (c) 2013, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

 The Linear Technology Linduino is not affiliated with the official Arduino team.
 However, the Linduino is only possible because of the Arduino team's commitment
 to the open-source community.  Please, visit http://www.arduino.cc and
 http://store.arduino.cc , and consider a purchase that will help fund their
 ongoing work.
 */

//! @defgroup LTC2946 LTC2946: 12-Bit Wide Range Power, Charge and Energy Monitor
/*! @file
 @ingroup LTC2946
 Library for LTC2946 12-Bit Wide Range Power, Charge and Energy Monitor
 */


#include <stdint.h>
#include "LTC2946.h"
#include "board.h"

//static I2C_XFER_T xfer;

//// Write an 8-bit code to the LTC2946.
//int8_t LTC2946_write(I2C_ID_T port, uint8_t i2c_address, uint8_t adc_command,
//		uint8_t code)
//// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
//{
//
//	return Chip_I2C_MasterSend(port, i2c_address, NULL, 1);
//}

// Write an 8-bit code to the LTC2946.
int8_t LTC2946_write(I2C_ID_T port, uint8_t i2c_address, uint8_t adc_command,
		uint8_t code) {
	uint8_t buffer[2];
	int Escritos;

	buffer[0] = adc_command;
	buffer[1] = code;

	Escritos = Chip_I2C_MasterSend(port, i2c_address, buffer, 2);

	return Escritos;
}

// Write a 16-bit code to the LTC2946.
int8_t LTC2946_write_16_bits(uint8_t i2c_address, uint8_t adc_command,
		uint16_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
	int8_t ack;

	ack = 1; //i2c_write_word_data(i2c_address, adc_command, code);
	return (ack);
}

// Write a 24-bit code to the LTC2946.
int8_t LTC2946_write_24_bits(uint8_t i2c_address, uint8_t adc_command,
		uint32_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
//	int8_t ack;

//  LT_union_int32_4bytes data;
//  data.LT_int32 = code;
//
//  ack = i2c_write_block_data(i2c_address, adc_command, (uint8_t) 3, data.LT_byte);

	return 1;
}

int8_t LTC2946_write_32_bits(uint8_t i2c_address, uint8_t adc_command,
		uint32_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
//	int8_t ack;

//  LT_union_int32_4bytes data;
//  data.LT_int32 = code;
//
//  ack = i2c_write_block_data(i2c_address, adc_command, (uint8_t) 4, data.LT_byte);

	return 1; //(ack);
}

// Reads an 8-bit adc_code from LTC2946
int8_t LTC2946_read(I2C_ID_T port, uint8_t i2c_address, uint8_t adc_command,
		uint8_t *adc_code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
	int Leidos;

	Leidos = Chip_I2C_MasterCmdRead(port, i2c_address, adc_command, adc_code,
			(uint8_t) 1);

	return Leidos;
}

// Reads a 12-bit adc_code from LTC2946
int8_t LTC2946_read_12_bits(I2C_ID_T port, uint8_t i2c_address,
		uint8_t adc_command, uint16_t *adc_code) {
	int Leidos;
//	int Escritos;

	volatile uint8_t buffer[4];

	Leidos = Chip_I2C_MasterCmdRead(port, i2c_address, adc_command,
			(uint8_t*) buffer, (uint8_t) 2);
	*adc_code = 0xFFFF & ((buffer[0] << 8) + (buffer[1]));
	*adc_code >>= 4;

	return Leidos;
}

// Reads a 16-bit adc_code from LTC2946
int8_t LTC2946_read_16_bits(I2C_ID_T port, uint8_t i2c_address,
		uint8_t adc_command, uint16_t *adc_code)
{
	int Leidos;
	volatile uint8_t buffer[4];

	Leidos = Chip_I2C_MasterCmdRead(port, i2c_address, adc_command,
			(uint8_t*) &buffer[0], (uint8_t) 2);
	*adc_code = 0xFFFF & ((buffer[0] << 8) + (buffer[1]));

	return Leidos;
}

// Reads a 24-bit adc_code from LTC2946
int8_t LTC2946_read_24_bits(I2C_ID_T port, uint8_t i2c_address,
		uint8_t adc_command, uint32_t *adc_code) {
	uint8_t buffer[4];
	int Leidos;

	Leidos = Chip_I2C_MasterCmdRead(port, i2c_address, adc_command, buffer,
			(uint8_t) 3);
	*adc_code = 0x0FFFFFF
			& ((buffer[0] << 16) + (buffer[1] << 8) + (buffer[2]));
	return Leidos;
}

// Reads a 32-bit adc_code from LTC2946
int8_t LTC2946_read_32_bits(I2C_ID_T port, uint8_t i2c_address,
		uint8_t adc_command, uint32_t *adc_code)
{
	volatile uint8_t buffer[4];
	int Leidos;

	Leidos = Chip_I2C_MasterCmdRead(port, i2c_address, adc_command,
			(uint8_t*) buffer, (uint8_t) 4);
	*adc_code = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8)
			+ (buffer[3]);
	return Leidos;
}

// Calculate the LTC2946 VIN voltage
float LTC2946_VIN_code_to_voltage(uint16_t adc_code, float LTC2946_VIN_lsb)
// Returns the VIN Voltage in Volts
{
	float voltage;
	voltage = (float) adc_code * LTC2946_VIN_lsb; //! 1) Calculate voltage from code and lsb
	return (voltage);
}

// Calculate the LTC2946 ADIN voltage
float LTC2946_ADIN_code_to_voltage(uint16_t adc_code, float LTC2946_ADIN_lsb)
// Returns the ADIN Voltage in Volts
{
	float adc_voltage;
	adc_voltage = (float) adc_code * LTC2946_ADIN_lsb; //! 1) Calculate voltage from code and ADIN lsb
	return (adc_voltage);
}

// Calculate the LTC2946 current with a sense resistor
float LTC2946_code_to_current(uint16_t adc_code, float resistor,
		float LTC2946_DELTA_SENSE_lsb)
// Returns the LTC2946 current in Amps
{
	float voltage, current;
	voltage = (float) adc_code * LTC2946_DELTA_SENSE_lsb; //! 1) Calculate voltage from ADC code and delta sense lsb
	current = voltage / resistor;      //! 2) Calculate current, I = V/R
	return (current);
}

// Calculate the LTC2946 power
float LTC2946_code_to_power(int32_t adc_code, float resistor,
		float LTC2946_Power_lsb)
// Returns The LTC2946 power in Watts
{
	float power;
	power = (float) adc_code * LTC2946_Power_lsb / resistor; //! 1) Calculate Power using Power lsb and resistor

	return (power);
}

// Calculate the LTC2946 energy
float LTC2946_code_to_energy(int32_t adc_code, float resistor,
		float LTC2946_Power_lsb, float LTC2946_TIME_lsb)
// Returns the LTC2946 energy in Joules
{
	float energy_lsb, energy;
	energy_lsb = (float) (LTC2946_Power_lsb / resistor) * 65536
			* LTC2946_TIME_lsb; //! 1) Calculate Energy lsb from Power lsb and Time lsb
	energy = adc_code * energy_lsb; //! 2) Calculate Energy using Energy lsb and adc code
	return (energy);
}

// Calculate the LTC2946 Coulombs
float LTC2946_code_to_coulombs(int32_t adc_code, float resistor,
		float LTC2946_DELTA_SENSE_lsb, float LTC2946_TIME_lsb)
// Returns the LTC2946 Coulombs
{
	float coulomb_lsb, coulombs;
	coulomb_lsb = (float) (LTC2946_DELTA_SENSE_lsb / resistor) * 16
			* LTC2946_TIME_lsb; //! 1) Calculate Coulomb lsb Current lsb and Time lsb
	coulombs = adc_code * coulomb_lsb; //! 2) Calculate Coulombs using Coulomb lsb and adc code
	return (coulombs);
}

//Calculate the LTC2946 Time in Seconds
float LTC2946_code_to_time(float time_code, float LTC2946_TIME_lsb) {
	float seconds;
	seconds = LTC2946_TIME_lsb * time_code;
	return seconds;
}

