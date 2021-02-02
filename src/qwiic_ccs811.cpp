/******************************************************************************
SparkFunCCS811.cpp
CCS811 Arduino library

Marshall Taylor @ SparkFun Electronics
Nathan Seidle @ SparkFun Electronics

April 4, 2017

https://github.com/sparkfun/CCS811_Air_Quality_Breakout
https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

Resources:
Uses Wire.h for i2c operation

Development environment specifics:
Arduino IDE 1.8.1

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

//See SparkFunCCS811.h for additional topology notes.

#include <stdio.h>
#include <math.h>
#include "sparkfun/qwiic_i2c.h"
#include "sparkfun/qwiic_ccs811.h"

Qwiic_I2C qwiic;

//****************************************************************************//
CCS811::CCS811(uint8_t address)
{
	refResistance = 10000; //Unsupported feature. 
	resistance = 0; //Unsupported feature. 
	temperature = 0;
	tVOC = 0;
	CO2 = 0;

	I2CAddress = address;
}

//****************************************************************************//
//
//  Begin
//
//  This starts the lower level begin, then applies settings
//
//****************************************************************************//
bool CCS811::begin()
{
	uint8_t data[4] = {0x11, 0xE5, 0x72, 0x8A};					   //Reset key

	// init I2C.

	if(!qwiic.init()){
		printf("Error: I2C subsystem failed to initialize.");
		return false;
	}
	sleep_ms(50);

	//Check communication with IC before anything else
	uint8_t chipID = qwiic.readRegister(I2CAddress, CSS811_HW_ID); 

	if(chipID != 0x81){
	  	printf("Invalid chip id. \n");
		return false; //Not the target chip
	}

	
	int rc = qwiic.writeRegisterRegion(I2CAddress, CSS811_SW_RESET, data, 4);
	if(rc == QWIIC_ERROR_GENERIC){
		printf("CCS811 - Error resetting the device on startup.\n");
		return false;
	}

	// let the sensor setup.
	sleep_ms(50);

	if (checkForStatusError() == true){
		printf("CCS811 Error - bad status after reset\n");
		return false;
	}

	if (appValid() == false){
		printf("CCS811 Error - Invalid application\n");		
		return false;
	}

	//Write 0 bytes to this register to start app
	qwiic.write(I2CAddress, CSS811_APP_START);

	return setDriveMode(1); //Read every second

}

//****************************************************************************//
//
//  Sensor functions
//
//****************************************************************************//
//Updates the total voltatile organic compounds (TVOC) in parts per billion (PPB)
//and the CO2 value
//Returns nothing
bool CCS811::readAlgorithmResults(void)
{
	uint8_t data[4];

	qwiic.readRegisterRegion(I2CAddress, CSS811_ALG_RESULT_DATA, data, 4 );

	// Data ordered:
	// co2MSB, co2LSB, tvocMSB, tvocLSB

	CO2 = ((uint16_t)data[0] << 8) | data[1];
	tVOC = ((uint16_t)data[2] << 8) | data[3];
	return true;
}

//Checks to see if error bit is set
bool CCS811::checkForStatusError(void)
{
	uint8_t value;
	//return the status bit
	value = qwiic.readRegister(I2CAddress, CSS811_STATUS);

	return (value & 1 << 0);
}

//Checks to see if DATA_READ flag is set in the status register
bool CCS811::dataAvailable(void)
{
	uint8_t value;
	
	value = qwiic.readRegister(I2CAddress, CSS811_STATUS);

	return (value & 1 << 3);

}

//Checks to see if APP_VALID flag is set in the status register
bool CCS811::appValid(void)
{

	uint8_t value = qwiic.readRegister(I2CAddress, CSS811_STATUS);	

	return (value & 1 << 4);

}

uint8_t CCS811::getErrorRegister(void)
{

	return 	qwiic.readRegister(I2CAddress, CSS811_ERROR_ID);	

}

//Returns the baseline value
//Used for telling sensor what 'clean' air is
//You must put the sensor in clean air and record this value
uint16_t CCS811::getBaseline(void)
{
	uint8_t data[2];

	qwiic.readRegisterRegion(I2CAddress, CSS811_BASELINE, data, 2);

	return ((uint16_t)data[0] << 8) | data[1];

}

bool CCS811::setBaseline(uint16_t input)
{
	uint8_t data[2];
	data[0] = (input >> 8) & 0x00FF;
	data[1] = input & 0x00FF;

	qwiic.writeRegisterRegion(I2CAddress, CSS811_BASELINE, data, 2);

	return true;
}

//Enable the nINT signal
bool CCS811::enableInterrupts(void)
{
	uint8_t value = qwiic.readRegister(I2CAddress, CSS811_MEAS_MODE);


	value |= (1 << 3); //Set INTERRUPT bit
	qwiic.writeRegister(I2CAddress, CSS811_MEAS_MODE, value);

	return true;
}

//Disable the nINT signal
bool CCS811::disableInterrupts(void)
{
	uint8_t value = qwiic.readRegister(I2CAddress, CSS811_MEAS_MODE);	

	value &= ~(1 << 3); //Clear INTERRUPT bit

	qwiic.writeRegister(I2CAddress, CSS811_MEAS_MODE, value);

	return true;
}

//Mode 0 = Idle
//Mode 1 = read every 1s
//Mode 2 = every 10s
//Mode 3 = every 60s
//Mode 4 = RAW mode
bool CCS811::setDriveMode(uint8_t mode)
{
	if (mode > 4)
		mode = 4; //sanitize input

	uint8_t value = qwiic.readRegister(I2CAddress, CSS811_MEAS_MODE);	

	value &= ~(0b00000111 << 4); //Clear DRIVE_MODE bits
	value |= (mode << 4);		 //Mask in mode
	
	qwiic.writeRegister(I2CAddress, CSS811_MEAS_MODE, value);
	return true;

}

//Given a temp and humidity, write this data to the CSS811 for better compensation
//This function expects the humidity and temp to come in as floats
bool CCS811::setEnvironmentalData(float relativeHumidity, float temperature)
{
	//Check for invalid temperatures
	if ((temperature < -25) || (temperature > 50))
		return false;

	//Check for invalid humidity
	if ((relativeHumidity < 0) || (relativeHumidity > 100))
		return false;

	uint32_t rH = relativeHumidity * 1000; //42.348 becomes 42348
	uint32_t temp = temperature * 1000;	//23.2 becomes 23200

	uint8_t envData[4];

	//Split value into 7-bit integer and 9-bit fractional

	//Incorrect way from datasheet.
	//envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
	//envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
	//if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
	//{
	//	envData[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
	//}

	//Correct rounding. See issue 8: https://github.com/sparkfun/Qwiic_BME280_CCS811_Combo/issues/8
	envData[0] = (rH + 250) / 500;
	envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero

	temp += 25000; //Add the 25C offset
	//Split value into 7-bit integer and 9-bit fractional
	//envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
	//envData[3] = 0;
	//if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
	//{
	//	envData[2] |= 1;  //Set 9th bit of fractional to indicate 0.5C
	//}

	//Correct rounding
	envData[2] = (temp + 250) / 500;
	envData[3] = 0;

	qwiic.writeRegisterRegion(I2CAddress, CSS811_ENV_DATA, envData, 4);

	return true;
}

uint16_t CCS811::getTVOC(void)
{
	return tVOC;
}

uint16_t CCS811::getCO2(void)
{
	return CO2;
}

//****************************************************************************//
//
//	The CCS811 no longer supports temperature compensation from an NTC thermistor.
//	NTC thermistor compensation will only work on boards purchased in 2017.
//	List of unsupported functions:
//		setRefResistance();
//		readNTC();
//		getResistance();
//		getTemperature();	
//
//****************************************************************************//

void CCS811::setRefResistance(float input)
{
	refResistance = input;
}

bool CCS811::readNTC(void)
{
	uint8_t data[4];

	qwiic.readRegisterRegion(I2CAddress, CSS811_NTC, data, 4);



	vrefCounts = ((uint16_t)data[0] << 8) | data[1];
	//Serial.print("vrefCounts: ");
	//Serial.println(vrefCounts);
	ntcCounts = ((uint16_t)data[2] << 8) | data[3];
	//Serial.print("ntcCounts: ");
	//Serial.println(ntcCounts);
	//Serial.print("sum: ");
	//Serial.println(ntcCounts + vrefCounts);
	resistance = ((float)ntcCounts * refResistance / (float)vrefCounts);

	//Code from Milan Malesevic and Zoran Stupic, 2011,
	//Modified by Max Mayfield,
	temperature = log((long)resistance);
	temperature = 1 / (0.001129148 + (0.000234125 * temperature) + (0.0000000876741 * temperature * temperature * temperature));
	temperature = temperature - 273.15; // Convert Kelvin to Celsius

	return true;
}

float CCS811::getResistance(void)
{
	return resistance;
}

float CCS811::getTemperature(void)
{
	return temperature;
}
