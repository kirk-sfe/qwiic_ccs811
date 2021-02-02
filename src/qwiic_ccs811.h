/******************************************************************************
SparkFunCCS811.h
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

#ifndef __CCS811_H__
#define __CCS811_H__


//Register addresses
#define CSS811_STATUS 0x00
#define CSS811_MEAS_MODE 0x01
#define CSS811_ALG_RESULT_DATA 0x02
#define CSS811_RAW_DATA 0x03
#define CSS811_ENV_DATA 0x05
#define CSS811_NTC 0x06 //NTC compensation no longer supported
#define CSS811_THRESHOLDS 0x10
#define CSS811_BASELINE 0x11
#define CSS811_HW_ID 0x20
#define CSS811_HW_VERSION 0x21
#define CSS811_FW_BOOT_VERSION 0x23
#define CSS811_FW_APP_VERSION 0x24
#define CSS811_ERROR_ID 0xE0
#define CSS811_APP_START 0xF4
#define CSS811_SW_RESET 0xFF

#define CCS811_DEFAULT_ADDRESS 0x5B

//This is the highest level class of the driver.
//
//  class CCS811 inherits the CCS811Core and makes use of the beginCore()
//method through its own begin() method.  It also contains user settings/values.

class CCS811
{
public:
	CCS811(uint8_t address=CCS811_DEFAULT_ADDRESS);

	//Call to check for errors, start app, and set default mode 1
	bool begin();							  

	bool readAlgorithmResults(void);
	bool checkForStatusError(void);
	bool dataAvailable(void);
	bool appValid(void);
	uint8_t getErrorRegister(void);
	uint16_t getBaseline(void);
	bool setBaseline(uint16_t);
	bool enableInterrupts(void);
	bool disableInterrupts(void);
	bool setDriveMode(uint8_t mode);
	bool setEnvironmentalData(float relativeHumidity, float temperature);
	void setRefResistance(float); //Unsupported feature. Refer to CPP file for more information.
	bool readNTC(void); //Unsupported feature. Refer to CPP file for more information.
	uint16_t getTVOC(void);
	uint16_t getCO2(void);
	float getResistance(void); //Unsupported feature. Refer to CPP file for more information.
	float getTemperature(void); //Unsupported feature. Refer to CPP file for more information.

private:
	//These are the air quality values obtained from the sensor
	float refResistance; //Unsupported feature. Refer to CPP file for more information.
	float resistance; //Unsupported feature. Refer to CPP file for more information.
	uint16_t tVOC;
	uint16_t CO2;
	uint16_t vrefCounts = 0;
	uint16_t ntcCounts = 0;
	float temperature;

	uint8_t I2CAddress;
};

#endif // End of definition check
