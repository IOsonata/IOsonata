/**-------------------------------------------------------------------------
@file	temp_sensor.h

@brief	Generic temperature sensor abstraction.

@author	Hoang Nguyen Hoan
@date	Feb. 12, 2017

@license

MIT

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __TEMP_SENSOR_H__
#define __TEMP_SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

/** @addtogroup Sensors
  * @{
  */

#pragma pack(push, 1)


/// @brief	Temperature sensor data
///
/// Structure defining temperature sensor data
typedef struct __TemperatureSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	int32_t  Temperature;	//!< Temperature in degree C, 2 decimals fixed point
} TempSensorData_t;

#pragma pack(pop)

#ifdef __cplusplus

class TempSensor;

typedef void (*TempDataRdyCb_t)(TempSensor * const pSensor, TempSensorData_t *pData);

#pragma pack(push, 4)

/// @brief	Temperature sensor configuration
///
typedef struct __TempSensor_Config {
	uint32_t DevAddr;			//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE OpMode;		//!< Operating mode
	uint32_t Freq;				//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	uint32_t IntPrio;			//!< Interrupt priority
	bool bIntEn;				//!< Interrupt enable
	int	TempOvrs;				//!< Oversampling measurement for temperature
	uint32_t FilterCoeff;		//!< Filter coefficient select value (this value is device dependent)
	TempDataRdyCb_t DataRdyCB;	//!< Data ready handler
} TempSensorCfg_t;

#pragma pack(pop)


/// Temperature sensor base class.  Sensor implementation must derive form this class
class TempSensor : public Sensor {
public:

	TempSensor() { Type(SENSOR_TYPE_TEMP); }

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	CfgData : Reference to configuration data
	 * @param	pIntrf 	: Pointer to interface to the sensor.
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Pointer to timer for retrieval of time stamp
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read temperature data.
	 *
	 * Read temperature value from the last UpdateData
	 *
	 * @param 	Buff : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(TempSensorData_t &Data) { Data = vData; }

	/**
	 * @brief	Read temperature (require implementation).
	 *
	 * @return	Temperature in degree C
	 */
	virtual float ReadTemperature() { return (float)vData.Temperature / 100.0; }

protected:

	TempSensorData_t vData;				//!< Last measured data
	TempDataRdyCb_t vDataRdyHandler;	//!< Data ready event handler
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __TEMP_SENSOR_H__
