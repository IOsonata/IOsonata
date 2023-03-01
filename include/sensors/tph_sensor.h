/**-------------------------------------------------------------------------
@file	tph_sensor.h

@brief	Generic environment sensor abstraction : Temperature, Pressure, Humidity

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
#ifndef __TPH_SENSOR_H__
#define __TPH_SENSOR_H__

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


/// @brief	TPH sensor data
///
/// Structure defining TPH sensor data
typedef struct __TPHSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	uint32_t Pressure;		//!< Barometric pressure in Pa no decimal
	int16_t  Temperature;	//!< Temperature in degree C, 2 decimals fixed point
	uint16_t Humidity;		//!< Relative humidity in %, 2 decimals fixed point
} TPHSensorData_t;

#pragma pack(pop)

class TphSensor;

typedef void (*TPHDataRdyHandler_t)(TphSensor * const pSensor, TPHSensorData_t *pData);

#pragma pack(push, 4)

/// @brief	TPH sensor configuration
///
typedef struct __TPHSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	int				TempOvrs;		//!< Oversampling measurement for temperature
	int				PresOvrs;		//!< Oversampling measurement for pressure
	int 			HumOvrs;		//!< Oversampling measurement for humidity
	uint32_t		FilterCoeff;	//!< Filter coefficient select value (this value is device dependent)
	TPHDataRdyHandler_t	DataRdyCB;		//!< Callback handler for data ready
} TPHSensorCfg_t;

#pragma pack(pop)

#ifdef __cplusplus

/// TPH sensor base class.  Sensor implementation must derive form this class
class TphSensor : public Sensor {
public:

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
	virtual bool Init(const TPHSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read TPH data (require implementation).
	 *
	 * Read TPH value from device if available. If not return previous data.
	 *
	 * @param 	TphData : Reference buffer to be filled with measured data
	 *
	 * @return
	 * 			- true	: If new data is returned
	 * 			- false	: If old data is returned
	 */
	virtual bool Read(TPHSensorData_t &TphData) = 0;

	/**
	 * @brief	Read temperature (require implementation).
	 *
	 * @return	Temperature in degree C
	 */
	virtual float ReadTemperature() = 0;

	/**
	 * @brief	Read barometric pressure (require implementation).
	 *
	 * @return	Barometric pressure in Pascal
	 */
	virtual float ReadPressure() = 0;

	/**
	 * @brief	Read relative humidity (require implementation).
	 *
	 * @return	Relative humidity in %
	 */
	virtual float ReadHumidity() = 0;

protected:

	TPHSensorData_t vTphData;			//!< Last measured data
	TPHDataRdyHandler_t vDataRdyHandler;	//!< Callback data ready handler
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __TPH_SENSOR_H__
