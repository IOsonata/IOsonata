/**-------------------------------------------------------------------------
@file	press_sensor.h

@brief	Generic pressure sensor abstraction

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
#ifndef __PRESS_SENSOR_H__
#define __PRESS_SENSOR_H__

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


/// @brief	Pressure sensor data
///
/// Structure defining pressure sensor data
typedef struct __PressureSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	uint32_t Pressure;		//!< Barometric pressure in Pa no decimal
} PressureSensorData_t;

#pragma pack(pop)

class PressSensor;

typedef void (*PressureSensorEvtHandler_t)(PressSensor * const pSensor, PressureSensorData_t *pData);

#pragma pack(push, 4)

/// @brief	Pressure sensor configuration
///
typedef struct __PressureSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	int				PresOvrs;		//!< Oversampling measurement for pressure
	uint32_t		FilterCoeff;	//!< Filter coefficient select value (this value is device dependent)
	PressureSensorEvtHandler_t EvtHandler;//!< Event handler
} PressureSensorCfg_t;

#pragma pack(pop)

#ifdef __cplusplus

/// Pressure sensor base class.  Sensor implementation must derive form this class
class PressSensor : public Sensor {
public:

	PressSensor() { Type(SENSOR_TYPE_PRESSURE); }

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
	virtual bool Init(const PressureSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read pressure data.
	 *
	 * Read pressure value from device if available. If not return previous data.
	 *
	 * @param	Buff : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(PressureSensorData_t &Data) { Data = vData; }

	/**
	 * @brief	Read pressure).
	 *
	 * @return	Barometric pressure in KPascal
	 */
	virtual float ReadPressure() { return (float)vData.Pressure / 1000.0; }

protected:

	PressureSensorData_t vData;			//!< Last measured data
	PressureSensorEvtHandler_t vEvtyHandler;	//!< Event handler
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __PRESS_SENSOR_H__
