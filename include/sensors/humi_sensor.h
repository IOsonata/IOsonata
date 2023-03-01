/**-------------------------------------------------------------------------
@file	humi_sensor.h

@brief	Generic humidity sensor abstraction

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
#ifndef __HUMI_SENSOR_H__
#define __HUMI_SENSOR_H__

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


/// @brief	Humidity sensor data
///
/// Structure defining humidity sensor data
typedef struct __HumiditySensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	uint16_t Humidity;		//!< Relative humidity in %, 2 decimals fixed point
} HumiSensorData_t;

#pragma pack(pop)

class HumiSensor;

typedef void (*HumiSensorEvtCb_t)(HumiSensor * const pSensor, HumiSensorData_t *pData);

#pragma pack(push, 4)

/// @brief	Humidity sensor configuration
///
typedef struct __HumiditySensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	int 			HumOvrs;		//!< Oversampling measurement for humidity
	uint32_t		FilterCoeff;	//!< Filter coefficient select value (this value is device dependent)
	HumiSensorEvtCb_t EvtHandler;	//!< Event handler callback
} HumiSensorCfg_t;

#pragma pack(pop)

#ifdef __cplusplus

/// Humidity sensor base class.  Sensor implementation must derive form this class
class HumiSensor : public Sensor {
public:

	HumiSensor() { Type(SENSOR_TYPE_HUMI); }

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
	virtual bool Init(const HumiSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read current data.
	 *
	 * Read current value that was updated by UpdateData().
	 *
	 * @param 	Data : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(HumiSensorData_t &Data) { Data = vData; }

	/**
	 * @brief	Read relative humidity
	 *
	 * @return	Relative humidity in %
	 */
	virtual float ReadHumidity() { return (float)vData.Humidity / 100.0; }

protected:

	HumiSensorData_t 	vData;			//!< Last measured data
	HumiSensorEvtCb_t	vEvtHandler;	//!< Event handler
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __HUMI_SENSOR_H__
