/**-------------------------------------------------------------------------
@file	dist_sensor.h

@brief	Generic distance sensor abstraction

Distance sensors such as the time of flight sensors measure the distance to
the nearest target in mm, with the return signal and the ambient light rates
that tell how much the measurement can be trusted.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __DIST_SENSOR_H__
#define __DIST_SENSOR_H__

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

#define DISTSENSOR_STATUS_VALID			0		//!< Distance measured, other values are device dependent

#pragma pack(push, 1)

/// @brief	Distance sensor data
///
/// Structure defining distance sensor data
typedef struct __DistSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	uint32_t Distance;		//!< Distance in mm
	uint32_t SignalRate;	//!< Return signal rate in kilo counts per second
	uint32_t AmbientRate;	//!< Ambient light rate in kilo counts per second
	uint8_t Status;			//!< DISTSENSOR_STATUS_VALID or a device dependent error code
} DistSensorData_t;

#pragma pack(pop)

#ifdef __cplusplus
class DistSensor;

typedef void (*DistSensorEvtHandler_t)(DistSensor * const pSensor, DistSensorData_t *pData);

#pragma pack(push, 4)

/// @brief	Distance sensor configuration
///
typedef struct __DistSensor_Config {
	uint32_t DevAddr;			//!< Either I2C 7 bits device address or CS index select if SPI is used
	SENSOR_OPMODE OpMode;		//!< Operating mode
	uint32_t Freq;				//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	uint32_t TimingBudget;		//!< Time of one measurement in usec, 0 for the device default
	uint32_t IntPrio;			//!< Interrupt priority
	uint8_t Inter;				//!< 0 - Disable, Bit0 - enable interrupt 1, Bit1 - enable interrupt 2,...
	DEVINTR_POL	IntPol;			//!< Interrupt polarity
	DistSensorEvtHandler_t DataRdyCB;	//!< Data ready handler
} DistSensorCfg_t;

#pragma pack(pop)

/// Distance sensor base class.  Sensor implementation must derive form this class
class DistSensor : public Sensor {
public:
	DistSensor() { Type(SENSOR_TYPE_DISTANCE); }

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	Cfg 	: Reference to configuration data
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
	virtual bool Init(const DistSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read distance data.
	 *
	 * Read the data of the last UpdateData
	 *
	 * @param 	Data : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(DistSensorData_t &Data) { Data = vData; }

	/**
	 * @brief	Read the distance.
	 *
	 * @return	Distance in mm of the last UpdateData
	 */
	virtual uint32_t ReadDistance() { return vData.Distance; }

protected:
	DistSensorData_t vData;					//!< Last measured data
	DistSensorEvtHandler_t vDataRdyHandler;	//!< Data ready event handler
};

extern "C" {
#endif // __cplusplus

#ifdef __cplusplus
}

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __DIST_SENSOR_H__
