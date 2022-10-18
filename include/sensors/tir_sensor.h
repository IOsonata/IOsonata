/**-------------------------------------------------------------------------
@file	tir_sensor.h

@brief	Generic Thermal IR sensor definitions

@author	Hoang Nguyen Hoan
@date	Apr. 14, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __TIR_SENSOR_H__
#define __TIR_SENSOR_H__

#include <stdint.h>

#include "coredev/timer.h"
#include "sensors/temp_sensor.h"

#pragma pack(push, 4)
typedef struct __Thermal_Ir_Data {
	uint32_t Timestamp;
	int32_t Tobject;
	int16_t Tambient;
	uint8_t Tpresence;
	uint8_t Tmotion;
	uint8_t Tambshock;
} ThermIrData_t;

/// Configuration
typedef struct __Thermal_Ir_Cfg {
	uint8_t DevAddr;	//!< I2C device address
	bool IntEn;			//!< Interrupt enable true/false
	DevEvtHandler_t EvHandler;
} ThermIrCfg_t;
#pragma pack(pop)

#ifdef __cplusplus

class ThermIRSensor : public TempSensor {
public:
	bool Init(const ThermIrCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

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
	virtual bool Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable() { return true; }

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	virtual void Disable() {}

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset() {}

	/**
	 * @brief	Read temperature data.
	 *
	 * Read temperature value from the last UpdateData
	 *
	 * @param 	Buff : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(TempSensorData_t &Data) { Data = vTempData; }

	/**
	 * @brief	Read temperature (require implementation).
	 *
	 * @return	Temperature in degree C
	 */
	virtual float ReadTemperature() { return (float)vTempData.Temperature / 100.0; }

	/**
	 * @brief	Start sampling data
	 *
	 * This is a require implementation by sensor implementer.\n
	 * This function initiates sensor to do actual measurement.
	 *
	 * @return	true - success
	 * 			false - in case of error or sensor busy measuring
	 */
	virtual bool StartSampling() { return true; }

	/**
	 * @brief	Read sensor and update internal data with new readings
	 *
	 * This function should be called by a periodic timer to update
	 * sensor data in SENSOR_OPMODE_CONTINUOUS or interrupt or when Read is called
	 * in SENSOR_OPMODE_SINGLE
	 *
	 * @return	true - New data is updated
	 */
	virtual bool UpdateData();

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler();


private:
//	void ReadTP();
	float CalcAmbientTemp(int16_t TPamb) {
		// Tamb [K] = (25 + 273.15) + (TPambient − PTAT25) · (1/M )
		return (25.0 + 273.15) + (float)(TPamb - vPTat25) / vM;
	}
	float CalcObjTemp(int32_t TPobj, int16_t TPamb);

	TempSensorData_t vTempData;
	CalipileData_t vData;
	uint8_t vLookup;
	int16_t vPTat25;
	float vM;
	int32_t vU0;
	int32_t vUout1;
	float vTobj1;	// in degree K
	int32_t vTamb;
	float vkFactor;
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // __TIR_SENSOR_H__
