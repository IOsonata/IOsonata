/**-------------------------------------------------------------------------
@file	accel_sensor.h

@brief	Generic accelerometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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

#ifndef __ACCEL_SENSOR_H__
#define __ACCEL_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

/** @addtogroup Sensors
  * @{
  */

#pragma pack(push, 1)

/// Accelerometer raw sensor data
typedef struct __AccelSensor_Raw_Data {
	uint64_t Timestamp;			//!< Time stamp count in usec
	float GFactor;				//!< g scale factor. Obtain g force = GFactor * RawData;
    int16_t Temp;				//!< Temperature in C x 100
	union {
		int32_t Val[3];
		struct {
			int32_t X;			//!< X axis
			int32_t Y;			//!< Y axis
			int32_t Z;			//!< Z axis
		};
	};
} AccelSensorRawData_t;

//typedef AccelSensorRawData_t	ACCELSENSOR_RAWDATA;

/// Accelerometer sensor data in G
typedef struct __AccelSensor_Data {
	uint64_t Timestamp;			//!< Time stamp count in usec
	float Temp;					//!< Temperature in degree C
	union {
		float Val[3];
		struct {
		    float X;			//!< X axis
		    float Y;			//!< Y axis
		    float Z;			//!< Z axis
		};
	};
} AccelSensorData_t;

//typedef AccelSensorData_t	ACCELSENSOR_DATA;

typedef void (*AccelSensorEvtCb_t)(AccelSensorData_t *pData);

/// Accel configuration data
typedef struct __AccelSensor_Config {
	uint32_t		DevAddr;	//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		//!< Operating mode
	uint32_t		Freq;		//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	uint16_t		Scale;		//!< Accelerometer sensor scale in g force (2g, 4g, ...
	uint32_t		FltrFreq;	//!< Filter cutoff frequency in mHz
	uint8_t 		Inter;		//!< 0 - Disable, Bit0 - enable interrupt 1, Bit1 - enable interrupt 2,...
	DEVINTR_POL		IntPol;		//!< interrupt polarity
	AccelSensorEvtCb_t IntHandler;
	bool			bFifoEn;		//!< Enable internal fifo (device dependent)
} AccelSensorCfg_t;

//typedef AccelSensorCfg_t	ACCELSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/// Accel. sensor base class
class AccelSensor : public Sensor {
public:

	/**
	 * @brief	Sensor initialization
	 *
	 * @param 	Cfg		: Sensor configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    /**
     * @brief   Read last updated sensor raw data
     *
     * This function read the currently stored data last updated by UdateData().
     * Device implementation can add validation if needed and return true or false
     * in the case of data valid or not.  This default implementation only returns
     * the stored data with success.
     *
     * @param   Data : Reference to data storage for the returned data
     *
     * @return  True - Success.
     */
	virtual bool Read(AccelSensorRawData_t &Data) {
		Data = vData;
		return true;
	}

	/**
	 * @brief	Read converted sensor data
	 *
	 * This function gets the currently stored raw data last updated by UdateData() and
	 * convert it to real G force unit.  Device implementation can add validation if needed and
	 * return true or false in the case of data valid or not.  This default implementation
	 * only returns the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(AccelSensorData_t &Data);

	/**
	 * @brief	Get the current G scale value.
	 *
	 * @return	G scale value
	 */
	virtual uint16_t Scale() { return vScale; }

	/**
	 * @brief	Set the current G scale value.
	 *
	 * NOTE : Implementer must overload this function to add require hardware implement then call
	 * this function to keep the scale value internally and return the real hardware scale value.
	 *
	 * @param 	Value : Wanted scale value
	 *
	 * @return	Real scale value
	 */
	virtual uint16_t Scale(uint16_t Value);

	/**
	 * @brief	Set max measurement range of the device
	 *
	 * This function sets the maximum positive value of the raw data that can be read from
	 * the sensor. Sensor device can implement this function to allow configuration variable
	 * range value.
	 *
	 * @param	Max positive range value
	 *
	 * @return	Actual maximum positive range value of the raw data
	 */
	virtual uint32_t Range(uint32_t Value);// { vData.GFactor = (float)Scale() / Sensor::Range(Value); return vData.Range; }

    virtual void SetCalibration(const float (&Gain)[3][3], const float (&Offset)[3]);
    virtual void SetCalibrationMatrix(const float (&Gain)[3][3]);
	virtual void SetCalibrationOffset(const float (&Offset)[3]);
	virtual void ClearCalibration();
	virtual bool StartSampling() { return true; }

	AccelSensor() {
		Type(SENSOR_TYPE_ACCEL);
		ClearCalibration();
	}
	AccelSensor(AccelSensor&);	// Copy ctor not allowed

protected:

	AccelSensorRawData_t vData;		//!< Current sensor data updated with UpdateData()
	AccelSensorEvtCb_t vIntHandler;

private:
	uint16_t vScale;			//!< Sensor data scale in g force (2g, 4g, ...)
	float vCalibGain[3][3];
	float vCalibOffset[3];
};

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __ACCEL_SENSOR_H__
