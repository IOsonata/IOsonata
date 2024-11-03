/**-------------------------------------------------------------------------
@file	gyro_sensor.h

@brief	Generic gyroscope sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

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

#ifndef __GYRO_SENSOR_H__
#define __GYRO_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

/** @addtogroup Sensors
  * @{
  */

#pragma pack(push, 1)

/// Gyroscope raw sensor data
typedef struct __GyroSensor_Raw_Data {
    uint64_t Timestamp; 	//!< Time stamp count in usec
    uint16_t Sensitivity;	//!< Scale in degree per second of the sensor
    uint16_t Range;     	//!< Sensor ADC range
    union {
        int16_t Val[3];
        struct {
            int16_t X;      //!< X axis
            int16_t Y;      //!< Y axis
            int16_t Z;      //!< Z axis
        };
    };
} GyroSensorRawData_t;

//typedef GyroSensorRawData_t	GYROSENSOR_RAWDATA;

/// Gyroscope sensor data
typedef struct __GyroSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	union {
	    float Val[3];
		struct {
	        float X;		//!< X axis
	        float Y;		//!< Y axis
	        float Z;		//!< Z axis
		};
	};
} GyroSensorData_t;

//typedef GyroSensorData_t	GYROSENSOR_DATA;

typedef struct __GyroSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	uint16_t		Sensitivity;	//!< Sensitivity level per degree per second
	uint32_t		FltrFreq;		//!< Filter cutoff frequency in mHz
	bool 			bInter;			//!< true - enable interrupt
	DEVINTR_POL		IntPol;			//!< Interrupt pin polarity
} GyroSensorCfg_t;

//typedef GyroSensorCfg_t	GYROSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class GyroSensor : public Sensor {
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
	virtual bool Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    virtual bool Read(GyroSensorRawData_t &Data) {
        Data = vData;
        return true;
    }

    /**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(GyroSensorData_t &Data);

	/**
	 * @brief	Get the current sensitivity value.
	 *
	 * @return	Sensitivity value
	 */
	virtual uint16_t Sensitivity() { return vSensitivity; }

	/**
	 * @brief	Set the current sensitivity value.
	 *
	 * NOTE : Implementer must overload this function to add require hardware implement then call
	 * this function to keep the scale value internally and return the real hardware scale value.
	 *
	 * @param 	Value : Wanted sensitivity value
	 *
	 * @return	Real sensitivity value
	 */
	virtual uint16_t Sensitivity(uint16_t Value);

    virtual void SetCalibration(float (&Gain)[3][3], float (&Offset)[3]);
	virtual void ClearCalibration();
	GyroSensor() {
		Type(SENSOR_TYPE_GYRO);
		ClearCalibration();
	}

protected:

	uint16_t vSensitivity;	    //!< Sensitivity level per degree per second
	GyroSensorRawData_t vData;	//!< Current sensor data updated with UpdateData()
	float vCalibGain[3][3];
	float vCalibOffset[3];
};

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __GYRO_SENSOR_H__
