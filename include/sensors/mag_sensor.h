/**-------------------------------------------------------------------------
@file	mag_sensor.h

@brief	Generic magnetometer sensor abstraction

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

#ifndef __MAG_SENSOR_H__
#define __MAG_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

/** @addtogroup Sensors
  * @{
  */

#pragma pack(push, 1)

/// Magnetometer raw sensor data
typedef struct __MagSensor_Raw_Data {
    uint64_t Timestamp; 		//!< Time stamp count in usec
    uint16_t Sensitivity[3];	//!< Scale factor in nanoTesla of the sensor
    union {
        int32_t Val[3];
        struct {
            int32_t X;          //!< X axis
            int32_t Y;          //!< Y axis
            int32_t Z;          //!< Z axis
        };
    };
} MagSensorRawData_t;

/// Magnetometer sensor data
typedef struct __MagSensor_Data {
	uint64_t Timestamp;			//!< Time stamp count in usec
	union {
	    float Val[3];	//!< Mag data in uT
		struct {
	        float X;			//!< X axis
	        float Y;			//!< Y axis
	        float Z;			//!< Z axis
		};
	};
} MagSensorData_t;

// Some mag may offer select-able sampling precision such as the MPU9250 allows 14 or 16 bits selection
/// Sampling precision
typedef enum __MagSensor_Precision {
	MAGSENSOR_PRECISION_LOW,
	MAGSENSOR_PRECISION_HIGH
} MAGSENSOR_PRECISION;

/// Mag configuration data
typedef struct __MagSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C 7 bits device address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	MAGSENSOR_PRECISION	Precision;	//!< Sampling precision
	uint8_t 		Inter;			//!< 0 - Disable, Bit0 - enable interrupt 1, Bit1 - enable interrupt 2,...
	DEVINTR_POL		IntPol;			//!< Interrupt polarity
} MagSensorCfg_t;

#pragma pack(pop)

#ifdef __cplusplus

class MagSensor : public Sensor {
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
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    virtual bool Read(MagSensorRawData_t &Data) {
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
    virtual bool Read(MagSensorData_t &Data);

    virtual MAGSENSOR_PRECISION Precision() { return vPrecision; }
    virtual MAGSENSOR_PRECISION Precision(MAGSENSOR_PRECISION Val) { vPrecision = Val; return vPrecision; }
    virtual void SetCalibration(float (&Gain)[3][3], float (&Offset)[3]);
    virtual void ClearCalibration();
    virtual void Sensitivity(uint16_t (&Sen)[3]);
    MagSensor() {
    	Type(SENSOR_TYPE_MAG);
    	ClearCalibration();
    }

protected:
    // These functions allow override the default interface for devices that are hooked up on
    // the secondary interface.
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return Read(DeviceAddress(), pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return Write(DeviceAddress(), pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	uint16_t vSensitivity[3];		//!< Sample scaling factor in nanoTesla
	MAGSENSOR_PRECISION vPrecision;	//!< Currently selected sampling precision
	MagSensorRawData_t vData;	//!< Current sensor data updated with UpdateData()
	float vCalibGain[3][3];
	float vCalibOffset[3];
private:

};

#endif // __cplusplus

/** @} End of group Sensors */

#endif // __MAG_SENSOR_H__
