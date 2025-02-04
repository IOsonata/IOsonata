/**-------------------------------------------------------------------------
@file	imu_xiot_fusion.h

@brief	Implementation of software imu class using x-iot Fusion

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#ifndef __IMU_XIOT_FUSION_H__
#define __IMU_XIOT_FUSION_H__

#include "Fusion/Fusion.h"

#include "imu/imu.h"


#ifdef __cplusplus

class ImuXiotFusion : public Imu {
public:
	bool Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler();
	uint32_t Rate(uint32_t DataRate);
	bool Calibrate();
	void SetAxisAlignmentMatrix(int8_t * const pMatrix);
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Euler(bool bEn) { return false; }
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);
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
	virtual bool Read(AccelSensorRawData_t &Data) { return vpAccel->Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return vpAccel->Read(Data); }

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
	virtual bool Read(GyroSensorRawData_t &Data) { return vpGyro->Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return vpGyro->Read(Data); }

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
	virtual bool Read(MagSensorRawData_t &Data) { return vpMag->Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return vpMag->Read(Data); }
    virtual bool Read(ImuQuat_t &Data) { Data = vQuat; return true; }
    virtual bool Read(ImuEuler_t &Data) { Data = vEuler; return true; }

protected:

private:	
    FusionAhrs vAhrs;
    FusionAhrsSettings vSettings;
    uint64_t vPrevTimeStamp;
};

extern "C" {
#endif

// C prototypes

#ifdef __cplusplus
}
#endif

#endif // __IMU_XIOT_FUSION_H__
