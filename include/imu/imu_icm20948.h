/**-------------------------------------------------------------------------
@file	imu_icm20948.h

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

@author	Hoang Nguyen Hoan
@date	Sept. 9, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
#ifndef __IMU_ICM20948_H__
#define __IMU_ICM20948_H__

#include "device_intrf.h"
#include "imu/imu.h"
#include "sensors/agm_icm20948.h"

class ImuIcm20948 : public Imu {
public:

	bool Init(const ImuCfg_t &Cfg, AgmIcm20948 *pIcm);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler();
	virtual IMU_FEATURE Feature(IMU_FEATURE FeatureBit, bool bEnDis);
	virtual bool Calibrate();
	virtual void SetAxisAlignmentMatrix(int8_t * const pMatrix);
	virtual bool Euler(bool bEn) { return false; }
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);

	virtual bool Read(ImuQuat_t &Data) { return Imu::Read(Data); }
	virtual bool Read(ImuEuler_t &Data) { return Imu::Read(Data); }

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
	virtual bool Read(AccelSensorData_t &Data) { return Imu::Read(Data); }

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
	virtual bool Read(GyroSensorData_t &Data) { return Imu::Read(Data); }

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
	virtual bool Read(MagSensorData_t &Data) { return Imu::Read(Data); }

protected:
	bool Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);

private:
	/**
	 * @brief	Read device's register/memory block
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior reading data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pBuff		: Data buffer container
	 * @param	BuffLen		: Data buffer size
	 *
	 * @return	Actual number of bytes read
	 */
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return vpIcm->Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	/**
	 * @brief	Write to device's register/memory block
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior writing data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pData		: Data buffer to be written to the device
	 * @param	DataLen		: Size of data
	 *
	 * @return	Actual number of bytes written
	 */
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return vpIcm->Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	AgmIcm20948 *vpIcm;
};



#endif // __IMU_INVN_ICM20948_H__
