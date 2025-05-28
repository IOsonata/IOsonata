/**-------------------------------------------------------------------------
@file	agm_icm456x_ak09940.h

@brief	Combine IMU ICM-45686 + AK09940

This file implements combinasion of accel & gyro part of the ICM456x with 
Mag AK09940 connected in aux interface

@author	Hoang Nguyen Hoan
@date	May. 23, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#ifndef __AGM_ICM456X_AK09940_H__
#define __AGM_ICM456X_AK09940_H__

#include "sensors/ag_icm456x.h"
#include "sensors/mag_ak09940.h"

/** @addtogroup Sensors
  * @{
  */



class AgmIcm456xAk09940 : public AgIcm456x, public MagAk09940 {
public:
	/**
	 * @brief	Initialize magnetometer sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const MagSensorCfg_t &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL) {
		vbSensorEnabled[4] = MagIcm20948::Init(Cfg, pIntrf, pTimer);
		return vbSensorEnabled[4];
	}

protected:
	virtual int Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	virtual int Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

private:
};


/** @} End of group Sensors */

#endif // __AGM_ICM456X_AK09940_H__
