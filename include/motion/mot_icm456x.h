/**-------------------------------------------------------------------------
@file	mot_icm456x.h

@brief	Motion device driver (attitude and activity) for Invensense ICM-45686

@author	Hoang Nguyen Hoan
@date	May 29, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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
#ifndef __MOT_ICM456x_H__
#define __MOT_ICM456x_H__

#include "motion/att.h"
#include "motion/act.h"

/** @addtogroup Motion
  * @{
  */

#ifdef __cplusplus

class MotIcm456x : public Att, public Act {
public:
	virtual bool Init(const AttCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool UpdateData();
	virtual void IntHandler();
	virtual bool Calibrate();
	virtual void SetAxisAlignmentMatrix(int8_t * const pMatrix);

	// Features
	virtual bool Euler(bool bEn);
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);
private:
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** @} end group Motion */

#endif // __MOT_ICM456x_H__
