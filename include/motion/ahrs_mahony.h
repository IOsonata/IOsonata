/**-------------------------------------------------------------------------
@file	ahrs_mahony.h

@brief	Implementation of the Ahrs class using Mahony fusion

Self contained Mahony explicit complementary filter for attitude estimation
with gyro bias correction. No external source and no CMSIS-DSP dependency. The
state is a unit quaternion plus an integral feedback term that tracks the gyro
bias. A PI feedback drives the gyro integration toward the accel gravity
direction (and the mag heading direction when 9-axis is enabled later).

This is the lowest cost backend in the set. It uses no matrix inverse and no
covariance, only cross products and a quaternion update per sample, so it runs
on soft-float parts and is a good fit for fast inner loops and low cost nodes.
For higher accuracy or online noise modelling use VQF, MEKF or EqF.

This is the 6-axis variant (accel + gyro). The accel supplies the gravity
direction; roll and pitch gyro bias are tracked by the integral term. Heading
stays unobservable without a mag source. The mag heading correction (9-axis)
adds into the same feedback path and can be enabled later.

@author	Hoang Nguyen Hoan
@date	Jun. 23, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __AHRS_MAHONY_H__
#define __AHRS_MAHONY_H__

#include "motion/ahrs.h"

/** @addtogroup AHRS
  * @{
  */

/// Mahony tuning parameters.
typedef struct __Mahony_Param {
	float Kp;		//!< Proportional gain, accel tracking. Time constant ~ 1/Kp sec
	float Ki;		//!< Integral gain, gyro bias learning rate. 0 disables bias estimation
	int initSamples;	//!< Accel samples averaged for initial attitude
	float accGateLo;	//!< Reject accel correction below this norm, g
	float accGateHi;	//!< Reject accel correction above this norm, g
} MahonyParam_t;

/// Mahony runtime state. The quaternion maps body to earth and matches the EqF
/// and MEKF output convention.
typedef struct __Mahony_State {
	float q[4];		//!< Attitude quaternion, body to earth, [w x y z]
	float integralFB[3];	//!< Integral feedback, gyro bias correction, rad/s
	float accSum[3];	//!< Accel accumulator during init
	int accInitCount;	//!< Accel sample count during init
	int mode;		//!< 0 = init accumulate, 1 = running
} MahonyState_t;

class AhrsMahony : public Ahrs {
public:
	AhrsMahony();
	virtual ~AhrsMahony() {}
	bool Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(MahonyParam_t &Param) { memcpy(&vParams, &Param, sizeof(MahonyParam_t)); }
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler();

	bool Calibrate();
	void SetAxisAlignmentMatrix(int8_t * const pMatrix);
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Euler(bool bEn) { return false; }
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);

	virtual bool Read(AccelSensorRawData_t &Data) { return vpAccel->Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return vpAccel->Read(Data); }
	virtual bool Read(GyroSensorRawData_t &Data) { return vpGyro->Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return vpGyro->Read(Data); }
	virtual bool Read(MagSensorRawData_t &Data) { return vpMag->Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return vpMag->Read(Data); }
	virtual bool Read(AhrsQuat_t &Data);
	virtual bool Read(AhrsEuler_t &Data) { Data = vEuler; return true; }

protected:

private:
	void Setup(void);
	/// Build the initial attitude quaternion from an averaged gravity vector.
	void GravityInit(const float accAvg[3]);

	MahonyParam_t vParams;
	MahonyState_t vState;
	float vGyrDt;		//!< Gyro sample period, sec
	float vAccDt;		//!< Accel sample period, sec
	int vNbAxis;		//!< 6 for accel/gyro (mag path not yet enabled)
	bool vbInitialized;
};

/** @} */

#endif // __AHRS_MAHONY_H__
