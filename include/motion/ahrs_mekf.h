/**-------------------------------------------------------------------------
@file	ahrs_mekf.h

@brief	Implementation of the Ahrs class using MEKF fusion

Self contained Multiplicative Extended Kalman Filter for attitude and gyro
bias estimation. No external source and no CMSIS-DSP dependency. The state is
a unit quaternion plus a 3 axis gyro bias. The error state is a 6 vector of a
small angle attitude error and a bias error, propagated with a 6x6 covariance
and reset into the quaternion after every update (the multiplicative reset).

This is the 6-axis variant (accel + gyro). The accel supplies the gravity
direction measurement. Roll and pitch gyro bias become observable through the
gravity update and the error state coupling; yaw and yaw bias stay unobservable
without a heading source. The mag direction update (9-axis) uses the same
vector update path and can be added later.

The filter linearises the error dynamics at the current estimate each step,
unlike the EqF which linearises at a fixed origin. Use MEKF when an error-state
EKF baseline is wanted; use EqF for stronger transient and reset behaviour.

The matrix work (6x6 covariance, quaternion and 3x3 ops per sample) needs an
FPU. Use this backend on FPU targets; on soft-float parts prefer VQF.

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
#ifndef __AHRS_MEKF_H__
#define __AHRS_MEKF_H__

#include "motion/ahrs.h"

/** @addtogroup AHRS
  * @{
  */

/// MEKF tuning parameters. Noise terms follow the Farrenkopf attitude and bias
/// model. Defaults are reasonable starting values for a consumer MEMS IMU.
typedef struct __Mekf_Param {
	float sigmaV;		//!< Gyro white noise, rad/s/sqrt(Hz)
	float sigmaU;		//!< Gyro bias random walk, rad/s^2/sqrt(Hz)
	float sigmaAcc;		//!< Accel measurement noise, unit vector
	float accAdaptK;	//!< Accel adaptive noise gain vs norm deviation
	int initSamples;	//!< Accel samples averaged for initial attitude
	float pInitAtt;		//!< Initial attitude variance, rad^2
	float pInitBias;	//!< Initial bias variance, (rad/s)^2
	float accGateLo;	//!< Reject accel update below this norm, g
	float accGateHi;	//!< Reject accel update above this norm, g
} MekfParam_t;

/// MEKF runtime state. The quaternion maps body to earth and matches the EqF
/// output convention. The covariance is symmetrised after each step to control
/// float32 drift.
typedef struct __Mekf_State {
	float q[4];		//!< Attitude quaternion, body to earth, [w x y z]
	float b[3];		//!< Gyro bias, rad/s
	float P[36];		//!< 6x6 error covariance, row-major
	float accSum[3];	//!< Accel accumulator during init
	int accInitCount;	//!< Accel sample count during init
	int mode;		//!< 0 = init accumulate, 1 = running
} MekfState_t;

class AhrsMekf : public Ahrs {
public:
	AhrsMekf();
	virtual ~AhrsMekf() {}
	bool Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(MekfParam_t &Param) { memcpy(&vParams, &Param, sizeof(MekfParam_t)); }
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
	/// One propagation step from a gyro sample (rad/s) over dt seconds.
	void Propagate(const float w[3], float dt);
	/// Vector measurement update. meas is the raw body-frame vector, ref is the
	/// known unit direction in the earth frame, sigma the measurement std-dev.
	void VecUpdate(const float meas[3], const float ref[3], float sigma);
	/// Build the initial attitude quaternion from an averaged gravity vector.
	void GravityInit(const float accAvg[3]);

	MekfParam_t vParams;
	MekfState_t vState;
	float vGyrDt;		//!< Gyro sample period, sec
	float vAccDt;		//!< Accel sample period, sec
	int vNbAxis;		//!< 6 for accel/gyro (mag path not yet enabled)
	bool vbInitialized;
};

/** @} */

#endif // __AHRS_MEKF_H__
