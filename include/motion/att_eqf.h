/**-------------------------------------------------------------------------
@file	att_eqf.h

@brief	Implementation of the Ahrs class using EqF fusion

Self contained port of the ABC-EqF n=0 (Attitude-Bias Equivariant Filter).
No external source and no CMSIS-DSP dependency. The estimator is a geometric
EKF on the symmetry group: SO(3) attitude plus gyro bias, with a 6x6 error
covariance propagated by a state transition matrix and a Riccati update, and
corrections applied through the group exponential.

This is the 6-axis variant (accel + gyro). The accel provides the gravity
direction measurement; gyro bias is estimated during detected rest. The mag
direction update (9-axis) uses the same direction-update path and can be added
later.

Reference: Fornasier et al., "Overcoming Bias: Equivariant Filter Design for
Biased Attitude Estimation with Online Calibration", RA-L 2022.

The matrix work (6x6 covariance, SO(3) exp/log per sample) needs an FPU. Use
this backend on FPU targets; on soft-float parts prefer VQF.

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
#ifndef __ATT_EQF_H__
#define __ATT_EQF_H__

#include "motion/att.h"

/** @addtogroup Motion
  * @{
  */

/// EqF tuning parameters. Defaults match the reference n=0 tuning.
typedef struct __Eqf_Param {
	float sigmaW;		//!< Gyro noise PSD, rad/s/sqrt(Hz)
	float sigmaB;		//!< Bias random walk PSD, rad/s^2/sqrt(Hz)
	float sigmaAcc;		//!< Accel measurement noise, unit vector
	float accAdaptK;	//!< Accel adaptive noise gain
	float sixAxisAccScale;	//!< Extra accel noise scale in 6-axis mode
	int initSamples;	//!< Accel samples averaged for initial attitude
	int orthoInterval;	//!< Re-orthonormalise R every N propagations
	float pInitAtt;		//!< Initial attitude variance, rad^2
	float pInitBias;	//!< Initial bias variance, (rad/s)^2
	float restTau;		//!< Rest detection low pass time constant, sec
	float restThGyr;	//!< Rest gyro deviation threshold, deg/s
	float restThAcc;	//!< Rest accel deviation threshold, g
	float restMinT;		//!< Min rest duration to trigger, sec
	float restSigma;	//!< Bias measurement noise during rest
	float restMaxBias;	//!< Max plausible bias, deg/s
	float restAccNormTh;	//!< Accel norm deviation from 1g for rest
} EqfParam_t;

/// EqF runtime state. All single precision; the covariance is symmetrised and
/// the attitude re-orthonormalised periodically to control float32 drift.
typedef struct __Eqf_State {
	float A[9];		//!< SO(3) attitude matrix, row-major
	float aVec[3];		//!< Bias in the group Lie algebra; bias = -A^T aVec
	float P[36];		//!< 6x6 error covariance, row-major
	float restGyrLp[3];	//!< Low pass gyro, rad/s
	float restAccLp[3];	//!< Low pass accel, g
	float restGyrDev;	//!< Last gyro squared deviation
	float restT;		//!< Accumulated rest time, sec
	float accSum[3];	//!< Accel accumulator during init
	int accInitCount;	//!< Accel sample count during init
	int orthoCounter;	//!< Propagations since last re-orthonormalise
	bool restDetected;
	bool restGyrLpInit;
	bool restAccLpInit;
	int mode;		//!< 0 = init accumulate, 1 = running
} EqfState_t;

class AhrsEqf : public Ahrs {
public:
	AhrsEqf();
	virtual ~AhrsEqf() {}
	bool Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(EqfParam_t &Param) { memcpy(&vParams, &Param, sizeof(EqfParam_t)); }
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
	/// Direction measurement update. y is the raw body-frame vector, d is the
	/// known unit direction in the earth frame, sigma the measurement std-dev.
	/// suppressYaw zeroes the heading and bias gains (6-axis accel update).
	void DirUpdate(const float y[3], const float d[3], float sigma, bool suppressYaw);
	/// Gyro bias Kalman update applied while at rest.
	void RestBiasUpdate(void);
	/// Build the initial attitude from an averaged gravity vector.
	void GravityInit(const float accAvg[3]);

	EqfParam_t vParams;
	EqfState_t vState;
	float vGyrDt;		//!< Gyro sample period, sec
	float vAccDt;		//!< Accel sample period, sec
	int vNbAxis;		//!< 6 for accel/gyro (mag path not yet enabled)
	bool vbInitialized;
};

/** @} */

#endif // __ATT_EQF_H__
