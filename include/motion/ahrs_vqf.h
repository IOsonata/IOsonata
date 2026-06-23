/**-------------------------------------------------------------------------
@file	ahrs_vqf.h

@brief	Implementation of the Ahrs class using vqf fusion

Self contained port of Daniel Laidig VQF orientation estimation algorithm.
No external vqf source and no CMSIS-DSP dependency. The full algorithm is
implemented directly here: gyro strapdown integration, accelerometer based
inclination correction, motion and rest gyro bias estimation, and
magnetometer disturbance rejection.

Reference: https://github.com/dlaidig/vqf
Laidig and Seel, "VQF: Highly accurate IMU orientation estimation with bias
estimation and magnetic disturbance rejection", Information Fusion, 2023.

@author	Hoang Nguyen Hoan
@date	May. 28, 2025

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
#ifndef __AHRS_VQF_H__
#define __AHRS_VQF_H__

#include "motion/ahrs.h"

/** @addtogroup AHRS
  * @{
  */

/// VQF tuning parameters. Main state is single precision; the low pass filter
/// states are double precision (see VqfState_t) because the mean initialised
/// biquad drifts in float. Defaults match the reference implementation.
typedef struct __Vqf_Param {
	float tauAcc;			//!< Time constant for accel low pass, sec
	float tauMag;			//!< Time constant for mag heading correction, sec
	bool motionBiasEstEnabled;	//!< Enable gyro bias estimation during motion
	bool restBiasEstEnabled;	//!< Enable gyro bias estimation at rest
	bool magDistRejectionEnabled;	//!< Enable magnetic disturbance rejection
	float biasSigmaInit;		//!< Initial bias uncertainty, deg/s
	float biasForgettingTime;	//!< Bias forgetting time, sec
	float biasClip;			//!< Bias estimate clip, deg/s
	float biasSigmaMotion;		//!< Motion bias estimation uncertainty, deg/s
	float biasVerticalForgettingFactor;	//!< Vertical bias forgetting factor
	float biasSigmaRest;		//!< Rest bias estimation uncertainty, deg/s
	float restMinT;			//!< Min duration to detect rest, sec
	float restFilterTau;		//!< Time constant for rest detection low pass, sec
	float restThGyr;		//!< Rest detection gyro threshold, deg/s
	float restThAcc;		//!< Rest detection accel threshold
	float magCurrentTau;		//!< Time constant for current mag norm/dip low pass, sec
	float magRefTau;		//!< Time constant for mag reference update, sec
	float magNormTh;		//!< Relative threshold for mag norm
	float magDipTh;			//!< Threshold for mag dip angle, deg
	float magNewTime;		//!< Duration accepting a new mag field, sec
	float magNewFirstTime;		//!< Duration accepting a new mag field at start, sec
	float magNewMinGyr;		//!< Min gyro rate to accept a new mag field, deg/s
	float magMinUndisturbedTime;	//!< Min undisturbed time before trusting mag, sec
	float magMaxRejectionTime;	//!< Max mag rejection time, sec
	float magRejectionFactor;	//!< Mag rejection recovery factor
} VqfParam_t;

/// VQF runtime state. Quaternion, bias and vector quantities are single
/// precision. The biquad low pass states are double precision; this is a
/// correctness requirement of the mean initialised filter, not a tuning choice.
typedef struct __Vqf_State {
	float gyrQuat[4];		//!< Strapdown integrated quaternion
	float accQuat[4];		//!< Inclination corrected quaternion
	float delta;			//!< Heading offset from mag, rad
	bool restDetected;
	bool magDistDetected;
	float lastAccLp[3];		//!< Last accel low pass output, earth frame
	double accLpState[3 * 2];	//!< Accel low pass biquad state
	float lastAccCorrAngularRate;
	float kMagInit;			//!< Mag gain during fast initial convergence
	float lastMagDisAngle;
	float lastMagCorrAngularRate;
	float bias[3];			//!< Gyro bias estimate, rad/s
	float biasP[9];			//!< Bias estimation covariance
	double motionBiasEstRLpState[9 * 2];	//!< Rotation matrix low pass state
	double motionBiasEstBiasLpState[2 * 2];	//!< R*bias low pass state
	float restLastSquaredDeviations[2];	//!< [gyr, acc] squared deviations
	float restT;			//!< Accumulated rest time, sec
	float restLastGyrLp[3];
	double restGyrLpState[3 * 2];
	float restLastAccLp[3];
	double restAccLpState[3 * 2];
	float magRefNorm;
	float magRefDip;
	float magUndisturbedT;
	float magRejectT;
	float magCandidateNorm;
	float magCandidateDip;
	float magCandidateT;
	float magNormDip[2];		//!< [norm, dip] of current mag in earth frame
	double magNormDipLpState[2 * 2];
} VqfState_t;

/// Coefficients precomputed once from parameters and sample periods. Computed
/// in Setup() at Init time, never recomputed in the per sample update path.
typedef struct __Vqf_Coeff {
	float gyrTs;			//!< Gyro sample period, sec
	float accTs;			//!< Accel sample period, sec
	float magTs;			//!< Mag sample period, sec
	double accLpB[3];		//!< Accel low pass numerator
	double accLpA[2];		//!< Accel low pass denominator
	float kMag;			//!< Mag heading correction gain
	float biasP0;			//!< Initial bias covariance
	float biasV;			//!< Bias system noise
	float biasMotionW;		//!< Motion update measurement noise
	float biasVerticalW;		//!< Vertical motion update measurement noise
	float biasRestW;		//!< Rest update measurement noise
	double restGyrLpB[3];
	double restGyrLpA[2];
	double restAccLpB[3];
	double restAccLpA[2];
	float kMagRef;			//!< Mag reference update gain
	double magNormDipLpB[3];
	double magNormDipLpA[2];
} VqfCoeff_t;

class AhrsVqf : public Ahrs {
public:
	AhrsVqf();
	virtual ~AhrsVqf() {}
	bool Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(VqfParam_t &Param) { memcpy(&vParams, &Param, sizeof(VqfParam_t)); }
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
	/// Recompute all coefficients from vParams and the sample periods.
	void Setup(void);
	/// Per sample VQF update steps (input arrays already in VQF units).
	void UpdateGyr(const float gyr[3]);	//!< gyr in rad/s
	void UpdateAcc(const float acc[3]);	//!< acc in any consistent unit
	void UpdateMag(const float mag[3]);	//!< mag in any consistent unit
	/// Mean initialised second order low pass over a vector. state holds two
	/// doubles per element; out receives the filtered vector.
	void FilterVec(const float vec[], size_t n, float tau, float Ts, const double b[3],
	               const double a[2], double state[], float out[]);
	/// Output quaternions: 6D uses accel and gyro only, 9D adds mag heading.
	void GetQuat6D(float out[4]) const;
	void GetQuat9D(float out[4]) const;

	VqfParam_t vParams;
	VqfState_t vState;
	VqfCoeff_t vCoeffs;
	int vNbAxis;		//!< 6 for accel/gyro, 9 for accel/gyro/mag
	bool vbInitialized;
};

/** @} */

#endif // __AHRS_VQF_H__
