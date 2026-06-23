/**-------------------------------------------------------------------------
@file	nav_eskf.h

@brief	Loosely coupled error-state EKF inertial navigation

15-state error-state Extended Kalman Filter for inertial navigation in a local
NED frame. The nominal state is position, velocity, attitude quaternion, accel
bias and gyro bias. The error state is the 15 vector

	[ d_pos(3), d_vel(3), d_theta(3), d_accelbias(3), d_gyrobias(3) ]

propagated with a 15x15 covariance. Accel and gyro drive the strapdown; GNSS
position and velocity, barometric height, optical flow and zero velocity
updates are fused as they are injected. The attitude error is multiplicative
and reset into the quaternion after each update, as in the MEKF.

This is the loosely coupled INS form used by typical small vehicle flight
stacks. It is the navigation tier baseline; the invariant EKF and the SE2(3)
equivariant filter share this interface and improve on its consistency.

Needs an FPU. The 15x15 covariance work uses general matrix routines from
nav_math.h.

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
#ifndef __NAV_ESKF_H__
#define __NAV_ESKF_H__

#include <string.h>

#include "imu/inertial_nav.h"

/** @addtogroup AHRS
  * @{
  */

#define ESKF_NX		15	//!< Error state dimension

/// ESKF tuning parameters. Noise terms are continuous-time spectral densities.
typedef struct __Eskf_Param {
	float accelNoise;	//!< Accel white noise, m/s^2/sqrt(Hz)
	float gyroNoise;	//!< Gyro white noise, rad/s/sqrt(Hz)
	float accelBiasNoise;	//!< Accel bias random walk, m/s^3/sqrt(Hz)
	float gyroBiasNoise;	//!< Gyro bias random walk, rad/s^2/sqrt(Hz)
	float gravity;		//!< Local gravity magnitude, m/s^2
	int initSamples;	//!< Accel samples averaged for initial levelling
	float pInitPos;		//!< Initial position variance, m^2
	float pInitVel;		//!< Initial velocity variance, (m/s)^2
	float pInitAtt;		//!< Initial attitude variance, rad^2
	float pInitAccBias;	//!< Initial accel bias variance, (m/s^2)^2
	float pInitGyroBias;	//!< Initial gyro bias variance, (rad/s)^2
} EskfParam_t;

/// ESKF runtime state. Nominal state plus the 15x15 error covariance.
typedef struct __Eskf_State {
	float pos[3];		//!< NED position, m
	float vel[3];		//!< NED velocity, m/s
	float q[4];		//!< Attitude quaternion, body to NED, [w x y z]
	float ab[3];		//!< Accel bias, m/s^2
	float gb[3];		//!< Gyro bias, rad/s
	float P[ESKF_NX * ESKF_NX];	//!< 15x15 error covariance, row-major
	float baroBias;		//!< Baro to NED height offset, m
	bool baroBiasSet;	//!< Baro offset has been initialised
	float accSum[3];	//!< Accel accumulator during levelling
	int accInitCount;	//!< Accel sample count during levelling
	int mode;		//!< 0 = level accumulate, 1 = running
} EskfState_t;

class NavEskf : public InertialNav {
public:
	NavEskf();
	virtual ~NavEskf() {}
	bool Init(const NavCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(EskfParam_t &Param) { memcpy(&vParams, &Param, sizeof(EskfParam_t)); }

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	virtual bool UpdateData();
	virtual void IntHandler();

	virtual bool InjectGnss(const NavGnssMeas_t &Meas);
	virtual bool InjectBaro(const NavBaroMeas_t &Meas);
	virtual bool InjectFlow(const NavFlowMeas_t &Meas);
	virtual bool ZeroVelocityUpdate();

	virtual bool Read(NavState_t &Data);

protected:

private:
	void Setup(void);
	/// Strapdown nominal integration and 15x15 covariance propagation.
	void Predict(const float accel[3], const float gyro[3], float dt);
	/// Initial attitude from an averaged gravity vector, level only.
	void LevelInit(const float accAvg[3]);
	/// Convert a geodetic point to NED metres against the current origin.
	void GeodeticToNed(double lat, double lon, float alt, float ned[3]);
	/// Generic 3-row position-type measurement update with diagonal noise.
	/// H selects which 3-block of the error state the measurement observes.
	bool FusePos(const float innov[3], const float Rdiag[3]);
	bool FuseVel(const float innov[3], const float Rdiag[3]);
	/// Single-row height update against the down axis.
	bool FuseHeight(float innov, float r);
	/// Apply a 15-vector correction and reset the multiplicative attitude part.
	void ApplyCorrection(const float dx[ESKF_NX]);
	/// Publish the nominal state into vState.
	void Publish(uint64_t timestamp);

	EskfParam_t vParams;
	EskfState_t vState;
	float vGyrDt;		//!< Gyro sample period, sec
	float vAccDt;		//!< Accel sample period, sec
	bool vbInitialized;
};

/** @} */

#endif // __NAV_ESKF_H__
