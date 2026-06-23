/**-------------------------------------------------------------------------
@file	ahrs_mahony.cpp

@brief	Implementation of the Ahrs class using Mahony fusion

Self contained Mahony explicit complementary filter. State is a unit quaternion
(body to earth) plus an integral feedback term that tracks the gyro bias. Each
step computes the error between the measured accel direction and the predicted
gravity direction as a cross product, feeds it back through a PI gain into the
gyro rate, and integrates the quaternion.

The correction direction matches the MEKF gravity update geometry, so the
filter aligns roll and pitch to gravity and leaves heading unobservable in
6-axis. Shared math comes from fusion_math.h.

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
#include <math.h>
#include <string.h>

#include "fusion/ahrs_mahony.h"
#include "fusion/fusion_math.h"

#define MAHONY_DEG2RAD	0.01745329251994329577f

using namespace FusionMath;

AhrsMahony::AhrsMahony()
{
	vNbAxis = 6;
	vbInitialized = false;
	vGyrDt = 0.005f;
	vAccDt = 0.005f;
	memset(&vState, 0, sizeof(vState));

	vParams.Kp = 1.0f;
	vParams.Ki = 0.1f;
	vParams.initSamples = 50;
	vParams.accGateLo = 0.1f;
	vParams.accGateHi = 5.0f;
}

bool AhrsMahony::Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr) {
		return false;
	}

	if (Ahrs::Init(Cfg, pAccel, pGyro, pMag) == false) {
		return false;
	}

	// IOsonata SamplingFrequency() returns mHz, so Ts = 1000 / f.
	float fg = (float)vpGyro->SamplingFrequency();
	float fa = (float)vpAccel->SamplingFrequency();
	vGyrDt = (fg > 0.0f) ? 1000.0f / fg : 0.005f;
	vAccDt = (fa > 0.0f) ? 1000.0f / fa : vGyrDt;

	vNbAxis = 6;	// mag path not yet enabled

	Setup();
	Reset();

	vbInitialized = true;

	return true;
}

void AhrsMahony::Setup(void)
{
	// Gains are applied directly from vParams each step. Nothing to precompute
	// beyond the sample periods captured in Init.
}

void AhrsMahony::Reset(void)
{
	memset(&vState, 0, sizeof(vState));
	vState.q[0] = 1.0f;	// identity quaternion
	vState.mode = 0;	// init accumulate
}

bool AhrsMahony::Enable()
{
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void AhrsMahony::Disable()
{
	if (vpMag) vpMag->Disable();
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
}

void AhrsMahony::GravityInit(const float accAvg[3])
{
	float A[9];
	DcmFromGravity(accAvg, A);
	MatToQuat(A, vState.q);
	memset(vState.integralFB, 0, sizeof(vState.integralFB));
}

bool AhrsMahony::UpdateData()
{
	if (vbInitialized == false) {
		return false;
	}

	AccelSensorData_t accel;
	GyroSensorData_t gyro;

	vpAccel->Read(accel);
	vpGyro->Read(gyro);

	float a[3] = { accel.X, accel.Y, accel.Z };	// g
	// Mahony expects gyro in rad/s. IOsonata gyro data is in deg/s.
	float w[3] = { gyro.X * MAHONY_DEG2RAD, gyro.Y * MAHONY_DEG2RAD, gyro.Z * MAHONY_DEG2RAD };

	if (vState.mode == 0) {
		// Accumulate accel for the initial attitude.
		vState.accSum[0] += a[0];
		vState.accSum[1] += a[1];
		vState.accSum[2] += a[2];
		vState.accInitCount++;
		if (vState.accInitCount >= vParams.initSamples) {
			float inv = 1.0f / (float)vState.accInitCount;
			float aa[3] = { vState.accSum[0] * inv, vState.accSum[1] * inv, vState.accSum[2] * inv };
			GravityInit(aa);
			vState.mode = 1;
		}
		memcpy(vQuat.Q, vState.q, sizeof(vQuat.Q));
		vQuat.Timestamp = gyro.Timestamp;
		return true;
	}

	// Accel error as a cross product between measured and predicted gravity
	// direction. Skipped when the accel norm is out of gate (linear accel).
	float e[3] = { 0.0f, 0.0f, 0.0f };
	float anorm = Norm3(a);
	if (anorm >= vParams.accGateLo && anorm <= vParams.accGateHi) {
		float au[3] = { a[0], a[1], a[2] };
		Normalize3(au);
		float Rbe[9];
		QuatToDcmBE(vState.q, Rbe);
		float dref[3] = { 0.0f, 0.0f, 1.0f };
		float pred[3];
		Mat3TVec(Rbe, dref, pred);	// predicted body gravity direction
		Cross3(au, pred, e);
		if (vParams.Ki > 0.0f) {
			vState.integralFB[0] += vParams.Ki * e[0] * vGyrDt;
			vState.integralFB[1] += vParams.Ki * e[1] * vGyrDt;
			vState.integralFB[2] += vParams.Ki * e[2] * vGyrDt;
		}
	}

	// PI corrected body rate, then integrate the quaternion.
	float wc[3] = {
		w[0] + vState.integralFB[0] + vParams.Kp * e[0],
		w[1] + vState.integralFB[1] + vParams.Kp * e[1],
		w[2] + vState.integralFB[2] + vParams.Kp * e[2]
	};
	float phi[3] = { wc[0] * vGyrDt, wc[1] * vGyrDt, wc[2] * vGyrDt };
	float dq[4];
	QuatFromRotVec(phi, dq);
	float qn[4];
	QuatMul(vState.q, dq, qn);
	QuatNormalize(qn);
	memcpy(vState.q, qn, sizeof(qn));

	memcpy(vQuat.Q, vState.q, sizeof(vQuat.Q));
	vQuat.Timestamp = gyro.Timestamp;

	// Euler from quaternion, ZYX, radians.
	float *q = vQuat.Q;
	vEuler.Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	float sp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	sp = fmaxf(-1.0f, fminf(1.0f, sp));
	vEuler.Pitch = asinf(sp);
	vEuler.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
	vEuler.Timestamp = gyro.Timestamp;

	return true;
}

bool AhrsMahony::Read(AhrsQuat_t &Data)
{
	Data = vQuat;
	return true;
}

void AhrsMahony::IntHandler()
{
	// Refresh the bound sensors then fuse, so this object works as a drop-in
	// pImuDev whose IntHandler is the data-ready entry point. When the caller
	// refreshes the sensors itself and calls UpdateData() directly, do not
	// call this.
	if (vpAccel) vpAccel->IntHandler();
	if (vpGyro) vpGyro->IntHandler();
	if (vpMag) vpMag->IntHandler();
	UpdateData();
}

bool AhrsMahony::Quaternion(bool bEn, int NbAxis)
{
	// Mag path not yet enabled; Mahony runs 6-axis.
	vNbAxis = 6;
	return true;
}

bool AhrsMahony::Compass(bool bEn)
{
	return false;
}

bool AhrsMahony::Calibrate()
{
	return false;
}

void AhrsMahony::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool AhrsMahony::Pedometer(bool bEn)
{
	return false;
}

bool AhrsMahony::Tap(bool bEn)
{
	return false;
}
