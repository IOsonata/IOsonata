/**-------------------------------------------------------------------------
@file	imu_vqf.cpp

@brief	Implementation of software imu class using vqf fusion

Self contained, single precision optimized port of the Daniel Laidig VQF
orientation estimation algorithm. No external vqf source and no CMSIS-DSP
dependency.

Design notes
 - Main state (quaternion, bias, vectors) is float for speed on the FPU
   targets and to stay usable on the soft float RISC-V targets.
 - The mean initialised second order low pass states and their coefficients
   are kept in double. This is required for numerical stability of the filter;
   running those in float drifts. This is the one place double is mandatory.
 - All coefficients are computed once in Setup() at Init time. The per sample
   update path does no coefficient recomputation.
 - The full algorithm is retained: gyro strapdown, accel inclination
   correction, motion plus rest gyro bias estimation, and magnetometer
   disturbance rejection. Only the offline and batch helpers of the reference
   are dropped.

Reference: https://github.com/dlaidig/vqf

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
#include <math.h>
#include <string.h>

#include "imu/imu_vqf.h"

// Constants. Double precision constants are used where the low pass filter
// math runs in double.
#define VQF_PI		3.14159265358979323846
#define VQF_SQRT2	1.41421356237309504880
#define VQF_EPS		1.0e-9f
#define VQF_DEG2RAD	(float)(VQF_PI / 180.0)

namespace {

inline float Square(float x) { return x * x; }

inline float VecNorm(const float v[], size_t n)
{
	float s = 0.0f;
	for (size_t i = 0; i < n; i++) {
		s += v[i] * v[i];
	}
	return sqrtf(s);
}

inline void VecNormalize(float v[], size_t n)
{
	float nrm = VecNorm(v, n);
	if (nrm < VQF_EPS) {
		return;
	}
	for (size_t i = 0; i < n; i++) {
		v[i] /= nrm;
	}
}

inline void VecClip(float v[], size_t n, float lo, float hi)
{
	for (size_t i = 0; i < n; i++) {
		if (v[i] < lo) {
			v[i] = lo;
		} else if (v[i] > hi) {
			v[i] = hi;
		}
	}
}

// Hamilton product out = q1 * q2.
void QuatMultiply(const float q1[4], const float q2[4], float out[4])
{
	float w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	float x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	float y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	float z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
	out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

// Rotate vector v by quaternion q, out = q v q*.
void QuatRotate(const float q[4], const float v[3], float out[3])
{
	float x = (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) * v[0] + 2 * v[1] * (q[2] * q[1] - q[0] * q[3]) + 2 * v[2] * (q[0] * q[2] + q[3] * q[1]);
	float y = 2 * v[0] * (q[0] * q[3] + q[2] * q[1]) + v[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) + 2 * v[2] * (q[2] * q[3] - q[1] * q[0]);
	float z = 2 * v[0] * (q[3] * q[1] - q[0] * q[2]) + 2 * v[1] * (q[0] * q[1] + q[3] * q[2]) + v[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);
	out[0] = x; out[1] = y; out[2] = z;
}

// Apply a heading only rotation: out = [cos(d/2), 0, 0, sin(d/2)] * q.
void QuatApplyDelta(const float q[4], float delta, float out[4])
{
	float c = cosf(delta / 2);
	float s = sinf(delta / 2);
	float w = c * q[0] - s * q[3];
	float x = c * q[1] - s * q[2];
	float y = c * q[2] + s * q[1];
	float z = c * q[3] + s * q[0];
	out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

inline void QuatSetIdentity(float q[4])
{
	q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

void Mat3ScaledIdentity(float scale, float out[9])
{
	out[0] = scale; out[1] = 0; out[2] = 0;
	out[3] = 0; out[4] = scale; out[5] = 0;
	out[6] = 0; out[7] = 0; out[8] = scale;
}

// out = a * b
void Mat3Multiply(const float a[9], const float b[9], float out[9])
{
	float t[9];
	t[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
	t[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
	t[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
	t[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
	t[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
	t[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
	t[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
	t[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
	t[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
	memcpy(out, t, sizeof(t));
}

// out = a^T * b
void Mat3MultiplyTpsFirst(const float a[9], const float b[9], float out[9])
{
	float t[9];
	t[0] = a[0] * b[0] + a[3] * b[3] + a[6] * b[6];
	t[1] = a[0] * b[1] + a[3] * b[4] + a[6] * b[7];
	t[2] = a[0] * b[2] + a[3] * b[5] + a[6] * b[8];
	t[3] = a[1] * b[0] + a[4] * b[3] + a[7] * b[6];
	t[4] = a[1] * b[1] + a[4] * b[4] + a[7] * b[7];
	t[5] = a[1] * b[2] + a[4] * b[5] + a[7] * b[8];
	t[6] = a[2] * b[0] + a[5] * b[3] + a[8] * b[6];
	t[7] = a[2] * b[1] + a[5] * b[4] + a[8] * b[7];
	t[8] = a[2] * b[2] + a[5] * b[5] + a[8] * b[8];
	memcpy(out, t, sizeof(t));
}

// out = a * b^T
void Mat3MultiplyTpsSecond(const float a[9], const float b[9], float out[9])
{
	float t[9];
	t[0] = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	t[1] = a[0] * b[3] + a[1] * b[4] + a[2] * b[5];
	t[2] = a[0] * b[6] + a[1] * b[7] + a[2] * b[8];
	t[3] = a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
	t[4] = a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
	t[5] = a[3] * b[6] + a[4] * b[7] + a[5] * b[8];
	t[6] = a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
	t[7] = a[6] * b[3] + a[7] * b[4] + a[8] * b[5];
	t[8] = a[6] * b[6] + a[7] * b[7] + a[8] * b[8];
	memcpy(out, t, sizeof(t));
}

// Inverse of a 3x3 matrix, computed in double for conditioning. Returns false
// for a singular matrix.
bool Mat3Inv(const float in[9], float out[9])
{
	double A = (double)(in[4] * in[8] - in[5] * in[7]);
	double D = (double)(in[2] * in[7] - in[1] * in[8]);
	double G = (double)(in[1] * in[5] - in[2] * in[4]);
	double B = (double)(in[5] * in[6] - in[3] * in[8]);
	double E = (double)(in[0] * in[8] - in[2] * in[6]);
	double H = (double)(in[2] * in[3] - in[0] * in[5]);
	double C = (double)(in[3] * in[7] - in[4] * in[6]);
	double F = (double)(in[1] * in[6] - in[0] * in[7]);
	double I = (double)(in[0] * in[4] - in[1] * in[3]);

	double det = (double)in[0] * A + (double)in[1] * B + (double)in[2] * C;

	if (det > -1.0e-12 && det < 1.0e-12) {
		memset(out, 0, 9 * sizeof(float));
		return false;
	}

	out[0] = (float)(A / det); out[1] = (float)(D / det); out[2] = (float)(G / det);
	out[3] = (float)(B / det); out[4] = (float)(E / det); out[5] = (float)(H / det);
	out[6] = (float)(C / det); out[7] = (float)(F / det); out[8] = (float)(I / det);
	return true;
}

// First order gain from a time constant. k=0 disables update (tau<0), k=1 for
// tau=0, otherwise 1 - exp(-Ts/tau).
float GainFromTau(float tau, float Ts)
{
	if (tau < 0.0f) {
		return 0.0f;
	} else if (tau == 0.0f) {
		return 1.0f;
	}
	return 1.0f - expf(-Ts / tau);
}

// Second order Butterworth low pass coefficients (double). Passthrough when
// tau < Ts/2 to avoid instability above 90 percent of Nyquist.
void FilterCoeffs(float tau, float Ts, double b[3], double a[2])
{
	if (tau < Ts / 2) {
		b[0] = 1; b[1] = 0; b[2] = 0;
		a[0] = 0; a[1] = 0;
		return;
	}

	double fc = (VQF_SQRT2 / (2.0 * VQF_PI)) / (double)tau;
	double C = tan(VQF_PI * fc * (double)Ts);
	double D = C * C + VQF_SQRT2 * C + 1;
	double b0 = C * C / D;
	b[0] = b0;
	b[1] = 2 * b0;
	b[2] = b0;
	a[0] = 2 * (C * C - 1) / D;
	a[1] = (1 - VQF_SQRT2 * C + C * C) / D;
}

// Steady state initial filter state for input held at x0.
void FilterInitialState(float x0, const double b[3], const double a[2], double out[2])
{
	out[0] = (double)x0 * (1 - b[0]);
	out[1] = (double)x0 * (b[2] - a[1]);
}

// One direct form II transposed biquad step. a0 is assumed 1. State is double.
float FilterStep(float x, const double b[3], const double a[2], double state[2])
{
	double y = b[0] * (double)x + state[0];
	state[0] = b[1] * (double)x - a[0] * y + state[1];
	state[1] = b[2] * (double)x - a[1] * y;
	return (float)y;
}

} // anonymous namespace

ImuVqf::ImuVqf()
{
	vNbAxis = 9;
	vbInitialized = false;
	memset(&vState, 0, sizeof(vState));
	memset(&vCoeffs, 0, sizeof(vCoeffs));

	// Reference defaults.
	vParams.tauAcc = 3.0f;
	vParams.tauMag = 9.0f;
	vParams.motionBiasEstEnabled = true;
	vParams.restBiasEstEnabled = true;
	vParams.magDistRejectionEnabled = true;
	vParams.biasSigmaInit = 0.5f;
	vParams.biasForgettingTime = 100.0f;
	vParams.biasClip = 2.0f;
	vParams.biasSigmaMotion = 0.1f;
	vParams.biasVerticalForgettingFactor = 0.0001f;
	vParams.biasSigmaRest = 0.03f;
	vParams.restMinT = 1.5f;
	vParams.restFilterTau = 0.5f;
	vParams.restThGyr = 2.0f;
	vParams.restThAcc = 0.5f;
	vParams.magCurrentTau = 0.05f;
	vParams.magRefTau = 20.0f;
	vParams.magNormTh = 0.1f;
	vParams.magDipTh = 10.0f;
	vParams.magNewTime = 20.0f;
	vParams.magNewFirstTime = 5.0f;
	vParams.magNewMinGyr = 20.0f;
	vParams.magMinUndisturbedTime = 0.5f;
	vParams.magMaxRejectionTime = 60.0f;
	vParams.magRejectionFactor = 2.0f;
}

bool ImuVqf::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr) {
		return false;
	}

	if (Imu::Init(Cfg, pAccel, pGyro, pMag) == false) {
		return false;
	}

	// Sample periods in seconds. IOsonata SamplingFrequency() returns mHz,
	// so Ts = 1000 / f.
	float fg = (float)vpGyro->SamplingFrequency();	// mHz
	float fa = (float)vpAccel->SamplingFrequency();	// mHz
	float fm = (vpMag != nullptr) ? (float)vpMag->SamplingFrequency() : 0.0f;

	vCoeffs.gyrTs = (fg > 0.0f) ? 1000.0f / fg : 0.005f;
	vCoeffs.accTs = (fa > 0.0f) ? 1000.0f / fa : vCoeffs.gyrTs;
	vCoeffs.magTs = (fm > 0.0f) ? 1000.0f / fm : vCoeffs.gyrTs;

	vNbAxis = (pMag != nullptr) ? 9 : 6;

	Setup();
	Reset();

	vbInitialized = true;

	return true;
}

void ImuVqf::Setup(void)
{
	FilterCoeffs(vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA);

	vCoeffs.kMag = GainFromTau(vParams.tauMag, vCoeffs.magTs);

	vCoeffs.biasP0 = Square(vParams.biasSigmaInit * 100.0f);
	// System noise grows variance from 0 to (0.1 deg/s)^2 in biasForgettingTime.
	vCoeffs.biasV = Square(0.1f * 100.0f) * vCoeffs.accTs / vParams.biasForgettingTime;

	float pMotion = Square(vParams.biasSigmaMotion * 100.0f);
	vCoeffs.biasMotionW = Square(pMotion) / vCoeffs.biasV + pMotion;
	vCoeffs.biasVerticalW = vCoeffs.biasMotionW / fmaxf(vParams.biasVerticalForgettingFactor, 1.0e-10f);

	float pRest = Square(vParams.biasSigmaRest * 100.0f);
	vCoeffs.biasRestW = Square(pRest) / vCoeffs.biasV + pRest;

	FilterCoeffs(vParams.restFilterTau, vCoeffs.gyrTs, vCoeffs.restGyrLpB, vCoeffs.restGyrLpA);
	FilterCoeffs(vParams.restFilterTau, vCoeffs.accTs, vCoeffs.restAccLpB, vCoeffs.restAccLpA);

	vCoeffs.kMagRef = GainFromTau(vParams.magRefTau, vCoeffs.magTs);
	if (vParams.magCurrentTau > 0.0f) {
		FilterCoeffs(vParams.magCurrentTau, vCoeffs.magTs, vCoeffs.magNormDipLpB, vCoeffs.magNormDipLpA);
	} else {
		vCoeffs.magNormDipLpB[0] = NAN;
		vCoeffs.magNormDipLpB[1] = NAN;
		vCoeffs.magNormDipLpB[2] = NAN;
		vCoeffs.magNormDipLpA[0] = NAN;
		vCoeffs.magNormDipLpA[1] = NAN;
	}
}

void ImuVqf::Reset(void)
{
	QuatSetIdentity(vState.gyrQuat);
	QuatSetIdentity(vState.accQuat);
	vState.delta = 0.0f;

	vState.restDetected = false;
	vState.magDistDetected = true;

	memset(vState.lastAccLp, 0, sizeof(vState.lastAccLp));
	for (size_t i = 0; i < 3 * 2; i++) { vState.accLpState[i] = NAN; }
	vState.lastAccCorrAngularRate = 0.0f;

	vState.kMagInit = 1.0f;
	vState.lastMagDisAngle = 0.0f;
	vState.lastMagCorrAngularRate = 0.0f;

	memset(vState.bias, 0, sizeof(vState.bias));
	Mat3ScaledIdentity(vCoeffs.biasP0, vState.biasP);
	for (size_t i = 0; i < 9 * 2; i++) { vState.motionBiasEstRLpState[i] = NAN; }
	for (size_t i = 0; i < 2 * 2; i++) { vState.motionBiasEstBiasLpState[i] = NAN; }

	memset(vState.restLastSquaredDeviations, 0, sizeof(vState.restLastSquaredDeviations));
	vState.restT = 0.0f;
	memset(vState.restLastGyrLp, 0, sizeof(vState.restLastGyrLp));
	for (size_t i = 0; i < 3 * 2; i++) { vState.restGyrLpState[i] = NAN; }
	memset(vState.restLastAccLp, 0, sizeof(vState.restLastAccLp));
	for (size_t i = 0; i < 3 * 2; i++) { vState.restAccLpState[i] = NAN; }

	vState.magRefNorm = 0.0f;
	vState.magRefDip = 0.0f;
	vState.magUndisturbedT = 0.0f;
	vState.magRejectT = vParams.magMaxRejectionTime;
	vState.magCandidateNorm = -1.0f;
	vState.magCandidateDip = 0.0f;
	vState.magCandidateT = 0.0f;
	vState.magNormDip[0] = 0.0f;
	vState.magNormDip[1] = 0.0f;
	for (size_t i = 0; i < 2 * 2; i++) { vState.magNormDipLpState[i] = NAN; }
}

bool ImuVqf::Enable()
{
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void ImuVqf::Disable()
{
	if (vpMag) vpMag->Disable();
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
}

void ImuVqf::FilterVec(const float vec[], size_t n, float tau, float Ts, const double b[3],
                       const double a[2], double state[], float out[])
{
	// During the first tau seconds, average the samples and seed the filter
	// from that mean instead of trusting a single sample. state[0] is NaN
	// while in this initialisation phase.
	if (isnan(state[0])) {
		if (isnan(state[1])) {		// first sample
			state[1] = 0;		// sample count
			for (size_t i = 0; i < n; i++) {
				state[2 + i] = 0;	// running sum
			}
		}
		state[1]++;
		for (size_t i = 0; i < n; i++) {
			state[2 + i] += (double)vec[i];
			out[i] = (float)(state[2 + i] / state[1]);
		}
		if ((float)state[1] * Ts >= tau) {
			for (size_t i = 0; i < n; i++) {
				FilterInitialState(out[i], b, a, state + 2 * i);
			}
		}
		return;
	}

	for (size_t i = 0; i < n; i++) {
		out[i] = FilterStep(vec[i], b, a, state + 2 * i);
	}
}

void ImuVqf::UpdateGyr(const float gyr[3])
{
	// Rest detection low pass and threshold check.
	if (vParams.restBiasEstEnabled || vParams.magDistRejectionEnabled) {
		FilterVec(gyr, 3, vParams.restFilterTau, vCoeffs.gyrTs, vCoeffs.restGyrLpB,
		          vCoeffs.restGyrLpA, vState.restGyrLpState, vState.restLastGyrLp);

		vState.restLastSquaredDeviations[0] = Square(gyr[0] - vState.restLastGyrLp[0])
				+ Square(gyr[1] - vState.restLastGyrLp[1]) + Square(gyr[2] - vState.restLastGyrLp[2]);

		float biasClip = vParams.biasClip * VQF_DEG2RAD;
		if (vState.restLastSquaredDeviations[0] >= Square(vParams.restThGyr * VQF_DEG2RAD)
				|| fabsf(vState.restLastGyrLp[0]) > biasClip || fabsf(vState.restLastGyrLp[1]) > biasClip
				|| fabsf(vState.restLastGyrLp[2]) > biasClip) {
			vState.restT = 0.0f;
			vState.restDetected = false;
		}
	}

	// Remove estimated bias then integrate.
	float gyrNoBias[3] = { gyr[0] - vState.bias[0], gyr[1] - vState.bias[1], gyr[2] - vState.bias[2] };

	float gyrNorm = VecNorm(gyrNoBias, 3);
	float angle = gyrNorm * vCoeffs.gyrTs;
	if (gyrNorm > VQF_EPS) {
		float c = cosf(angle / 2);
		float s = sinf(angle / 2) / gyrNorm;
		float gyrStepQuat[4] = { c, s * gyrNoBias[0], s * gyrNoBias[1], s * gyrNoBias[2] };
		QuatMultiply(vState.gyrQuat, gyrStepQuat, vState.gyrQuat);
		VecNormalize(vState.gyrQuat, 4);
	}
}

void ImuVqf::UpdateAcc(const float acc[3])
{
	if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) {
		return;
	}

	// Rest detection from accel deviation.
	if (vParams.restBiasEstEnabled) {
		FilterVec(acc, 3, vParams.restFilterTau, vCoeffs.accTs, vCoeffs.restAccLpB,
		          vCoeffs.restAccLpA, vState.restAccLpState, vState.restLastAccLp);

		vState.restLastSquaredDeviations[1] = Square(acc[0] - vState.restLastAccLp[0])
				+ Square(acc[1] - vState.restLastAccLp[1]) + Square(acc[2] - vState.restLastAccLp[2]);

		if (vState.restLastSquaredDeviations[1] >= Square(vParams.restThAcc)) {
			vState.restT = 0.0f;
			vState.restDetected = false;
		} else {
			vState.restT += vCoeffs.accTs;
			if (vState.restT >= vParams.restMinT) {
				vState.restDetected = true;
			}
		}
	}

	float accEarth[3];

	// Low pass the accel in the inertial frame.
	QuatRotate(vState.gyrQuat, acc, accEarth);
	FilterVec(accEarth, 3, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA,
	          vState.accLpState, vState.lastAccLp);

	// Bring into the 6D earth frame and normalize.
	QuatRotate(vState.accQuat, vState.lastAccLp, accEarth);
	VecNormalize(accEarth, 3);

	// Inclination correction quaternion.
	float accCorrQuat[4];
	float q_w = sqrtf((accEarth[2] + 1) / 2);
	if (q_w > 1.0e-6f) {
		accCorrQuat[0] = q_w;
		accCorrQuat[1] = 0.5f * accEarth[1] / q_w;
		accCorrQuat[2] = -0.5f * accEarth[0] / q_w;
		accCorrQuat[3] = 0.0f;
	} else {
		accCorrQuat[0] = 0.0f;
		accCorrQuat[1] = 1.0f;
		accCorrQuat[2] = 0.0f;
		accCorrQuat[3] = 0.0f;
	}
	QuatMultiply(accCorrQuat, vState.accQuat, vState.accQuat);
	VecNormalize(vState.accQuat, 4);

	vState.lastAccCorrAngularRate = acosf(accEarth[2]) / vCoeffs.accTs;

	// Gyro bias estimation Kalman filter.
	if (vParams.motionBiasEstEnabled || vParams.restBiasEstEnabled) {
		float biasClip = vParams.biasClip * VQF_DEG2RAD;

		float accGyrQuat[4];
		float R[9];
		float biasLp[2];

		GetQuat6D(accGyrQuat);
		R[0] = 1 - 2 * Square(accGyrQuat[2]) - 2 * Square(accGyrQuat[3]);
		R[1] = 2 * (accGyrQuat[2] * accGyrQuat[1] - accGyrQuat[0] * accGyrQuat[3]);
		R[2] = 2 * (accGyrQuat[0] * accGyrQuat[2] + accGyrQuat[3] * accGyrQuat[1]);
		R[3] = 2 * (accGyrQuat[0] * accGyrQuat[3] + accGyrQuat[2] * accGyrQuat[1]);
		R[4] = 1 - 2 * Square(accGyrQuat[1]) - 2 * Square(accGyrQuat[3]);
		R[5] = 2 * (accGyrQuat[2] * accGyrQuat[3] - accGyrQuat[1] * accGyrQuat[0]);
		R[6] = 2 * (accGyrQuat[3] * accGyrQuat[1] - accGyrQuat[0] * accGyrQuat[2]);
		R[7] = 2 * (accGyrQuat[0] * accGyrQuat[1] + accGyrQuat[3] * accGyrQuat[2]);
		R[8] = 1 - 2 * Square(accGyrQuat[1]) - 2 * Square(accGyrQuat[2]);

		biasLp[0] = R[0] * vState.bias[0] + R[1] * vState.bias[1] + R[2] * vState.bias[2];
		biasLp[1] = R[3] * vState.bias[0] + R[4] * vState.bias[1] + R[5] * vState.bias[2];

		FilterVec(R, 9, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA,
		          vState.motionBiasEstRLpState, R);
		FilterVec(biasLp, 2, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA,
		          vState.motionBiasEstBiasLpState, biasLp);

		float w[3];
		float e[3];
		if (vState.restDetected && vParams.restBiasEstEnabled) {
			e[0] = vState.restLastGyrLp[0] - vState.bias[0];
			e[1] = vState.restLastGyrLp[1] - vState.bias[1];
			e[2] = vState.restLastGyrLp[2] - vState.bias[2];
			Mat3ScaledIdentity(1.0f, R);
			w[0] = w[1] = w[2] = vCoeffs.biasRestW;
		} else if (vParams.motionBiasEstEnabled) {
			e[0] = -accEarth[1] / vCoeffs.accTs + biasLp[0] - R[0] * vState.bias[0] - R[1] * vState.bias[1] - R[2] * vState.bias[2];
			e[1] = accEarth[0] / vCoeffs.accTs + biasLp[1] - R[3] * vState.bias[0] - R[4] * vState.bias[1] - R[5] * vState.bias[2];
			e[2] = -R[6] * vState.bias[0] - R[7] * vState.bias[1] - R[8] * vState.bias[2];
			w[0] = vCoeffs.biasMotionW;
			w[1] = vCoeffs.biasMotionW;
			w[2] = vCoeffs.biasVerticalW;
		} else {
			w[0] = w[1] = w[2] = -1.0f;	// disable update
		}

		// Step 1: P = P + V, also when there is no measurement update.
		if (vState.biasP[0] < vCoeffs.biasP0) { vState.biasP[0] += vCoeffs.biasV; }
		if (vState.biasP[4] < vCoeffs.biasP0) { vState.biasP[4] += vCoeffs.biasV; }
		if (vState.biasP[8] < vCoeffs.biasP0) { vState.biasP[8] += vCoeffs.biasV; }

		if (w[0] >= 0.0f) {
			VecClip(e, 3, -biasClip, biasClip);

			// Step 2: K = P R^T inv(W + R P R^T)
			float K[9];
			Mat3MultiplyTpsSecond(vState.biasP, R, K);	// K = P R^T
			Mat3Multiply(R, K, K);				// K = R P R^T
			K[0] += w[0];
			K[4] += w[1];
			K[8] += w[2];					// K = W + R P R^T
			Mat3Inv(K, K);					// K = inv(W + R P R^T)
			Mat3MultiplyTpsFirst(R, K, K);			// K = R^T inv(...)
			Mat3Multiply(vState.biasP, K, K);		// K = P R^T inv(...)

			// Step 3: bias = bias + K e
			vState.bias[0] += K[0] * e[0] + K[1] * e[1] + K[2] * e[2];
			vState.bias[1] += K[3] * e[0] + K[4] * e[1] + K[5] * e[2];
			vState.bias[2] += K[6] * e[0] + K[7] * e[1] + K[8] * e[2];

			// Step 4: P = P - K R P
			Mat3Multiply(K, R, K);				// K = K R
			Mat3Multiply(K, vState.biasP, K);		// K = K R P
			for (size_t i = 0; i < 9; i++) {
				vState.biasP[i] -= K[i];
			}

			VecClip(vState.bias, 3, -biasClip, biasClip);
		}
	}
}

void ImuVqf::UpdateMag(const float mag[3])
{
	if (mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f) {
		return;
	}

	float magEarth[3];
	float accGyrQuat[4];
	GetQuat6D(accGyrQuat);
	QuatRotate(accGyrQuat, mag, magEarth);

	if (vParams.magDistRejectionEnabled) {
		vState.magNormDip[0] = VecNorm(magEarth, 3);
		vState.magNormDip[1] = -asinf(magEarth[2] / vState.magNormDip[0]);

		if (vParams.magCurrentTau > 0.0f) {
			FilterVec(vState.magNormDip, 2, vParams.magCurrentTau, vCoeffs.magTs,
			          vCoeffs.magNormDipLpB, vCoeffs.magNormDipLpA, vState.magNormDipLpState, vState.magNormDip);
		}

		// Disturbance detection against the tracked reference.
		if (fabsf(vState.magNormDip[0] - vState.magRefNorm) < vParams.magNormTh * vState.magRefNorm
				&& fabsf(vState.magNormDip[1] - vState.magRefDip) < vParams.magDipTh * VQF_DEG2RAD) {
			vState.magUndisturbedT += vCoeffs.magTs;
			if (vState.magUndisturbedT >= vParams.magMinUndisturbedTime) {
				vState.magDistDetected = false;
				vState.magRefNorm += vCoeffs.kMagRef * (vState.magNormDip[0] - vState.magRefNorm);
				vState.magRefDip += vCoeffs.kMagRef * (vState.magNormDip[1] - vState.magRefDip);
			}
		} else {
			vState.magUndisturbedT = 0.0f;
			vState.magDistDetected = true;
		}

		// New magnetic field acceptance.
		if (fabsf(vState.magNormDip[0] - vState.magCandidateNorm) < vParams.magNormTh * vState.magCandidateNorm
				&& fabsf(vState.magNormDip[1] - vState.magCandidateDip) < vParams.magDipTh * VQF_DEG2RAD) {
			if (VecNorm(vState.restLastGyrLp, 3) >= vParams.magNewMinGyr * VQF_DEG2RAD) {
				vState.magCandidateT += vCoeffs.magTs;
			}
			vState.magCandidateNorm += vCoeffs.kMagRef * (vState.magNormDip[0] - vState.magCandidateNorm);
			vState.magCandidateDip += vCoeffs.kMagRef * (vState.magNormDip[1] - vState.magCandidateDip);

			if (vState.magDistDetected && (vState.magCandidateT >= vParams.magNewTime
					|| (vState.magRefNorm == 0.0f && vState.magCandidateT >= vParams.magNewFirstTime))) {
				vState.magRefNorm = vState.magCandidateNorm;
				vState.magRefDip = vState.magCandidateDip;
				vState.magDistDetected = false;
				vState.magUndisturbedT = vParams.magMinUndisturbedTime;
			}
		} else {
			vState.magCandidateT = 0.0f;
			vState.magCandidateNorm = vState.magNormDip[0];
			vState.magCandidateDip = vState.magNormDip[1];
		}
	}

	// Heading disagreement angle, wrapped to [-pi, pi].
	vState.lastMagDisAngle = atan2f(magEarth[0], magEarth[1]) - vState.delta;
	if (vState.lastMagDisAngle > (float)VQF_PI) {
		vState.lastMagDisAngle -= (float)(2 * VQF_PI);
	} else if (vState.lastMagDisAngle < -(float)VQF_PI) {
		vState.lastMagDisAngle += (float)(2 * VQF_PI);
	}

	float k = vCoeffs.kMag;

	if (vParams.magDistRejectionEnabled) {
		if (vState.magDistDetected) {
			if (vState.magRejectT <= vParams.magMaxRejectionTime) {
				vState.magRejectT += vCoeffs.magTs;
				k = 0.0f;
			} else {
				k /= vParams.magRejectionFactor;
			}
		} else {
			vState.magRejectT = fmaxf(vState.magRejectT - vParams.magRejectionFactor * vCoeffs.magTs, 0.0f);
		}
	}

	// Fast initial convergence: keep k at least 1/N for the first samples.
	if (vState.kMagInit != 0.0f) {
		if (k < vState.kMagInit) {
			k = vState.kMagInit;
		}
		vState.kMagInit = vState.kMagInit / (vState.kMagInit + 1);
		if (vState.kMagInit * vParams.tauMag < vCoeffs.magTs) {
			vState.kMagInit = 0.0f;
		}
	}

	// First order heading correction, wrapped to [-pi, pi].
	vState.delta += k * vState.lastMagDisAngle;
	vState.lastMagCorrAngularRate = k * vState.lastMagDisAngle / vCoeffs.magTs;
	if (vState.delta > (float)VQF_PI) {
		vState.delta -= (float)(2 * VQF_PI);
	} else if (vState.delta < -(float)VQF_PI) {
		vState.delta += (float)(2 * VQF_PI);
	}
}

void ImuVqf::GetQuat6D(float out[4]) const
{
	QuatMultiply(vState.accQuat, vState.gyrQuat, out);
}

void ImuVqf::GetQuat9D(float out[4]) const
{
	QuatMultiply(vState.accQuat, vState.gyrQuat, out);
	QuatApplyDelta(out, vState.delta, out);
}

bool ImuVqf::UpdateData()
{
	if (vbInitialized == false) {
		return false;
	}

	AccelSensorData_t accel;
	GyroSensorData_t gyro;

	vpAccel->Read(accel);
	vpGyro->Read(gyro);

	// VQF expects gyro in rad/s. IOsonata gyro data is in deg/s.
	float g[3] = { gyro.X * VQF_DEG2RAD, gyro.Y * VQF_DEG2RAD, gyro.Z * VQF_DEG2RAD };
	// Accel is normalized inside VQF, so g or m/s^2 are both fine.
	float a[3] = { accel.X, accel.Y, accel.Z };

	UpdateGyr(g);
	UpdateAcc(a);

	if (vpMag != nullptr && vNbAxis == 9) {
		MagSensorData_t mag;
		vpMag->Read(mag);
		float m[3] = { mag.X, mag.Y, mag.Z };
		UpdateMag(m);
	}

	if (vNbAxis == 9) {
		GetQuat9D(vQuat.Q);
	} else {
		GetQuat6D(vQuat.Q);
	}
	vQuat.Timestamp = gyro.Timestamp;

	// Euler from quaternion, ZYX convention, radians.
	float *q = vQuat.Q;
	vEuler.Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	float sp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	sp = fmaxf(-1.0f, fminf(1.0f, sp));
	vEuler.Pitch = asinf(sp);
	vEuler.Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
	vEuler.Timestamp = gyro.Timestamp;

	return true;
}

bool ImuVqf::Read(ImuQuat_t &Data)
{
	Data = vQuat;
	return true;
}

void ImuVqf::IntHandler()
{
	UpdateData();
}

bool ImuVqf::Quaternion(bool bEn, int NbAxis)
{
	if (NbAxis == 9 && vpMag != nullptr) {
		vNbAxis = 9;
	} else {
		vNbAxis = 6;
	}
	return true;
}

bool ImuVqf::Compass(bool bEn)
{
	if (bEn && vpMag == nullptr) {
		return false;
	}
	vNbAxis = bEn ? 9 : 6;
	return true;
}

bool ImuVqf::Calibrate()
{
	return false;
}

void ImuVqf::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
	// Axis alignment is applied at the sensor driver level in IOsonata.
}

bool ImuVqf::Pedometer(bool bEn)
{
	return false;
}

bool ImuVqf::Tap(bool bEn)
{
	return false;
}
