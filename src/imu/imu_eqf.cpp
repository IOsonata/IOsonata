/**-------------------------------------------------------------------------
@file	imu_eqf.cpp

@brief	Implementation of software imu class using EqF fusion

Self contained, single precision port of the ABC-EqF n=0 (Attitude-Bias
Equivariant Filter). No external source and no CMSIS-DSP dependency.

Design notes
 - State is the SO(3) attitude A and the gyro bias carried in the group Lie
   algebra as aVec; the body bias is b = -A^T aVec. The error covariance P is
   6x6, single precision.
 - Propagation lifts the gyro input, advances A through the group exponential
   (Rodrigues plus the SO(3) left Jacobian), and advances P with the state
   transition matrix Phi and a Riccati update.
 - Updates are direction measurements. The accel gives the gravity direction;
   in 6-axis mode the heading and bias gains are suppressed so the accel only
   levels the attitude, and bias is estimated during detected rest.
 - The covariance is symmetrised and A is re-orthonormalised periodically to
   control float32 drift. The covariance update uses Joseph form.
 - All coefficients are static; the per sample path does no allocation.

Reference: Fornasier et al., RA-L 2022 (ABC-EqF). The 6-axis realisation and
the rest-detection front-end follow the public n=0 implementation.

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

#include "imu/imu_eqf.h"

#define EQF_DEG2RAD	0.01745329251994329577f

namespace {

inline float Dot3(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline float Norm3(const float v[3])
{
	return sqrtf(Dot3(v, v));
}

inline void Cross3(const float a[3], const float b[3], float out[3])
{
	float x = a[1] * b[2] - a[2] * b[1];
	float y = a[2] * b[0] - a[0] * b[2];
	float z = a[0] * b[1] - a[1] * b[0];
	out[0] = x; out[1] = y; out[2] = z;
}

inline void Normalize3(float v[3])
{
	float n = Norm3(v);
	if (n > 1.0e-10f) {
		float inv = 1.0f / n;
		v[0] *= inv; v[1] *= inv; v[2] *= inv;
	}
}

inline void Mat3Eye(float M[9])
{
	M[0] = 1; M[1] = 0; M[2] = 0;
	M[3] = 0; M[4] = 1; M[5] = 0;
	M[6] = 0; M[7] = 0; M[8] = 1;
}

inline void Mat3Copy(float dst[9], const float src[9])
{
	memcpy(dst, src, 9 * sizeof(float));
}

// C = A * B
void Mat3Mul(const float A[9], const float B[9], float C[9])
{
	float t[9];
	t[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	t[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	t[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	t[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	t[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	t[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	t[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	t[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	t[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
	memcpy(C, t, sizeof(t));
}

// C = A * B^T
void Mat3MulBT(const float A[9], const float B[9], float C[9])
{
	float t[9];
	t[0] = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
	t[1] = A[0] * B[3] + A[1] * B[4] + A[2] * B[5];
	t[2] = A[0] * B[6] + A[1] * B[7] + A[2] * B[8];
	t[3] = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
	t[4] = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];
	t[5] = A[3] * B[6] + A[4] * B[7] + A[5] * B[8];
	t[6] = A[6] * B[0] + A[7] * B[1] + A[8] * B[2];
	t[7] = A[6] * B[3] + A[7] * B[4] + A[8] * B[5];
	t[8] = A[6] * B[6] + A[7] * B[7] + A[8] * B[8];
	memcpy(C, t, sizeof(t));
}

void Mat3Transpose(const float A[9], float out[9])
{
	float t[9];
	t[0] = A[0]; t[1] = A[3]; t[2] = A[6];
	t[3] = A[1]; t[4] = A[4]; t[5] = A[7];
	t[6] = A[2]; t[7] = A[5]; t[8] = A[8];
	memcpy(out, t, sizeof(t));
}

// out = M * v
inline void Mat3Vec(const float M[9], const float v[3], float out[3])
{
	float x = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
	float y = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
	float z = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
	out[0] = x; out[1] = y; out[2] = z;
}

// out = M^T * v
inline void Mat3TVec(const float M[9], const float v[3], float out[3])
{
	float x = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
	float y = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
	float z = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
	out[0] = x; out[1] = y; out[2] = z;
}

// Skew symmetric matrix of v (wedge).
inline void Skew3(const float v[3], float M[9])
{
	M[0] = 0;     M[1] = -v[2]; M[2] = v[1];
	M[3] = v[2];  M[4] = 0;     M[5] = -v[0];
	M[6] = -v[1]; M[7] = v[0];  M[8] = 0;
}

// Square of the skew of a unit direction: skew(d)^2 = d d^T - I.
inline void Skew3Sq(const float d[3], float M[9])
{
	M[0] = d[0] * d[0] - 1.0f; M[1] = d[0] * d[1];        M[2] = d[0] * d[2];
	M[3] = d[1] * d[0];        M[4] = d[1] * d[1] - 1.0f; M[5] = d[1] * d[2];
	M[6] = d[2] * d[0];        M[7] = d[2] * d[1];        M[8] = d[2] * d[2] - 1.0f;
}

inline void Mat3Sym(float M[9])
{
	float a;
	a = 0.5f * (M[1] + M[3]); M[1] = M[3] = a;
	a = 0.5f * (M[2] + M[6]); M[2] = M[6] = a;
	a = 0.5f * (M[5] + M[7]); M[5] = M[7] = a;
}

// SO(3) exponential. v = angle * axis -> rotation matrix R.
void Rodrigues(const float v[3], float R[9])
{
	float angle = Norm3(v);
	if (angle < 1.0e-8f) {
		Mat3Eye(R);
		R[1] -= v[2]; R[2] += v[1];
		R[3] += v[2]; R[5] -= v[0];
		R[6] -= v[1]; R[7] += v[0];
		return;
	}
	float c = cosf(angle), s = sinf(angle), t = 1.0f - c;
	float ia = 1.0f / angle;
	float x = v[0] * ia, y = v[1] * ia, z = v[2] * ia;
	R[0] = c + t * x * x;     R[1] = t * x * y - s * z; R[2] = t * x * z + s * y;
	R[3] = t * y * x + s * z; R[4] = c + t * y * y;     R[5] = t * y * z - s * x;
	R[6] = t * z * x - s * y; R[7] = t * z * y + s * x; R[8] = c + t * z * z;
}

// SO(3) left Jacobian.
void So3LeftJ(const float v[3], float J[9])
{
	float angle = Norm3(v);
	if (angle < 1.0e-6f) {
		Mat3Eye(J);
		float h = 0.5f;
		J[1] -= h * v[2]; J[2] += h * v[1];
		J[3] += h * v[2]; J[5] -= h * v[0];
		J[6] -= h * v[1]; J[7] += h * v[0];
		return;
	}
	float s = sinf(angle), c = cosf(angle), ia = 1.0f / angle;
	float x = v[0] * ia, y = v[1] * ia, z = v[2] * ia;
	float sa = s * ia;
	float ms = 1.0f - sa;
	float mc = (1.0f - c) * ia;
	J[0] = sa + ms * x * x;     J[1] = ms * x * y - mc * z; J[2] = ms * x * z + mc * y;
	J[3] = ms * y * x + mc * z; J[4] = sa + ms * y * y;     J[5] = ms * y * z - mc * x;
	J[6] = ms * z * x - mc * y; J[7] = ms * z * y + mc * x; J[8] = sa + ms * z * z;
}

// Gram-Schmidt re-orthonormalise a row-major rotation matrix.
void Reortho(float A[9])
{
	float r0[3] = { A[0], A[1], A[2] };
	float r1[3] = { A[3], A[4], A[5] };
	float r2[3];
	Normalize3(r0);
	float d = Dot3(r1, r0);
	r1[0] -= d * r0[0]; r1[1] -= d * r0[1]; r1[2] -= d * r0[2];
	Normalize3(r1);
	Cross3(r0, r1, r2);
	A[0] = r0[0]; A[1] = r0[1]; A[2] = r0[2];
	A[3] = r1[0]; A[4] = r1[1]; A[5] = r1[2];
	A[6] = r2[0]; A[7] = r2[1]; A[8] = r2[2];
}

// 3x3 inverse with a scale-aware determinant guard. Returns false if singular.
bool Mat3Inv(const float M[9], float out[9])
{
	float det = M[0] * (M[4] * M[8] - M[5] * M[7])
		  - M[1] * (M[3] * M[8] - M[5] * M[6])
		  + M[2] * (M[3] * M[7] - M[4] * M[6]);
	float dmax = fabsf(M[0]);
	if (fabsf(M[4]) > dmax) dmax = fabsf(M[4]);
	if (fabsf(M[8]) > dmax) dmax = fabsf(M[8]);
	float thr = 1.0e-8f * dmax * dmax * dmax;
	if (thr < 1.0e-20f) thr = 1.0e-20f;
	if (fabsf(det) < thr) {
		return false;
	}
	float id = 1.0f / det;
	out[0] = (M[4] * M[8] - M[5] * M[7]) * id;
	out[1] = (M[2] * M[7] - M[1] * M[8]) * id;
	out[2] = (M[1] * M[5] - M[2] * M[4]) * id;
	out[3] = (M[5] * M[6] - M[3] * M[8]) * id;
	out[4] = (M[0] * M[8] - M[2] * M[6]) * id;
	out[5] = (M[2] * M[3] - M[0] * M[5]) * id;
	out[6] = (M[3] * M[7] - M[4] * M[6]) * id;
	out[7] = (M[1] * M[6] - M[0] * M[7]) * id;
	out[8] = (M[0] * M[4] - M[1] * M[3]) * id;
	return true;
}

// Rotation matrix to quaternion [w, x, y, z], Shepperd's method, positive w.
void MatToQuat(const float R[9], float q[4])
{
	float tr = R[0] + R[4] + R[8];
	if (tr > 0.0f) {
		float s = 0.5f / sqrtf(tr + 1.0f);
		q[0] = 0.25f / s;
		q[1] = (R[7] - R[5]) * s;
		q[2] = (R[2] - R[6]) * s;
		q[3] = (R[3] - R[1]) * s;
	} else if (R[0] > R[4] && R[0] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
		q[0] = (R[7] - R[5]) / s;
		q[1] = 0.25f * s;
		q[2] = (R[1] + R[3]) / s;
		q[3] = (R[2] + R[6]) / s;
	} else if (R[4] > R[8]) {
		float s = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
		q[0] = (R[2] - R[6]) / s;
		q[1] = (R[1] + R[3]) / s;
		q[2] = 0.25f * s;
		q[3] = (R[5] + R[7]) / s;
	} else {
		float s = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
		q[0] = (R[3] - R[1]) / s;
		q[1] = (R[2] + R[6]) / s;
		q[2] = (R[5] + R[7]) / s;
		q[3] = 0.25f * s;
	}
	if (q[0] < 0.0f) {
		q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
	}
}

// 6x6 covariance is stored row-major as P[36]; access 3x3 blocks (bi,bj).
inline void PGet(const float P[36], int bi, int bj, float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		const float *row = P + (r + i) * 6 + c;
		B[i * 3 + 0] = row[0];
		B[i * 3 + 1] = row[1];
		B[i * 3 + 2] = row[2];
	}
}

inline void PSet(float P[36], int bi, int bj, const float B[9])
{
	int r = bi * 3, c = bj * 3;
	for (int i = 0; i < 3; i++) {
		float *row = P + (r + i) * 6 + c;
		row[0] = B[i * 3 + 0];
		row[1] = B[i * 3 + 1];
		row[2] = B[i * 3 + 2];
	}
}

} // anonymous namespace

ImuEqf::ImuEqf()
{
	vNbAxis = 6;
	vbInitialized = false;
	vGyrDt = 0.005f;
	vAccDt = 0.005f;
	memset(&vState, 0, sizeof(vState));

	vParams.sigmaW = 0.003f;
	vParams.sigmaB = 0.0003f;
	vParams.sigmaAcc = 0.01f;
	vParams.accAdaptK = 100.0f;
	vParams.sixAxisAccScale = 50.0f;
	vParams.initSamples = 50;
	vParams.orthoInterval = 100;
	vParams.pInitAtt = 0.1f;
	vParams.pInitBias = 0.01f;
	vParams.restTau = 0.5f;
	vParams.restThGyr = 2.0f;
	vParams.restThAcc = 0.5f;
	vParams.restMinT = 1.5f;
	vParams.restSigma = 0.03f;
	vParams.restMaxBias = 10.0f;
	vParams.restAccNormTh = 0.1f;
}

bool ImuEqf::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr) {
		return false;
	}

	if (Imu::Init(Cfg, pAccel, pGyro, pMag) == false) {
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

void ImuEqf::Setup(void)
{
	// All EqF coefficients are static; nothing to precompute beyond the
	// sample periods captured in Init. Kept for symmetry with the interface.
}

void ImuEqf::Reset(void)
{
	memset(&vState, 0, sizeof(vState));
	Mat3Eye(vState.A);
	vState.mode = 0;	// init accumulate
}

bool ImuEqf::Enable()
{
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void ImuEqf::Disable()
{
	if (vpMag) vpMag->Disable();
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
}

void ImuEqf::GravityInit(const float accAvg[3])
{
	float gn = Norm3(accAvg);
	if (gn < 1.0e-6f) {
		Mat3Eye(vState.A);
	} else {
		float inv = 1.0f / gn;
		float b1[3] = { accAvg[0] * inv, accAvg[1] * inv, accAvg[2] * inv };

		float ref[3] = { 1.0f, 0.0f, 0.0f };
		if (fabsf(Dot3(b1, ref)) > 0.9f) {
			ref[0] = 0.0f; ref[1] = 1.0f; ref[2] = 0.0f;
		}
		float b2[3];
		Cross3(b1, ref, b2);
		float b2n = Norm3(b2);
		if (b2n < 1.0e-6f) {
			Mat3Eye(vState.A);
		} else {
			inv = 1.0f / b2n;
			b2[0] *= inv; b2[1] *= inv; b2[2] *= inv;
			float b3[3];
			Cross3(b1, b2, b3);
			vState.A[0] = -b3[0]; vState.A[1] = -b3[1]; vState.A[2] = -b3[2];
			vState.A[3] =  b2[0]; vState.A[4] =  b2[1]; vState.A[5] =  b2[2];
			vState.A[6] =  b1[0]; vState.A[7] =  b1[1]; vState.A[8] =  b1[2];
		}
	}

	memset(vState.aVec, 0, sizeof(vState.aVec));
	memset(vState.P, 0, sizeof(vState.P));
	// Earth-z (yaw) is unobservable in 6-axis; start with a large variance.
	vState.P[0] = vParams.pInitAtt;
	vState.P[7] = vParams.pInitAtt;
	vState.P[14] = 3.14f * 3.14f;
	for (int i = 3; i < 6; i++) {
		vState.P[i * 6 + i] = vParams.pInitBias;
	}
}

void ImuEqf::Propagate(const float w[3], float dt)
{
	// b = -A^T aVec
	float bh[3];
	Mat3TVec(vState.A, vState.aVec, bh);
	bh[0] = -bh[0]; bh[1] = -bh[1]; bh[2] = -bh[2];

	// Lift: om = w - b, lb = b x w
	float om[3] = { w[0] - bh[0], w[1] - bh[1], w[2] - bh[2] };
	float lb[3];
	Cross3(bh, w, lb);

	float lrDt[3] = { om[0] * dt, om[1] * dt, om[2] * dt };
	float lbDt[3] = { lb[0] * dt, lb[1] * dt, lb[2] * dt };

	// State update: X = X * exp(L dt)
	float dA[9];
	Rodrigues(lrDt, dA);
	float J[9];
	So3LeftJ(lrDt, J);
	float da[3];
	Mat3Vec(J, lbDt, da);

	float Anew[9];
	Mat3Mul(vState.A, dA, Anew);

	float Ada[3];
	Mat3Vec(vState.A, da, Ada);
	vState.aVec[0] += Ada[0];
	vState.aVec[1] += Ada[1];
	vState.aVec[2] += Ada[2];
	Mat3Copy(vState.A, Anew);

	// Covariance: W0 = [A om]x
	float Aw[3];
	Mat3Vec(vState.A, om, Aw);
	float W0[9];
	Skew3(Aw, W0);
	float W0sq[9];
	Mat3Mul(W0, W0, W0sq);

	float dt2 = dt * dt;

	// F = Phi12 = -dt (I + dt/2 W0 + dt^2/6 W0^2)
	float F[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			float eye = (i == j) ? 1.0f : 0.0f;
			F[k] = -dt * (eye + 0.5f * dt * W0[k] + (dt2 / 6.0f) * W0sq[k]);
		}
	}

	// G = Phi22 = I + dt W0 + dt^2/2 W0^2
	float G[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			float eye = (i == j) ? 1.0f : 0.0f;
			G[k] = eye + dt * W0[k] + 0.5f * dt2 * W0sq[k];
		}
	}

	float P11[9], P12[9], P21[9], P22[9];
	PGet(vState.P, 0, 0, P11);
	PGet(vState.P, 0, 1, P12);
	PGet(vState.P, 1, 0, P21);
	PGet(vState.P, 1, 1, P22);

	// Phi = [[I, F], [0, G]]
	float T1[9], T2[9];
	Mat3Mul(F, P21, T1);	// F P21
	Mat3Mul(F, P22, T2);	// F P22
	float FT[9];
	Mat3Transpose(F, FT);
	float T2FT[9];
	Mat3Mul(T2, FT, T2FT);	// F P22 F^T

	float nP11[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			nP11[k] = P11[k] + T1[k] + T1[j * 3 + i] + T2FT[k];
		}
	}

	float mid[9];	// P12 + F P22
	for (int i = 0; i < 9; i++) {
		mid[i] = P12[i] + T2[i];
	}
	float GT[9];
	Mat3Transpose(G, GT);
	float nP12[9];
	Mat3Mul(mid, GT, nP12);

	float nP21[9];
	Mat3Transpose(nP12, nP21);

	float GP22[9];
	Mat3Mul(G, P22, GP22);
	float nP22[9];
	Mat3Mul(GP22, GT, nP22);

	// Process noise M = diag(sigmaW^2, sigmaB^2) dt
	float sw2dt = vParams.sigmaW * vParams.sigmaW * dt;
	float sb2dt = vParams.sigmaB * vParams.sigmaB * dt;
	nP11[0] += sw2dt; nP11[4] += sw2dt; nP11[8] += sw2dt;
	nP22[0] += sb2dt; nP22[4] += sb2dt; nP22[8] += sb2dt;

	Mat3Sym(nP11);
	Mat3Sym(nP22);

	PSet(vState.P, 0, 0, nP11);
	PSet(vState.P, 0, 1, nP12);
	PSet(vState.P, 1, 0, nP21);
	PSet(vState.P, 1, 1, nP22);

	if (++vState.orthoCounter >= vParams.orthoInterval) {
		Reortho(vState.A);
		vState.orthoCounter = 0;
	}
}

void ImuEqf::DirUpdate(const float yRaw[3], const float d[3], float sigma, bool suppressYaw)
{
	float y[3] = { yRaw[0], yRaw[1], yRaw[2] };
	float n = Norm3(y);
	if (n < 1.0e-10f) {
		return;
	}
	float inv = 1.0f / n;
	y[0] *= inv; y[1] *= inv; y[2] *= inv;

	// Cd = skew(d)^2
	float Cd[9];
	Skew3Sq(d, Cd);

	// innovation delta = [d]x (A y)
	float Ay[3];
	Mat3Vec(vState.A, y, Ay);
	float skD[9];
	Skew3(d, skD);
	float delta[3];
	Mat3Vec(skD, Ay, delta);

	// S = Cd P11 Cd^T + sigma^2 I
	float P11[9];
	PGet(vState.P, 0, 0, P11);
	float CdP[9];
	Mat3Mul(Cd, P11, CdP);
	float S[9];
	Mat3MulBT(CdP, Cd, S);
	float s2 = sigma * sigma;
	S[0] += s2; S[4] += s2; S[8] += s2;

	float Si[9];
	if (Mat3Inv(S, Si) == false) {
		return;
	}

	// K = P C^T S^-1, C = [Cd | 0]
	float P21[9];
	PGet(vState.P, 1, 0, P21);
	float CdT[9];
	Mat3Transpose(Cd, CdT);
	float P11Ct[9], P21Ct[9];
	Mat3Mul(P11, CdT, P11Ct);
	Mat3Mul(P21, CdT, P21Ct);
	float Ku[9], Kl[9];
	Mat3Mul(P11Ct, Si, Ku);
	Mat3Mul(P21Ct, Si, Kl);

	// 6-axis: accel must not drive heading (Ku row 2) or bias (Kl).
	if (suppressYaw) {
		Ku[6] = 0.0f; Ku[7] = 0.0f; Ku[8] = 0.0f;
		memset(Kl, 0, 9 * sizeof(float));
	}

	// InnovationLift = diag(1,1,1,-1,-1,-1)
	float Delta[6];
	Mat3Vec(Ku, delta, &Delta[0]);
	Mat3Vec(Kl, delta, &Delta[3]);
	Delta[3] = -Delta[3]; Delta[4] = -Delta[4]; Delta[5] = -Delta[5];

	// State correction: X = exp(Delta) X (left multiply)
	float dA[9];
	Rodrigues(&Delta[0], dA);
	float J[9];
	So3LeftJ(&Delta[0], J);
	float da[3];
	Mat3Vec(J, &Delta[3], da);

	float Anew[9];
	Mat3Mul(dA, vState.A, Anew);
	float dAa[3];
	Mat3Vec(dA, vState.aVec, dAa);
	vState.aVec[0] = da[0] + dAa[0];
	vState.aVec[1] = da[1] + dAa[1];
	vState.aVec[2] = da[2] + dAa[2];
	Mat3Copy(vState.A, Anew);

	// Covariance Joseph form. M = I - KC = [[A_, 0], [B_, I]]
	float KuCd[9], KlCd[9];
	Mat3Mul(Ku, Cd, KuCd);
	Mat3Mul(Kl, Cd, KlCd);

	float Ai[9];
	Mat3Eye(Ai);
	for (int i = 0; i < 9; i++) {
		Ai[i] -= KuCd[i];
	}
	float Bi[9];
	for (int i = 0; i < 9; i++) {
		Bi[i] = -KlCd[i];
	}

	float P12[9], P22[9];
	PGet(vState.P, 0, 1, P12);
	PGet(vState.P, 1, 1, P22);

	float AP11[9], BP11[9];
	Mat3Mul(Ai, P11, AP11);
	Mat3Mul(Bi, P11, BP11);

	float T[9];
	float nP11[9];
	Mat3MulBT(AP11, Ai, nP11);
	Mat3MulBT(Ku, Ku, T);
	for (int i = 0; i < 9; i++) {
		nP11[i] += s2 * T[i];
	}

	float nP12[9], AP12[9];
	Mat3MulBT(AP11, Bi, nP12);
	Mat3Mul(Ai, P12, AP12);
	Mat3MulBT(Ku, Kl, T);
	for (int i = 0; i < 9; i++) {
		nP12[i] += AP12[i] + s2 * T[i];
	}

	float nP21[9];
	Mat3Transpose(nP12, nP21);

	float nP22[9], BP12[9], P21BT[9];
	Mat3MulBT(BP11, Bi, nP22);
	Mat3Mul(Bi, P12, BP12);
	Mat3MulBT(P21, Bi, P21BT);
	Mat3MulBT(Kl, Kl, T);
	for (int i = 0; i < 9; i++) {
		nP22[i] += BP12[i] + P21BT[i] + P22[i] + s2 * T[i];
	}

	Mat3Sym(nP11);
	Mat3Sym(nP22);

	PSet(vState.P, 0, 0, nP11);
	PSet(vState.P, 0, 1, nP12);
	PSet(vState.P, 1, 0, nP21);
	PSet(vState.P, 1, 1, nP22);

	for (int i = 0; i < 6; i++) {
		if (vState.P[i * 6 + i] < 1.0e-12f) {
			vState.P[i * 6 + i] = 1.0e-10f;
		}
	}
}

void ImuEqf::RestBiasUpdate(void)
{
	// b = -A^T aVec
	float bh[3];
	Mat3TVec(vState.A, vState.aVec, bh);
	bh[0] = -bh[0]; bh[1] = -bh[1]; bh[2] = -bh[2];

	float e[3] = { vState.restGyrLp[0] - bh[0],
	               vState.restGyrLp[1] - bh[1],
	               vState.restGyrLp[2] - bh[2] };

	// C = [0, Cb], Cb = -A^T. S = A^T P22 A + sigma^2 I
	float P22[9];
	PGet(vState.P, 1, 1, P22);
	float AT[9];
	Mat3Transpose(vState.A, AT);
	float ATP22[9], S[9];
	Mat3Mul(AT, P22, ATP22);
	Mat3Mul(ATP22, vState.A, S);
	float s2 = vParams.restSigma * vParams.restSigma;
	S[0] += s2; S[4] += s2; S[8] += s2;

	float Si[9];
	if (Mat3Inv(S, Si) == false) {
		return;
	}

	// K = P C^T S^-1, C^T = [[0], [-A]]
	float P12[9];
	PGet(vState.P, 0, 1, P12);
	float mA[9];
	for (int i = 0; i < 9; i++) {
		mA[i] = -vState.A[i];
	}
	float P12mA[9], P22mA[9];
	Mat3Mul(P12, mA, P12mA);
	Mat3Mul(P22, mA, P22mA);
	float Ku[9], Kl[9];
	Mat3Mul(P12mA, Si, Ku);
	Mat3Mul(P22mA, Si, Kl);

	// Delta = K e (identity lift)
	float Delta[6];
	Mat3Vec(Ku, e, &Delta[0]);
	Mat3Vec(Kl, e, &Delta[3]);

	float dA[9];
	Rodrigues(&Delta[0], dA);
	float J[9];
	So3LeftJ(&Delta[0], J);
	float da[3];
	Mat3Vec(J, &Delta[3], da);

	float Anew[9];
	Mat3Mul(dA, vState.A, Anew);
	float dAa[3];
	Mat3Vec(dA, vState.aVec, dAa);
	vState.aVec[0] = da[0] + dAa[0];
	vState.aVec[1] = da[1] + dAa[1];
	vState.aVec[2] = da[2] + dAa[2];
	Mat3Copy(vState.A, Anew);

	// Covariance Joseph form. M = [[I, F_], [0, E_]]
	// F_ = Ku A^T, E_ = I + Kl A^T
	float Fi[9], Ei[9];
	Mat3Mul(Ku, AT, Fi);
	Mat3Mul(Kl, AT, Ei);
	Ei[0] += 1.0f; Ei[4] += 1.0f; Ei[8] += 1.0f;

	float P11[9], P21[9];
	PGet(vState.P, 0, 0, P11);
	PGet(vState.P, 1, 0, P21);

	float nP11[9], T[9], T2[9];
	Mat3MulBT(P12, Fi, nP11);	// P12 F_^T
	Mat3Mul(Fi, P21, T);		// F_ P21
	for (int i = 0; i < 9; i++) {
		nP11[i] += P11[i] + T[i];
	}
	Mat3Mul(Fi, P22, T);		// F_ P22
	Mat3MulBT(T, Fi, T2);		// F_ P22 F_^T
	Mat3MulBT(Ku, Ku, T);
	for (int i = 0; i < 9; i++) {
		nP11[i] += T2[i] + s2 * T[i];
	}

	float nP12[9], FP22[9], sum12[9];
	Mat3Mul(Fi, P22, FP22);
	for (int i = 0; i < 9; i++) {
		sum12[i] = P12[i] + FP22[i];
	}
	Mat3MulBT(sum12, Ei, nP12);
	Mat3MulBT(Ku, Kl, T);
	for (int i = 0; i < 9; i++) {
		nP12[i] += s2 * T[i];
	}

	float nP21[9];
	Mat3Transpose(nP12, nP21);

	float nP22[9], EP22[9];
	Mat3Mul(Ei, P22, EP22);
	Mat3MulBT(EP22, Ei, nP22);
	Mat3MulBT(Kl, Kl, T);
	for (int i = 0; i < 9; i++) {
		nP22[i] += s2 * T[i];
	}

	Mat3Sym(nP11);
	Mat3Sym(nP22);

	PSet(vState.P, 0, 0, nP11);
	PSet(vState.P, 0, 1, nP12);
	PSet(vState.P, 1, 0, nP21);
	PSet(vState.P, 1, 1, nP22);

	for (int i = 0; i < 6; i++) {
		if (vState.P[i * 6 + i] < 1.0e-12f) {
			vState.P[i * 6 + i] = 1.0e-10f;
		}
	}
}

bool ImuEqf::UpdateData()
{
	if (vbInitialized == false) {
		return false;
	}

	AccelSensorData_t accel;
	GyroSensorData_t gyro;

	vpAccel->Read(accel);
	vpGyro->Read(gyro);

	float a[3] = { accel.X, accel.Y, accel.Z };	// g
	// VQF and EqF expect gyro in rad/s. IOsonata gyro data is in deg/s.
	float w[3] = { gyro.X * EQF_DEG2RAD, gyro.Y * EQF_DEG2RAD, gyro.Z * EQF_DEG2RAD };

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
		MatToQuat(vState.A, vQuat.Q);
		vQuat.Timestamp = gyro.Timestamp;
		return true;
	}

	// Propagate from gyro, then rest gyro low pass.
	Propagate(w, vGyrDt);

	float alphaG = 1.0f - expf(-vGyrDt / vParams.restTau);
	if (vState.restGyrLpInit == false) {
		vState.restGyrLp[0] = w[0]; vState.restGyrLp[1] = w[1]; vState.restGyrLp[2] = w[2];
		vState.restGyrLpInit = true;
	} else {
		vState.restGyrLp[0] += alphaG * (w[0] - vState.restGyrLp[0]);
		vState.restGyrLp[1] += alphaG * (w[1] - vState.restGyrLp[1]);
		vState.restGyrLp[2] += alphaG * (w[2] - vState.restGyrLp[2]);
	}
	float dg0 = w[0] - vState.restGyrLp[0];
	float dg1 = w[1] - vState.restGyrLp[1];
	float dg2 = w[2] - vState.restGyrLp[2];
	vState.restGyrDev = dg0 * dg0 + dg1 * dg1 + dg2 * dg2;

	// Accel direction update (gravity), 6-axis: suppress heading and bias.
	float anorm = Norm3(a);
	if (anorm >= 0.1f && anorm <= 5.0f) {
		float dev = anorm - 1.0f;
		float baseSigma = vParams.sigmaAcc * vParams.sixAxisAccScale;	// 6-axis weakening
		float sigma = baseSigma * (1.0f + vParams.accAdaptK * dev * dev);
		static const float dAcc[3] = { 0.0f, 0.0f, 1.0f };
		DirUpdate(a, dAcc, sigma, true);
	}

	// Rest detection: accel low pass and combined thresholds.
	float alphaA = 1.0f - expf(-vAccDt / vParams.restTau);
	if (vState.restAccLpInit == false) {
		vState.restAccLp[0] = a[0]; vState.restAccLp[1] = a[1]; vState.restAccLp[2] = a[2];
		vState.restAccLpInit = true;
	} else {
		vState.restAccLp[0] += alphaA * (a[0] - vState.restAccLp[0]);
		vState.restAccLp[1] += alphaA * (a[1] - vState.restAccLp[1]);
		vState.restAccLp[2] += alphaA * (a[2] - vState.restAccLp[2]);
	}
	float da0 = a[0] - vState.restAccLp[0];
	float da1 = a[1] - vState.restAccLp[1];
	float da2 = a[2] - vState.restAccLp[2];
	float accDevSq = da0 * da0 + da1 * da1 + da2 * da2;

	float gyrThRad = vParams.restThGyr * EQF_DEG2RAD;
	float maxBiasRad = vParams.restMaxBias * EQF_DEG2RAD;

	if (vState.restGyrDev >= gyrThRad * gyrThRad
			|| accDevSq >= vParams.restThAcc * vParams.restThAcc
			|| fabsf(anorm - 1.0f) > vParams.restAccNormTh
			|| fabsf(vState.restGyrLp[0]) > maxBiasRad
			|| fabsf(vState.restGyrLp[1]) > maxBiasRad
			|| fabsf(vState.restGyrLp[2]) > maxBiasRad) {
		vState.restT = 0.0f;
		vState.restDetected = false;
	} else {
		vState.restT += vAccDt;
		if (vState.restT >= vParams.restMinT) {
			vState.restDetected = true;
		}
	}

	if (vState.restDetected && vState.restGyrLpInit) {
		RestBiasUpdate();
	}

	MatToQuat(vState.A, vQuat.Q);
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

bool ImuEqf::Read(ImuQuat_t &Data)
{
	Data = vQuat;
	return true;
}

void ImuEqf::IntHandler()
{
	UpdateData();
}

bool ImuEqf::Quaternion(bool bEn, int NbAxis)
{
	// Mag path not yet enabled; EqF runs 6-axis.
	vNbAxis = 6;
	return true;
}

bool ImuEqf::Compass(bool bEn)
{
	return false;
}

bool ImuEqf::Calibrate()
{
	return false;
}

void ImuEqf::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuEqf::Pedometer(bool bEn)
{
	return false;
}

bool ImuEqf::Tap(bool bEn)
{
	return false;
}
