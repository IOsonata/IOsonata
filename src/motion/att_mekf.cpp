/**-------------------------------------------------------------------------
@file	ahrs_mekf.cpp

@brief	Implementation of the Att class using MEKF fusion

Self contained Multiplicative Extended Kalman Filter for attitude and gyro
bias. State is a unit quaternion (body to earth) plus a 3 axis gyro bias. The
6 element error state (small angle attitude error, bias error) is propagated
with a 6x6 covariance and reset into the quaternion after each update.

Process noise follows the Farrenkopf attitude and bias model. The measurement
update is a vector direction update in Joseph form. The 6-axis path uses the
accel gravity direction only; the gravity geometry leaves the about-gravity
heading component unobservable, so no explicit yaw suppression is needed.

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

#include "motion/att_mekf.h"
#include "math_linalg.h"
#include "math_so3.h"

#define MEKF_DEG2RAD	0.01745329251994329577f


using namespace LinAlg;
using namespace So3;

AttMekf::AttMekf()
{
	vNbAxis = 6;
	vbInitialized = false;
	vGyrDt = 0.005f;
	vAccDt = 0.005f;
	memset(&vState, 0, sizeof(vState));

	vParams.sigmaV = 0.003f;
	vParams.sigmaU = 0.0003f;
	vParams.sigmaAcc = 0.5f;
	vParams.accAdaptK = 100.0f;
	vParams.initSamples = 50;
	vParams.pInitAtt = 0.1f;
	vParams.pInitBias = 0.01f;
	vParams.accGateLo = 0.1f;
	vParams.accGateHi = 5.0f;
}

bool AttMekf::Init(const AttCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr) {
		return false;
	}

	if (Att::Init(Cfg, pAccel, pGyro, pMag) == false) {
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

void AttMekf::Setup(void)
{
	// All MEKF coefficients are derived per step from vParams and the sample
	// periods captured in Init. Kept for symmetry with the interface.
}

void AttMekf::Reset(void)
{
	memset(&vState, 0, sizeof(vState));
	vState.q[0] = 1.0f;	// identity quaternion
	vState.mode = 0;	// init accumulate
}

bool AttMekf::Enable()
{
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void AttMekf::Disable()
{
	if (vpMag) vpMag->Disable();
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
}

void AttMekf::GravityInit(const float accAvg[3])
{
	float A[9];
	DcmFromGravity(accAvg, A);

	MatToQuat(A, vState.q);

	memset(vState.b, 0, sizeof(vState.b));
	memset(vState.P, 0, sizeof(vState.P));
	// Isotropic initial attitude variance. The gravity update naturally leaves
	// the about-gravity heading component unobservable, so its variance is not
	// reduced by the filter and no special yaw seeding is required.
	for (int i = 0; i < 3; i++) {
		vState.P[i * 6 + i] = vParams.pInitAtt;
	}
	for (int i = 3; i < 6; i++) {
		vState.P[i * 6 + i] = vParams.pInitBias;
	}
}

void AttMekf::Propagate(const float w[3], float dt)
{
	// Bias corrected body rate.
	float wh[3] = { w[0] - vState.b[0], w[1] - vState.b[1], w[2] - vState.b[2] };
	float phi[3] = { wh[0] * dt, wh[1] * dt, wh[2] * dt };

	// Quaternion update: q = q (x) dq, body frame increment.
	float dq[4];
	QuatFromRotVec(phi, dq);
	float qn[4];
	QuatMul(vState.q, dq, qn);
	QuatNormalize(qn);
	memcpy(vState.q, qn, sizeof(qn));

	// Error state transition Phi = [[Phi11, Phi12], [0, I]].
	// Phi11 = exp(-[phi]x), Phi12 = -(I dt - 0.5 [wh]x dt^2).
	float nphi[3] = { -phi[0], -phi[1], -phi[2] };
	float Phi11[9];
	Rodrigues(nphi, Phi11);

	float skewW[9];
	Skew3(wh, skewW);
	float dt2 = dt * dt;
	float Phi12[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = i * 3 + j;
			float eye = (i == j) ? 1.0f : 0.0f;
			Phi12[k] = -(eye * dt) + 0.5f * dt2 * skewW[k];
		}
	}

	float P11[9], P12[9], P21[9], P22[9];
	Mat3BlockGet(vState.P, 6, 0, 0, P11);
	Mat3BlockGet(vState.P, 6, 0, 1, P12);
	Mat3BlockGet(vState.P, 6, 1, 0, P21);
	Mat3BlockGet(vState.P, 6, 1, 1, P22);

	// nP11 = Phi11 P11 Phi11^T + Phi11 P12 Phi12^T
	//      + Phi12 P21 Phi11^T + Phi12 P22 Phi12^T
	float T[9], T2[9], nP11[9];
	Mat3Mul(Phi11, P11, T);
	Mat3MulBT(T, Phi11, nP11);
	Mat3Mul(Phi11, P12, T);
	Mat3MulBT(T, Phi12, T2);
	for (int i = 0; i < 9; i++) nP11[i] += T2[i];
	Mat3Mul(Phi12, P21, T);
	Mat3MulBT(T, Phi11, T2);
	for (int i = 0; i < 9; i++) nP11[i] += T2[i];
	Mat3Mul(Phi12, P22, T);
	Mat3MulBT(T, Phi12, T2);
	for (int i = 0; i < 9; i++) nP11[i] += T2[i];

	// nP12 = Phi11 P12 + Phi12 P22
	float nP12[9], Tb[9];
	Mat3Mul(Phi11, P12, T);
	Mat3Mul(Phi12, P22, Tb);
	for (int i = 0; i < 9; i++) nP12[i] = T[i] + Tb[i];

	// nP22 = P22
	float nP22[9];
	memcpy(nP22, P22, sizeof(nP22));

	// Process noise (Farrenkopf): q11 angle, q12 cross, q22 bias walk.
	float su2 = vParams.sigmaU * vParams.sigmaU;
	float q11 = vParams.sigmaV * vParams.sigmaV * dt + (1.0f / 3.0f) * su2 * dt * dt2;
	float q12 = -0.5f * su2 * dt2;
	float q22 = su2 * dt;
	nP11[0] += q11; nP11[4] += q11; nP11[8] += q11;
	nP12[0] += q12; nP12[4] += q12; nP12[8] += q12;
	nP22[0] += q22; nP22[4] += q22; nP22[8] += q22;

	float nP21[9];
	Mat3Transpose(nP12, nP21);

	Mat3Sym(nP11);
	Mat3Sym(nP22);

	Mat3BlockSet(vState.P, 6, 0, 0, nP11);
	Mat3BlockSet(vState.P, 6, 0, 1, nP12);
	Mat3BlockSet(vState.P, 6, 1, 0, nP21);
	Mat3BlockSet(vState.P, 6, 1, 1, nP22);
}

void AttMekf::VecUpdate(const float meas[3], const float ref[3], float sigma)
{
	float y[3] = { meas[0], meas[1], meas[2] };
	float n = Norm3(y);
	if (n < 1.0e-10f) {
		return;
	}
	float inv = 1.0f / n;
	y[0] *= inv; y[1] *= inv; y[2] *= inv;

	// Predicted body-frame direction = A r, A = earth to body = R(q)^T.
	float Rbe[9];
	QuatToDcmBE(vState.q, Rbe);
	float pred[3];
	Mat3TVec(Rbe, ref, pred);

	// innovation
	float resid[3] = { y[0] - pred[0], y[1] - pred[1], y[2] - pred[2] };

	// H attitude block = skew(pred), bias block zero.
	float Hs[9];
	Skew3(pred, Hs);

	// S = Hs P11 Hs^T + sigma^2 I
	float P11[9];
	Mat3BlockGet(vState.P, 6, 0, 0, P11);
	float HsP[9];
	Mat3Mul(Hs, P11, HsP);
	float S[9];
	Mat3MulBT(HsP, Hs, S);
	float s2 = sigma * sigma;
	S[0] += s2; S[4] += s2; S[8] += s2;

	float Si[9];
	if (Mat3Inv(S, Si) == false) {
		return;
	}

	// K = P H^T S^-1, H^T = [Hs^T ; 0]
	float P21[9];
	Mat3BlockGet(vState.P, 6, 1, 0, P21);
	float P11Ht[9], P21Ht[9];
	Mat3MulBT(P11, Hs, P11Ht);
	Mat3MulBT(P21, Hs, P21Ht);
	float Ku[9], Kl[9];
	Mat3Mul(P11Ht, Si, Ku);
	Mat3Mul(P21Ht, Si, Kl);

	// State correction.
	float du[3], dl[3];
	Mat3Vec(Ku, resid, du);
	Mat3Vec(Kl, resid, dl);

	float dq[4];
	QuatFromRotVec(du, dq);
	float qn[4];
	QuatMul(vState.q, dq, qn);
	QuatNormalize(qn);
	memcpy(vState.q, qn, sizeof(qn));

	vState.b[0] += dl[0];
	vState.b[1] += dl[1];
	vState.b[2] += dl[2];

	// Covariance Joseph form. M = I - KH = [[Ai, 0], [Bi, I]].
	float KuHs[9], KlHs[9];
	Mat3Mul(Ku, Hs, KuHs);
	Mat3Mul(Kl, Hs, KlHs);

	float Ai[9];
	Mat3Eye(Ai);
	for (int i = 0; i < 9; i++) {
		Ai[i] -= KuHs[i];
	}
	float Bi[9];
	for (int i = 0; i < 9; i++) {
		Bi[i] = -KlHs[i];
	}

	float P12[9], P22[9];
	Mat3BlockGet(vState.P, 6, 0, 1, P12);
	Mat3BlockGet(vState.P, 6, 1, 1, P22);

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

	Mat3BlockSet(vState.P, 6, 0, 0, nP11);
	Mat3BlockSet(vState.P, 6, 0, 1, nP12);
	Mat3BlockSet(vState.P, 6, 1, 0, nP21);
	Mat3BlockSet(vState.P, 6, 1, 1, nP22);

	for (int i = 0; i < 6; i++) {
		if (vState.P[i * 6 + i] < 1.0e-12f) {
			vState.P[i * 6 + i] = 1.0e-10f;
		}
	}
}

bool AttMekf::UpdateData()
{
	if (vbInitialized == false) {
		return false;
	}

	AccelSensorData_t accel;
	GyroSensorData_t gyro;

	vpAccel->Read(accel);
	vpGyro->Read(gyro);

	float a[3] = { accel.X, accel.Y, accel.Z };	// g
	// MEKF expects gyro in rad/s. IOsonata gyro data is in deg/s.
	float w[3] = { gyro.X * MEKF_DEG2RAD, gyro.Y * MEKF_DEG2RAD, gyro.Z * MEKF_DEG2RAD };

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

	// Propagate from gyro.
	Propagate(w, vGyrDt);

	// Accel direction update (gravity). 6-axis: gravity geometry leaves the
	// heading component unobservable; no explicit yaw suppression needed.
	float anorm = Norm3(a);
	if (anorm >= vParams.accGateLo && anorm <= vParams.accGateHi) {
		float dev = anorm - 1.0f;
		float sigma = vParams.sigmaAcc * (1.0f + vParams.accAdaptK * dev * dev);
		static const float dAcc[3] = { 0.0f, 0.0f, 1.0f };
		VecUpdate(a, dAcc, sigma);
	}

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

bool AttMekf::Read(AttQuat_t &Data)
{
	Data = vQuat;
	return true;
}

void AttMekf::IntHandler()
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

bool AttMekf::Quaternion(bool bEn, int NbAxis)
{
	// Mag path not yet enabled; MEKF runs 6-axis.
	vNbAxis = 6;
	return true;
}

bool AttMekf::Compass(bool bEn)
{
	return false;
}

bool AttMekf::Calibrate()
{
	return false;
}

void AttMekf::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

