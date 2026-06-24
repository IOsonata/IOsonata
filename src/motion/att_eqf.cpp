/**-------------------------------------------------------------------------
@file	ahrs_eqf.cpp

@brief	Implementation of the Ahrs class using EqF fusion

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

#include "motion/att_eqf.h"
#include "math_linalg.h"
#include "math_so3.h"

#define EQF_DEG2RAD	0.01745329251994329577f


using namespace LinAlg;
using namespace So3;

AhrsEqf::AhrsEqf()
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

bool AhrsEqf::Init(const AhrsCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
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

void AhrsEqf::Setup(void)
{
	// All EqF coefficients are static; nothing to precompute beyond the
	// sample periods captured in Init. Kept for symmetry with the interface.
}

void AhrsEqf::Reset(void)
{
	memset(&vState, 0, sizeof(vState));
	Mat3Eye(vState.A);
	vState.mode = 0;	// init accumulate
}

bool AhrsEqf::Enable()
{
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void AhrsEqf::Disable()
{
	if (vpMag) vpMag->Disable();
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
}

void AhrsEqf::GravityInit(const float accAvg[3])
{
	DcmFromGravity(accAvg, vState.A);

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

void AhrsEqf::Propagate(const float w[3], float dt)
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
	Mat3BlockGet(vState.P, 6, 0, 0, P11);
	Mat3BlockGet(vState.P, 6, 0, 1, P12);
	Mat3BlockGet(vState.P, 6, 1, 0, P21);
	Mat3BlockGet(vState.P, 6, 1, 1, P22);

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

	Mat3BlockSet(vState.P, 6, 0, 0, nP11);
	Mat3BlockSet(vState.P, 6, 0, 1, nP12);
	Mat3BlockSet(vState.P, 6, 1, 0, nP21);
	Mat3BlockSet(vState.P, 6, 1, 1, nP22);

	if (++vState.orthoCounter >= vParams.orthoInterval) {
		Reortho(vState.A);
		vState.orthoCounter = 0;
	}
}

void AhrsEqf::DirUpdate(const float yRaw[3], const float d[3], float sigma, bool suppressYaw)
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
	Mat3BlockGet(vState.P, 6, 0, 0, P11);
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
	Mat3BlockGet(vState.P, 6, 1, 0, P21);
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

void AhrsEqf::RestBiasUpdate(void)
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
	Mat3BlockGet(vState.P, 6, 1, 1, P22);
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
	Mat3BlockGet(vState.P, 6, 0, 1, P12);
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
	Mat3BlockGet(vState.P, 6, 0, 0, P11);
	Mat3BlockGet(vState.P, 6, 1, 0, P21);

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

bool AhrsEqf::UpdateData()
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

bool AhrsEqf::Read(AhrsQuat_t &Data)
{
	Data = vQuat;
	return true;
}

void AhrsEqf::IntHandler()
{
	// Refresh the bound sensors then fuse. IntHandler is the data-ready entry point. When the caller
	// refreshes the sensors itself and calls UpdateData() directly, do not
	// call this.
	if (vpAccel) vpAccel->IntHandler();
	if (vpGyro) vpGyro->IntHandler();
	if (vpMag) vpMag->IntHandler();
	UpdateData();
}

bool AhrsEqf::Quaternion(bool bEn, int NbAxis)
{
	// Mag path not yet enabled; EqF runs 6-axis.
	vNbAxis = 6;
	return true;
}

bool AhrsEqf::Compass(bool bEn)
{
	return false;
}

bool AhrsEqf::Calibrate()
{
	return false;
}

void AhrsEqf::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool AhrsEqf::Pedometer(bool bEn)
{
	return false;
}

bool AhrsEqf::Tap(bool bEn)
{
	return false;
}
