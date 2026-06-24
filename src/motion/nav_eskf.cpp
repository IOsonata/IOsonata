/**-------------------------------------------------------------------------
@file	nav_eskf.cpp

@brief	Loosely coupled error-state EKF inertial navigation

Implementation of the NavEskf class declared in nav_eskf.h. The error state is
the body frame form

	[ d_pos(3), d_vel(3), d_theta(3), d_accelbias(3), d_gyrobias(3) ]

with d_theta the multiplicative body frame attitude error. The nominal state is
integrated by a strapdown step and the 15x15 covariance is propagated by a first
order transition. GNSS position and velocity, barometric height and zero
velocity updates observe a selected 3 or 1 block of the error state, so the gain
uses the 3x3 inverse from fusion_math.h directly. Optical flow is mapped to a
body velocity pseudo measurement and fused through the velocity path.

Frame and sign conventions follow fusion_math.h: QuatToDcmBE returns body to NED
(v_nav = R v_body) and DcmFromGravity maps the measured specific force to NED +Z.
The nominal velocity update is v_dot = R (accel - accelbias) + gNav with
gNav = (0, 0, -gravity), which cancels the specific force at rest.

@author	Hoang Nguyen Hoan
@date	Jun. 24, 2026

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

#include "motion/nav_eskf.h"
#include "math_linalg.h"
#include "math_so3.h"

using namespace LinAlg;
using namespace So3;

#define NX		ESKF_NX			// 15
#define I_POS		0
#define I_VEL		3
#define I_ATT		6
#define I_AB		9
#define I_GB		12

#define DEG_TO_RAD	0.01745329252f

// WGS84 ellipsoid for the local tangent conversion.
#define WGS84_A		6378137.0
#define WGS84_E2	6.69437999014e-3

// ---- file scope Kalman helpers, selector measurement on a 3 or 1 block ----

// Copy the three columns c0..c0+2 of P into a 15 x 3 destination.
static void GetBlockCols(const float P[NX * NX], int c0, float dst[NX * 3])
{
	for (int i = 0; i < NX; i++) {
		const float *row = P + i * NX + c0;
		dst[i * 3 + 0] = row[0];
		dst[i * 3 + 1] = row[1];
		dst[i * 3 + 2] = row[2];
	}
}

// Update for a 3 row measurement that observes the error block at column c0.
// innov is measured minus predicted. Rdiag is the diagonal measurement noise.
// Fills dx with the 15 vector correction and updates P. Returns false when the
// innovation covariance is not invertible.
static bool KalmanBlock3(float P[NX * NX], int c0, const float innov[3],
			 const float Rdiag[3], float dx[NX])
{
	float PHt[NX * 3];			// P H^T, 15 x 3, columns c0..c0+2 of P
	GetBlockCols(P, c0, PHt);

	float S[9];				// H P H^T + R, the c0 block of P plus R
	S[0] = P[(c0 + 0) * NX + c0 + 0] + Rdiag[0];
	S[1] = P[(c0 + 0) * NX + c0 + 1];
	S[2] = P[(c0 + 0) * NX + c0 + 2];
	S[3] = P[(c0 + 1) * NX + c0 + 0];
	S[4] = P[(c0 + 1) * NX + c0 + 1] + Rdiag[1];
	S[5] = P[(c0 + 1) * NX + c0 + 2];
	S[6] = P[(c0 + 2) * NX + c0 + 0];
	S[7] = P[(c0 + 2) * NX + c0 + 1];
	S[8] = P[(c0 + 2) * NX + c0 + 2] + Rdiag[2];

	float Si[9];
	if (!Mat3Inv(S, Si)) {
		return false;
	}

	float K[NX * 3];			// K = P H^T S^-1, 15 x 3
	for (int i = 0; i < NX; i++) {
		const float *ph = PHt + i * 3;
		float *kr = K + i * 3;
		kr[0] = ph[0] * Si[0] + ph[1] * Si[3] + ph[2] * Si[6];
		kr[1] = ph[0] * Si[1] + ph[1] * Si[4] + ph[2] * Si[7];
		kr[2] = ph[0] * Si[2] + ph[1] * Si[5] + ph[2] * Si[8];
	}

	for (int i = 0; i < NX; i++) {
		const float *kr = K + i * 3;
		dx[i] = kr[0] * innov[0] + kr[1] * innov[1] + kr[2] * innov[2];
	}

	// P = P - K (P H^T)^T
	for (int i = 0; i < NX; i++) {
		const float *kr = K + i * 3;
		for (int j = 0; j < NX; j++) {
			const float *ph = PHt + j * 3;
			P[i * NX + j] -= kr[0] * ph[0] + kr[1] * ph[1] + kr[2] * ph[2];
		}
	}
	Symmetrize(P, NX);
	return true;
}

// Single row measurement that observes error element at index idx.
static bool KalmanScalar(float P[NX * NX], int idx, float innov, float r,
			 float dx[NX])
{
	float ph[NX];				// column idx of P
	for (int i = 0; i < NX; i++) {
		ph[i] = P[i * NX + idx];
	}
	float s = ph[idx] + r;
	if (fabsf(s) < 1.0e-12f) {
		return false;
	}
	float invs = 1.0f / s;
	float k[NX];
	for (int i = 0; i < NX; i++) {
		k[i] = ph[i] * invs;		// K = P H^T / s
		dx[i] = k[i] * innov;
	}
	for (int i = 0; i < NX; i++) {
		for (int j = 0; j < NX; j++) {
			P[i * NX + j] -= k[i] * ph[j];
		}
	}
	Symmetrize(P, NX);
	return true;
}

// ---- class ----

NavEskf::NavEskf()
{
	vpAccel = nullptr;
	vpGyro = nullptr;
	vpMag = nullptr;
	vGyrDt = 0.0f;
	vAccDt = 0.0f;
	vbInitialized = false;
	memset(&vParams, 0, sizeof(vParams));
	memset(&vState, 0, sizeof(vState));
	memset(&vOrigin, 0, sizeof(vOrigin));
	NavState_t &ns = InertialNav::vState;
	memset(&ns, 0, sizeof(NavState_t));
}

bool NavEskf::Init(const NavCfg_t &Cfg, AccelSensor * const pAccel,
		   GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr) {
		return false;
	}
	vpAccel = pAccel;
	vpGyro = pGyro;
	vpMag = pMag;
	vOrigin = Cfg.Origin;

	// Default tuning. The application overrides with SetParam as needed.
	vParams.accelNoise = 0.35f;
	vParams.gyroNoise = 0.015f;
	vParams.accelBiasNoise = 1.0e-3f;
	vParams.gyroBiasNoise = 1.0e-4f;
	vParams.gravity = 9.80665f;
	vParams.initSamples = 64;
	vParams.pInitPos = 100.0f;
	vParams.pInitVel = 25.0f;
	vParams.pInitAtt = 0.1f;
	vParams.pInitAccBias = 0.04f;
	vParams.pInitGyroBias = 1.0e-3f;

	Setup();
	vbInitialized = true;
	return true;
}

void NavEskf::Setup(void)
{
	memset(&vState, 0, sizeof(vState));
	vState.q[0] = 1.0f;
	vState.mode = 0;			// level accumulate
	vState.baroBiasSet = false;

	float *P = vState.P;
	Eye(P, NX);
	for (int i = 0; i < 3; i++) {
		P[(I_POS + i) * NX + (I_POS + i)] = vParams.pInitPos;
		P[(I_VEL + i) * NX + (I_VEL + i)] = vParams.pInitVel;
		P[(I_ATT + i) * NX + (I_ATT + i)] = vParams.pInitAtt;
		P[(I_AB + i) * NX + (I_AB + i)] = vParams.pInitAccBias;
		P[(I_GB + i) * NX + (I_GB + i)] = vParams.pInitGyroBias;
	}

	NavState_t &ns = InertialNav::vState;
	memset(&ns, 0, sizeof(NavState_t));
	ns.Q[0] = 1.0f;
	if (vOrigin.bValid) {
		ns.Flags |= NAV_FLAG_ORIGIN_SET;
	}
}

bool NavEskf::Enable()
{
	Setup();
	if (vpAccel) vpAccel->Enable();
	if (vpGyro) vpGyro->Enable();
	if (vpMag) vpMag->Enable();
	return true;
}

void NavEskf::Disable()
{
	if (vpGyro) vpGyro->Disable();
	if (vpAccel) vpAccel->Disable();
	if (vpMag) vpMag->Disable();
}

void NavEskf::Reset()
{
	Setup();
}

void NavEskf::LevelInit(const float accAvg[3])
{
	float R[9];
	DcmFromGravity(accAvg, R);
	MatToQuat(R, vState.q);
	QuatNormalize(vState.q);
}

void NavEskf::Predict(const float accel[3], const float gyro[3], float dt)
{
	if (dt <= 0.0f) {
		return;
	}
	float *q = vState.q;
	float *ab = vState.ab;
	float *gb = vState.gb;

	float fb[3] = { accel[0] - ab[0], accel[1] - ab[1], accel[2] - ab[2] };
	float wb[3] = { gyro[0] - gb[0], gyro[1] - gb[1], gyro[2] - gb[2] };

	float R[9];
	QuatToDcmBE(q, R);

	float Rf[3];				// specific force in NED
	Mat3Vec(R, fb, Rf);

	float aN[3] = { Rf[0], Rf[1], Rf[2] - vParams.gravity };

	// Nominal integration, semi implicit.
	for (int i = 0; i < 3; i++) {
		vState.vel[i] += aN[i] * dt;
		vState.pos[i] += vState.vel[i] * dt;
	}

	float dth[3] = { wb[0] * dt, wb[1] * dt, wb[2] * dt };
	float dq[4];
	QuatFromRotVec(dth, dq);
	float qn[4];
	QuatMul(q, dq, qn);
	QuatNormalize(qn);
	memcpy(q, qn, sizeof(qn));

	// ---- error covariance, P = Phi P Phi^T + Q ----
	float Phi[NX * NX];
	Eye(Phi, NX);

	// d_pos / d_vel = I dt
	for (int i = 0; i < 3; i++) {
		Phi[(I_POS + i) * NX + (I_VEL + i)] = dt;
	}
	// d_vel / d_theta = -[Rf]x dt
	float sk[9];
	Skew3(Rf, sk);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Phi[(I_VEL + i) * NX + (I_ATT + j)] = -sk[i * 3 + j] * dt;
		}
	}
	// d_vel / d_accelbias = -R dt
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Phi[(I_VEL + i) * NX + (I_AB + j)] = -R[i * 3 + j] * dt;
		}
	}
	// d_theta / d_theta = I - [wb]x dt
	float skw[9];
	Skew3(wb, skw);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Phi[(I_ATT + i) * NX + (I_ATT + j)] -= skw[i * 3 + j] * dt;
		}
	}
	// d_theta / d_gyrobias = -I dt
	for (int i = 0; i < 3; i++) {
		Phi[(I_ATT + i) * NX + (I_GB + i)] = -dt;
	}

	float *P = vState.P;
	float tmp[NX * NX];
	Mul(Phi, P, tmp, NX, NX, NX);		// tmp = Phi P
	MulABt(tmp, Phi, P, NX, NX, NX);	// P = tmp Phi^T

	// Process noise, continuous spectral densities times dt on the diagonal.
	float qv = vParams.accelNoise * vParams.accelNoise * dt;
	float qa = vParams.gyroNoise * vParams.gyroNoise * dt;
	float qab = vParams.accelBiasNoise * vParams.accelBiasNoise * dt;
	float qgb = vParams.gyroBiasNoise * vParams.gyroBiasNoise * dt;
	for (int i = 0; i < 3; i++) {
		P[(I_VEL + i) * NX + (I_VEL + i)] += qv;
		P[(I_ATT + i) * NX + (I_ATT + i)] += qa;
		P[(I_AB + i) * NX + (I_AB + i)] += qab;
		P[(I_GB + i) * NX + (I_GB + i)] += qgb;
	}
	Symmetrize(P, NX);
}

void NavEskf::ApplyCorrection(const float dx[NX])
{
	for (int i = 0; i < 3; i++) {
		vState.pos[i] += dx[I_POS + i];
		vState.vel[i] += dx[I_VEL + i];
		vState.ab[i] += dx[I_AB + i];
		vState.gb[i] += dx[I_GB + i];
	}
	float dth[3] = { dx[I_ATT + 0], dx[I_ATT + 1], dx[I_ATT + 2] };
	float dq[4];
	QuatFromRotVec(dth, dq);
	float qn[4];
	QuatMul(vState.q, dq, qn);		// body frame right multiply
	QuatNormalize(qn);
	memcpy(vState.q, qn, sizeof(qn));
}

bool NavEskf::FusePos(const float innov[3], const float Rdiag[3])
{
	float dx[NX];
	if (!KalmanBlock3(vState.P, I_POS, innov, Rdiag, dx)) {
		return false;
	}
	ApplyCorrection(dx);
	return true;
}

bool NavEskf::FuseVel(const float innov[3], const float Rdiag[3])
{
	float dx[NX];
	if (!KalmanBlock3(vState.P, I_VEL, innov, Rdiag, dx)) {
		return false;
	}
	ApplyCorrection(dx);
	return true;
}

bool NavEskf::FuseHeight(float innov, float r)
{
	float dx[NX];
	if (!KalmanScalar(vState.P, I_POS + 2, innov, r, dx)) {
		return false;
	}
	ApplyCorrection(dx);
	return true;
}

void NavEskf::GeodeticToNed(double lat, double lon, float alt, float ned[3])
{
	double lat0 = vOrigin.Lat * (double)DEG_TO_RAD;
	double s = sin(lat0);
	double w = 1.0 - WGS84_E2 * s * s;
	double sw = sqrt(w);
	double Rn = WGS84_A * (1.0 - WGS84_E2) / (w * sw);	// meridian
	double Re = WGS84_A / sw;				// prime vertical
	double dLat = (lat - vOrigin.Lat) * (double)DEG_TO_RAD;
	double dLon = (lon - vOrigin.Lon) * (double)DEG_TO_RAD;
	ned[0] = (float)(dLat * (Rn + (double)vOrigin.Alt));
	ned[1] = (float)(dLon * (Re + (double)vOrigin.Alt) * cos(lat0));
	ned[2] = -(alt - vOrigin.Alt);
}

bool NavEskf::UpdateData()
{
	if (vpAccel == nullptr || vpGyro == nullptr) {
		return false;
	}
	AccelSensorData_t accel;
	GyroSensorData_t gyro;
	vpAccel->Read(accel);
	vpGyro->Read(gyro);

	// Accel in g, scaled to m/s^2 by the local gravity so a level sample reads
	// gravity exactly. Gyro in deg/s, to rad/s.
	float a[3] = { accel.Val[0] * vParams.gravity,
		       accel.Val[1] * vParams.gravity,
		       accel.Val[2] * vParams.gravity };
	float g[3] = { gyro.Val[0] * DEG_TO_RAD,
		       gyro.Val[1] * DEG_TO_RAD,
		       gyro.Val[2] * DEG_TO_RAD };

	float fa = (float)vpAccel->SamplingFrequency();		// mHz
	float dt = (fa > 0.0f) ? (1000.0f / fa) : vAccDt;
	vAccDt = dt;

	if (vState.mode == 0) {
		// Level accumulate, average gravity for the initial attitude.
		for (int i = 0; i < 3; i++) {
			vState.accSum[i] += a[i];
		}
		vState.accInitCount++;
		if (vState.accInitCount >= vParams.initSamples) {
			float avg[3] = { vState.accSum[0] / vState.accInitCount,
					 vState.accSum[1] / vState.accInitCount,
					 vState.accSum[2] / vState.accInitCount };
			LevelInit(avg);
			vState.mode = 1;
		}
		Publish(accel.Timestamp);
		return true;
	}

	Predict(a, g, dt);
	Publish(accel.Timestamp);
	return true;
}

void NavEskf::IntHandler()
{
	if (vpAccel) vpAccel->IntHandler();
	if (vpGyro) vpGyro->IntHandler();
	UpdateData();
}

bool NavEskf::InjectGnss(const NavGnssMeas_t &Meas)
{
	if (!vOrigin.bValid) {
		vOrigin.Lat = Meas.Lat;
		vOrigin.Lon = Meas.Lon;
		vOrigin.Alt = Meas.Alt;
		vOrigin.bValid = true;
		vState.pos[0] = vState.pos[1] = vState.pos[2] = 0.0f;
		InertialNav::vState.Flags |= NAV_FLAG_ORIGIN_SET;
		return true;
	}
	if (vState.mode != 1) {
		return false;
	}
	float ned[3];
	GeodeticToNed(Meas.Lat, Meas.Lon, Meas.Alt, ned);
	float innov[3] = { ned[0] - vState.pos[0],
			   ned[1] - vState.pos[1],
			   ned[2] - vState.pos[2] };
	float Rdiag[3] = { Meas.PosAccH * Meas.PosAccH,
			   Meas.PosAccH * Meas.PosAccH,
			   Meas.PosAccV * Meas.PosAccV };
	bool ok = FusePos(innov, Rdiag);

	if (ok && Meas.bVelValid) {
		float vi[3] = { Meas.Vel[0] - vState.vel[0],
				Meas.Vel[1] - vState.vel[1],
				Meas.Vel[2] - vState.vel[2] };
		float vr[3] = { Meas.VelAcc * Meas.VelAcc,
				Meas.VelAcc * Meas.VelAcc,
				Meas.VelAcc * Meas.VelAcc };
		FuseVel(vi, vr);
	}
	if (ok) {
		InertialNav::vState.Flags |= NAV_FLAG_GNSS_FUSED;
	}
	return ok;
}

bool NavEskf::InjectBaro(const NavBaroMeas_t &Meas)
{
	if (vState.mode != 1) {
		return false;
	}
	float originAlt = vOrigin.bValid ? vOrigin.Alt : 0.0f;
	if (!vState.baroBiasSet) {
		// Reference the first sample so the initial innovation is zero.
		vState.baroBias = Meas.Alt - originAlt + vState.pos[2];
		vState.baroBiasSet = true;
		return true;
	}
	float dMeas = originAlt + vState.baroBias - Meas.Alt;	// NED down
	float innov = dMeas - vState.pos[2];
	return FuseHeight(innov, Meas.Acc * Meas.Acc);
}

bool NavEskf::InjectFlow(const NavFlowMeas_t &Meas)
{
	// Baseline model. The translational flow is the measured flow minus the
	// rotational part. It is mapped to a body velocity pseudo measurement and
	// fused through the velocity path. The axis and sign mapping below is the
	// common down facing sensor convention and must be checked against the
	// specific flow device before use.
	if (vState.mode != 1 || Meas.Range <= 0.0f || Meas.Quality <= 0.0f) {
		return false;
	}
	float R[9];
	QuatToDcmBE(vState.q, R);
	float vBody[3];
	Mat3TVec(R, vState.vel, vBody);			// nav to body

	float tx = Meas.Flow[0] - Meas.Gyro[0];
	float ty = Meas.Flow[1] - Meas.Gyro[1];
	float vbMeas[3];
	vbMeas[0] =  ty * Meas.Range;
	vbMeas[1] = -tx * Meas.Range;
	vbMeas[2] = vBody[2];				// down not observed, hold

	float vNed[3];
	Mat3Vec(R, vbMeas, vNed);			// body to nav
	float innov[3] = { vNed[0] - vState.vel[0],
			   vNed[1] - vState.vel[1],
			   0.0f };
	float rxy = (Meas.Acc * Meas.Range) * (Meas.Acc * Meas.Range);
	float Rdiag[3] = { rxy, rxy, 1.0e6f };		// down weighted out
	return FuseVel(innov, Rdiag);
}

bool NavEskf::ZeroVelocityUpdate()
{
	if (vState.mode != 1) {
		return false;
	}
	float innov[3] = { -vState.vel[0], -vState.vel[1], -vState.vel[2] };
	float Rdiag[3] = { 1.0e-4f, 1.0e-4f, 1.0e-4f };
	return FuseVel(innov, Rdiag);
}

void NavEskf::Publish(uint64_t timestamp)
{
	NavState_t &ns = InertialNav::vState;
	ns.Timestamp = timestamp;
	for (int i = 0; i < 3; i++) {
		ns.Pos[i] = vState.pos[i];
		ns.Vel[i] = vState.vel[i];
		ns.AccelBias[i] = vState.ab[i];
		ns.GyroBias[i] = vState.gb[i];
	}
	for (int i = 0; i < 4; i++) {
		ns.Q[i] = vState.q[i];
	}
	if (vState.mode == 1) {
		ns.Flags |= NAV_FLAG_ATTITUDE_ALIGNED;
	}
}

bool NavEskf::Read(NavState_t &Data)
{
	Data = InertialNav::vState;
	return true;
}
