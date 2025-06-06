/**-------------------------------------------------------------------------
@file	imu_vqf.cpp

@brief	Implementation of software imu class using vqf fusion

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

#include "imu/imu_vqf.h"

#ifdef __ARM_ARCH

#include "dsp/quaternion_math_functions.h"

#define Quat2Rotation	arm_quaternion2rotation_f32
#define QuatNormalize	arm_quaternion_normalize_f32
/*
void arm_quaternion2rotation_f32(const float32_t *pInputQuaternions,
    float32_t *pOutputRotations,
    uint32_t nbQuaternions)*/

#else
#endif

static inline float square(float x) { return x * x; }

void QuatRotate(const float q[4], const float v[3], float out[3])
{
	out[0] = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
	out[1] = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
	out[2] = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
}

void QuatMultiply(const float q1[4], const float q2[4], float out[4])
{
	out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void NormalizeVect(float *pVect, size_t VectSize)
{
	size_t n = VectSize - 1;
	float div = 0;

	while (n >= 0)
	{
		div += pVect[n] * pVect[n];
		n--;
	}

	div = sqrt(div);

	while (--VectSize >= 0)
	{
		pVect[VectSize] /= div;
	}
}

void Matrix3SetToScaledIdentity(float scale, float out[9])
{
    out[0] = scale;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = scale;
    out[5] = 0.0;
    out[6] = 0.0;
    out[7] = 0.0;
    out[8] = scale;
}

void Matrix3Multiply(const float in1[9], const float in2[9], float out[9])
{
    out[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6];
    out[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7];
    out[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    out[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6];
    out[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7];
    out[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    out[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6];
    out[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7];
    out[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
}

void Matrix3MultiplyTpsFirst(const float in1[9], const float in2[9], float out[9])
{
    out[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6];
    out[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7];
    out[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    out[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6];
    out[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7];
    out[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    out[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6];
    out[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7];
    out[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
}

void Matrix3MultiplyTpsSecond(const float in1[9], const float in2[9], float out[9])
{
    out[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2];
    out[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5];
    out[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    out[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2];
    out[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
    out[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    out[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2];
    out[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5];
    out[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
}

bool Matrix3Inv(const float in[9], float out[9])
{
    // in = [a b c; d e f; g h i]
    double A = in[4]*in[8] - in[5]*in[7]; // (e*i - f*h)
    double D = in[2]*in[7] - in[1]*in[8]; // -(b*i - c*h)
    double G = in[1]*in[5] - in[2]*in[4]; // (b*f - c*e)
    double B = in[5]*in[6] - in[3]*in[8]; // -(d*i - f*g)
    double E = in[0]*in[8] - in[2]*in[6]; // (a*i - c*g)
    double H = in[2]*in[3] - in[0]*in[5]; // -(a*f - c*d)
    double C = in[3]*in[7] - in[4]*in[6]; // (d*h - e*g)
    double F = in[1]*in[6] - in[0]*in[7]; // -(a*h - b*g)
    double I = in[0]*in[4] - in[1]*in[3]; // (a*e - b*d)

    double det = in[0]*A + in[1]*B + in[2]*C; // a*A + b*B + c*C;

#define EPS std::numeric_limits<float>::epsilon()

    if (det >= -EPS && det <= EPS) {
        memset(out, 0, 9 * sizeof(float));
        return false;
    }

    // out = [A D G; B E H; C F I]/det
    out[0] = A/det;
    out[1] = D/det;
    out[2] = G/det;
    out[3] = B/det;
    out[4] = E/det;
    out[5] = H/det;
    out[6] = C/det;
    out[7] = F/det;
    out[8] = I/det;

    return true;
}

void Clip(float vec[], size_t N, float min, float max)
{
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        } else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}

ImuVqf::ImuVqf()
{

}

bool ImuVqf::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr)
	{
		return false;
	}

	bool res = Imu::Init(Cfg, pAccel, pGyro, pMag);

	if (res == true)
	{
	    // Much more conservative mag correction, slightly slower acc
	    vParams.tauAcc = 0.02f;
	    vParams.tauMag = 9.0f;

	    // Enable all protection features
	    vParams.restBiasEstEnabled = true;
	    vParams.magDistRejectionEnabled = true;

	    vParams.biasSigmaInit = 0.1f;
	    vParams.biasForgettingTime = 0.5f;
	    vParams.biasClip = 8.0f;

	    vParams.biasSigmaRest = 0.02f;
	    vParams.restMinT = 0.05f;
	    vParams.restFilterTau = 0.20f;

	    // Much slower magnetic field updates
	    vParams.restThGyr = 0.3f;
	    vParams.restThAcc = 0.3f;
	    vParams.magCurrentTau = 0.1f;

	    // Much stricter magnetic field validation
	    vParams.magRefTau = 6.0f;
	    vParams.magNormTh = 0.08f;
	    vParams.magDipTh = 8.0f;
	    vParams.magNewTime = 20.0f;
	    vParams.magNewFirstTime = 5.0f;
	    vParams.magNewMinGyr = 20.0f;

		// Much longer undisturbed time requirement
		vParams.magMinUndisturbedTime = 2.0f;
	    vParams.magMaxRejectionTime = 90.0f;

	    vParams.magRejectionFactor = 3.0f;
	}

	return res;
}

bool ImuVqf::Enable()
{
	bool res = vpAccel->Enable();

	if (res == true)
	{
		res = vpGyro->Enable();
	}

	if (vpMag)
	{
		res = vpMag->Enable();
	}

	return res;
}

void ImuVqf::Disable()
{

}

void ImuVqf::Reset()
{

}

bool ImuVqf::UpdateData()
{
	AccelSensorData_t acc;
	GyroSensorData_t gyro;
	MagSensorData_t mag;

#if 0
	vpAccel->UpdateData();
	vpGyro->UpdateData();
	if (vpMag)
	{
		vpMag->UpdateData();
	}
#endif
	Read(acc);
	Read(gyro);

	if (vpMag)
	{
		Read(mag);
	}
	float deltatime = 0;

	float vqf_gyro[3] = { gyro.X, gyro.Y, gyro.Z };
	float vqf_acc[3] = { acc.X, acc.Y, acc.Z };
	float q[4];

	if (vPrevTimeStamp != 0)
	{
		deltatime = (gyro.Timestamp - vPrevTimeStamp) / 1000000.0;
	}
	vPrevTimeStamp = gyro.Timestamp;

	if (vpMag)
	{
		float vqf_mag[3] = { mag.X, mag.Y, mag.Z };
		//vVqf.update(vqf_gyro, vqf_acc, vqf_mag);
		//vVqf.getQuat9D(q);
	}
	else
	{
//		vVqf.update(vqf_gyro, vqf_acc);
//		vVqf.getQuat6D(q);
	}

    vQuat.Q[0] = q[0];
    vQuat.Q[1] = q[1];
    vQuat.Q[2] = q[2];
    vQuat.Q[3] = q[3];

	return true;
}

void ImuVqf::IntHandler()
{
	vpAccel->IntHandler();
	vpGyro->IntHandler();
	if (vpMag)
	{
		vpMag->IntHandler();
	}

	UpdateData();
}

uint32_t ImuVqf::Rate(uint32_t DataRate)
{
	return Imu::Rate(DataRate);
}

bool ImuVqf::Calibrate()
{

	return true;
}

void ImuVqf::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{

}

bool ImuVqf::Compass(bool bEn)
{

	return true;
}

bool ImuVqf::Pedometer(bool bEn)
{

	return true;
}

bool ImuVqf::Quaternion(bool bEn, int NbAxis)
{
	//FusionQuaternion fvquat = FusionAhrsGetQuaternion(&vAhrs);

	return true;
}

bool ImuVqf::Tap(bool bEn)
{

	return true;
}

void ImuVqf::ProcessAccel(void)
{
	AccelSensorData_t acc;

	Read(acc);

    // ignore [0 0 0] samples
    if (acc.X == 0.0 && acc.Y == 0.0 && acc.Z == 0.0)
    {
        return;
    }

    // rest detection
    if (vParams.restBiasEstEnabled)
    {
        FilterVec(acc.Val, 3, vParams.restFilterTau, vCoeffs.accTs, vCoeffs.restAccLpB, vCoeffs.restAccLpA,
                  vState.restAccLpState, vState.restLastAccLp);

        acc.X -= vState.restLastAccLp[0];
        acc.Y -= vState.restLastAccLp[1];
        acc.Z -= vState.restLastAccLp[2];

        vState.restLastSquaredDeviations[1] = acc.X * acc.X + acc.Y * acc.Y + acc.Z * acc.Z;

        if (vState.restLastSquaredDeviations[1] >= (vParams.restThAcc * vParams.restThAcc))
        {
            vState.restT = 0.0;
            vState.restDetected = false;
        } else
        {
            vState.restT += vCoeffs.accTs;
            if (vState.restT >= vParams.restMinT)
            {
                vState.restDetected = true;
            }
        }
    }

    float accEarth[3];

    // filter acc in inertial frame
    QuatRotate(vState.gyrQuat, acc.Val, accEarth);
    FilterVec(accEarth, 3, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA, vState.accLpState, vState.lastAccLp);

    // transform to 6D earth frame and normalize
    QuatRotate(vState.accQuat, vState.lastAccLp, accEarth);
    NormalizeVect(accEarth, 3);

    // inclination correction
    float accCorrQuat[4];
    float q_w = sqrt((accEarth[2]+1)/2);
    if (q_w > 1e-6)
    {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    }
    else
    {
        // to avoid numeric issues when acc is close to [0 0 -1], i.e. the correction step is close (<= 0.00011°) to 180°:
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    QuatMultiply(accCorrQuat, vState.accQuat, vState.accQuat);
    NormalizeVect(vState.accQuat, 4);

    // calculate correction angular rate to facilitate debugging
    vState.lastAccCorrAngularRate = acos(accEarth[2])/vCoeffs.accTs;

    // bias estimation
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    if (vParams.motionBiasEstEnabled || vParams.restBiasEstEnabled)
    {
        float biasClip = vParams.biasClip*float(M_PI/180.0);

        float accGyrQuat[4];
        float R[9];
        float biasLp[2];

        // get rotation matrix corresponding to accGyrQuat
//        getQuat6D(accGyrQuat);
        QuatMultiply(vState.accQuat, vState.gyrQuat, accGyrQuat);

        R[0] = 1 - 2*square(accGyrQuat[2]) - 2*square(accGyrQuat[3]); // r11
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]); // r12
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]); // r13
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]); // r21
        R[4] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[3]); // r22
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]); // r23
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]); // r31
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]); // r32
        R[8] = 1 - 2*square(accGyrQuat[1]) - 2*square(accGyrQuat[2]); // r33

        // calculate R*b_hat (only the x and y component, as z is not needed)
        biasLp[0] = R[0]*vState.bias[0] + R[1]*vState.bias[1] + R[2]*vState.bias[2];
        biasLp[1] = R[3]*vState.bias[0] + R[4]*vState.bias[1] + R[5]*vState.bias[2];

        // low-pass filter R and R*b_hat
        FilterVec(R, 9, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA, vState.motionBiasEstRLpState, R);
        FilterVec(biasLp, 2, vParams.tauAcc, vCoeffs.accTs, vCoeffs.accLpB, vCoeffs.accLpA, vState.motionBiasEstBiasLpState,
                  biasLp);

        // set measurement error and covariance for the respective Kalman filter update
        float w[3];
        float e[3];
        if (vState.restDetected && vParams.restBiasEstEnabled)
        {
            e[0] = vState.restLastGyrLp[0] - vState.bias[0];
            e[1] = vState.restLastGyrLp[1] - vState.bias[1];
            e[2] = vState.restLastGyrLp[2] - vState.bias[2];
            Matrix3SetToScaledIdentity(1.0, R);
            std::fill(w, w+3, vCoeffs.biasRestW);
        }
        else if (vParams.motionBiasEstEnabled)
        {
            e[0] = -accEarth[1]/vCoeffs.accTs + biasLp[0] - R[0]*vState.bias[0] - R[1]*vState.bias[1] - R[2]*vState.bias[2];
            e[1] = accEarth[0]/vCoeffs.accTs + biasLp[1] - R[3]*vState.bias[0] - R[4]*vState.bias[1] - R[5]*vState.bias[2];
            e[2] = - R[6]*vState.bias[0] - R[7]*vState.bias[1] - R[8]*vState.bias[2];
            w[0] = vCoeffs.biasMotionW;
            w[1] = vCoeffs.biasMotionW;
            w[2] = vCoeffs.biasVerticalW;
        }
        else
        {
            std::fill(w, w+3, -1); // disable update
        }

        // Kalman filter update
        // step 1: P = P + V (also increase covariance if there is no measurement update!)
        if (vState.biasP[0] < vCoeffs.biasP0) {
            vState.biasP[0] += vCoeffs.biasV;
        }
        if (vState.biasP[4] < vCoeffs.biasP0) {
            vState.biasP[4] += vCoeffs.biasV;
        }
        if (vState.biasP[8] < vCoeffs.biasP0) {
            vState.biasP[8] += vCoeffs.biasV;
        }
        if (w[0] >= 0) {
            // clip disagreement to -2..2 °/s
            // (this also effectively limits the harm done by the first inclination correction step)
            Clip(e, 3, -biasClip, biasClip);

            // step 2: K = P R^T inv(W + R P R^T)
            float K[9];
            Matrix3MultiplyTpsSecond(vState.biasP, R, K); // K = P R^T
            Matrix3Multiply(R, K, K); // K = R P R^T
            K[0] += w[0];
            K[4] += w[1];
            K[8] += w[2]; // K = W + R P R^T
            Matrix3Inv(K, K); // K = inv(W + R P R^T)
            Matrix3MultiplyTpsFirst(R, K, K); // K = R^T inv(W + R P R^T)
            Matrix3Multiply(vState.biasP, K, K); // K = P R^T inv(W + R P R^T)

            // step 3: bias = bias + K (y - R bias) = bias + K e
            vState.bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            vState.bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            vState.bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

            // step 4: P = P - K R P
            Matrix3Multiply(K, R, K); // K = K R
            Matrix3Multiply(K,vState.biasP, K); // K = K R P
            for(size_t i = 0; i < 9; i++) {
                vState.biasP[i] -= K[i];
            }

            // clip bias estimate to -2..2 °/s
            Clip(vState.bias, 3, -biasClip, biasClip);
        }
    }
#else
    // simplified implementation of bias estimation for the special case in which only rest bias estimation is enabled
    if (params.restBiasEstEnabled) {
        float biasClip = params.biasClip*float(M_PI/180.0);
        if (state.biasP < coeffs.biasP0) {
            state.biasP += coeffs.biasV;
        }
        if (state.restDetected) {
            float e[3];
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            clip(e, 3, -biasClip, biasClip);

            // Kalman filter update, simplified scalar version for rest update
            // (this version only uses the first entry of P as P is diagonal and all diagonal elements are the same)
            // step 1: P = P + V (done above!)
            // step 2: K = P R^T inv(W + R P R^T)
            float k = state.biasP/(coeffs.biasRestW + state.biasP);
            // step 3: bias = bias + K (y - R bias) = bias + K e
            state.bias[0] += k*e[0];
            state.bias[1] += k*e[1];
            state.bias[2] += k*e[2];
            // step 4: P = P - K R P
            state.biasP -= k*state.biasP;
            clip(state.bias, 3, -biasClip, biasClip);
        }
    }
#endif
#if 0
    QuatMultiply(vState.accQuat, vState.gyrQuat, Imu::vQuat.Q);
#endif
}

void ImuVqf::ProcessGyro(void)
{
	GyroSensorData_t gyro;

	Read(gyro);

    // rest detection
    if (vParams.restBiasEstEnabled || vParams.magDistRejectionEnabled) {
        FilterVec(gyro.Val, 3, vParams.restFilterTau, vCoeffs.gyrTs, vCoeffs.restGyrLpB, vCoeffs.restGyrLpA,
                  vState.restGyrLpState, vState.restLastGyrLp);

        vState.restLastSquaredDeviations[0] = square(gyro.Val[0] - vState.restLastGyrLp[0])
                + square(gyro.Val[1] - vState.restLastGyrLp[1]) + square(gyro.Val[2] - vState.restLastGyrLp[2]);

        float biasClip = vParams.biasClip * M_PI / 180.0;
        if (vState.restLastSquaredDeviations[0] >= square(vParams.restThGyr * M_PI / 180.0)
                || fabs(vState.restLastGyrLp[0]) > biasClip || fabs(vState.restLastGyrLp[1]) > biasClip
                || fabs(vState.restLastGyrLp[2]) > biasClip) {
            vState.restT = 0.0;
            vState.restDetected = false;
        }
    }

    // remove estimated gyro bias
    float gyrNoBias[3] = {gyro.Val[0]-vState.bias[0], gyro.Val[1]-vState.bias[1], gyro.Val[2]-vState.bias[2]};

    // gyroscope prediction step
    float gyrNorm = 0.0;

    for (int i = 0; i < 3; i++)
    {
    	gyrNorm += gyrNoBias[i];
    }

    gyrNorm = sqrt(gyrNorm);

    float angle = gyrNorm * vCoeffs.gyrTs;
    if (gyrNorm > EPS) {
        float c = cos(angle/2);
        float s = sin(angle/2)/gyrNorm;
        float gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        QuatMultiply(vState.gyrQuat, gyrStepQuat, vState.gyrQuat);
        NormalizeVect(vState.gyrQuat, 4);
    }

#if 0
    QuatMultiply(vState.accQuat, vState.gyrQuat, Imu::vQuat.Q);
#endif
}

void ImuVqf::ProcessMag(void)
{
	GyroSensorData_t mag;

	Read(mag);

	// ignore [0 0 0] samples
    if (mag.Val[0] == 0.0 && mag.Val[1] == 0.0 && mag.Val[2] == 0.0) {
        return;
    }

    float magEarth[3];

    // bring magnetometer measurement into 6D earth frame
    float accGyrQuat[4];
//    getQuat6D(accGyrQuat);

    QuatMultiply(vState.accQuat, vState.gyrQuat, accGyrQuat);

    QuatRotate(accGyrQuat, mag.Val, magEarth);

    if (vParams.magDistRejectionEnabled)
    {
    	vState.magNormDip[0] = 0.0;

    	for (int i = 0; i < 3; i++)
    	{
    		vState.magNormDip[0] += magEarth[i];
    	}
        vState.magNormDip[0] = sqrt(vState.magNormDip[0]);//norm(magEarth, 3);
        vState.magNormDip[1] = -asin(magEarth[2]/vState.magNormDip[0]);

        if (vParams.magCurrentTau > 0) {
            FilterVec(vState.magNormDip, 2, vParams.magCurrentTau, vCoeffs.magTs, vCoeffs.magNormDipLpB,
                      vCoeffs.magNormDipLpA, vState.magNormDipLpState, vState.magNormDip);
        }

        // magnetic disturbance detection
        if (fabs(vState.magNormDip[0] - vState.magRefNorm) < vParams.magNormTh*vState.magRefNorm
                && fabs(vState.magNormDip[1] - vState.magRefDip) < vParams.magDipTh* M_PI / 180.0) {
            vState.magUndisturbedT += vCoeffs.magTs;
            if (vState.magUndisturbedT >= vParams.magMinUndisturbedTime) {
                vState.magDistDetected = false;
                vState.magRefNorm += vCoeffs.kMagRef*(vState.magNormDip[0] - vState.magRefNorm);
                vState.magRefDip += vCoeffs.kMagRef*(vState.magNormDip[1] - vState.magRefDip);
            }
        } else {
            vState.magUndisturbedT = 0.0;
            vState.magDistDetected = true;
        }

        // new magnetic field acceptance
        if (fabs(vState.magNormDip[0] - vState.magCandidateNorm) < vParams.magNormTh*vState.magCandidateNorm
                && fabs(vState.magNormDip[1] - vState.magCandidateDip) < vParams.magDipTh* M_PI/180.0)
        {
        	float div = 0.0;

        	for (int i = 0; i < 3; i++)
        	{
        		div += vState.restLastGyrLp[i];
        	}

            //if (norm(vState.restLastGyrLp, 3) >= vParams.magNewMinGyr*M_PI/180.0)
        	if (sqrt(div) >= vParams.magNewMinGyr*M_PI/180.0)
            {
                vState.magCandidateT += vCoeffs.magTs;
            }
            vState.magCandidateNorm += vCoeffs.kMagRef*(vState.magNormDip[0] - vState.magCandidateNorm);
            vState.magCandidateDip += vCoeffs.kMagRef*(vState.magNormDip[1] - vState.magCandidateDip);

            if (vState.magDistDetected && (vState.magCandidateT >= vParams.magNewTime || (
                    vState.magRefNorm == 0.0 && vState.magCandidateT >= vParams.magNewFirstTime))) {
                vState.magRefNorm = vState.magCandidateNorm;
                vState.magRefDip = vState.magCandidateDip;
                vState.magDistDetected = false;
                vState.magUndisturbedT = vParams.magMinUndisturbedTime;
            }
        } else {
            vState.magCandidateT = 0.0;
            vState.magCandidateNorm = vState.magNormDip[0];
            vState.magCandidateDip = vState.magNormDip[1];
        }
    }

    // calculate disagreement angle based on current magnetometer measurement
    vState.lastMagDisAngle = atan2(magEarth[0], magEarth[1]) - vState.delta;

    // make sure the disagreement angle is in the range [-pi, pi]
    if (vState.lastMagDisAngle > M_PI) {
        vState.lastMagDisAngle -= (2.0*M_PI);
    } else if (vState.lastMagDisAngle < (-M_PI)) {
        vState.lastMagDisAngle += (2.0*M_PI);
    }

    float k = vCoeffs.kMag;

    if (vParams.magDistRejectionEnabled) {
        // magnetic disturbance rejection
        if (vState.magDistDetected) {
            if (vState.magRejectT <= vParams.magMaxRejectionTime) {
                vState.magRejectT += vCoeffs.magTs;
                k = 0;
            } else {
                k /= vParams.magRejectionFactor;
            }
        } else {
            vState.magRejectT = std::max(vState.magRejectT - vParams.magRejectionFactor*vCoeffs.magTs, (float)0.0);
        }
    }

    // ensure fast initial convergence
    if (vState.kMagInit != 0.0) {
        // make sure that the gain k is at least 1/N, N=1,2,3,... in the first few samples
        if (k < vState.kMagInit) {
            k = vState.kMagInit;
        }

        // iterative expression to calculate 1/N
        vState.kMagInit = vState.kMagInit/(vState.kMagInit+1);

        // disable if t > tauMag
        if (vState.kMagInit*vParams.tauMag < vCoeffs.magTs) {
            vState.kMagInit = 0.0;
        }
    }

    // first-order filter step
    vState.delta += k*vState.lastMagDisAngle;
    // calculate correction angular rate to facilitate debugging
    vState.lastMagCorrAngularRate = k*vState.lastMagDisAngle/vCoeffs.magTs;

    // make sure delta is in the range [-pi, pi]
    if (vState.delta > M_PI) {
        vState.delta -= (2.0*M_PI);
    } else if (vState.delta < (-M_PI)) {
        vState.delta += (2.0*M_PI);
    }

#if 0
    QuatMultiply(vState.accQuat, vState.gyrQuat, Imu::vQuat.Q);
    //    QuatApplyDelta(Imu::vQuat.Q, vState.delta, Imu::vQuat.Q);

    float c = cos(vState.delta / 2.0);
    float s = sin(vState.delta / 2.0);

	float w = c * Imu::vQuat.Q[0] - s * Imu::vQuat.Q[3];
	float x = c * Imu::vQuat.Q[1] - s * Imu::vQuat.Q[2];
	float y = c * Imu::vQuat.Q[2] + s * Imu::vQuat.Q[1];
	float z = c * Imu::vQuat.Q[3] + s * Imu::vQuat.Q[0];

	Imu::vQuat.Q1 = w;
	Imu::vQuat.Q2 = x;
	Imu::vQuat.Q3 = y;
	Imu::vQuat.Q4 = z;
#endif
}

bool ImuVqf::Read(ImuQuat_t &Data)
{
    QuatMultiply(vState.accQuat, vState.gyrQuat, Imu::vQuat.Q);

    if (vpMag)
	{
        //    QuatApplyDelta(Imu::vQuat.Q, vState.delta, Imu::vQuat.Q);

        float c = cos(vState.delta / 2.0);
        float s = sin(vState.delta / 2.0);

    	float w = c * Imu::vQuat.Q[0] - s * Imu::vQuat.Q[3];
    	float x = c * Imu::vQuat.Q[1] - s * Imu::vQuat.Q[2];
    	float y = c * Imu::vQuat.Q[2] + s * Imu::vQuat.Q[1];
    	float z = c * Imu::vQuat.Q[3] + s * Imu::vQuat.Q[0];

    	Imu::vQuat.Q1 = w;
    	Imu::vQuat.Q2 = x;
    	Imu::vQuat.Q3 = y;
    	Imu::vQuat.Q4 = z;
	}

    Data = vQuat;

	return true;
}

void FilterInitialState(float x0, const double b[3], const double a[2], double out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0 * (1 - b[0]);
    out[1] = x0 * (b[2] - a[1]);
}

float FilterStep(float Val, const double b[3], const double a[2], double State[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    double y = b[0] * Val + State[0];

    State[0] = b[1] * Val - a[0] * y + State[1];
    State[1] = b[2] * Val - a[1] * y;

    return y;
}

/**
 * @brief Performs filter step for vector-valued signal with averaging-based initialization.
 *
 * During the first \f$\tau\f$ seconds, the filter output is the mean of the previous samples. At \f$t=\tau\f$, the
 * initial conditions for the low-pass filter are calculated based on the current mean value and from then on,
 * regular filtering with the rational transfer function described by the coefficients b and a is performed.
 *
 * @param Vec input values (array of size N)
 * @param VectSize number of values in vector-valued signal
 * @param tau filter time constant \f$\tau\f$ in seconds (used for initialization)
 * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds (used for initialization)
 * @param b numerator coefficients
 * @param a denominator coefficients (without \f$a_0=1\f$)
 * @param state filter state (array of size N*2, will be modified)
 * @param out output array for filtered values (size N)
 */
void ImuVqf::FilterVec(const float Vec[], size_t VectSize, float tau, float Ts, const double b[3],
                      const double a[2], double state[], float out[])
{
    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0]))
    { // initialization phase
        if (isnan(state[1]))
        { // first sample
            state[1] = 0; // state[1] is used to store the sample count
            for(size_t i = 0; i < VectSize; i++) {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < VectSize; i++)
        {
            state[2+i] += Vec[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau)
        {
            for(size_t i = 0; i < VectSize; i++)
            {
               FilterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }

    for (size_t i = 0; i < VectSize; i++)
    {
        out[i] = FilterStep(Vec[i], b, a, state + 2 * i);
    }
}

#if 0
VQFParams::VQFParams() :
    tauAcc(0.02f), tauMag(9.0f),  // Much more conservative mag correction, slightly slower acc
    restBiasEstEnabled(true), magDistRejectionEnabled(true),  // Enable all protection features
    biasSigmaInit(0.1f), biasForgettingTime(0.5f), biasClip(8.0f),
    biasSigmaRest(0.02f), restMinT(0.05f), restFilterTau(0.20f),
    restThGyr(0.3f), restThAcc(0.3f), magCurrentTau(0.1f),  // Much slower magnetic field updates
    magRefTau(6.0f), magNormTh(0.08f), magDipTh(8.0f),  // Much stricter magnetic field validation
    magNewTime(20.0f), magNewFirstTime(5.0f), magNewMinGyr(20.0f),
    magMinUndisturbedTime(2.0f), magMaxRejectionTime(90.0f),  // Much longer undisturbed time requirement
    magRejectionFactor(3.0f)
//	maxAngularRate(360.0f),
 //   vrStabilityThreshold(0.25f) {
{
        // Set cutoff filters
//        accLpB[0] = 8.0f;  // Filter accelerometer noise
//        magLpB[0] = 5.0f;  // Filter magnetometer noise
}

VQF::VQF(float gyrTs, float accTs, float magTs) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    params = VQFParams();
    setup();
}

VQF::VQF(const VQFParams& params, float gyrTs, float accTs, float magTs) : params(params) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    setup();
}
#endif
