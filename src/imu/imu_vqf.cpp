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

ImuVqf::ImuVqf() : vVqf(0, 0, 0)
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

	vqf_real_t vqf_gyro[3] = { gyro.X, gyro.Y, gyro.Z };
	vqf_real_t vqf_acc[3] = { acc.X, acc.Y, acc.Z };
	vqf_real_t q[4];

	if (vPrevTimeStamp != 0)
	{
		deltatime = (gyro.Timestamp - vPrevTimeStamp) / 1000000.0;
	}
	vPrevTimeStamp = gyro.Timestamp;

	if (vpMag)
	{
		vqf_real_t vqf_mag[3] = { mag.X, mag.Y, mag.Z };
		vVqf.update(vqf_gyro, vqf_acc, vqf_mag);
		vVqf.getQuat9D(q);
	}
	else
	{
		vVqf.update(vqf_gyro, vqf_acc);
		vVqf.getQuat6D(q);
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

VQF::VQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    params = VQFParams();
    setup();
}

VQF::VQF(const VQFParams& params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs) : params(params) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    setup();
}
#endif
