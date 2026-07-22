/**-------------------------------------------------------------------------
@file	att_xiot_fusion.cpp

@brief	Implementation of the Att class using the x-io Fusion library

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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

#include "Fusion/Fusion.h"

#include "motion/att_xiot_fusion.h"

#define SAMPLE_RATE		50

bool AttXiotFusion::Init(const AttCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == nullptr || pGyro == nullptr)
	{
		return false;
	}

	bool res = Att::Init(Cfg, pAccel, pGyro, pMag);

	if (res == true)
	{
		// Init fusion lib
	    // Initialise algorithms
	    FusionBias bias = {
	    	.settings = { (float)(pAccel->SamplingFrequency() / 1000), },
	    };
	    //FusionAhrs ahrs;

	    FusionBiasInitialise(&bias);
	    FusionAhrsInitialise(&vAhrs);

	    // Set the Fusion algorithm settings
	    const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.gyroscopeRange = (float)pGyro->Sensitivity(), // actual gyroscope range in degrees/s
			.accelerationRejection = 10.0f,
			.magneticRejection = 10.0f,
			// The rejection recovery timeout field name differs across Fusion
			// versions; leave it default (0) by omitting it here.
	    };
	    vSettings = settings;
	    FusionAhrsSetSettings(&vAhrs, &vSettings);

	}

	return res;
}

bool AttXiotFusion::Enable()
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

void AttXiotFusion::Disable()
{

}

void AttXiotFusion::Reset()
{

}

bool AttXiotFusion::UpdateData()
{
	AccelSensorData_t acc;
	GyroSensorData_t gyro;
	MagSensorData_t mag;

	Read(acc);
	Read(gyro);

	if (vpMag)
	{
		Read(mag);
	}
	float deltatime = 0;


	if (vPrevTimeStamp != 0)
	{
		deltatime = (gyro.Timestamp - vPrevTimeStamp) / 1000000.0;
	}
	vPrevTimeStamp = gyro.Timestamp;

    // Update the Fusion algorithm

	FusionVector fvgyro = { gyro.X,  gyro.Y, gyro.Z };
	FusionVector fvacc = { acc.X,  acc.Y, acc.Z };

	if (vpMag)
	{
		FusionVector fvmag = { mag.X,  mag.Y, mag.Z };
	    FusionAhrsUpdate(&vAhrs, fvgyro, fvacc, fvmag);
	}
	else
	{
		FusionAhrsUpdateNoMagnetometer(&vAhrs, fvgyro, fvacc);
	}

    FusionAhrsGetQuaternion(&vAhrs);

    vQuat.Q[0] = vAhrs.quaternion.array[0];
    vQuat.Q[1] = vAhrs.quaternion.array[1];
    vQuat.Q[2] = vAhrs.quaternion.array[2];
    vQuat.Q[3] = vAhrs.quaternion.array[3];

	return true;
}

void AttXiotFusion::IntHandler()
{
	vpAccel->IntHandler();
	vpGyro->IntHandler();
	if (vpMag)
	{
		vpMag->IntHandler();
	}

	UpdateData();
}

uint32_t AttXiotFusion::Rate(uint32_t DataRate)
{
	return Att::Rate(DataRate);
}

bool AttXiotFusion::Calibrate()
{

	return true;
}

void AttXiotFusion::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
	(void)pMatrix;
}

bool AttXiotFusion::Compass(bool bEn)
{
	(void)bEn;

	return true;
}

bool AttXiotFusion::Quaternion(bool bEn, int NbAxis)
{
	(void)bEn;
	(void)NbAxis;

	FusionQuaternion fvquat = FusionAhrsGetQuaternion(&vAhrs);

	return true;
}

