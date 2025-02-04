/**-------------------------------------------------------------------------
@file	imu_xiot_fusion.cpp

@brief	Implementation of software imu class using fusion ahrs

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

#include "imu/imu_xiot_fusion.h"

#define SAMPLE_RATE		50

bool ImuXiotFusion::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	bool res = Imu::Init(Cfg, pAccel, pGyro, pMag);

	if (res == true)
	{
		// Init fusion lib
	    // Initialise algorithms
	    FusionOffset offset;
	    //FusionAhrs ahrs;

	    FusionOffsetInitialise(&offset, pAccel->SamplingFrequency() / 1000);
	    FusionAhrsInitialise(&vAhrs);

	    // Set AHRS algorithm settings
	    /*const FusionAhrsSettings*/
	    vSettings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.gyroscopeRange = (float)pGyro->Sensitivity(),// 2000.0f, /* replace this with actual gyroscope range in degrees/s */
			.accelerationRejection = 10.0f,
			.magneticRejection = 10.0f,
			.recoveryTriggerPeriod = 5 * pAccel->SamplingFrequency() / 1000, /* 5 seconds */
	    };
	    FusionAhrsSetSettings(&vAhrs, &vSettings);

	}

	return res;
}

bool ImuXiotFusion::Enable()
{
	bool res = vpAccel->Enable();

	if (res == true)
	{
		res = vpGyro->Enable();
	}

	if (vpMag && res == true)
	{
		res = vpMag->Enable();
	}

	return res;
}

void ImuXiotFusion::Disable()
{

}

void ImuXiotFusion::Reset()
{

}

bool ImuXiotFusion::UpdateData()
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
	float deltatime = (acc.Timestamp - vPrevTimeStamp) / 1000000.0;


    // Update gyroscope AHRS algorithm

	FusionVector fvgyro = { .axis = { gyro.X,  gyro.Y, gyro.Z} };
	FusionVector fvacc = { .axis = { acc.X,  acc.Y, acc.Z} };

	if (vpMag)
	{
		FusionVector fvmag = { .axis = { mag.X,  mag.Y, mag.Z} };
	    FusionAhrsUpdate(&vAhrs, fvgyro, fvacc, fvmag, deltatime);
	}
	else
	{
		FusionAhrsUpdateNoMagnetometer(&vAhrs, fvgyro, fvacc, 0.02);
	}

    FusionAhrsGetQuaternion(&vAhrs);

    vQuat.Q[0] = vAhrs.quaternion.array[0];
    vQuat.Q[1] = vAhrs.quaternion.array[1];
    vQuat.Q[2] = vAhrs.quaternion.array[2];
    vQuat.Q[3] = vAhrs.quaternion.array[3];

	return true;
}

void ImuXiotFusion::IntHandler()
{
	vpAccel->IntHandler();
	vpGyro->IntHandler();
	if (vpMag)
	{
		vpMag->IntHandler();
	}

	UpdateData();
}

uint32_t ImuXiotFusion::Rate(uint32_t DataRate)
{
	return Imu::Rate(DataRate);
}

bool ImuXiotFusion::Calibrate()
{

	return true;
}

void ImuXiotFusion::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{

}

bool ImuXiotFusion::Compass(bool bEn)
{

	return true;
}

bool ImuXiotFusion::Pedometer(bool bEn)
{

	return true;
}

bool ImuXiotFusion::Quaternion(bool bEn, int NbAxis)
{
	FusionQuaternion fvquat = FusionAhrsGetQuaternion(&vAhrs);

	return true;
}

bool ImuXiotFusion::Tap(bool bEn)
{

	return true;
}
