/**-------------------------------------------------------------------------
@file	imu_icm20948.cpp

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

@author	Hoang Nguyen Hoan
@date	Sept. 9, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
#include "idelay.h"
#include "imu/imu_icm20948.h"
#include "sensors/agm_icm20948.h"

//#define ICM20948_WHO_AM_I_ID		0xEA

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

bool ImuIcm20948::Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == NULL)
	{
		return false;
	}

	Imu::Init(Cfg, pAccel, pGyro, pMag);

	vpIcm = (AgmIcm20948*)pAccel;
	vEvtHandler = Cfg.EvtHandler;

	return true;
}

bool ImuIcm20948::Enable()
{
	return vpIcm->Enable();
}

void ImuIcm20948::Disable()
{
	vpIcm->Disable();
}

void ImuIcm20948::Reset()
{
	vpIcm->Reset();
}

IMU_FEATURE ImuIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	return Imu::Feature();
}

bool ImuIcm20948::Calibrate()
{
	return true;
}

void ImuIcm20948::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuIcm20948::Compass(bool bEn)
{
	return true;
}

bool ImuIcm20948::Pedometer(bool bEn)
{
	return true;
}

bool ImuIcm20948::Quaternion(bool bEn, int NbAxis)
{
	return true;
}

bool ImuIcm20948::Tap(bool bEn)
{
	return true;
}

bool ImuIcm20948::UpdateData()
{
	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}

	return true;
}

void ImuIcm20948::IntHandler()
{
	vpIcm->IntHandler();
}

int ImuIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int ImuIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

