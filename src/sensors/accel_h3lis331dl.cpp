/**-------------------------------------------------------------------------
@file	accel_h3lis331dl.cpp

@brief	Implementation of ST H3LIS331DL accel. sensor

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2019

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

#include <stdint.h>

#include "coredev/spi.h"
#include "sensors/accel_h3lis331dl.h"

/**
 * @brief	Initialize accelerometer sensor.
 *
 * NOTE: This sensor must be the first to be initialized.
 *
 * @param 	Cfg		: Accelerometer configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool AccelH3lis331dl::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
}

bool AccelH3lis331dl::Enable()
{
	return true;
}
void AccelH3lis331dl::Disable()
{

}
void AccelH3lis331dl::Reset()
{

}

// Default base initialization. Does detection and set default config for all sensor.
// All sensor init must call this first prio to initializing itself
bool AccelH3lis331dl::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;
	uint8_t mst = 0;

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || (DevAddr != H3LIS331DL_I2C_DEVADDR && DevAddr != H3LIS331DL_I2C_DEVADDR1))
	{
		regaddr = H3LIS331DL_CTRL_REG4_REG;
		if (((SPI*)vpIntrf)->Mode() == SPIMODE_3WIRE)
		{
			d = Read8(&regaddr, 1) | H3LIS331DL_CTRL_REG4_SIM_3WIRE;
			Write8(&regaddr, 1, d);
		}
	}

	// Read chip id
	regaddr = H3LIS331DL_WHO_AM_I_REG;
	d = Read8(&regaddr, 1);
printf("Chip Id %x\r\n", d);

	if (d != H3LIS331DL_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	return true;
}

bool AccelH3lis331dl::UpdateData()
{
	uint8_t regaddr = H3LIS331DL_STATUS_REG_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d & 0xf)
	{
		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		regaddr = H3LIS331DL_OUT_X_L_REG;
		Device::Read(&regaddr, 1, (uint8_t*)vData.Val, 6);

		return true;
	}

	return false;
}

