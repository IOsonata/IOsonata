/**-------------------------------------------------------------------------
@file	ag_icm456x.cpp

@brief	TDK Invensen ICM456x series accel gyro implementation

This file implements only accel & gyro part of the ICM456x. IMU features are
implemented in imu implementation file.


@author	Hoang Nguyen Hoan
@date	Mar. 19, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#include <memory.h>

#include "sensors/ag_icm456x.h"


bool AccelIcm456x::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	return true;
}
uint32_t AccelIcm456x::SamplingFrequency(uint32_t Freq)
{
	return 0;
}
uint8_t AccelIcm456x::Scale(uint8_t Value)
{
	return 0;
}

/**
 * @brief	Set and enable filter cutoff frequency
 *
 * Optional implementation can override this to implement filtering supported by the device
 *
 * @param	Freq : Filter frequency in mHz
 *
 * @return	Actual frequency in mHz
 */
uint32_t AccelIcm456x::FilterFreq(uint32_t Freq)
{
	return 0;
}
bool AccelIcm456x::Enable()
{
	return false;
}

void AccelIcm456x::Disable()
{

}

bool GyroIcm456x::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return false;
}
uint32_t GyroIcm456x::SamplingFrequency(uint32_t Freq)
{
	return 0;
}
uint32_t GyroIcm456x::Sensitivity(uint32_t Value)
{
	return 0;
}

/**
 * @brief	Set and enable filter cutoff frequency
 *
 * Optional implementation can override this to implement filtering supported by the device
 *
 * @param	Freq : Filter frequency in mHz
 *
 * @return	Actual frequency in mHz
 */
uint32_t GyroIcm456x::FilterFreq(uint32_t Freq)
{
	return 0;
}
bool GyroIcm456x::Enable()
{
	return false;
}

void GyroIcm456x::Disable()
{

}

/**
 * @brief	Initialize sensor (require implementation).
 *
 * @param 	CfgData : Reference to configuration data
 * @param	pIntrf 	: Pointer to interface to the sensor.
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 * @param	pTimer	: Pointer to timer for retrieval of time stamp
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool TempIcm456x::Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool TempIcm456x::Enable()
{
	return false;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * @return	None
 */
void TempIcm456x::Disable()
{

}

AgIcm456x::AgIcm456x()
{
	memset(vbSensorEnabled, 0, sizeof(vbSensorEnabled));
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO;
}

bool AgIcm456x::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return false;
}

bool AgIcm456x::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);

	uint8_t regaddr = ICM456X_WHO_AM_I_REG;
	uint8_t d;

	if (DevAddr == ICM456X_I2C_7BITS_DEVADDR0 || DevAddr == ICM456X_I2C_7BITS_DEVADDR1)
	{
		if (pIntrf->Type() != DEVINTRF_TYPE_I2C)
		{
			// Device address is set to I2C but interface is not
			return false;
		}

		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_WHO_AM_I_REG)
		{
			return false;
		}
	}
	else if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		// Interface is I2C but device address is not set.
		// Detect device
		DeviceAddress(ICM456X_I2C_7BITS_DEVADDR0);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_CHIP_ID)
		{
			// Try alternate address
			DeviceAddress(ICM456X_I2C_7BITS_DEVADDR1);
			d = Read8((uint8_t*)&regaddr, 1);
			if (d != ICM456X_CHIP_ID)
			{
				return false;
			}
		}
	}
	else
	{
		// Not I2C interface
		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_CHIP_ID)
		{
			return false;
		}
	}

	// Device found, save device id
	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Valid(true);

	Reset();

	regaddr = ICM456X_FIFO_CONFIG0_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG0_DEPTH_2K | ICM456X_FIFO_CONFIG0_MODE_STREAM);

	regaddr = ICM456X_FIFO_CONFIG1_0_REG;

	uint16_t wm = ICM456X_FIFO_SIZE * 80 / 100; // 80%

	Write16(&regaddr, 1, wm);

	regaddr = ICM456X_FIFO_CONFIG2_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG2_WR_WM_GT_TH);


	return true;
}

bool AgIcm456x::Enable()
{
	return false;
}

void AgIcm456x::Disable()
{

}

void AgIcm456x::Reset()
{

}

void AgIcm456x::IntHandler()
{

}

bool AgIcm456x::UpdateData()
{
	return false;
}

//int AgIcm45686::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
//{
//	return 0;
//}
