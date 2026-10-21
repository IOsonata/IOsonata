/**-------------------------------------------------------------------------
@file	mag_lis3mdl.cpp

@brief	ST LIS3MDL 3 axis magnetometer implementation

UpdateData reads the status, the 3 axes and the temperature in one access,
from STATUS to TEMP_OUT_H. The scale of the outputs, 6842 LSB per gauss at
4 gauss, is 14.6 nT per LSB: the calibration gain keeps it in floating point.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>

#include "sensors/mag_lis3mdl.h"

// Bound of the reads that wait for the software reset to end
#define LIS3MDL_RESET_POLL_MAX			100

// Highest rate set by DO, above it the rate depends on the X and Y mode
#define LIS3MDL_DO_FREQ_MAX				80000
#define LIS3MDL_FAST_ODR_FREQ_UHP		155000
#define LIS3MDL_FAST_ODR_FREQ_LP		1000000

// nT per LSB rounded, 100000 nT per gauss
static inline uint16_t Lis3mdlNtPerLsb(uint16_t LsbPerGauss)
{
	return (uint16_t)((100000U + LsbPerGauss / 2U) / LsbPerGauss);
}

MagLis3mdl::MagLis3mdl()
{
	vScale = 4;
	vLsbPerGauss = LIS3MDL_SENSITIVITY_4G;
	vPrecision = MAGSENSOR_PRECISION_LOW;
	memset(&vData, 0, sizeof(vData));

	uint16_t nt = Lis3mdlNtPerLsb(vLsbPerGauss);

	for (int i = 0; i < 3; i++)
	{
		vSensitivity[i] = nt;
		vData.Sensitivity[i] = nt;
	}

	ClearCalibration();
}

bool MagLis3mdl::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
	{
		return true;
	}

	if (pIntrf == NULL)
	{
		return false;
	}

	Interface(pIntrf);

	uint8_t regaddr = LIS3MDL_WHO_AM_I_REG;
	uint8_t d = 0;

	if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		if (DevAddr == LIS3MDL_I2C_DEV_ADDR0 || DevAddr == LIS3MDL_I2C_DEV_ADDR1)
		{
			DeviceAddress(DevAddr);
			d = Read8(&regaddr, 1);
		}
		else
		{
			// Address not given, try both
			DeviceAddress(LIS3MDL_I2C_DEV_ADDR0);
			d = Read8(&regaddr, 1);

			if (d != LIS3MDL_WHO_AM_I_ID)
			{
				DeviceAddress(LIS3MDL_I2C_DEV_ADDR1);
				d = Read8(&regaddr, 1);
			}
		}
	}
	else
	{
		// SPI, DevAddr is the chip select index
		DeviceAddress(DevAddr);
		d = Read8(&regaddr, 1);
	}

	if (d != LIS3MDL_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	regaddr = LIS3MDL_CTRL_REG2_REG;
	if (Read8(&regaddr, 1) & LIS3MDL_CTRL_REG2_SOFT_RST)
	{
		// Reset did not end
		return false;
	}

	// Coherent 16 bits samples
	regaddr = LIS3MDL_CTRL_REG5_REG;
	Write8(&regaddr, 1, LIS3MDL_CTRL_REG5_BDU);

	// Temperature read with the field
	regaddr = LIS3MDL_CTRL_REG1_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | LIS3MDL_CTRL_REG1_TEMP_EN);

	Range(LIS3MDL_ADC_RANGE);

	Valid(true);

	return true;
}

bool MagLis3mdl::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	vOpMode = Cfg.OpMode;

	Precision(Cfg.Precision);

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE && Cfg.Freq > LIS3MDL_DO_FREQ_MAX)
	{
		// A single conversion uses the DO rates only
		SamplingFrequency(LIS3MDL_DO_FREQ_MAX);
	}
	else
	{
		SamplingFrequency(Cfg.Freq);
	}

	if (Cfg.Inter)
	{
		InterruptId(Cfg.Inter);
		InterruptEnabled(true);
	}

	return true;
}

uint32_t MagLis3mdl::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = LIS3MDL_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LIS3MDL_CTRL_REG1_DO_MASK | LIS3MDL_CTRL_REG1_FAST_ODR);

	if (Freq > LIS3MDL_DO_FREQ_MAX)
	{
		d |= LIS3MDL_CTRL_REG1_FAST_ODR;
		Freq = vPrecision == MAGSENSOR_PRECISION_HIGH ? LIS3MDL_FAST_ODR_FREQ_UHP : LIS3MDL_FAST_ODR_FREQ_LP;
	}
	else
	{
		// 625 mHz doubled DO times
		uint8_t dout = 0;
		uint32_t f = 625;

		while (dout < 7 && f < Freq)
		{
			dout++;
			f <<= 1;
		}
		d |= dout << LIS3MDL_CTRL_REG1_DO_POS;
		Freq = f;
	}

	Write8(&regaddr, 1, d);

	return MagSensor::SamplingFrequency(Freq);
}

MAGSENSOR_PRECISION MagLis3mdl::Precision(MAGSENSOR_PRECISION Val)
{
	bool high = Val == MAGSENSOR_PRECISION_HIGH;
	uint8_t regaddr = LIS3MDL_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG1_OM_MASK;

	vPrecision = Val;

	// X and Y, then Z in the same mode
	Write8(&regaddr, 1, d | (high ? LIS3MDL_CTRL_REG1_OM_UHP : LIS3MDL_CTRL_REG1_OM_LP));

	bool fast = (d & LIS3MDL_CTRL_REG1_FAST_ODR) != 0;

	regaddr = LIS3MDL_CTRL_REG4_REG;
	d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG4_OMZ_MASK;
	Write8(&regaddr, 1, d | (high ? LIS3MDL_CTRL_REG4_OMZ_UHP : LIS3MDL_CTRL_REG4_OMZ_LP));

	if (fast)
	{
		// Above 80 Hz the rate follows the mode
		MagSensor::SamplingFrequency(high ? LIS3MDL_FAST_ODR_FREQ_UHP : LIS3MDL_FAST_ODR_FREQ_LP);
	}

	return vPrecision;
}

uint16_t MagLis3mdl::Scale(uint16_t Gauss)
{
	uint8_t fs;
	uint16_t lsb;

	if (Gauss <= 4)
	{
		fs = LIS3MDL_CTRL_REG2_FS_4G;
		lsb = LIS3MDL_SENSITIVITY_4G;
		Gauss = 4;
	}
	else if (Gauss <= 8)
	{
		fs = LIS3MDL_CTRL_REG2_FS_8G;
		lsb = LIS3MDL_SENSITIVITY_8G;
		Gauss = 8;
	}
	else if (Gauss <= 12)
	{
		fs = LIS3MDL_CTRL_REG2_FS_12G;
		lsb = LIS3MDL_SENSITIVITY_12G;
		Gauss = 12;
	}
	else
	{
		fs = LIS3MDL_CTRL_REG2_FS_16G;
		lsb = LIS3MDL_SENSITIVITY_16G;
		Gauss = 16;
	}

	uint8_t regaddr = LIS3MDL_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG2_FS_MASK;

	Write8(&regaddr, 1, d | fs);

	// Gain in uT per LSB of the new scale, the offset is in uT
	float f = (float)vLsbPerGauss / (float)lsb;

	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] *= f;
		vCalibGain[1][i] *= f;
		vCalibGain[2][i] *= f;
	}

	vLsbPerGauss = lsb;
	vScale = Gauss;

	uint16_t nt = Lis3mdlNtPerLsb(lsb);

	for (int i = 0; i < 3; i++)
	{
		vSensitivity[i] = nt;
		vData.Sensitivity[i] = nt;
	}

	return vScale;
}

void MagLis3mdl::SetCalibration(const float (&Gain)[3][3], const float (&Offset)[3])
{
	// uT per LSB, 100 uT per gauss
	float s = 100.0f / (float)vLsbPerGauss;

	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] = Gain[0][i] * s;
		vCalibGain[1][i] = Gain[1][i] * s;
		vCalibGain[2][i] = Gain[2][i] * s;
		vCalibOffset[i] = Offset[i] * s;
	}
}

void MagLis3mdl::ClearCalibration()
{
	memset(vCalibOffset, 0, sizeof(vCalibOffset));
	memset(vCalibGain, 0, sizeof(vCalibGain));

	// uT per LSB, 100 uT per gauss
	vCalibGain[0][0] = 100.0f / (float)vLsbPerGauss;
	vCalibGain[1][1] = vCalibGain[0][0];
	vCalibGain[2][2] = vCalibGain[0][0];
}

bool MagLis3mdl::Enable()
{
	if (vOpMode == SENSOR_OPMODE_SINGLE)
	{
		// StartSampling starts each conversion
		return true;
	}

	uint8_t regaddr = LIS3MDL_CTRL_REG3_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG3_MD_MASK;

	return Write8(&regaddr, 1, d | LIS3MDL_CTRL_REG3_MD_CONTINUOUS);
}

void MagLis3mdl::Disable()
{
	uint8_t regaddr = LIS3MDL_CTRL_REG3_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG3_MD_MASK;

	Write8(&regaddr, 1, d | LIS3MDL_CTRL_REG3_MD_POWER_DOWN);
}

void MagLis3mdl::Reset()
{
	uint8_t regaddr = LIS3MDL_CTRL_REG2_REG;

	// Registers to their default values: 4 gauss, low power at 10 Hz,
	// power down
	Write8(&regaddr, 1, LIS3MDL_CTRL_REG2_SOFT_RST);

	for (int i = 0; i < LIS3MDL_RESET_POLL_MAX; i++)
	{
		if ((Read8(&regaddr, 1) & LIS3MDL_CTRL_REG2_SOFT_RST) == 0)
		{
			break;
		}
	}
}

bool MagLis3mdl::StartSampling()
{
	if (vOpMode != SENSOR_OPMODE_SINGLE)
	{
		// Converts continuously
		return true;
	}

	uint8_t regaddr = LIS3MDL_CTRL_REG3_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LIS3MDL_CTRL_REG3_MD_MASK;

	return Write8(&regaddr, 1, d | LIS3MDL_CTRL_REG3_MD_SINGLE);
}

bool MagLis3mdl::UpdateData()
{
	// Status, X, Y, Z then temperature, little endian
	uint8_t d[9];
	uint8_t regaddr = LIS3MDL_STATUS_REG;

	if (Read(DeviceAddress(), &regaddr, 1, d, sizeof(d)) != (int)sizeof(d))
	{
		return false;
	}

	if ((d[0] & LIS3MDL_STATUS_ZYXDA) == 0)
	{
		return false;
	}

	int32_t t = (int16_t)(d[7] | (d[8] << 8));

	vData.Timestamp = vpTimer != NULL ? vpTimer->uSecond() : 0;
	vData.X = (int16_t)(d[1] | (d[2] << 8));
	vData.Y = (int16_t)(d[3] | (d[4] << 8));
	vData.Z = (int16_t)(d[5] | (d[6] << 8));
	// Degree C x 100
	vData.Temp = (int16_t)(t * 100 / LIS3MDL_TEMP_SENSITIVITY + LIS3MDL_TEMP_OFFSET * 100);
	vSampleCnt++;

	return true;
}

void MagLis3mdl::IntHandler()
{
	if (UpdateData() == true && Device::vEvtHandler != NULL)
	{
		Device::vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

int MagLis3mdl::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen != 1)
	{
		// Register addresses are one byte
		return 0;
	}

	uint8_t cmd = *pCmdAddr;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd |= LIS3MDL_SPI_READ;
		if (BuffLen > 1)
		{
			cmd |= LIS3MDL_SPI_ADDR_INC;
		}
	}
	else if (BuffLen > 1)
	{
		cmd |= LIS3MDL_I2C_ADDR_INC;
	}

	return vpIntrf->Read(DevAddr, &cmd, 1, pBuff, BuffLen);
}

int MagLis3mdl::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen)
{
	if (CmdAddrLen != 1)
	{
		// Register addresses are one byte
		return 0;
	}

	uint8_t cmd = *pCmdAddr;

	if (DataLen > 1)
	{
		cmd |= vpIntrf->Type() == DEVINTRF_TYPE_SPI ? LIS3MDL_SPI_ADDR_INC : LIS3MDL_I2C_ADDR_INC;
	}

	return vpIntrf->Write(DevAddr, &cmd, 1, pData, DataLen);
}
