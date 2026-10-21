/**-------------------------------------------------------------------------
@file	press_lps22hb.cpp

@brief	ST LPS22HB pressure and temperature sensor implementation

UpdateData reads the status and both outputs in one access, from STATUS to
TEMP_OUT_H, with the register address increment that CTRL_REG2 enables by
default on I2C and SPI.

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
#include "sensors/press_lps22hb.h"

// Bound of the reads that wait for the software reset to end
#define LPS22HB_RESET_POLL_MAX			100

PressLps22hb::PressLps22hb()
{
	vOdr = LPS22HB_CTRL_REG1_ODR_1HZ;
	vLpfp = 0;
	PressSensor::vEvtyHandler = NULL;
	TempSensor::vDataRdyHandler = NULL;
}

bool PressLps22hb::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
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

	uint8_t regaddr = LPS22HB_WHO_AM_I_REG;
	uint8_t d = 0;

	if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		if (DevAddr == LPS22HB_I2C_DEV_ADDR0 || DevAddr == LPS22HB_I2C_DEV_ADDR1)
		{
			DeviceAddress(DevAddr);
			d = Read8(&regaddr, 1);
		}
		else
		{
			// Address not given, try both
			DeviceAddress(LPS22HB_I2C_DEV_ADDR0);
			d = Read8(&regaddr, 1);

			if (d != LPS22HB_WHO_AM_I_ID)
			{
				DeviceAddress(LPS22HB_I2C_DEV_ADDR1);
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

	if (d != LPS22HB_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	regaddr = LPS22HB_CTRL_REG2_REG;
	if (Read8(&regaddr, 1) & LPS22HB_CTRL_REG2_SWRESET)
	{
		// Reset did not end
		return false;
	}

	// Power down, coherent pressure and temperature samples
	regaddr = LPS22HB_CTRL_REG1_REG;
	Write8(&regaddr, 1, LPS22HB_CTRL_REG1_BDU);

	Valid(true);

	return true;
}

bool PressLps22hb::Init(const PressureSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	PressSensor::vEvtyHandler = Cfg.EvtHandler;
	PressSensor::vOpMode = Cfg.OpMode;

	if (Cfg.FilterCoeff >= 20)
	{
		vLpfp = LPS22HB_CTRL_REG1_EN_LPFP | LPS22HB_CTRL_REG1_LPFP_CFG;
	}
	else if (Cfg.FilterCoeff >= 9)
	{
		vLpfp = LPS22HB_CTRL_REG1_EN_LPFP;
	}
	else
	{
		vLpfp = 0;
	}

	uint8_t regaddr = LPS22HB_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LPS22HB_CTRL_REG1_EN_LPFP | LPS22HB_CTRL_REG1_LPFP_CFG);

	Write8(&regaddr, 1, d | vLpfp);

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_ONE_SHOT;
		PressSensor::SamplingFrequency(0);
		TempSensor::SamplingFrequency(0);
	}
	else
	{
		SamplingFrequency(Cfg.Freq);
	}

	return true;
}

bool PressLps22hb::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	TempSensor::vDataRdyHandler = Cfg.DataRdyCB;
	TempSensor::vOpMode = Cfg.OpMode;

	uint8_t regaddr = LPS22HB_CTRL_REG3_REG;

	if (Cfg.Inter)
	{
		// Push-pull INT_DRDY pin with the data ready signal, high while new
		// data waits unless active low
		uint8_t d = LPS22HB_CTRL_REG3_DRDY;

		if (Cfg.IntPol == DEVINTR_POL_LOW)
		{
			d |= LPS22HB_CTRL_REG3_INT_H_L;
		}
		Write8(&regaddr, 1, d);
		InterruptId(Cfg.Inter);
		InterruptEnabled(true);
	}
	else
	{
		Write8(&regaddr, 1, 0);
	}

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_ONE_SHOT;
		PressSensor::SamplingFrequency(0);
		TempSensor::SamplingFrequency(0);
	}
	else
	{
		SamplingFrequency(Cfg.Freq);
	}

	return true;
}

uint32_t PressLps22hb::SamplingFrequency(uint32_t Freq)
{
	if (Freq <= 1000)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_1HZ;
		Freq = 1000;
	}
	else if (Freq <= 10000)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_10HZ;
		Freq = 10000;
	}
	else if (Freq <= 25000)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_25HZ;
		Freq = 25000;
	}
	else if (Freq <= 50000)
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_50HZ;
		Freq = 50000;
	}
	else
	{
		vOdr = LPS22HB_CTRL_REG1_ODR_75HZ;
		Freq = 75000;
	}

	uint8_t regaddr = LPS22HB_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1);

	// Change the rate now only when running, else Enable writes it
	if (d & LPS22HB_CTRL_REG1_ODR_MASK)
	{
		Write8(&regaddr, 1, (d & ~LPS22HB_CTRL_REG1_ODR_MASK) | vOdr);
	}

	PressSensor::SamplingFrequency(Freq);

	return TempSensor::SamplingFrequency(Freq);
}

bool PressLps22hb::Enable()
{
	uint8_t regaddr;

	if (vLpfp)
	{
		// Start the filter from the first sample
		regaddr = LPS22HB_LPFP_RES_REG;
		Read8(&regaddr, 1);
	}

	regaddr = LPS22HB_CTRL_REG1_REG;

	return Write8(&regaddr, 1, vOdr | vLpfp | LPS22HB_CTRL_REG1_BDU);
}

void PressLps22hb::Disable()
{
	uint8_t regaddr = LPS22HB_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LPS22HB_CTRL_REG1_ODR_MASK;

	Write8(&regaddr, 1, d);
}

void PressLps22hb::Reset()
{
	uint8_t regaddr = LPS22HB_CTRL_REG2_REG;

	// Control registers to their default values: power down, register
	// address increment on, INT_DRDY pin off
	Write8(&regaddr, 1, LPS22HB_CTRL_REG2_IF_ADD_INC | LPS22HB_CTRL_REG2_SWRESET);

	for (int i = 0; i < LPS22HB_RESET_POLL_MAX; i++)
	{
		if ((Read8(&regaddr, 1) & LPS22HB_CTRL_REG2_SWRESET) == 0)
		{
			break;
		}
	}
}

bool PressLps22hb::StartSampling()
{
	if (vOdr != LPS22HB_CTRL_REG1_ODR_ONE_SHOT)
	{
		// Measures continuously
		return true;
	}

	uint8_t regaddr = LPS22HB_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d & LPS22HB_CTRL_REG2_ONE_SHOT)
	{
		// Previous measurement not done
		return false;
	}

	return Write8(&regaddr, 1, d | LPS22HB_CTRL_REG2_ONE_SHOT);
}

bool PressLps22hb::UpdateData()
{
	// Status, pressure 24 bits then temperature 16 bits, little endian
	uint8_t d[6];
	uint8_t regaddr = LPS22HB_STATUS_REG;

	if (Device::Read(&regaddr, 1, d, sizeof(d)) != (int)sizeof(d))
	{
		return false;
	}

	uint8_t status = d[0];

	if ((status & (LPS22HB_STATUS_P_DA | LPS22HB_STATUS_T_DA)) == 0)
	{
		return false;
	}

	uint64_t t = vpTimer != NULL ? vpTimer->uSecond() : 0;

	if (status & LPS22HB_STATUS_P_DA)
	{
		uint32_t u = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16);
		int32_t raw = (u & 0x800000U) ? (int32_t)u - 0x1000000 : (int32_t)u;

		// Pa rounded, from hPa x 4096: raw x 100 / 4096 = raw x 25 / 1024
		int32_t pa = raw > 0 ? (raw * 25 + 512) / 1024 : 0;

		PressSensor::vData.Timestamp = t;
		PressSensor::vData.Pressure = (uint32_t)pa;
		PressSensor::vSampleCnt++;

		if (PressSensor::vEvtyHandler != NULL)
		{
			PressSensor::vEvtyHandler(this, &(PressSensor::vData));
		}
	}

	if (status & LPS22HB_STATUS_T_DA)
	{
		// Degree C x 100 is the output itself
		TempSensor::vData.Timestamp = t;
		TempSensor::vData.Temperature = (int16_t)(d[4] | (d[5] << 8));
		TempSensor::vSampleCnt++;

		if (TempSensor::vDataRdyHandler != NULL)
		{
			TempSensor::vDataRdyHandler(this, &(TempSensor::vData));
		}
	}

	return true;
}

void PressLps22hb::IntHandler()
{
	if (UpdateData() == true && Device::vEvtHandler != NULL)
	{
		Device::vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}
