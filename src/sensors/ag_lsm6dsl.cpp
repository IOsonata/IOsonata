/**-------------------------------------------------------------------------
@file	ag_lsm6dsl.cpp

@brief	ST LSM6DSL accel gyro implementation

The accelerometer and the gyroscope run in high performance mode. Enable
writes the output data rate, which powers each of them on, Disable writes 0,
which powers it down. The temperature is measured while either one runs.

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

#include "sensors/ag_lsm6dsl.h"

// Bound of the status reads that wait for the software reset to end. The
// reset takes 50 us, shorter than one register read.
#define LSM6DSL_RESET_POLL_MAX			100

// Output data rates in mHz of the codes LSM6DSL_ODR_12_5HZ to
// LSM6DSL_ODR_6660HZ, high performance mode.
static const uint32_t s_Lsm6dslOdrFreq[] = {
	12500, 26000, 52000, 104000, 208000, 416000, 833000, 1660000, 3330000, 6660000
};

static const int s_Lsm6dslOdrCount = sizeof(s_Lsm6dslOdrFreq) / sizeof(s_Lsm6dslOdrFreq[0]);

// Smallest rate at or above Freq, the highest rate above it. Returns the ODR
// code and updates Freq to the actual rate.
static uint8_t Lsm6dslOdr(uint32_t &Freq)
{
	int i = 0;

	while (i < s_Lsm6dslOdrCount - 1 && s_Lsm6dslOdrFreq[i] < Freq)
	{
		i++;
	}

	Freq = s_Lsm6dslOdrFreq[i];

	return (uint8_t)(LSM6DSL_ODR_12_5HZ + i);
}

bool AccelLsm6dsl::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	vOdr = LSM6DSL_ODR_POWER_DOWN;
	Range(LSM6DSL_ACCEL_ADC_RANGE);
	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);

	return true;
}

uint32_t AccelLsm6dsl::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = LSM6DSL_CTRL1_XL_REG;
	uint8_t d = Read8(&regaddr, 1);

	vOdr = Lsm6dslOdr(Freq);

	// Change the rate now only when running, else Enable writes it
	if (d & LSM6DSL_CTRL1_XL_ODR_XL_MASK)
	{
		d = (d & ~LSM6DSL_CTRL1_XL_ODR_XL_MASK) | (vOdr << LSM6DSL_CTRL1_XL_ODR_XL_POS);
		Write8(&regaddr, 1, d);
	}

	return AccelSensor::SamplingFrequency(Freq);
}

uint16_t AccelLsm6dsl::Scale(uint16_t Value)
{
	uint8_t regaddr = LSM6DSL_CTRL1_XL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM6DSL_CTRL1_XL_FS_XL_MASK;

	if (Value <= 2)
	{
		d |= LSM6DSL_CTRL1_XL_FS_XL_2G;
		Value = 2;
	}
	else if (Value <= 4)
	{
		d |= LSM6DSL_CTRL1_XL_FS_XL_4G;
		Value = 4;
	}
	else if (Value <= 8)
	{
		d |= LSM6DSL_CTRL1_XL_FS_XL_8G;
		Value = 8;
	}
	else
	{
		d |= LSM6DSL_CTRL1_XL_FS_XL_16G;
		Value = 16;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::Scale(Value);
}

bool AccelLsm6dsl::Enable()
{
	uint8_t regaddr = LSM6DSL_CTRL1_XL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM6DSL_CTRL1_XL_ODR_XL_MASK;

	d |= vOdr << LSM6DSL_CTRL1_XL_ODR_XL_POS;

	return Write8(&regaddr, 1, d);
}

void AccelLsm6dsl::Disable()
{
	uint8_t regaddr = LSM6DSL_CTRL1_XL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM6DSL_CTRL1_XL_ODR_XL_MASK;

	Write8(&regaddr, 1, d);
}

bool GyroLsm6dsl::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	vOdr = LSM6DSL_ODR_POWER_DOWN;
	Range(LSM6DSL_GYRO_ADC_RANGE);
	SamplingFrequency(Cfg.Freq);
	Sensitivity(Cfg.Sensitivity);

	return true;
}

uint32_t GyroLsm6dsl::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = LSM6DSL_CTRL2_G_REG;
	uint8_t d = Read8(&regaddr, 1);

	vOdr = Lsm6dslOdr(Freq);

	// Change the rate now only when running, else Enable writes it
	if (d & LSM6DSL_CTRL2_G_ODR_G_MASK)
	{
		d = (d & ~LSM6DSL_CTRL2_G_ODR_G_MASK) | (vOdr << LSM6DSL_CTRL2_G_ODR_G_POS);
		Write8(&regaddr, 1, d);
	}

	return GyroSensor::SamplingFrequency(Freq);
}

uint16_t GyroLsm6dsl::Sensitivity(uint16_t Value)
{
	uint8_t regaddr = LSM6DSL_CTRL2_G_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LSM6DSL_CTRL2_G_FS_125 | LSM6DSL_CTRL2_G_FS_G_MASK);

	if (Value <= 125)
	{
		d |= LSM6DSL_CTRL2_G_FS_125;
		Value = 125;
	}
	else if (Value <= 250)
	{
		d |= LSM6DSL_CTRL2_G_FS_G_250;
		Value = 250;
	}
	else if (Value <= 500)
	{
		d |= LSM6DSL_CTRL2_G_FS_G_500;
		Value = 500;
	}
	else if (Value <= 1000)
	{
		d |= LSM6DSL_CTRL2_G_FS_G_1000;
		Value = 1000;
	}
	else
	{
		d |= LSM6DSL_CTRL2_G_FS_G_2000;
		Value = 2000;
	}

	Write8(&regaddr, 1, d);

	return GyroSensor::Sensitivity(Value);
}

bool GyroLsm6dsl::Enable()
{
	uint8_t regaddr = LSM6DSL_CTRL2_G_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM6DSL_CTRL2_G_ODR_G_MASK;

	d |= vOdr << LSM6DSL_CTRL2_G_ODR_G_POS;

	return Write8(&regaddr, 1, d);
}

void GyroLsm6dsl::Disable()
{
	uint8_t regaddr = LSM6DSL_CTRL2_G_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM6DSL_CTRL2_G_ODR_G_MASK;

	Write8(&regaddr, 1, d);
}

bool TempLsm6dsl::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	vDataRdyHandler = Cfg.DataRdyCB;

	return Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer);
}

bool TempLsm6dsl::Enable()
{
	// Measured while the accelerometer or the gyroscope runs
	return true;
}

void TempLsm6dsl::Disable()
{
	// Stops with the accelerometer and the gyroscope
}

AgLsm6dsl::AgLsm6dsl()
{
	memset(vbSensorEnabled, 0, sizeof(vbSensorEnabled));
	AccelLsm6dsl::vOdr = LSM6DSL_ODR_POWER_DOWN;
	GyroLsm6dsl::vOdr = LSM6DSL_ODR_POWER_DOWN;
	TempSensor::vDataRdyHandler = NULL;
}

bool AgLsm6dsl::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
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

	uint8_t regaddr = LSM6DSL_WHO_AM_I_REG;
	uint8_t d = 0;

	if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		if (DevAddr == LSM6DSL_I2C_DEV_ADDR0 || DevAddr == LSM6DSL_I2C_DEV_ADDR1)
		{
			DeviceAddress(DevAddr);
			d = Read8(&regaddr, 1);
		}
		else
		{
			// Address not given, try both
			DeviceAddress(LSM6DSL_I2C_DEV_ADDR0);
			d = Read8(&regaddr, 1);

			if (d != LSM6DSL_WHO_AM_I_ID)
			{
				DeviceAddress(LSM6DSL_I2C_DEV_ADDR1);
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

	if (d != LSM6DSL_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	regaddr = LSM6DSL_CTRL3_C_REG;
	d = Read8(&regaddr, 1);

	if (d & LSM6DSL_CTRL3_C_SW_RESET)
	{
		// Reset did not end
		return false;
	}

	// Coherent 16 bits samples, register address increment, push-pull INT pins
	d = LSM6DSL_CTRL3_C_BDU | LSM6DSL_CTRL3_C_IF_INC;

	if (IntPol == DEVINTR_POL_LOW)
	{
		d |= LSM6DSL_CTRL3_C_H_LACTIVE;
	}

	Write8(&regaddr, 1, d);

	// No data ready while the filters settle after a power up or a rate change
	regaddr = LSM6DSL_CTRL4_C_REG;
	Write8(&regaddr, 1, LSM6DSL_CTRL4_C_DRDY_MASK);

	if (Inter)
	{
		// Data ready as pulses, a sample that is not read does not hold the
		// pin and the next sample makes a new edge
		regaddr = LSM6DSL_DRDY_PULSE_CFG_G_REG;
		Write8(&regaddr, 1, LSM6DSL_DRDY_PULSE_CFG_G_DRDY_PULSED);

		if (Inter & 1)
		{
			regaddr = LSM6DSL_INT1_CTRL_REG;
			Write8(&regaddr, 1, LSM6DSL_INT1_CTRL_DRDY_XL | LSM6DSL_INT1_CTRL_DRDY_G);
		}

		if (Inter & 2)
		{
			regaddr = LSM6DSL_INT2_CTRL_REG;
			Write8(&regaddr, 1, LSM6DSL_INT2_CTRL_DRDY_XL | LSM6DSL_INT2_CTRL_DRDY_G);
		}

		InterruptId(Inter);
		InterruptEnabled(true);
	}

	Valid(true);

	return true;
}

bool AgLsm6dsl::Enable()
{
	bool res = AccelLsm6dsl::Enable();

	if (res == true)
	{
		res = GyroLsm6dsl::Enable();
	}

	return res;
}

void AgLsm6dsl::Disable()
{
	AccelLsm6dsl::Disable();
	GyroLsm6dsl::Disable();
}

void AgLsm6dsl::Reset()
{
	uint8_t regaddr = LSM6DSL_CTRL3_C_REG;

	Write8(&regaddr, 1, LSM6DSL_CTRL3_C_SW_RESET);

	for (int i = 0; i < LSM6DSL_RESET_POLL_MAX; i++)
	{
		if ((Read8(&regaddr, 1) & LSM6DSL_CTRL3_C_SW_RESET) == 0)
		{
			break;
		}
	}
}

void AgLsm6dsl::IntHandler()
{
	if (UpdateData() == true && vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

bool AgLsm6dsl::UpdateData()
{
	uint8_t regaddr = LSM6DSL_STATUS_REG;
	uint8_t status = Read8(&regaddr, 1);

	if ((status & (LSM6DSL_STATUS_XLDA | LSM6DSL_STATUS_GDA | LSM6DSL_STATUS_TDA)) == 0)
	{
		return false;
	}

	// Temperature, gyroscope then accelerometer, little endian
	uint8_t d[14];

	regaddr = LSM6DSL_OUT_TEMP_L_REG;

	if (Device::Read(&regaddr, 1, d, sizeof(d)) != (int)sizeof(d))
	{
		return false;
	}

	uint64_t t = vpTimer != NULL ? vpTimer->uSecond() : 0;

	// Degree C x 100
	int32_t temp = ((int32_t)(int16_t)(d[0] | (d[1] << 8)) * 100) / LSM6DSL_TEMP_SENSITIVITY +
				   LSM6DSL_TEMP_OFFSET * 100;

	if (status & LSM6DSL_STATUS_TDA)
	{
		TempSensor::vData.Timestamp = t;
		TempSensor::vData.Temperature = temp;
	}

	if (status & LSM6DSL_STATUS_GDA)
	{
		GyroSensor::vData.Timestamp = t;
		GyroSensor::vData.Temp = (int16_t)temp;
		GyroSensor::vData.X = (int16_t)(d[2] | (d[3] << 8));
		GyroSensor::vData.Y = (int16_t)(d[4] | (d[5] << 8));
		GyroSensor::vData.Z = (int16_t)(d[6] | (d[7] << 8));
		GyroSensor::vSampleCnt++;
	}

	if (status & LSM6DSL_STATUS_XLDA)
	{
		AccelSensor::vData.Timestamp = t;
		AccelSensor::vData.Temp = (int16_t)temp;
		AccelSensor::vData.X = (int16_t)(d[8] | (d[9] << 8));
		AccelSensor::vData.Y = (int16_t)(d[10] | (d[11] << 8));
		AccelSensor::vData.Z = (int16_t)(d[12] | (d[13] << 8));
		AccelSensor::vSampleCnt++;
	}

	if ((status & LSM6DSL_STATUS_TDA) && TempSensor::vDataRdyHandler != NULL)
	{
		TempSensor::vDataRdyHandler(this, &(TempSensor::vData));
	}

	return true;
}
