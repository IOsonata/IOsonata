/**-------------------------------------------------------------------------
@file	mag_ak09940.cpp

@brief	Implementation of Asahi Kasei AK09940x mag sensor


@author	Hoang Nguyen Hoan
@date	May 3, 2025

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

#include "idelay.h"
#include "sensors/mag_ak09940.h"

bool MagAk09940::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t regaddr = AK09940_WIA1_REG;
	uint16_t d = 0;

	MagSensor::Type(SENSOR_TYPE_MAG);
	Interface(pIntrf);

	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		DeviceAddress(Cfg.DevAddr);
	}
	else
	{
		for (uint32_t ad = AK09940_I2C_7BITS_DEVADDR0; ad <= AK09940_I2C_7BITS_DEVADDR3; ad++)
		{
			DeviceAddress(AK09940_I2C_7BITS_DEVADDR0);
			Read(&regaddr, 1, (uint8_t*)&d, 2);

			if (d == AK09940_COMPANY_DEVICE_ID)
			{
				break;
			}
		}
	}

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Read(&regaddr, 1, (uint8_t*)&d, 2);

	if (d != AK09940_COMPANY_DEVICE_ID)
	{
		return false;
	}

	DataReadyClear();

	Reset();

	Range(AK09940_ADC_RANGE);

	vbFifoEn = Cfg.bFifoEn;

	vSensitivity[0] = AK09940_SENSITIVITY;
	vSensitivity[1] = vSensitivity[0];
	vSensitivity[2] = vSensitivity[0];

	vData.Sensitivity[0] = vSensitivity[0];
	vData.Sensitivity[1] = vSensitivity[1];
	vData.Sensitivity[2] = vSensitivity[2];

	ClearCalibration();

	vOpMode = Cfg.OpMode;

	vCtrl1Val = 0;
	vCtrl3Val = 0;

#if 0
	regaddr = AK09940_CTRL2_REG;
	d = 0;	// Disable temp sensor
	Write(&regaddr, 1, (uint8_t*)&d, 1);
#endif

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE || Cfg.OpMode == SENSOR_OPMODE_TIMER)
	{
		// No fifo in single shot
		regaddr = AK09940_CTRL3_REG;
		vCtrl3Val = AK09940_CTRL3_MODE_SINGLE;

		Write(&regaddr, 1, &vCtrl3Val, 1);
	}
	else
	{
		// Default fifo enable in continuous mode

		if (vbFifoEn)
		{
			vCtrl3Val = AK09940_CTRL3_FIFO_EN;
			vCtrl1Val = AK09940_FIFO_WM_LEVEL_MIN;
		}

		if (vOpMode == SENSOR_OPMODE_LOW_POWER)
		{
			vCtrl1Val |= AK09940_CTRL1_MT2_EN;
		}
		else
		{
			vCtrl3Val |= AK09940_CTRL3_MT_LN1;
		}

		SamplingFrequency(Cfg.Freq);
	}

	regaddr = AK09940_CTRL1_REG;
	Write(&regaddr, 1, (uint8_t*)&vCtrl1Val, 1);

	return true;
}

uint32_t MagAk09940::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = AK09940_CTRL3_REG;
	uint8_t d = 0;
	bool forcelp = false;

	vCtrl3Val &= ~AK09940_CTRL3_MODE_MASK;

	Write(&regaddr, 1, &d, 1);

	usDelay(100);

	if (Freq < 20000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_10HZ;
		Freq = 10000;
	}
	else if (Freq < 50000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_20HZ;
		Freq = 20000;
	}
	else if (Freq < 100000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_50HZ;
		Freq = 50000;
	}
	else if (Freq < 200000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_100HZ;
		Freq = 100000;
	}
	else if (Freq < 400000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_200HZ;
		Freq = 200000;
	}
	else if (Freq < 1000000)
	{
		// Must be in one of the low power mode
		vCtrl3Val &= AK09940_CTRL3_MT_MASK;
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_400HZ;
		Freq = 400000;
		forcelp = true;
	}
	else if (Freq < 2500000)
	{
		// Must be in one of the low power mode 1 or ultra only
		vCtrl3Val &= AK09940_CTRL3_MT_MASK;
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_1000HZ;
		Freq = 1000000;
		forcelp = true;
	}
	else
	{
		// Must be in ultra low power only
		vCtrl3Val &= AK09940_CTRL3_MT_MASK;
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_2500HZ;
		Freq = 2500000;
		forcelp = true;
	}

	if (forcelp && vOpMode != SENSOR_OPMODE_LOW_POWER)
	{
		vCtrl1Val &= ~vOpMode == SENSOR_OPMODE_LOW_POWER;
		regaddr = AK09940_CTRL1_REG;
		Write(&regaddr, 1, &vCtrl1Val, 1);
	}

	return MagSensor::SamplingFrequency(Freq);
}

/**
 * @brief	Power off the device completely.
 *
 * If supported, this will put the device in complete power down.
 * Full re-intialization is required to re-enable the device.
 */
void MagAk09940::PowerOff()
{
	uint8_t regaddr = AK09940_CTRL3_REG;
	uint8_t d = 0;

	Write(&regaddr, 1, &d, 1);
}

bool MagAk09940::Enable()
{
	uint8_t regaddr = AK09940_ST_REG;
	uint16_t d;

	regaddr = AK09940_CTRL3_REG;
	Write(&regaddr, 1, &vCtrl3Val, 1);
	Read(&regaddr, 1, (uint8_t*)&d, 1);

	// Reset DRDY.  Do not remove
	regaddr = AK09940_ST1_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);
	regaddr = AK09940_ST2_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);

	State(SENSOR_STATE_SAMPLING);

	return true;
}

void MagAk09940::Disable()
{
	uint8_t regaddr = AK09940_CTRL3_REG;
	uint8_t d = 0;

	Read(&regaddr, 1, &d, 1);
	d &= AK09940_CTRL3_MODE_MASK;

	Write(&regaddr, 1, &d, 1);
}

void MagAk09940::Reset()
{
	uint8_t regaddr = AK09940_CTRL4_REG;
	uint8_t d = AK09940_CTRL4_SRST;

	Write(&regaddr, 1, (uint8_t*)&d, 1);
}

bool MagAk09940::StartSampling(void)
{
	uint8_t regaddr = AK09940_CTRL3_REG;

	Write(&regaddr, 1, &vCtrl3Val, 1);

	State(SENSOR_STATE_SAMPLING);

	return true;
}

bool MagAk09940::UpdateData()
{
	uint8_t regaddr = AK09940_ST1_REG;
	uint8_t d[2];
	uint64_t t;

	Read(&regaddr, 1, d, 1);

	//if (d[0] & AK09940_ST_DRDY)
	if (isDataReady())
	{
		int nb = (d[0] & AK09940_ST1_FNUM_MASK) >> 1;
		if (vpTimer)
		{
			t = vpTimer->uSecond();
		}

		uint8_t dd[16];

		if (vbFifoEn == false)
		{
			regaddr = AK09940_HXL_REG;
			Read(&regaddr, 1, (uint8_t*)dd, 10);
		}
		else
		{
			for (int i = 0; i < nb; i++)
			{
				regaddr = AK09940_HXL_REG;
				Read(&regaddr, 1, (uint8_t*)dd, 10);
			}
		}
		regaddr = AK09940_ST2_REG;
		Read(&regaddr, 1, (uint8_t*)d, 1);

		if ((d[0] & AK09940_ST2_INV) == 0)
		{
			vData.Timestamp = t;
			vData.X = dd[0] | (dd[1] << 8) | ((int8_t)dd[2] << 16);
			vData.Y = dd[3] | (dd[4] << 8) | ((int8_t)dd[5] << 16);
			vData.Z = dd[6] | (dd[7] << 8) | ((int8_t)dd[8] << 16);
		}

		DataReadyClear();

		return true;
	}

	return false;
}

void MagAk09940::Flush(void)
{
	UpdateData();
}

/**
 * @brief	Interrupt handler (optional)
 *
 * Sensor that supports interrupt can implement this to handle interrupt.
 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
 */
void MagAk09940::IntHandler(void)
{
	uint8_t regaddr = AK09940_ST_REG;
	uint8_t st, st1, d[2];

	Read(&regaddr, 1, &st, 1);

	if (st & AK09940_ST_DRDY)
	{
		DataReadySet();
		UpdateData();
		if (vCtrl3Val < 3)
		{
			State(SENSOR_STATE_IDLE);
		}
	}
}

