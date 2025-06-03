/**-------------------------------------------------------------------------
@file	accel_lisxdh.cpp

@brief	Implementation of ST LISxDH accel. sensor

Implementation for LIS2DH12 & LIS3DH

Note : More details about the registers programming are described in
       application AN5005. Those important details are not in the datasheet

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "sensors/accel_lisxdh.h"
#include "idelay.h"

bool AccelLisxdh::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	// Read chip id
	uint8_t regaddr = LISXDH_WHO_AM_I_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d != LISXDH_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);
	Valid(true);

	return true;
}

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
bool AccelLisxdh::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	if (Cfg.IntHandler)
	{
		vIntHandler = Cfg.IntHandler;
	}

	// High res 12bits default
	uint8_t regaddr = LISXDH_CTRL_REG4_REG;
	uint8_t d = Read8(&regaddr, 1) | LISXDH_CTRL_REG4_HR_EN;
	Write8(&regaddr, 1, d);

	AccelSensor::Range(2047);

	Scale(Cfg.Scale);
	uint32_t f = SamplingFrequency(Cfg.Freq);

	if (Cfg.Inter)
	{

		regaddr = LISXDH_CTRL_REG3_REG;
		Write8(&regaddr, 1, LISXDH_CTRL_REG3_I1_OVERRUN | LISXDH_CTRL_REG3_I1_WTM);// |
				//LISXDH_CTRL_REG3_I1_ZYXDA | LISXDH_CTRL_REG3_I1_IA1);

		regaddr = LISXDH_CTRL_REG5_REG;
		Write8(&regaddr, 1, LISXDH_CTRL_REG5_LIR_INT1 | LISXDH_CTRL_REG5_D4D_INT1);

		regaddr = LISXDH_INT1_CFG_REG;
		Write8(&regaddr, 1, 0xf);

		regaddr = LISXDH_CTRL_REG6_REG;
		Write8(&regaddr, 1, LISXDH_CTRL_REG6_INT_POLARITY_LOW);

		regaddr = LISXDH_INT1_DURATION_REG;
		Write8(&regaddr, 1, 1000 / f);

		regaddr = LISXDH_INT1_THS_REG;
		Write8(&regaddr, 1, 1);

		vbIntEn = true;
	}
	else
	{
		regaddr = LISXDH_INT1_CFG_REG;
		Write8(&regaddr, 1, 0);
	}

	msDelay(1);

	// Flush FIFO
 	regaddr = LISXDH_FIFO_SRC_REG;
	d = Read8(&regaddr, 1) & LISXDH_FIFO_SRC_REG_FSS_MASK;

	uint8_t b[196];

	regaddr = LISXDH_OUT_X_L_REG | 0x40;
	if (d > 0)
	{
		Device::Read(&regaddr, 1, b, d * 6);
	}

	Enable();

	return true;
}

bool AccelLisxdh::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	uint8_t regaddr = LISXDH_TEMP_CFG_REG;
	Write8(&regaddr, 1, LISXDH_TEMP_CFG_TEMP_EN);

	regaddr = LISXDH_CTRL_REG4_REG;
	uint8_t d = Read8(&regaddr, 1) | LISXDH_CTRL_REG4_BDU;
	Write8(&regaddr, 1, d);

	regaddr = LISXDH_CTRL_REG1_REG;
	d = Read8(&regaddr, 1);


	if (d & LISXDH_CTRL_REG1_LPEN)
	{
		TempSensor::Range(127);
	}
	else
	{
		TempSensor::Range(511);
	}

	return true;
}


uint16_t AccelLisxdh::Scale(uint16_t Value)
{
	uint8_t regaddr = LISXDH_CTRL_REG4_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LISXDH_CTRL_REG4_FS_MASK;

	if (Value < 3)
	{
		Value = 2;
	}
	else if (Value < 6)
	{
		Value = 4;
		d |= LISXDH_CTRL_REG4_FS_4G;
	}
	else if (Value < 10)
	{
		Value = 8;
		d |= LISXDH_CTRL_REG4_FS_8G;
	}
	else
	{
		Value = 16;
		d |= LISXDH_CTRL_REG4_FS_16G;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::Scale(Value);
}

/**
 * @brief	Set sampling frequency.
 *
 * The sampling frequency is relevant only in continuous mode.
 *
 * @return	Frequency in mHz (milliHerz)
 */
uint32_t AccelLisxdh::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = LISXDH_CTRL_REG4_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LISXDH_CTRL_REG4_HR_EN);
	uint32_t f = 0;
	uint32_t range = 2047;
	uint32_t trange = 511;

	regaddr = LISXDH_CTRL_REG1_REG;
	d = Read8(&regaddr, 1) & ~(LISXDH_CTRL_REG1_ODR_MASK | LISXDH_CTRL_REG1_LPEN);

	if (Freq == 0 || Freq >= 2000000)
	{
		// High freq.
		// 5.376 KHz, LP mode only
		f = 5376000;
		d |= LISXDH_CTRL_REG1_ODR_HR_LP | LISXDH_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}
	else if (Freq < 5000)
	{
		// 1 Hz
		f = 1;
		d |= LISXDH_CTRL_REG1_ODR_1HZ;
	}
	else if (Freq < 17500)
	{
		// 10 Hz
		f = 10;
		d |= LISXDH_CTRL_REG1_ODR_10HZ;
	}
	else if (Freq < 37500)
	{
		// 25 Hz
		f = 25;
		d |= LISXDH_CTRL_REG1_ODR_25HZ;
	}
	else if (Freq < 75000)
	{
		// 50 Hz
		f = 50;
		d |= LISXDH_CTRL_REG1_ODR_50HZ;
	}
	else if (Freq < 150000)
	{
		// 100 Hz
		f = 100;
		d |= LISXDH_CTRL_REG1_ODR_100HZ;
	}
	else if (Freq < 300000)
	{
		// 200 Hz
		f = 200;
		d |= LISXDH_CTRL_REG1_ODR_200HZ;
	}
	else if (Freq < 600000)
	{
		// 400 Hz
		f = 400;
		d |= LISXDH_CTRL_REG1_ODR_400HZ;
	}
	else if (Freq < 1400000)
	{
		// 1.344 KHz, HR/Normal
		f = 1344000;
		d |= LISXDH_CTRL_REG1_ODR_HR_LP;
	}
	else if (Freq < 2000000)
	{
		// 1.62 KHz, LP mode only
		f = 1620000;
		d |= LISXDH_CTRL_REG1_ODR_1620HZ | LISXDH_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}
	else
	{
		// 5.376 KHz, LP mode only
		f = 5376000;
		d |= LISXDH_CTRL_REG1_ODR_HR_LP | LISXDH_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}

	Write8(&regaddr, 1, d);

	if ((d & LISXDH_CTRL_REG1_LPEN) == 0)
	{
		regaddr = LISXDH_CTRL_REG4_REG;
		d = Read8(&regaddr, 1) | LISXDH_CTRL_REG4_HR_EN;
		Write8(&regaddr, 1, d);
	}

	// Read this register for changes to take effect
	regaddr = LISXDH_REFERENCE_REG;
	d = Read8(&regaddr, 1);

	AccelSensor::Range(range);
	TempSensor::Range(trange);

	return  AccelSensor::SamplingFrequency(f);
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
uint32_t AccelLisxdh::FilterFreq(uint32_t Freq)
{
	uint8_t regaddr = LISXDH_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LISXDH_CTRL_REG2_HPM_MASK | LISXDH_CTRL_REG2_FDS);

	if (Freq != 0)
	{
		d |= LISXDH_CTRL_REG2_FDS;
	}

	Write8(&regaddr, 1, d);

	regaddr = LISXDH_REFERENCE_REG;
	d = Read8(&regaddr, 1);

	return AccelSensor::FilterFreq(Freq);
}

bool AccelLisxdh::Enable()
{
	uint8_t regaddr = LISXDH_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1);

	Write8(&regaddr, 1, d | LISXDH_CTRL_REG1_XEN |
			LISXDH_CTRL_REG1_YEN | LISXDH_CTRL_REG1_ZEN);

	if (vbIntEn)
	{
		regaddr = LISXDH_CTRL_REG5_REG;
		d = Read8(&regaddr, 1) | LISXDH_CTRL_REG5_FIFO_EN;
		Write8(&regaddr, 1, d);

		regaddr = LISXDH_FIFO_CTRL_REG;
		d = Read8(&regaddr, 1) & ~(LISXDH_FIFO_CTRL_FM_MASK | LISXDH_FIFO_CTRL_FTH_MASK);

		Write8(&regaddr, 1, d);

		d |= LISXDH_FIFO_CTRL_FM_STREAM | 1;
		Write8(&regaddr, 1, d);

	}
	else
	{
		regaddr = LISXDH_FIFO_CTRL_REG;
		d = Read8(&regaddr, 1) & ~(LISXDH_FIFO_CTRL_FM_MASK | LISXDH_FIFO_CTRL_FTH_MASK);

		Write8(&regaddr, 1, d);
	}

	return true;
}

void AccelLisxdh::Disable()
{
	uint8_t regaddr = LISXDH_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LISXDH_CTRL_REG1_XEN |
			LISXDH_CTRL_REG1_YEN | LISXDH_CTRL_REG1_ZEN);

	Write8(&regaddr, 1, d);

	regaddr = LISXDH_FIFO_CTRL_REG;
	d = Read8(&regaddr, 1) & ~(LISXDH_FIFO_CTRL_FM_MASK | LISXDH_FIFO_CTRL_FTH_MASK);
	Write8(&regaddr, 1, d);

	regaddr = LISXDH_CTRL_REG5_REG;
	d = Read8(&regaddr, 1) & ~LISXDH_CTRL_REG5_FIFO_EN;
	Write8(&regaddr, 1, d);
}

void AccelLisxdh::Reset()
{
	uint8_t regaddr = LISXDH_CTRL_REG1_REG;
	uint8_t d = 0;

	Write8(&regaddr, 1, 0);

	regaddr = LISXDH_CTRL_REG2_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LISXDH_CTRL_REG3_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LISXDH_FIFO_CTRL_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LISXDH_CTRL_REG5_REG;
	Write8(&regaddr, 1, LISXDH_CTRL_REG5_BOOT);
	msDelay(1);
	Write8(&regaddr, 1, 0);

	regaddr = LISXDH_REFERENCE_REG;
	d = Read8(&regaddr,1);

	regaddr = LISXDH_INT1_SRC_REG;
	d = Read8(&regaddr, 1);

}

void AccelLisxdh::PowerOff()
{
	uint8_t regaddr = LISXDH_CTRL_REG1_REG;

	Write8(&regaddr, 1, 0);
}

void AccelLisxdh::IntHandler()
{
	uint8_t regaddr = LISXDH_STATUS_REG;
	uint8_t status = Read8(&regaddr, 1);

	regaddr = LISXDH_FIFO_SRC_REG;
	uint8_t fstatus = Read8(&regaddr, 1);

	regaddr = LISXDH_INT1_SRC_REG;
	uint8_t isrc1 = Read8(&regaddr, 1);

	regaddr = LISXDH_INT2_SRC_REG;
	uint8_t isrc2 = Read8(&regaddr, 1);

	if ((fstatus & LISXDH_FIFO_SRC_REG_WTM) || isrc1 || isrc2)
	{
		if (UpdateData() == true)
		{
			if (vIntHandler)
			{
				AccelSensorData_t data;

				AccelSensor::Read(data);

				vIntHandler(&data);
			}
		}
	}

	if (fstatus & LISXDH_FIFO_SRC_REG_OVRN_FIFO)
	{
		// Overrun, clear fifo
		regaddr = LISXDH_FIFO_CTRL_REG;
		uint8_t d = Read8(&regaddr, 1) & ~LISXDH_FIFO_CTRL_FM_MASK;
		Write8(&regaddr, 1, d);
		d |= LISXDH_FIFO_CTRL_FM_FIFO;
		Write8(&regaddr, 1, d);
	}
}

bool AccelLisxdh::UpdateData()
{
	uint8_t regaddr = LISXDH_STATUS_REG;
	uint8_t d = Read8(&regaddr, 1);
	uint8_t fstatus;
	uint32_t ts = 0;

	if (vpTimer)
	{
		ts = vpTimer->uSecond();
	}

	regaddr = LISXDH_FIFO_SRC_REG;
	fstatus = Read8(&regaddr, 1);

	bool avail = false;

	if ((fstatus & LISXDH_FIFO_SRC_REG_FSS_MASK) & vbIntEn)
	{
		avail = true;
	}
	else if (d & 0xf)
	{
		avail = true;
	}

	if (avail == true)
	{
		// New data avail
		if (vpTimer)
		{
			AccelSensor::vData.Timestamp = ts;
		}
		for (int i = 0; i < (fstatus & 0x1f); i++)
		{
			regaddr = LISXDH_OUT_X_L_REG | 0x40;

			uint16_t dd[3];

			Device::Read(&regaddr, 1, (uint8_t*)dd, 6);
			AccelSensor::vData.X = dd[0];
			AccelSensor::vData.Y = dd[1];
			AccelSensor::vData.Z = dd[2];
		}
	}

	regaddr = LISXDH_STATUS_REG_AUX_REG;
	d = Read8(&regaddr, 1);
	if (d & LISXDH_STATUS_REG_TDA_AUX_3DA)
	{
		if (vpTimer)
		{
			TempSensor::vData.Timestamp = ts;
		}

		regaddr = LISXDH_OUT_TEMP_ADC3_L_REG | 0x40;
		uint16_t t = 0;
		Device::Read(&regaddr, 1, (uint8_t*)&t, 2);

		int32_t r = TempSensor::Range();
		if (r <= 127)
		{
			TempSensor::vData.Temperature = t * 100 / 256 + 2500;
		}
		else
		{
			TempSensor::vData.Temperature = t * 100 / (64 * r) + 2500;
		}

		avail = true;
	}

	return avail;
}

