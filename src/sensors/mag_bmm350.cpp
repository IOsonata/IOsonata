/**-------------------------------------------------------------------------
@file	mag_bmm350.cpp

@brief	Bosch BMM350 magnetometer implementation


@author	Hoang Nguyen Hoan
@date	July 24, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#include "sensors/mag_bmm350.h"

/**
 * @brief	Initialize mag sensor.
 *
 * @param 	Cfg		: Configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool MagBmm350::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	MagSensor::Type(SENSOR_TYPE_MAG);

	Interface(pIntrf);

	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		DeviceAddress(Cfg.DevAddr);
	}
	else
	{
		DeviceAddress(BMM350_I2C_7BITS_DEVADDR);
	}
	//vData.Range = Range(((1<<15) - 1) >> 1);
	//vData.Scale = 2500;

	uint8_t regaddr = BMM350_CMD_REG;
	uint8_t d = BMM350_CMD_SOFTRESET;

	// use this way to allow hook up on the secondary interface of a combo device
	MagBmm350::Write(&regaddr, 1, &d, 1);

	msDelay(10);

	regaddr = BMM350_CHIP_ID_REG;
	MagBmm350::Read(&regaddr, 1, &d, 1);

	if (d != BMM350_CHIP_ID)
	{
		if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
		{
			DeviceAddress(BMM350_I2C_7BITS_DEVADDR2);
			MagBmm350::Read(&regaddr, 1, &d, 1);

			if (d != BMM350_CHIP_ID)
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	// There is no setting to change precision
	// fix it to lowest value.
	// X, Y : 13 bits, Z : 15 bits, RHALL : 14 bits
	vPrecision = MAGSENSOR_PRECISION_LOW;
	vSensitivity[0] = vSensitivity[1] = vSensitivity[2] = BMM350_FLUX_DENSITY / BMM350_ADC_RANGE;

	vRange = BMM350_ADC_RANGE;
	vData.Sensitivity[0] = vSensitivity[0];
	vData.Sensitivity[1] = vSensitivity[1];
	vData.Sensitivity[2] = vSensitivity[2];

	ClearCalibration();

	SamplingFrequency(Cfg.Freq);

	Enable();

	return true;
}

uint32_t MagBmm350::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMM350_PMU_CMD_AGGR_SET_REG;
	uint8_t d;
	uint32_t f = 0;

	MagBmm350::Read(&regaddr, 1, &d, 1);

	d &= ~BMM350_PMU_CMD_AGGR_SET_ODR_MASK;

	if (Freq < 2000)
	{
		f = 1562;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_1_5625HZ;
	}
	else if (Freq < 6000)
	{
		f = 3125;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_3_125HZ;
	}
	else if (Freq < 10000)
	{
		f = 6250;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_6_25HZ;
	}
	else if (Freq < 17000)
	{
		f = 12500;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_12_5HZ;
	}
	else if (Freq < 35000)
	{
		f = 25000;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_25HZ;
	}
	else if (Freq < 80000)
	{
		f = 50000;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_50HZ;
	}
	else if (Freq < 150000)
	{
		f = 100000;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_100HZ;
	}
	else if (Freq < 300000)
	{
		f = 200000;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_200HZ;
	}
	else
	{
		f = 400000;
		d |= BMM350_PMU_CMD_AGGR_SET_ODR_400HZ;
	}

	MagBmm350::Write(&regaddr, 1, &d, 1);

	return Sensor::SamplingFrequency(f);
}

bool MagBmm350::Enable()
{
	uint8_t regaddr = BMM350_PMU_CMD_REG;
	uint8_t d;

	MagBmm350::Read(&regaddr, 1, &d, 1);
	d &= ~BMM350_PMU_CMD_MASK;
	d |= BMM350_PMU_CMD_NM;
	MagBmm350::Write(&regaddr, 1, &d, 1);

	msDelay(10);

	regaddr = BMM350_PMU_CMD_AXIS_EN_REG;
	MagBmm350::Read(&regaddr, 1, &d, 1);

	d |= (BMM350_PMU_CMD_AXIS_EN_X | BMM350_PMU_CMD_AXIS_EN_Y | BMM350_PMU_CMD_AXIS_EN_Z);
	MagBmm350::Write(&regaddr, 1, &d, 1);

	return true;
}

void MagBmm350::Disable()
{
	uint8_t regaddr = BMM350_PMU_CMD_AXIS_EN_REG;
	uint8_t d = 0;

//	MagBmm350::Read(&regaddr, 1, &d, 1);

//	d |= (BMM150_CTRL3_CHAN_X_DIS | BMM150_CTRL3_CHAN_Y_DIS | BMM150_CTRL3_CHAN_Y_DIS);
	MagBmm350::Write(&regaddr, 1, &d, 1);

	regaddr = BMM350_PMU_CMD_REG;
	MagBmm350::Read(&regaddr, 1, &d, 1);
	d &= ~BMM350_PMU_CMD_MASK;
	d |= BMM350_PMU_CMD_SUSP;
	MagBmm350::Write(&regaddr, 1, &d, 1);
}

void MagBmm350::Reset()
{
	uint8_t regaddr = BMM350_CMD_REG;
	uint8_t d = BMM350_CMD_SOFTRESET;

	MagBmm350::Write(&regaddr, 1, &d, 1);

	msDelay(1);
}

bool MagBmm350::UpdateData()
{
	uint8_t regaddr = BMM350_INT_STATUS_REG;
	uint8_t d = 0;
	bool res = false;

	MagBmm350::Read(&regaddr, 1, &d, 1);
	if (d)
	{
		int8_t buff[20];
		regaddr = BMM350_MAG_X_XLSB_REG;

		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		MagBmm350::Read(&regaddr, 1, (uint8_t*)buff, 15);

		vData.X = buff[0];
		vData.Y = buff[1];
		vData.Z = buff[2];

		res = true;
	}

	return res;
}
