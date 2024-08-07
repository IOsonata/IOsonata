/**-------------------------------------------------------------------------
@file	ag_bmi323.cpp

@brief	Bosch BMI323 accel gyro implementation


@author	Hoang Nguyen Hoan
@date	July 20, 2024

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
#include <math.h>

#include "idelay.h"
#include "convutil.h"
#include "sensors/ag_bmi323.h"

static const uint32_t s_BMI323Odr[] = {
	781, 1562, 3125,
};

bool AccelBmi323::Init(const AccelSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	vData.Range = Range(BMI323_ADC_RANGE);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	if (CfgData.bInter)
	{
		uint8_t regaddr = BMI323_INT_MAP2_REG;
		uint16_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_INT_MAP2_ACC_DRDY_MASK;

		d |= BMI323_INT_MAP2_ACC_DRDY_INT1;
		Write16(&regaddr, 1, d);

		regaddr = BMI323_FIFO_WATERMARK_REG;
		d = 0x1FF;
		Write16(&regaddr, 1, d);

		regaddr = BMI323_INT_CONFIG_REG;
		Write16(&regaddr, 1, BMI323_INT_CONFIG_LATCHED);

		regaddr = BMI323_INT_MAP2_REG;
		d = EndianCvt16(Read16(&regaddr, 1)) & ~(BMI323_INT_MAP2_ACC_DRDY_MASK |
				BMI323_INT_MAP2_FIFO_FULL_MASK | BMI323_INT_MAP2_FIFO_WATERMARK_MASK |
				BMI323_INT_MAP2_ERR_STATUS_MASK);
		d |= BMI323_INT_MAP2_ACC_DRDY_INT1 | BMI323_INT_MAP2_FIFO_FULL_INT1 |
			 BMI323_INT_MAP2_ERR_STATUS_INT1 | BMI323_INT_MAP2_FIFO_WATERMARK_INT1;
		Write16(&regaddr, 1, d);

		regaddr = BMI323_IO_CTRL_REG;
		d = BMI323_IO_CTRL_INT1_PUSHPULL | BMI323_IO_CTRL_INT1_OUTPUT_EN;

		if (CfgData.IntPol == DEVINTR_POL_HIGH)
		{
			d |= BMI323_IO_CTRL_INT2_ACTIVE_HIGH;
		}

		Write16(&regaddr, 1, d);

	}

	AccelBmi323::Enable();

	return true;
}

uint8_t AccelBmi323::Scale(uint8_t Value)
{
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_ACC_CONFIG_RANGE_MASK;

	if (Value < 3)
	{
		d |= BMI323_ACC_CONFIG_RANGE_2G;
		Value = 2;
	}
	else if (Value < 6)
	{
		d |= BMI323_ACC_CONFIG_RANGE_4G;
		Value = 4;
	}
	else if (Value < 12)
	{
		d |= BMI323_ACC_CONFIG_RANGE_8G;
		Value = 8;
	}
	else
	{
		d |= BMI323_ACC_CONFIG_RANGE_16G;
		Value = 16;
	}

	Write16(&regaddr, 1, BMI323_ACC_CONFIG_RANGE_2G);
	msDelay(1);	// Require delay, donot remove

	vData.Scale = Value;

	return AccelSensor::Scale(Value);
}

uint32_t AccelBmi323::FilterFreq(uint32_t Freq)
{
	uint8_t t = AccelSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_ACC_CONFIG_AVG_NUM_MASK;

	if ( t < 4)
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_32;
		t = 5;
	}
	else
	{
		d |= BMI323_ACC_CONFIG_AVG_NUM_64;
		t = 6;
	}

	Write16(&regaddr, 1, d);

	return AccelSensor::FilterFreq(AccelSensor::SamplingFrequency() >> t);
}

uint32_t AccelBmi323::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint32_t accconf = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_ACC_CONFIG_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 1000)
	{
		accconf = 0;
		f = 781;
	}
	else if (Freq < 2500)
	{
		accconf = 1;
		f = 1562;
	}
	else
	{
		for (int i = 0; i < 12; i++)
		{
			uint32_t t = 3125 << i;
			uint32_t x = labs(Freq - t);

			if (x < dif)
			{
				accconf &= ~BMI323_ACC_CONFIG_ODR_MASK;
				accconf |= i + 2;
				dif = x;
				f = t;
			}

			if (t > Freq)
			{
				break;
			}
		}
	}
	printf("acconfig = %d\n", accconf);
	Write16(&regaddr, 1, accconf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

bool AccelBmi323::Enable()
{
	uint8_t regaddr;
	uint16_t d;

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) | BMI323_FIFO_CONFIG_ACC_EN;

	printf("fifo cfg %x\n", d);

	Write16(&regaddr, 1, d);

	msDelay(1); // Require delay, do not remove

	regaddr = BMI323_FIFO_CTRL_REG;
	Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

	regaddr = BMI323_ACC_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_ACC_CONFIG_MODE_MASK;
	d |= BMI323_ACC_CONFIG_MODE_CONT_EN;
	printf("Enable : %x\n", d);
	Write16(&regaddr, 1, d);

	msDelay(20); // Require delay, do not remove

	regaddr = BMI323_ERR_REG;
	d = EndianCvt16(Read16(&regaddr, 1));

	if (d != 0)
	{
		return false;
	}

	FifoDataFlagSet(BMI323_FIFO_DATA_FLAG_ACC);

	return true;
}

void AccelBmi323::Disable()
{
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_ACC_CONFIG_MODE_MASK;

	Write16(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_FIFO_CONFIG_ACC_EN;
	Write16(&regaddr, 1, d);

//	regaddr = BMI323_ERR_REG;
//	Read16(&regaddr, 1);
}

bool GyroBmi323::Init(const GyroSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	vData.Range = Range(0x7FFF);

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	GyroBmi323::Enable();

	return true;
}

uint32_t GyroBmi323::FilterFreq(uint32_t Freq)
{
	uint8_t t = GyroSensor::SamplingFrequency() / Freq;
/*	uint8_t regaddr = BMI160_GYR_CONF;
	uint8_t d = Read16(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_BWP_MASK;

	if ( t < 4)
	{
		d |= BMI160_GYR_CONF_GYR_BWP_OSR2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI160_GYR_CONF_GYR_BWP_OSR4;
		t = 2;
	}
	else
	{
		d |= BMI160_GYR_CONF_GYR_BWP_NORMAL;
		t = 3;
	}

	Write16(&regaddr, 1, d);*/

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() >> t);
}

uint32_t GyroBmi323::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint32_t accconf = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_GYR_CONFIG_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 1000)
	{
		accconf = 0;
		f = 781;
	}
	else if (Freq < 2500)
	{
		accconf = 1;
		f = 1562;
	}
	else
	{
		for (int i = 0; i < 12; i++)
		{
			uint32_t t = 3125 << i;
			uint32_t x = labs(Freq - t);

			if (x < dif)
			{
				accconf &= ~BMI323_GYR_CONFIG_ODR_MASK;
				accconf |= i = 2;
				dif = x;
				f = t;
			}

			if (t > Freq)
			{
				break;
			}
		}
	}

	Write16(&regaddr, 1, accconf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

uint32_t GyroBmi323::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint32_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_GYR_CONFIG_RANGE_MASK;
	uint32_t range = 0;

	if (Value < 250)
	{
		d |= BMI323_GYR_CONFIG_RANGE_125;
		range = 125;
	}
	else if (Value < 500)
	{
		d |= BMI323_GYR_CONFIG_RANGE_250;
		range = 250;
	}
	else if (Value < 1000)
	{
		d |= BMI323_GYR_CONFIG_RANGE_500;
		range = 500;
	}
	else if (Value < 2000)
	{
		d |= BMI323_GYR_CONFIG_RANGE_1000;
		range = 1000;
	}
	else
	{
		d |= BMI323_GYR_CONFIG_RANGE_2000;
		range = 2000;
	}

	Write16(&regaddr, 1, d);

	vData.Sensitivity = range;

	return GyroSensor::Sensitivity(range);
}

bool GyroBmi323::Enable()
{
	uint8_t regaddr;
	uint16_t d;

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) | BMI323_FIFO_CONFIG_GYR_EN;

	Write16(&regaddr, 1, d);

	msDelay(5); // Require delay, do not remove

	regaddr = BMI323_GYR_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) | BMI323_GYR_CONFIG_MODE_CONT_EN;
	Write16(&regaddr, 1, d);

	msDelay(40); // Require delay, do not remove

	regaddr = BMI323_ERR_REG;
	d = EndianCvt16(Read16(&regaddr, 1));

	if (d != 0)
	{
		return false;
	}

	return true;
}

void GyroBmi323::Disable()
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint16_t d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_GYR_CONFIG_MODE_MASK;
	Write16(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI323_ERR_REG;
	d = EndianCvt16(Read16(&regaddr, 1));

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_FIFO_CONFIG_GYR_EN;

	Write16(&regaddr, 1, d);
}

bool AgBmi323::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint16_t d;

	Interface(pIntrf);
	Device::DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	// Read chip id
	regaddr = BMI323_CHIP_ID_REG;
	d = EndianCvt16(Read16(&regaddr, 1));

	if (d != BMI323_CHIP_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	regaddr = BMI323_FIFO_CTRL_REG;
	Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

	msDelay(10);

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) | BMI323_FIFO_CONFIG_TIME_EN | BMI323_FIFO_CONFIG_TEMP_EN;

	Write16(&regaddr, 1, d);

	vFifoDataFlag = BMI323_FIFO_DATA_FLAG_TEMP | BMI323_FIFO_DATA_FLAG_TIME;
	vFifoFrameSize = 2;

	return true;
}

bool AgBmi323::Enable()
{
	AccelBmi323::Enable();
	GyroBmi323::Enable();

	return true;
}

void AgBmi323::Disable()
{
	AccelBmi323::Disable();
	GyroBmi323::Disable();
}

void AgBmi323::Reset()
{
	uint8_t regaddr = BMI323_CMD_REG;

	Write16(&regaddr, 1, BMI323_CMD_SOFTRESET);

	msDelay(1);

	regaddr = BMI323_FIFO_CTRL_REG;
	Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

	msDelay(10);

	// Read err register to clear
	regaddr = BMI323_ERR_REG;
	Read16(&regaddr, 1);
}

bool AgBmi323::UpdateData()
{
	bool res = false;
	uint8_t dflag = 0;
	uint8_t regaddr = BMI323_FIFO_FILL_LEVEL_REG;
	int len = EndianCvt16(Read16(&regaddr, 1));
	uint8_t fifo[256];

	//Device::Read(&regaddr, 1, (uint8_t*)&len, 2);

	//printf("fifo len = %d\n", len);

	if (len > vFifoFrameSize)
	{
		regaddr = BMI323_FIFO_DATA_REG;
		Read(&regaddr, 1, fifo, vFifoFrameSize << 1);

		int16_t *p = (int16_t*)fifo;

		if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_ACC)
		{
			AccelSensor::vData.X = EndianCvt16(p[0]);
			AccelSensor::vData.Y = EndianCvt16(p[1]);
			AccelSensor::vData.Z = EndianCvt16(p[2]);
			//memcpy(AccelSensor::vData.Val, p, 6);
			p += 3;
		}
		if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_GYR)
		{
			GyroSensor::vData.X = EndianCvt16(p[0]);
			GyroSensor::vData.Y = EndianCvt16(p[1]);
			GyroSensor::vData.Z = EndianCvt16(p[2]);
			p += 3;
		}
		if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_TEMP)
		{
			int16_t temp = EndianCvt16(p[0]);
			///memcpy(GyroSensor::vData.Val, p, 6);
			//AccelSensor::vData.Timestamp = *p;
			p++;
		}
		if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_TIME)
		{
			AccelSensor::vData.Timestamp = EndianCvt16(p[0]);
			GyroSensor::vData.Timestamp = EndianCvt16(p[0]);
		}
		else if (vpTimer)
		{
			uint64_t t = vpTimer->mSecond();
			AccelSensor::vData.Timestamp = t;
			GyroSensor::vData.Timestamp = t;
		}
	}

	return res;
}

void AgBmi323::IntHandler()
{
	uint8_t regaddr = BMI323_INT1_STATUS_REG;
	uint16_t d;

	// Read all status
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	if (d & (BMI323_INT1_STATUS_ACC_DRDY | BMI323_INT1_STATUS_GYR_DRDY |
			BMI323_INT1_STATUS_FWM | BMI323_INT1_STATUS_FFULL))
	{
		UpdateData();

		if (vEvtHandler)
		{
			vEvtHandler(this, DEV_EVT_DATA_RDY);
		}
	}
}

static inline size_t FifoFrameSize(uint8_t Flag)
{
	size_t retval = 0;

	if (Flag & BMI323_FIFO_DATA_FLAG_ACC)
	{
		retval += 3;
	}

	if (Flag & BMI323_FIFO_DATA_FLAG_GYR)
	{
		retval += 3;
	}

	if (Flag & BMI323_FIFO_DATA_FLAG_TEMP)
	{
		retval++;
	}

	if (Flag & BMI323_FIFO_DATA_FLAG_TIME)
	{
		retval += 1;
	}

	return retval;
}

void AgBmi323::FifoDataFlagSet(uint8_t Flag)
{
	vFifoDataFlag = (vFifoDataFlag & ~Flag) | Flag;

	vFifoFrameSize = FifoFrameSize(vFifoDataFlag);
}

void AgBmi323::FifoDataFlagClr(uint8_t Flag)
{
	vFifoDataFlag = (vFifoDataFlag & ~Flag);

	vFifoFrameSize = FifoFrameSize(vFifoDataFlag);
}

