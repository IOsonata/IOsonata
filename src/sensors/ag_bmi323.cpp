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
		return false;

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	vData.Range = Range(BMI323_ADC_RANGE);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	AccelBmi323::Enable();

	if (CfgData.bInter)
	{
		uint8_t regaddr = BMI323_INT_MAP2_REG;
		uint16_t d = 0;//BMI160_INT_EN_0_INT_ANYMO_X_EN | BMI160_INT_EN_0_INT_ANYMO_Y_EN | BMI160_INT_EN_0_INT_ANYMO_Z_EN;

		Write16(&regaddr, 1, d);

		regaddr = BMI323_INT_CONFIG_REG;
		Write16(&regaddr, 1, BMI323_INT_CONFIG_LATCHED);

		regaddr = BMI323_INT_MAP2_REG;
		d = EndianCvt16(Read16(&regaddr, 1)) & ~BMI323_INT_MAP2_ACC_DRDY_MASK;
		d |= BMI323_INT_MAP2_ACC_DRDY_INT1;
		Write16(&regaddr, 1, d);///BMI160_INT_EN_1_INT_HIGHG_X_EN | BMI160_INT_EN_1_INT_HIGHG_Y_EN |
				//BMI160_INT_EN_1_INT_HIGHG_Z_EN |*/ BMI160_INT_EN_1_INT_DRDY_EN);

		regaddr = BMI323_IO_CTRL_REG;
		d = BMI323_IO_CTRL_INT1_PUSHPULL | BMI323_IO_CTRL_INT1_OUTPUT_EN;

		if (CfgData.IntPol == DEVINTR_POL_HIGH)
		{
			d |= BMI323_IO_CTRL_INT2_ACTIVE_HIGH;
		}

		Write16(&regaddr, 1, d);

		//regaddr = BMI160_INT_MAP_0;
		//Write16(&regaddr, 1, BMI160_INT_MAP_0_INT1_HIGHG | BMI160_INT_MAP_0_INT1_ANYMOTION);

		//regaddr = BMI160_INT_MAP_1;
		//Write16(&regaddr, 1, BMI160_INT_MAP_1_INT1_DRDY);
	}

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
/*	uint8_t regaddr = BMI160_ACC_CONF;
	uint8_t d = Read16(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_BWP_MASK;

	if ( t < 4)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_OSR2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_OSR4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG32;
		t = 5;
	}
	else if (t < 128)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG64;
		t = 6;
	}
	else
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG128;
		t = 7;
	}

	Write16(&regaddr, 1, d);*/

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
	d = EndianCvt16(Read16(&regaddr, 1)) | EndianCvt16(BMI323_FIFO_CONFIG_ACC_EN);

	Write16(&regaddr, 1, d);

	msDelay(1); // Require delay, do not remove

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

	msDelay(50);

	regaddr = BMI323_FIFO_CTRL_REG;
	Write16(&regaddr, 1, EndianCvt16(BMI323_FIFO_CTRL_FLUSH));

	msDelay(10);

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = EndianCvt16(Read16(&regaddr, 1)) | EndianCvt16(BMI323_FIFO_CONFIG_TIME_EN | BMI323_FIFO_CONFIG_TEMP_EN);

	Write16(&regaddr, 1, d);

	msDelay(2);

	//regaddr = BMI160_FIFO_DOWNS;
	//Write16(&regaddr, 1, (2<<BMI160_FIFO_DOWNS_GYR_FIFO_DOWNS_BITPOS) | BMI160_FIFO_DOWNS_GYR_FIFO_FILT_DATA |
	//					(2<<BMI160_FIFO_DOWNS_ACC_FIFO_DOWNS_BITPOS) | BMI160_FIFO_DOWNS_ACC_FIFO_FILT_DATA);

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
	//MagBmi160::Reset();

//	msDelay(5);

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


	//Device::Read(&regaddr, 1, (uint8_t*)&len, 2);

	printf("fifo len = %d\n", len);

	regaddr = BMI323_ACC_DATA_X_REG;
	uint16_t x = EndianCvt16(Read16(&regaddr, 1));
	printf("x = %d\n", x);

	if (len <= 0)
	{
		return false;
	}
/*
#if 0
	if (len > 1024)
	{
		uint8_t regaddr = BMI323_FIFO_CTRL_REG;
		Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

		msDelay(10);

		return false;
	}
#else
	len = min(len, BMI160_FIFO_MAX_SIZE);
#endif
	//printf("len %d\r\n", len);

	uint8_t buff[BMI160_FIFO_MAX_SIZE];
	uint64_t t = 0;

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	//regaddr = BMI160_DATA_MAG_X_LSB;
	//Device::Read(&regaddr, 1, (uint8_t*)MagSensor::vData.Val, 6);
	//printf("mx %d %d %d\r\n", MagSensor::vData.X, MagSensor::vData.Y, MagSensor::vData.Z);

	len += 4; // read time stamp

	regaddr = BMI160_FIFO_DATA;
	len = Device::Read(&regaddr, 1, buff, len);

	uint8_t *p = buff;

	while (len > 0)
	{
		BMI160_HEADER *hdr = (BMI160_HEADER *)p;

		len--;

		if (*p == 0x80 || len < 1)
		{
			break;
		}

		p++;

		if (hdr->Type == BMI160_FRAME_TYPE_DATA)
		{
			//printf("Data frame %x %d\r\n", *p, len);
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_MAG)
			{
				if (len >= 8)
				{
					dflag |= (1<<2);
					memcpy(MagSensor::vData.Val, p, 6);

					MagSensor::vData.Timestamp = t;
					MagSensor::vData.Val[0] >>= 3;
					MagSensor::vData.Val[1] >>= 3;
					MagSensor::vData.Val[2] >>= 1;
				}
				p += 8;
				len -= 8;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_GYRO)
			{
				if (len >= 6)
				{
					dflag |= (1<<1);
					memcpy(GyroBmi323::vData.Val, p, 6);
					GyroBmi323::vData.Timestamp = t;
					GyroBmi323::vData.Sensitivity = GyroSensor::Sensitivity();
				}
				p += 6;
				len -= 6;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_ACCEL)
			{
				if (len >= 6)
				{
					dflag |= (1<<0);
					memcpy(AccelBmi323::vData.Val, p, 6);
					AccelBmi323::vData.Timestamp = t;
					AccelBmi323::vData.Scale = AccelSensor::Scale();
				}
				p += 6;
				len -= 6;
			}
		}
		else if (hdr->Type == BMI160_FRAME_TYPE_CONTROL)
		{
			switch (hdr->Parm)
			{
				case BMI160_FRAME_CONTROL_PARM_SKIP:
					AccelBmi323::vDropCnt += *p;
					GyroBmi323::vDropCnt = AccelBmi323::vDropCnt;
					len--;
					p++;
					break;
				case BMI160_FRAME_CONTROL_PARM_TIME:
					if (len >= 3)
					{
						dflag |= (1<<3);
						uint64_t t = 0;

						memcpy(&t, p, 3);
						t &= 0xFFFFFF;
						t *= BMI160_TIME_RESOLUTION_USEC;

						if (vpTimer == nullptr)
						{
							if (dflag & 1)
							{
								AccelBmi323::vData.Timestamp = t;
							}
							if (dflag & 2)
							{
								GyroBmi323::vData.Timestamp = t;
							}
							if (dflag & 4)
							{
								//MagBmi160::vData.Timestamp = t;
							}
						}
					}
					len -= 3;
					p += 3;
					break;
				case BMI160_FRAME_CONTROL_PARM_INPUT:
					//Read16(&regaddr, 1);
					//printf("Input\r\n");
					p++;
					len--;
					break;
//				default:
					//printf("Control??\r\n");
			}
		}
		else
		{
			//printf("Unknown\r\n");
		}
	}
*/
	if (dflag != 0 && vEvtHandler)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}

	return res;
}

void AgBmi323::IntHandler()
{
	uint8_t regaddr = BMI323_INT1_STATUS_REG;
	uint16_t d;

	// Read all status
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	if (d & (BMI323_INT_STATUS_IBI_GYR_DRDY | BMI323_INT_STATUS_IBI_ACC_DRDY |
			 BMI323_INT_STATUS_IBI_FWM | BMI323_INT_STATUS_IBI_FFULL))
	{
		UpdateData();
	}
}
