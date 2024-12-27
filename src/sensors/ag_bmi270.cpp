/**-------------------------------------------------------------------------
@file	ag_bmi270.cpp

@brief	Bosch BMI270 accel gyro implementation

This file implements only accel & gyro part of the BMI270. IMU features are
implemented in imu implementation file.


@author	Hoang Nguyen Hoan
@date	Dec. 26, 2024

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

#include "istddef.h"
#include "idelay.h"
#include "sensors/ag_Bmi270.h"

bool AccelBmi270::Init(const AccelSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	vData.Range = Range(BMI270_ADC_RANGE);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	if (CfgData.bInter)
	{
		uint8_t regaddr = BMI270_INT_MAP_DATA_REG;
		uint8_t d = BMI270_INT_MAP_DATA_FFULL_INT1 |
					BMI270_INT_MAP_DATA_FWM_INT1 |
					BMI270_INT_MAP_DATA_DRDY_INT1 |
					BMI270_INT_MAP_ERR_INT1;

		Write8(&regaddr, 1, d);

		regaddr = BMI270_INT_LATCH_REG;
		Write8(&regaddr, 1, BMI270_INT_LATCH_PERM);

		if (vpTimer == nullptr)
		{
//			d |= BMI270_INT_MAP2_ERR_STATUS_INT1 | Bmi270_INT_MAP2_FIFO_WATERMARK_INT1 | Bmi270_INT_MAP2_FIFO_FULL_INT1;
		}
//		Write16(&regaddr, 1, d);

		regaddr = BMI270_INT1_IO_CTRL_REG;
		d = BMI270_INT1_IO_CTRL_OD_PUSH_PULL | BMI270_INT1_IO_CTRL_OUTPUT_EN;

		if (CfgData.IntPol == DEVINTR_POL_HIGH)
		{
			d |= BMI270_INT1_IO_CTRL_LVL_ACT_HIGH;
		}

		Write8(&regaddr, 1, d);

	}

	AccelBmi270::Enable();

	return true;
}

uint8_t AccelBmi270::Scale(uint8_t Value)
{
	uint8_t regaddr = BMI270_ACC_RANGE_REG;
	uint8_t d = 0;//Read16(&regaddr, 1) & ~BMI270_ACC_RANGE_MASK;

	if (Value < 3)
	{
		d |= BMI270_ACC_RANGE_2G;
		Value = 2;
	}
	else if (Value < 6)
	{
		d |= BMI270_ACC_RANGE_4G;
		Value = 4;
	}
	else if (Value < 12)
	{
		d |= BMI270_ACC_RANGE_8G;
		Value = 8;
	}
	else
	{
		d |= BMI270_ACC_RANGE_16G;
		Value = 16;
	}

	Write8(&regaddr, 1, d);
	msDelay(1);	// Require delay, donot remove

	vData.Scale = Value;

	return AccelSensor::Scale(Value);
}

uint32_t AccelBmi270::FilterFreq(uint32_t Freq)
{
	uint8_t t = AccelSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI270_ACC_CONF_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(BMI270_ACC_CONF_ACC_BWP_MASK | BMI270_ACC_CONF_ACC_FILTER_PERF_HP);

	if ( t < 4)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_OSR2_AVG2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_NORM_AVG4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_CIC_AVG8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_RES_AVG16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_RES_AVG32;
		t = 5;
	}
	else if (t < 128)
	{
		d |= BMI270_ACC_CONF_ACC_BWP_RES_AVG64;
		t = 6;
	}
	else
	{
		d |= BMI270_ACC_CONF_ACC_BWP_RES_AVG128;
		t = 7;
	}


	Write8(&regaddr, 1, d);

	return AccelSensor::FilterFreq(AccelSensor::SamplingFrequency() >> t);
}

uint32_t AccelBmi270::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI270_ACC_CONF_REG;
	uint8_t accconf = Read8(&regaddr, 1) & ~BMI270_ACC_CONF_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 1000)
	{
		accconf |= 1;
		f = 781;
	}
	else if (Freq < 2500)
	{
		accconf |= 2;
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
				accconf &= ~BMI270_ACC_CONF_ODR_MASK;
				accconf |= i + 3;
				dif = x;
				f = t;
			}

			if (t > Freq)
			{
				break;
			}
		}
	}
	//printf("AccelBmi270::SamplingFrequency %d %d %x\n", Freq, f, accconf);
	Write8(&regaddr, 1, accconf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

bool AccelBmi270::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	if (vpTimer == nullptr)
	{
		regaddr = BMI270_FIFO_CONFIG1_REG;
		d = Read8(&regaddr, 1) | BMI270_FIFO_CONFIG1_FIFO_ACC_EN;
		Write16(&regaddr, 1, d);

		msDelay(1); // Require delay, do not remove

		regaddr = BMI270_CMD_REG;
		Write8(&regaddr, 1, BMI270_CMD_FIFO_FLUSH);

		FifoDataFlagSet(BMI270_FIFO_DATA_FLAG_ACC);
	}

	regaddr = BMI270_PWR_CTRL_REG;
	d = Read8(&regaddr, 1) & ~BMI270_PWR_CTRL_ACC_EN;
	d |= BMI270_PWR_CTRL_ACC_EN;
	printf("Enable : %x\n", d);
	Write8(&regaddr, 1, d);

	msDelay(20); // Require delay, do not remove

	regaddr = BMI270_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void AccelBmi270::Disable()
{
	uint8_t regaddr = BMI270_PWR_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~BMI270_PWR_CTRL_ACC_EN;

	Write8(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI270_FIFO_CONFIG1_REG;
	d = Read8(&regaddr, 1) & ~BMI270_FIFO_CONFIG1_FIFO_ACC_EN;
	Write8(&regaddr, 1, d);

//	regaddr = Bmi270_ERR_REG;
//	Read16(&regaddr, 1);
}

bool GyroBmi270::Init(const GyroSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	vData.Range = Range(BMI270_ADC_RANGE);

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	uint8_t regaddr = BMI270_ERR_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d != 0)
	{
		printf("GyroBmi270::Init error %x\n\r", d);
		return false;
	}

	if (vpTimer == nullptr)
	{
//		d |= Bmi270_INT_MAP2_ERR_STATUS_INT1 | Bmi270_INT_MAP2_FIFO_FULL_INT1 | Bmi270_INT_MAP2_FIFO_WATERMARK_INT1;
	}
//	Write16(&regaddr, 1, d);

	GyroBmi270::Enable();

	return true;
}

uint32_t GyroBmi270::FilterFreq(uint32_t Freq)
{
	uint8_t t = GyroSensor::SamplingFrequency() / Freq;
#if 0
	uint8_t regaddr = Bmi270_GYR_CONFIG_REG;
	uint8_t d = Read16(&regaddr, 1) & ~Bmi270_GYR_CONFIG_AVG_NUM_MASK;

	if (t < 4)
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_32;
		t = 5;
	}
	else
	{
		d |= Bmi270_GYR_CONFIG_AVG_NUM_64;
		t = 6;
	}

	Write16(&regaddr, 1, d);
#endif

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() >> t);
}

uint32_t GyroBmi270::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI270_GYR_CONF_REG;
	uint8_t conf = Read8(&regaddr, 1) & ~BMI270_GYR_CONF_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 1000)
	{
		conf |= 1;
		f = 781;
	}
	else if (Freq < 2500)
	{
		conf |= 2;
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
				conf &= ~BMI270_GYR_CONF_ODR_MASK;
				conf |= i + 3;
				dif = x;
				f = t;
			}

			if (t > Freq)
			{
				break;
			}
		}
	}
//printf("SamplingFrequency %d %d %x\r\n", Freq, f, conf);
	Write8(&regaddr, 1, conf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

uint32_t GyroBmi270::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = BMI270_GYR_RANGE_REG;
	uint32_t d = Read16(&regaddr, 1) & ~BMI270_GYR_RANGE_GYR_RANGE_MASK;
	uint32_t range = 0;

	if (Value < 250)
	{
		d |= BMI270_GYR_RANGE_GYR_RANGE_125;
		range = 125;
	}
	else if (Value < 500)
	{
		d |= BMI270_GYR_RANGE_GYR_RANGE_250;
		range = 250;
	}
	else if (Value < 1000)
	{
		d |= BMI270_GYR_RANGE_GYR_RANGE_500;
		range = 500;
	}
	else if (Value < 2000)
	{
		d |= BMI270_GYR_RANGE_GYR_RANGE_1000;
		range = 1000;
	}
	else
	{
		d |= BMI270_GYR_RANGE_GYR_RANGE_2000;
		range = 2000;
	}

	Write16(&regaddr, 1, d);

	vData.Sensitivity = range;

	return GyroSensor::Sensitivity(range);
}

bool GyroBmi270::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	if (vpTimer == nullptr)
	{
		regaddr = BMI270_FIFO_CONFIG1_REG;
		d = Read8(&regaddr, 1) | BMI270_FIFO_CONFIG1_FIFO_GYR_EN;

		Write8(&regaddr, 1, d);

		msDelay(1); // Require delay, do not remove

		regaddr = BMI270_CMD_REG;
		Write8(&regaddr, 1, BMI270_CMD_FIFO_FLUSH);

		FifoDataFlagSet(BMI270_FIFO_DATA_FLAG_GYR);
	}

	regaddr = BMI270_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
		printf("Bmi270_FIFO_CTRL_FLUSH err %x\r\n", d);
		return false;
	}

	regaddr = BMI270_PWR_CTRL_REG;
	d = Read8(&regaddr, 1) & ~BMI270_PWR_CTRL_GYR_EN;
	d |= BMI270_PWR_CTRL_GYR_EN;

	printf("Gyr Cfg : %x\r\n", d);

	Write8(&regaddr, 1, d);

	msDelay(20); // Require delay, do not remove

	regaddr = BMI270_ERR_REG;
	d = Read8(&regaddr, 1);

	if (d != 0)
	{
		printf("Gyr err %x\r\n", d);
		return false;
	}

	return true;
}

void GyroBmi270::Disable()
{
	uint8_t regaddr = BMI270_PWR_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~BMI270_PWR_CTRL_GYR_EN;
	Write8(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI270_ERR_REG;
	d = Read8(&regaddr, 1);

	regaddr = BMI270_FIFO_CONFIG1_REG;
	d = Read8(&regaddr, 1) & ~BMI270_FIFO_CONFIG1_FIFO_GYR_EN;

	Write8(&regaddr, 1, d);
}

bool TempBmi270::Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	TempSensor::Type(SENSOR_TYPE_TEMP);


	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool TempBmi270::Enable()
{
	bool retval = false;

	FifoDataFlagSet(BMI270_FIFO_DATA_FLAG_TEMP);

	return retval;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * @return	None
 */
void TempBmi270::Disable()
{
}

bool AgBmi270::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	Device::DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	// Read chip id
	regaddr = BMI270_CHIP_ID_REG;
	d = Read8(&regaddr, 1);

	if (d != BMI270_CHIP_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	if (vpTimer == nullptr)
	{
		regaddr = BMI270_CMD_REG;
		Write8(&regaddr, 1, BMI270_CMD_FIFO_FLUSH);

		msDelay(10);

		regaddr = BMI270_FIFO_CONFIG0_REG;
		d = Read8(&regaddr, 1) | BMI270_FIFO_CONFIG0_FIFO_TIME_EN;

		Write8(&regaddr, 1, d);

		vFifoDataFlag = BMI270_FIFO_DATA_FLAG_TIME;
		vFifoFrameSize = 2;
	}

	return true;
}

bool AgBmi270::Enable()
{
	AccelBmi270::Enable();
	GyroBmi270::Enable();

	return true;
}

void AgBmi270::Disable()
{
	AccelBmi270::Disable();
	GyroBmi270::Disable();
}

void AgBmi270::Reset()
{
	uint8_t regaddr = BMI270_CMD_REG;

	Write8(&regaddr, 1, BMI270_CMD_SOFTRESET);

	msDelay(1);

	regaddr = BMI270_CMD_REG;
	Write8(&regaddr, 1, BMI270_CMD_FIFO_FLUSH);

	msDelay(10);

	// Read err register to clear
	regaddr = BMI270_ERR_REG;
	Read8(&regaddr, 1);
}

bool AgBmi270::UpdateData()
{
	bool res = false;
	uint8_t regaddr;
	int len = 0;

	if (vpTimer == nullptr)
	{
		uint8_t fifo[256];

		regaddr = BMI270_FIFO_WATERMARK_REG;
		len = Read16(&regaddr, 1);

		// Re-adjust length to read full frame only
		len = (min(len, 128) / vFifoFrameSize) * vFifoFrameSize;

		if (len >= vFifoFrameSize)
		{
			regaddr = BMI270_FIFO_DATA_REG;
			len = Read(&regaddr, 1, fifo, len << 1) >> 1;

			int16_t *p = (int16_t*)fifo;

			while (len >= vFifoFrameSize)
			{
				if (vFifoDataFlag & BMI270_FIFO_DATA_FLAG_ACC)
				{
					if (p[0] != BMI270_ACC_DUMMY_X)
					{
						// Take valid data only
						memcpy(AccelSensor::vData.Val, p, 6);
						AccelSensor::vSampleCnt++;
					}
					p += 3;
				}
				if (vFifoDataFlag & BMI270_FIFO_DATA_FLAG_GYR)
				{
					if (p[0] != BMI270_GYR_DUMMY_X)
					{
						// Take valid data only
						memcpy(GyroSensor::vData.Val, p, 6);
						GyroSensor::vSampleCnt++;
					}
					p += 3;
				}
				if (vFifoDataFlag & BMI270_FIFO_DATA_FLAG_TEMP)
				{
					if (p[0] != BMI270_TEMP_DUMMY)
					{
						// Take valid data only
						TempSensor::vData.Temperature = p[0];
						TempSensor::vSampleCnt++;
					}
					p++;
				}
				if (vFifoDataFlag & BMI270_FIFO_DATA_FLAG_TIME)
				{
					if (vPrevTime > (uint16_t)p[0])
					{
						vRollover += 0x10000U;
					}
					vPrevTime = (uint16_t)p[0];
					uint64_t t = vRollover + ((uint64_t)p[0] & 0xFFFFULL);
					AccelSensor::vData.Timestamp = t;
					GyroSensor::vData.Timestamp = t;
					TempSensor::vData.Timestamp = t;
					p++;
				}
				else if (vpTimer)
				{
					uint64_t t = vpTimer->mSecond();
					AccelSensor::vData.Timestamp = t;
					GyroSensor::vData.Timestamp = t;
					TempSensor::vData.Timestamp = t;
				}

				len -= vFifoFrameSize;
			}
			res = true;
		}
	}
	else
	{
		// Non FIFO
		uint64_t t = vpTimer->uSecond();

		regaddr = BMI270_STATUS_REG;
		uint16_t d = Read16(&regaddr, 1);

		if (d & BMI270_STATUS_DRDY_ACC)
		{
			regaddr = BMI270_ACC_X_LSB_REG;
			Read(&regaddr, 1, (uint8_t*)AccelSensor::vData.Val, 6);
			AccelSensor::vData.Timestamp = t;

			res = true;
		}
		if (d & BMI270_STATUS_DRDY_GYR)
		{
			regaddr = BMI270_GYR_X_LSB_REG;
			Read(&regaddr, 1, (uint8_t*)GyroSensor::vData.Val, 6);
			GyroSensor::vData.Timestamp = t;
			res = true;
		}
		//if (d & Bmi270_STATUS_DRDY_TEMP)
		{
			regaddr = BMI270_TEMPERATURE_LSB_REG;
			Read(&regaddr, 1, (uint8_t*)&TempSensor::vData.Temperature, 2);
			TempSensor::vData.Timestamp = t;
			res = true;
		}
	}

	return res;
}

void AgBmi270::IntHandler()
{
	uint8_t regaddr = BMI270_INT_STATUS1_REG;
	uint8_t d;

	// Read all status
	//Read(&regaddr, 1, (uint8_t*)&d, 2);
	d = Read8(&regaddr, 1);

	if (d & (BMI270_INT_STATUS1_AUX_DRDY | BMI270_INT_STATUS1_GYR_DRDY |
			 BMI270_INT_STATUS1_ACC_DRDY | BMI270_INT_STATUS1_FIFO_FULL |
			 BMI270_INT_STATUS1_FIFO_WATERMARK))
	{
		UpdateData();

		if (vEvtHandler)
		{
			vEvtHandler(this, DEV_EVT_DATA_RDY);
		}
	}
}

int AgBmi270::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t b[BuffLen + 1];
	int n = Device::Read(pCmdAddr, CmdAddrLen, b, BuffLen + 1);

	memcpy(pBuff, &b[1], BuffLen);

	return n;
}

static size_t FifoFrameSize(uint8_t Flag)
{
	size_t retval = 0;

	if (Flag & BMI270_FIFO_DATA_FLAG_ACC)
	{
		retval += 3;
	}

	if (Flag & BMI270_FIFO_DATA_FLAG_GYR)
	{
		retval += 3;
	}

	if (Flag & BMI270_FIFO_DATA_FLAG_TEMP)
	{
		retval++;
	}

	if (Flag & BMI270_FIFO_DATA_FLAG_TIME)
	{
		retval ++;
	}

	return retval;
}

void AgBmi270::FifoDataFlagSet(uint8_t Flag)
{
	vFifoDataFlag = (vFifoDataFlag & ~Flag) | Flag;

	vFifoFrameSize = FifoFrameSize(vFifoDataFlag);
}

void AgBmi270::FifoDataFlagClr(uint8_t Flag)
{
	vFifoDataFlag = (vFifoDataFlag & ~Flag);

	vFifoFrameSize = FifoFrameSize(vFifoDataFlag);
}

