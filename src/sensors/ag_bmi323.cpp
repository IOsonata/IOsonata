/**-------------------------------------------------------------------------
@file	ag_bmi323.cpp

@brief	Bosch BMI323 accel gyro implementation

This file implements only accel & gyro part of the BMI323. IMU features are
implemented in imu implementation file.

NOTE: BMI323 read always send a dummy byte first.  Se datasheet for detail.

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

#include "istddef.h"
#include "idelay.h"
#include "convutil.h"
#include "sensors/ag_bmi323.h"

//#include "coredev/uart.h"
//extern UART g_Uart;

//static const uint32_t s_BMI323Odr[] = {
//	781, 1562, 3125,
//};

bool AccelBmi323::Init(const AccelSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	Range(BMI323_ADC_RANGE);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	if (CfgData.Inter)
	{
		uint8_t regaddr = BMI323_INT_MAP2_REG;
		uint16_t d = Read16(&regaddr, 1) & ~BMI323_INT_MAP2_ACC_DRDY_MASK;

		d |= BMI323_INT_MAP2_ACC_DRDY_INT1;
		Write16(&regaddr, 1, d);

		regaddr = BMI323_FIFO_WATERMARK_REG;
		d = Read16(&regaddr, 1);
		d = 800;
		Write16(&regaddr, 1, d);
		d = Read16(&regaddr, 1);

		regaddr = BMI323_INT_MAP2_REG;
		d = (Read16(&regaddr, 1));
		d &= ~BMI323_INT_MAP2_ACC_DRDY_MASK;
		d |= BMI323_INT_MAP2_ACC_DRDY_INT1;//

		if (vpTimer == nullptr)
		{
			d |= BMI323_INT_MAP2_ERR_STATUS_INT1 | BMI323_INT_MAP2_FIFO_WATERMARK_INT1 | BMI323_INT_MAP2_FIFO_FULL_INT1;
		}
		Write16(&regaddr, 1, d);

		regaddr = BMI323_IO_CTRL_REG;
		d = BMI323_IO_CTRL_INT1_PUSHPULL | BMI323_IO_CTRL_INT1_OUTPUT_EN;

		if (CfgData.IntPol == DEVINTR_POL_HIGH)
		{
			d |= BMI323_IO_CTRL_INT1_ACTIVE_HIGH;
			regaddr = BMI323_INT_CONFIG_REG;
			Write16(&regaddr, 1, BMI323_INT_CONFIG_LATCHED);
		}

		regaddr = BMI323_IO_CTRL_REG;
		Write16(&regaddr, 1, d);

	}

	AccelBmi323::Enable();

	return true;
}

uint8_t AccelBmi323::Scale(uint8_t Value)
{
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = Read16(&regaddr, 1) & ~BMI323_ACC_CONFIG_RANGE_MASK;

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

	Write16(&regaddr, 1, d);//BMI323_ACC_CONFIG_RANGE_2G);
	msDelay(1);	// Require delay, donot remove

	return AccelSensor::Scale(Value);
}

uint32_t AccelBmi323::FilterFreq(uint32_t Freq)
{
	uint8_t t = AccelSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = Read16(&regaddr, 1) & ~BMI323_ACC_CONFIG_AVG_NUM_MASK;

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
	uint32_t accconf = Read16(&regaddr, 1) & ~BMI323_ACC_CONFIG_ODR_MASK;
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
				accconf &= ~BMI323_ACC_CONFIG_ODR_MASK;
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
	//printf("AccelBmi323::SamplingFrequency %d %d %x\n", Freq, f, accconf);
	Write16(&regaddr, 1, accconf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

bool AccelBmi323::Enable()
{
	uint8_t regaddr;
	uint16_t d;

	if (vpTimer == nullptr)
	{
		regaddr = BMI323_FIFO_CONFIG_REG;
		d = Read16(&regaddr, 1) | BMI323_FIFO_CONFIG_ACC_EN;
		Write16(&regaddr, 1, d);

		msDelay(1); // Require delay, do not remove

		regaddr = BMI323_FIFO_CTRL_REG;
		Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

		FifoDataFlagSet(BMI323_FIFO_DATA_FLAG_ACC);
	}

	regaddr = BMI323_ACC_CONFIG_REG;
	d = Read16(&regaddr, 1) & ~BMI323_ACC_CONFIG_MODE_MASK;
	d |= BMI323_ACC_CONFIG_MODE_CONT_EN;

	Write16(&regaddr, 1, d);

	msDelay(20); // Require delay, do not remove

	regaddr = BMI323_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void AccelBmi323::Disable()
{
	uint8_t regaddr = BMI323_ACC_CONFIG_REG;
	uint16_t d = Read16(&regaddr, 1) & ~BMI323_ACC_CONFIG_MODE_MASK;

	Write16(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = Read16(&regaddr, 1) & ~BMI323_FIFO_CONFIG_ACC_EN;
	Write16(&regaddr, 1, d);

//	regaddr = BMI323_ERR_REG;
//	Read16(&regaddr, 1);
}

bool GyroBmi323::Init(const GyroSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	vData.Range = Range(BMI323_ADC_RANGE);

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	uint8_t regaddr = BMI323_ERR_REG;
	uint16_t d = Read16(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	regaddr = BMI323_INT_MAP2_REG;
	d = (Read16(&regaddr, 1));
	d &= ~BMI323_INT_MAP2_GYR_DRDY_MASK;
	d |= BMI323_INT_MAP2_GYR_DRDY_INT1;//

	if (vpTimer == nullptr)
	{
		d |= BMI323_INT_MAP2_ERR_STATUS_INT1 | BMI323_INT_MAP2_FIFO_FULL_INT1 | BMI323_INT_MAP2_FIFO_WATERMARK_INT1;
	}
	Write16(&regaddr, 1, d);

	GyroBmi323::Enable();

	return true;
}

uint32_t GyroBmi323::FilterFreq(uint32_t Freq)
{
	uint8_t t = GyroSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint8_t d = Read16(&regaddr, 1) & ~BMI323_GYR_CONFIG_AVG_NUM_MASK;

	if (t < 4)
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_32;
		t = 5;
	}
	else
	{
		d |= BMI323_GYR_CONFIG_AVG_NUM_64;
		t = 6;
	}

	Write16(&regaddr, 1, d);

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() >> t);
}

uint32_t GyroBmi323::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint32_t conf = Read16(&regaddr, 1) & ~BMI323_GYR_CONFIG_ODR_MASK;
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
				conf &= ~BMI323_GYR_CONFIG_ODR_MASK;
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

	Write16(&regaddr, 1, conf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

uint32_t GyroBmi323::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint32_t d = Read16(&regaddr, 1) & ~BMI323_GYR_CONFIG_RANGE_MASK;
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

	if (vpTimer == nullptr)
	{
		regaddr = BMI323_FIFO_CONFIG_REG;
		d = Read16(&regaddr, 1) | BMI323_FIFO_CONFIG_GYR_EN;

		Write16(&regaddr, 1, d);

		msDelay(1); // Require delay, do not remove

		regaddr = BMI323_FIFO_CTRL_REG;
		Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

		FifoDataFlagSet(BMI323_FIFO_DATA_FLAG_GYR);
	}

	regaddr = BMI323_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
//		printf("BMI323_FIFO_CTRL_FLUSH err %x\r\n", d);
		return false;
	}

	regaddr = BMI323_GYR_CONFIG_REG;
	d = Read16(&regaddr, 1) & ~BMI323_GYR_CONFIG_MODE_MASK;
	d |= BMI323_GYR_CONFIG_MODE_CONT_EN;

//	printf("Gyr Cfg : %x\r\n", d);

	Write16(&regaddr, 1, d);

	msDelay(20); // Require delay, do not remove

	regaddr = BMI323_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
//		printf("Gyr err %x\r\n", d);
		return false;
	}

	return true;
}

void GyroBmi323::Disable()
{
	uint8_t regaddr = BMI323_GYR_CONFIG_REG;
	uint16_t d = Read16(&regaddr, 1) & ~BMI323_GYR_CONFIG_MODE_MASK;
	Write16(&regaddr, 1, d);

	msDelay(10);

	regaddr = BMI323_ERR_REG;
	d = Read16(&regaddr, 1);

	regaddr = BMI323_FIFO_CONFIG_REG;
	d = Read16(&regaddr, 1) & ~BMI323_FIFO_CONFIG_GYR_EN;

	Write16(&regaddr, 1, d);
}

bool TempBmi323::Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
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
bool TempBmi323::Enable()
{
	bool retval = false;

	FifoDataFlagSet(BMI323_FIFO_DATA_FLAG_TEMP);

	return retval;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * @return	None
 */
void TempBmi323::Disable()
{
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
		vpTimer = pTimer;
	}

	if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		switch (DevAddr)
		{
			case 1:
			case BMI323_I2C_7BITS_DEVADDR1:
				DeviceAddress(BMI323_I2C_7BITS_DEVADDR1);
				break;
			case 0:
			case BMI323_I2C_7BITS_DEVADDR0:
			default:
				DeviceAddress(BMI323_I2C_7BITS_DEVADDR0);
				break;
		}
	}
	else
	{
		// SPI
		DeviceAddress(DevAddr);

		// Read dummy as per datasheet
		regaddr = BMI323_CHIP_ID_REG;
		d = Read16(&regaddr, 1);
	}

	// Read chip id
	regaddr = BMI323_CHIP_ID_REG;
	d = Read16(&regaddr, 1) & 0xFF;	// Bit 8-15 must be ignored

	if (d != BMI323_CHIP_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	regaddr = BMI323_ERR_REG;
	d = Read16(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	// Read to clear
	regaddr = BMI323_STATUS_REG;
	d = Read16(&regaddr, 1);

	if ((d & BMI323_STATUS_POR_DETECTED) == 0)
	{
		return false;
	}

	regaddr = BMI323_FIFO_CTRL_REG;
	Write16(&regaddr, 1, BMI323_FIFO_CTRL_FLUSH);

	msDelay(10);

	if (vpTimer == nullptr)
	{

		regaddr = BMI323_FIFO_CONFIG_REG;
		d = Read16(&regaddr, 1) | BMI323_FIFO_CONFIG_TIME_EN | BMI323_FIFO_CONFIG_TEMP_EN;

		Write16(&regaddr, 1, d);

		vFifoDataFlag = BMI323_FIFO_DATA_FLAG_TEMP | BMI323_FIFO_DATA_FLAG_TIME;
		vFifoFrameSize = 2;
	}

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
	uint8_t regaddr;
	int len = 0;

	if (vpTimer == nullptr)
	{
		uint8_t fifo[256];

		regaddr = BMI323_FIFO_FILL_LEVEL_REG;
		len = Read16(&regaddr, 1);

		// Re-adjust length to read full frame only
		len = (min(len, 128) / vFifoFrameSize) * vFifoFrameSize;

		if (len >= vFifoFrameSize)
		{
			regaddr = BMI323_FIFO_DATA_REG;
			len = Read(&regaddr, 1, fifo, len << 1) >> 1;

			int16_t *p = (int16_t*)fifo;

			while (len >= vFifoFrameSize)
			{
				if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_ACC)
				{
					if (p[0] != BMI323_ACC_DUMMY_X)
					{
						// Take valid data only
						memcpy(AccelSensor::vData.Val, p, 6);
						AccelSensor::vSampleCnt++;
					}
					p += 3;
				}
				if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_GYR)
				{
					if (p[0] != BMI323_GYR_DUMMY_X)
					{
						// Take valid data only
						memcpy(GyroSensor::vData.Val, p, 6);
						GyroSensor::vSampleCnt++;
					}
					p += 3;
				}
				if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_TEMP)
				{
					if (p[0] != BMI323_TEMP_DUMMY)
					{
						// Take valid data only
						TempSensor::vData.Temperature = p[0];
						TempSensor::vSampleCnt++;
					}
					p++;
				}
				if (vFifoDataFlag & BMI323_FIFO_DATA_FLAG_TIME)
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

		regaddr = BMI323_STATUS_REG;
		uint16_t d = Read16(&regaddr, 1);

		if (d & BMI323_STATUS_DRDY_ACC)
		{
			regaddr = BMI323_ACC_DATA_X_REG;
			Read(&regaddr, 1, (uint8_t*)AccelSensor::vData.Val, 6);
			AccelSensor::vData.Timestamp = t;

			res = true;
		}
		if (d & BMI323_STATUS_DRDY_GYR)
		{
			regaddr = BMI323_GYR_DATA_X_REG;
			Read(&regaddr, 1, (uint8_t*)GyroSensor::vData.Val, 6);
			GyroSensor::vData.Timestamp = t;
			res = true;
		}
		if (d & BMI323_STATUS_DRDY_TEMP)
		{
			regaddr = BMI323_TEMP_DATA_REG;
			Read(&regaddr, 1, (uint8_t*)&TempSensor::vData.Temperature, 2);
			TempSensor::vData.Timestamp = t;
			res = true;
		}
	}

	return res;
}

void AgBmi323::IntHandler()
{
	uint8_t regaddr = BMI323_INT1_STATUS_REG;
	uint16_t d;

	// Read all status
	//Read(&regaddr, 1, (uint8_t*)&d, 2);
	d = Read16(&regaddr, 1);

	if (d & (BMI323_INT1_STATUS_ACC_DRDY | BMI323_INT1_STATUS_GYR_DRDY |
			BMI323_INT1_STATUS_FWM | BMI323_INT1_STATUS_FFULL))
//	if (d & BMI323_INT1_STATUS_FWM | BMI323_INT1_STATUS_FFULL)
	{
		UpdateData();

		if (vEvtHandler)
		{
			vEvtHandler(this, DEV_EVT_DATA_RDY);
		}
	}
}

int AgBmi323::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t b[BuffLen + 1];
	int n = Device::Read(pCmdAddr, CmdAddrLen, b, BuffLen + 1);

	memcpy(pBuff, &b[1], BuffLen);

	return n;
}

static size_t FifoFrameSize(uint8_t Flag)
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
		retval ++;
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

