/**-------------------------------------------------------------------------
@file	ag_bmi160.cpp

@brief	Implementation of BOSCH BMI160 sensor Accel, Gyro


@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <math.h>

#include "istddef.h"
#include "coredev/spi.h"
#include "sensors/ag_bmi160.h"
#include "idelay.h"

bool AccelBmi160::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	vData.Range = Range(0x7FFF);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	AccelBmi160::Enable();

	return true;
}

uint8_t AccelBmi160::Scale(uint8_t Value)
{
	uint8_t regaddr = BMI160_ACC_RANGE;

	if (Value < 4)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_2G);
		Value = 2;
	}
	else if (Value < 8)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_4G);
		Value = 4;
	}
	else
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_8G);
		Value = 8;
	}

	vData.Scale = Value;

	return AccelSensor::Scale(Value);
}

uint32_t AccelBmi160::FilterFreq(uint32_t Freq)
{
	uint8_t t = AccelSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI160_ACC_CONF;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_BWP_MASK;

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

	Write8(&regaddr, 1, d);

	return AccelSensor::FilterFreq(AccelSensor::SamplingFrequency() >> t);
}

uint32_t AccelBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_ACC_CONF;
	uint32_t accconf = Read8(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 100000)
	{
		for (int i = 1; i < 8; i++)
		{
			uint32_t t = 100000 >> (8 - i);
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				accconf &= ~BMI160_ACC_CONF_ACC_ODR_MASK;
				accconf |= i;
				f = t;
				dif = x;
			}
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			uint32_t t = 100000 << i;
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				accconf &= ~BMI160_ACC_CONF_ACC_ODR_MASK;
				accconf |= i | 0x8;
				f = t;
				dif = x;
			}
		}
	}

	if (f < 12500)
	{
		// under sampling
		accconf |= BMI160_ACC_CONF_ACC_US;
	}
	else
	{
		accconf &= ~BMI160_ACC_CONF_ACC_US;
	}

	Write8(&regaddr, 1, accconf);

	return Sensor::SamplingFrequency(f);
}

bool AccelBmi160::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_ACC_EN;

	Write8(&regaddr, 1, d);

	msDelay(1); // Require delay, do not remove

	regaddr = BMI160_CMD;

	if (Sensor::SamplingFrequency() > 12500)
	{
		Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_NORMAL);
	}
	else
	{
		Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_LOWPOWER);
	}

	msDelay(20); // Require delay, do not remove

	regaddr = BMI160_ERR_REG;
	d = Read8(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void AccelBmi160::Disable()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_SUSPEND);

	msDelay(10);

	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);

	regaddr = BMI160_FIFO_CONFIG_1;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_FIFO_CONFIG_1_FIFO_ACC_EN;

	Write8(&regaddr, 1, d);
}

bool GyroBmi160::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Sensor::Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	vData.Range = Range(0x7FFF);

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	GyroBmi160::Enable();

	return true;
}

uint32_t GyroBmi160::FilterFreq(uint32_t Freq)
{
	uint8_t t = GyroSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI160_GYR_CONF;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_BWP_MASK;

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

	Write8(&regaddr, 1, d);

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() >> t);
}

uint32_t GyroBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_GYR_CONF;
	uint32_t odrval = Read8(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 100000)
	{
		for (int i = 6; i < 8; i++)
		{
			uint32_t t = 100000 >> (8 - i);
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_GYR_CONF_GYR_ODR_MASK;
				odrval |= i;
				f = t;
				dif = x;
			}
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			uint32_t t = 100000 << i;
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_GYR_CONF_GYR_ODR_MASK;
				odrval |= i | 0x8;
				f = t;
				dif = x;
			}
		}
	}

	Write8(&regaddr, 1, odrval);

	return Sensor::SamplingFrequency(f);
}

uint32_t GyroBmi160::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = BMI160_GYR_RANGE;
	uint32_t range = 0;

	if (Value < 250)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_125);
		range = 125;
	}
	else if (Value < 500)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_250);
		range = 250;
	}
	else if (Value < 1000)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_500);
		range = 500;
	}
	else if (Value < 2000)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_1000);
		range = 1000;
	}
	else
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_2000);
		range = 2000;
	}

	vData.Scale = Value;

	return GyroSensor::Sensitivity(range);
}

bool GyroBmi160::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_GYR_EN;

	Write8(&regaddr, 1, d);

	msDelay(5); // Require delay, do not remove

	regaddr = BMI160_CMD;

	if (Sensor::SamplingFrequency() < 25000)
	{
		Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_FASTSTARTUP);
	}
	else
	{
		Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_NORMAL);
	}

	msDelay(40); // Require delay, do not remove

	regaddr = BMI160_ERR_REG;
	d = Read8(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void GyroBmi160::Disable()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_SUSPEND);

	msDelay(10);

	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);

	regaddr = BMI160_FIFO_CONFIG_1;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_FIFO_CONFIG_1_FIFO_GYR_EN;

	Write8(&regaddr, 1, d);
}

bool AgBmi160::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	// Read chip id
	regaddr = BMI160_CHIP_ID_REG;
	d = Read8(&regaddr, 1);

	if (d != BMI160_CHIP_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	msDelay(10);

	regaddr = BMI160_CMD;
	Write8(&regaddr, 1, BMI160_CMD_FIFO_FLUSH);

	regaddr = BMI160_FIFO_CONFIG_0;
	Write8(&regaddr, 1, 7);

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_HEADER_EN | BMI160_FIFO_CONFIG_1_FIFO_TIME_EN;
	Write8(&regaddr, 1, d);

	return true;
}

bool AgBmi160::Enable()
{
	AccelBmi160::Enable();
	GyroBmi160::Enable();

	return true;
}

void AgBmi160::Disable()
{
	AccelBmi160::Disable();
	GyroBmi160::Disable();
}

void AgBmi160::Reset()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_SOFT_RESET);

	msDelay(1);

	// Read err register to clear
	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);
}

bool AgBmi160::UpdateData()
{
	uint8_t regaddr = BMI160_FIFO_LENGTH_0;
	int len = 0;

	Device::Read(&regaddr, 1, (uint8_t*)&len, 2);
	if (len <= 0)
	{
		return false;
	}

	uint8_t buff[BMI160_FIFO_MAX_SIZE];

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
				if (len >= 6)
				{
//				memcpy(MagBmi160::vData.Val, p, 6);
				}
				p += 6;
				len -= 6;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_GYRO)
			{
				if (len >= 6)
				{
					memcpy(GyroBmi160::vData.Val, p, 6);
				}
				p += 6;
				len -= 6;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_ACCEL)
			{
				if (len >= 6)
				{
					memcpy(AccelBmi160::vData.Val, p, 6);
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
					AccelBmi160::vDropCnt += *p;
					GyroBmi160::vDropCnt = AccelBmi160::vDropCnt;
					//printf("Skip frame %d %x\r\n", AccelBmi160::vDropCnt, AccelBmi160::vDropCnt);
					len--;
					p++;
					break;
				case BMI160_FRAME_CONTROL_PARM_TIME:
					if (len >= 3)
					{
						memcpy(&AccelBmi160::vData.Timestamp, p, 3);
						memcpy(&GyroBmi160::vData.Timestamp, p, 3);

						//printf("Time\r\n");
					}
					len -= 3;
					p += 3;
					break;
				case BMI160_FRAME_CONTROL_PARM_INPUT:
					//Read8(&regaddr, 1);
					printf("Input\r\n");
					p++;
					len--;
					break;
				default:
					printf("Control??\r\n");
			}
		}
		else
		{
			printf("Unknown\r\n");
		}
	}

	return true;
}

int AgBmi160::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t regaddr;


}

int AgBmi160::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{

}

void AgBmi160::IntHandler()
{

}

