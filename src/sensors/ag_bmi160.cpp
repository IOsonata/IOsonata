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

#include "coredev/spi.h"
#include "sensors/ag_bmi160.h"


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

	return true;
}

bool AccelBmi160::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, (DeviceIntrf *)pIntrf, (Timer *)pTimer) == false)
		return false;

	Scale(CfgData.Scale);
/*
	uint8_t regaddr = BMI160_ACC_RANGE;
	if (CfgData.Scale < 4)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_2G);
	}
	else if (CfgData.Scale < 8)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_4G);
	}
	else
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_8G);
	}
*/
	SamplingFrequency(CfgData.Freq);

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	return true;
}

uint32_t AccelBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_ACC_CONF;
	uint32_t odrval = Read8(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_ODR_MASK;
	uint32_t f = 0;

	if (Freq < 100)
	{
		for (int i = 1; i < 8; i++)
		{
			uint32_t t = 25000 >> (0x80 >> i);

			if (t > Freq)
			{
				break;
			}
			odrval |= i;
			f = t;
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			uint32_t t = 100000 << i;
			if (t > Freq)
			{
				break;
			}
			odrval |= i | 0x8;
			f = t;
		}
	}


	Write8(&regaddr, 1, odrval);

	return Sensor::SamplingFrequency(f);
}

bool GyroBmi160::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	return true;
}

uint32_t GyroBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_GYR_CONF;
	uint32_t odrval = Read8(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_ODR_MASK;
	uint32_t f = 0;

	if (Freq < 100)
	{
		for (int i = 6; i < 8; i++)
		{
			uint32_t t = 25000 >> (0x80 >> i);

			if (t > Freq)
			{
				break;
			}
			odrval |= i;
			f = t;
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			uint32_t t = 100000 << i;
			if (t > Freq)
			{
				break;
			}
			odrval |= i | 0x8;
			f = t;
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

	return GyroSensor::Sensitivity(range);
}

bool AgBmi160::Enable()
{
	return true;
}

void AgBmi160::Disable()
{

}

void AgBmi160::Reset()
{

}

bool AgBmi160::StartSampling()
{
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

	return AccelSensor::Scale(Value);
}

bool AgBmi160::UpdateData()
{
	return true;
}

