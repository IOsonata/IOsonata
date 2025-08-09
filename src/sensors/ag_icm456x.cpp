/**-------------------------------------------------------------------------
@file	ag_icm456x.cpp

@brief	TDK Invensen ICM456x series accel gyro implementation

This file implements only accel & gyro part of the ICM456x. IMU features are
implemented in imu implementation file.


@author	Hoang Nguyen Hoan
@date	Mar. 19, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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
#include <memory.h>

#include "istddef.h"
#include "idelay.h"
#include "convutil.h"
#include "sensors/ag_icm456x.h"

#if 0
#include "coredev/uart.h"

extern UART g_Uart;
#endif

bool AccelIcm456x::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	Type(SENSOR_TYPE_ACCEL);
	//vData.Range =
	Range(ICM456X_ADC_RANGE_HIRES);
	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);

	return true;
}
uint32_t AccelIcm456x::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = ICM456X_ACCEL_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_ACCEL_CONFIG0_ODR_MASK;

	if (Freq < 2000)
	{
		Freq = 1652;
		d |= ICM456X_ACCEL_CONFIG0_ODR_1_6525;
	}
	else if (Freq < 4000)
	{
		Freq = 3125;
		d |= ICM456X_ACCEL_CONFIG0_ODR_3_125;
	}
	else if (Freq < 7000)
	{
		Freq = 6250;
		d |= ICM456X_ACCEL_CONFIG0_ODR_6_25;
	}
	else if (Freq < 13000)
	{
		Freq = 12500;
		d |= ICM456X_ACCEL_CONFIG0_ODR_12_5;
	}
	else if (Freq < 26000)
	{
		Freq = 25000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_25;
	}
	else if (Freq < 51000)
	{
		Freq = 50000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_50;
	}
	else if (Freq < 101000)
	{
		Freq = 100000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_100;
	}
	else if (Freq < 201000)
	{
		Freq = 200000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_200;
	}
	else if (Freq < 410000)
	{
		Freq = 400000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_400;
	}
	else if (Freq < 810000)
	{
		Freq = 800000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_800;
	}
	else if (Freq < 1700000)
	{
		Freq = 1600000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_1600;
	}
	else if (Freq < 3300000)
	{
		Freq = 3200000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_3200;
	}
	else
	{
		Freq = 6400000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_6400;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::SamplingFrequency(Freq);
}

uint8_t AccelIcm456x::Scale(uint8_t Value)
{
	uint8_t regaddr = ICM456X_ACCEL_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_ACCEL_CONFIG0_UI_FS_SEL_MASK;

	((AgIcm456x*)this)->vHires = false;

	if (Value <= 2)
	{
		d |= ICM456X_ACCEL_CONFIG0_UI_FS_SEL_2G;
		Value = 2;
	}
	else if (Value <= 4)
	{
		d |= ICM456X_ACCEL_CONFIG0_UI_FS_SEL_4G;
		Value = 4;
	}
	else if (Value <= 8)
	{
		d |= ICM456X_ACCEL_CONFIG0_UI_FS_SEL_8G;
		Value = 8;
	}
	else if (Value <= 16)
	{
		d |= ICM456X_ACCEL_CONFIG0_UI_FS_SEL_16G;
		Value = 16;
	}
	else
	{
		d |= ICM456X_ACCEL_CONFIG0_UI_FS_SEL_32G;
		Value = 32;
		((AgIcm456x*)this)->vHires = true;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::Scale(Value);
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
uint32_t AccelIcm456x::FilterFreq(uint32_t Freq)
{
	uint32_t div = AccelSensor::SamplingFrequency() / Freq;
	uint16_t regaddr = ICM456X_IPREG_SYS2_REG_131_REG;
	uint8_t d = 0, avg = 0;

	Read(regaddr, &d, 1);

	d &= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_SEL_MASK;

	regaddr = ICM456X_IPREG_SYS2_REG_129_REG;
	Read(regaddr, &avg, 1);

	avg &= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_SEL_MASK;

	if (div < 2)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_BYPASS;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_1X;
		div = 1;
	}
	else if (div < 4)
	{
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_2X;
		div = 2;
	}
	else if (div < 5)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_4X;
		div = 4;
	}
	else if (div < 7)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_5X;
		div = 5;
	}
	else if (div < 8)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_7X;
		div = 7;
	}
	else if (div < 10)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_8X;
		div = 8;
	}
	else if (div < 11)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_10X;
		div = 10;
	}
	else if (div < 16)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_11X;
		div = 11;
	}
	else if (div < 18)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_16X;
		div = 16;
	}
	else if (div < 20)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_18X;
		div = 18;
	}
	else if (div < 32)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_20X;
		div = 20;
	}
	else if (div < 64)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV32;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_32X;
		div = 32;
	}
	else if (div < 128)
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV64;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_64X;
		div = 64;
	}
	else
	{
		d |= ICM456X_IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_ODR_DIV128;
		avg |= ICM456X_IPREG_SYS2_REG_129_ACCEEL_LP_AVG_64X;
		div = 128;
	}

	Write(regaddr, &avg, 1);

	regaddr = ICM456X_IPREG_SYS2_REG_131_REG;
	Write(regaddr, &d, 1);


	return AccelSensor::FilterFreq(AccelSensor::SamplingFrequency() / div);
}

bool AccelIcm456x::Enable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_ACCEL_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_ACCEL_MODE_LOW_NOISE;
//	d |= ICM456X_PWR_MGMT0_ACCEL_MODE_LOW_PWR;
	Write8(&regaddr, 1, d);

	regaddr = ICM456X_FIFO_CONFIG3_REG;
	d = Read8(&regaddr, 1) | ICM456X_FIFO_CONFIG3_ACCEL_EN;
	Write8(&regaddr, 1, d);

	if (((AgIcm456x*)this)->vFifoFrameSize < 8)
	{
		((AgIcm456x*)this)->vFifoFrameSize += 8;
	}
	else
	{
		((AgIcm456x*)this)->vFifoFrameSize += 6;
	}

	return true;
}

void AccelIcm456x::Disable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_ACCEL_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_ACCEL_MODE_STDBY;
	Write8(&regaddr, 1, d);

}

bool GyroIcm456x::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	Type(SENSOR_TYPE_GYRO);
	//vData.Range =
	Range(ICM456X_ADC_RANGE_HIRES);
	SamplingFrequency(Cfg.Freq);
	Sensitivity(Cfg.Sensitivity);

	return true;
}
uint32_t GyroIcm456x::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = ICM456X_GYRO_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_GYRO_CONFIG0_ODR_MASK;

	if (Freq < 2000)
	{
		Freq = 1652;
		d |= ICM456X_GYRO_CONFIG0_ODR_1_6525;
	}
	else if (Freq < 4000)
	{
		Freq = 3125;
		d |= ICM456X_GYRO_CONFIG0_ODR_3_125;
	}
	else if (Freq < 7000)
	{
		Freq = 6250;
		d |= ICM456X_GYRO_CONFIG0_ODR_6_25;
	}
	else if (Freq < 13000)
	{
		Freq = 12500;
		d |= ICM456X_GYRO_CONFIG0_ODR_12_5;
	}
	else if (Freq < 26000)
	{
		Freq = 25000;
		d |= ICM456X_GYRO_CONFIG0_ODR_25;
	}
	else if (Freq < 51000)
	{
		Freq = 50000;
		d |= ICM456X_GYRO_CONFIG0_ODR_50;
	}
	else if (Freq < 101000)
	{
		Freq = 100000;
		d |= ICM456X_GYRO_CONFIG0_ODR_100;
	}
	else if (Freq < 201000)
	{
		Freq = 200000;
		d |= ICM456X_GYRO_CONFIG0_ODR_200;
	}
	else if (Freq < 410000)
	{
		Freq = 400000;
		d |= ICM456X_GYRO_CONFIG0_ODR_400;
	}
	else if (Freq < 810000)
	{
		Freq = 800000;
		d |= ICM456X_GYRO_CONFIG0_ODR_800;
	}
	else if (Freq < 1700000)
	{
		Freq = 1600000;
		d |= ICM456X_ACCEL_CONFIG0_ODR_1600;
	}
	else if (Freq < 3300000)
	{
		Freq = 3200000;
		d |= ICM456X_GYRO_CONFIG0_ODR_3200;
	}
	else
	{
		Freq = 6400000;
		d |= ICM456X_GYRO_CONFIG0_ODR_6400;
	}

	Write8(&regaddr, 1, d);

	return GyroSensor::SamplingFrequency(Freq);
}

uint32_t GyroIcm456x::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = ICM456X_GYRO_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_GYRO_CONFIG0_UI_FS_SEL_MASK;

	((AgIcm456x*)this)->vHires = false;

	if (Value <= 16)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_15_625;
		Value = 16;
	}
	else if (Value <= 32)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_31_25;
		Value = 31;
	}
	else if (Value <= 63)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_62_5;
		Value = 63;
	}
	else if (Value <= 125)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_125;
		Value = 125;
	}
	else if (Value <= 250)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_250;
		Value = 250;
	}
	else if (Value <= 500)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_500;
		Value = 500;
	}
	else if (Value <= 1000)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_1000;
		Value = 1000;
	}
	else if (Value <= 2000)
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_2000;
		Value = 2000;
	}
	else
	{
		d |= ICM456X_GYRO_CONFIG0_UI_FS_SEL_4000;
		Value = 4000;
		((AgIcm456x*)this)->vHires = true;
	}

	Write8(&regaddr, 1, d);

	return GyroSensor::Sensitivity(Value);
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
uint32_t GyroIcm456x::FilterFreq(uint32_t Freq)
{
	uint32_t div = GyroSensor::SamplingFrequency() / Freq;
	uint16_t regaddr = ICM456X_IPREG_SYS1_REG_172_REG;
	uint8_t d = 0, avg = 0;

	Read(regaddr, &d, 1);

	d &= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_SEL_MASK;

	regaddr = ICM456X_IPREG_SYS1_REG_170_REG;
	Read(regaddr, &avg, 1);

	avg &= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_SEL_MASK;

	if (div < 2)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_BYPASS;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_1X;
		div = 1;
	}
	else if (div < 4)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_BYPASS;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_2X;
		div = 2;
	}
	else if (div < 5)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_4X;
		div = 4;
	}
	else if (div < 7)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_5X;
		div = 5;
	}
	else if (div < 8)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV4;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_7X;
		div = 7;
	}
	else if (div < 10)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_8X;
		div = 8;
	}
	else if (div < 11)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_10X;
		div = 10;
	}
	else if (div < 16)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV8;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_11X;
		div = 11;
	}
	else if (div < 18)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_16X;
		div = 16;
	}
	else if (div < 20)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_18X;
		div = 18;
	}
	else if (div < 32)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV16;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_20X;
		div = 20;
	}
	else if (div < 64)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV32;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_32X;
		div = 32;
	}
	else if (div < 128)
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV64;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_64X;
		div = 64;
	}
	else
	{
		d |= ICM456X_IPREG_SYS1_REG_172_GYRO_UI_LPFBW_ODR_DIV128;
		avg |= ICM456X_IPREG_SYS1_REG_170_GYRO_LP_AVG_64X;
		div = 128;
	}

	Write(regaddr, &avg, 1);

	regaddr = ICM456X_IPREG_SYS1_REG_172_REG;
	Write(regaddr, &d, 1);

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() / div);
}

bool GyroIcm456x::Enable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_GYRO_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_GYRO_MODE_LOW_NOISE;
	Write8(&regaddr, 1, d);

	regaddr = ICM456X_FIFO_CONFIG3_REG;
	d = Read8(&regaddr, 1) | ICM456X_FIFO_CONFIG3_GYRO_EN;
	Write8(&regaddr, 1, d);

	if (((AgIcm456x*)this)->vFifoFrameSize < 8)
	{
		((AgIcm456x*)this)->vFifoFrameSize += 8;
	}
	else
	{
		((AgIcm456x*)this)->vFifoFrameSize += 6;
	}
	return true;
}

void GyroIcm456x::Disable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_GYRO_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_GYRO_MODE_STDBY;
	Write8(&regaddr, 1, d);

}

/**
 * @brief	Initialize sensor (require implementation).
 *
 * @param 	CfgData : Reference to configuration data
 * @param	pIntrf 	: Pointer to interface to the sensor.
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 * @param	pTimer	: Pointer to timer for retrieval of time stamp
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool TempIcm456x::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

	return true;
}

/**
 * @brief	Enable temperature sensor
 *
 * @return	true - If success
 */
bool TempIcm456x::Enable()
{
	// Temperature sensor is always enable for this device
	return true;
}

/**
 * @brief	Disable temperature sensor
 *
 * @return	None
 */
void TempIcm456x::Disable()
{
	// Temperature sensor is always enable for this device
}

AgIcm456x::AgIcm456x()
{
	memset(vbSensorEnabled, 0, sizeof(vbSensorEnabled));
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO;
	vPrevTime = 0;
	vRollover = 0;
	vFifoFrameSize = 2;	// Min count 1 byte header + 1 byte temperature
	vHires = false;
}

bool AgIcm456x::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);

	uint8_t regaddr = ICM456X_WHO_AM_I_REG;
	uint8_t d;

	if (DevAddr == ICM456X_I2C_7BITS_DEVADDR0 || DevAddr == ICM456X_I2C_7BITS_DEVADDR1)
	{
		if (pIntrf->Type() != DEVINTRF_TYPE_I2C)
		{
			// Device address is set to I2C but interface is not
			return false;
		}

		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_WHO_AM_I_REG)
		{
			return false;
		}
	}
	else if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		// Interface is I2C but device address is not set.
		// Detect device
		DeviceAddress(ICM456X_I2C_7BITS_DEVADDR0);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_CHIP_ID)
		{
			// Try alternate address
			DeviceAddress(ICM456X_I2C_7BITS_DEVADDR1);
			d = Read8((uint8_t*)&regaddr, 1);
			if (d != ICM456X_CHIP_ID)
			{
				return false;
			}
		}
	}
	else
	{
		// Not I2C interface
		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 1);

		if (d != ICM456X_CHIP_ID)
		{
			return false;
		}
	}

	// Device found, save device id
	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Valid(true);

	Reset();

	regaddr = ICM456X_FIFO_CONFIG0_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG0_DEPTH_2K | ICM456X_FIFO_CONFIG0_MODE_STREAM);

	regaddr = ICM456X_FIFO_CONFIG1_0_REG;

	uint16_t wm = ICM456X_FIFO_SIZE * 80 / 100; // 80%

	//printf("wm = %d\r\n", wm);

	Write16(&regaddr, 1, wm);

	regaddr = ICM456X_FIFO_CONFIG2_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG2_WR_WM_GT_TH | ICM456X_FIFO_CONFIG2_FLUSH);

	uint16_t sreg = ICM456X_SREG_CTRL_REG;
	d = ICM456X_SREG_CTRL_DATA_LITTLE_ENDIAN;
	Write(sreg, &d, 1);

	ByteOrder(DEV_BYTEORDER_LITTLE);

	sreg = ICM456X_SMC_CONTROL_0_REG;
	d = 0;
	Read(sreg, &d, 1);

	d |= ICM456X_SMC_CONTROL_0_TMST_EN | ICM456X_SMC_CONTROL_0_TMST_FSYNC_EN;
	Write(sreg, &d, 1);

	//regaddr = ICM456X_FIFO_CONFIG3_REG;
	//d = Read8(&regaddr, 1) | ICM456X_FIFO_CONFIG3_IF_EN | ICM456X_FIFO_CONFIG3_HIRES_EN;

	//Write8(&regaddr, 1, d);

	regaddr = ICM456X_FIFO_CONFIG4_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG4_TMST_FSYNC_EN);

	regaddr = ICM456X_TMST_WOM_CONFIG_REG;
	d = Read8(&regaddr, 1);

	//printf("d = %x\r\n", d);

	uint32_t intcfg = 0;

	regaddr = ICM456X_INT1_CONFIG0_REG;

	if (Inter)
	{
		intcfg = ICM456X_INT_CONFIG_FIFO_FULL_EN | ICM456X_INT_CONFIG_FIFO_THRS_EN |
				 ICM456X_INT_CONFIG_DRDY_EN | ICM456X_INT_CONFIG_AUX1_DRDY_EN |
				 ICM456X_INT_CONFIG_WOM_X_EN | ICM456X_INT_CONFIG_WOM_Y_EN | ICM456X_INT_CONFIG_WOM_Z_EN |
				 ICM456X_INT_CONFIG_I2CM_DONE_EN | ICM456X_INT_CONFIG_APEX_EVENT_EN;

		if (IntPol == DEVINTR_POL_HIGH)
		{
			intcfg |= ICM456X_INT_CONFIG_POLARITY_HIGH | ICM456X_INT_CONFIG_MODE_LATCH;
		}

		if (Inter == 1)
		{
			regaddr = ICM456X_INT1_CONFIG0_REG;
		}
		else
		{
			regaddr = ICM456X_INT2_CONFIG0_REG;
		}

		InterruptId(Inter);
	}
	Write(&regaddr, 1, (uint8_t*)&intcfg, 3);

//	Enable();

	vAuxIntrf.Init(this);

	return true;
}

bool AgIcm456x::Enable()
{
	uint8_t regaddr = ICM456X_FIFO_CONFIG3_REG;
	uint8_t d = Read8(&regaddr, 1) | ICM456X_FIFO_CONFIG3_IF_EN;

	vFifoFrameSize = 16;

	if (vHires)
	{
		d |= ICM456X_FIFO_CONFIG3_HIRES_EN;
		vFifoFrameSize = 20;
	}

	Write8(&regaddr, 1, d);

	bool res = AccelIcm456x::Enable();

	if (res == true)
	{
		res = GyroIcm456x::Enable();
	}



	return res;
}

void AgIcm456x::Disable()
{

}

void AgIcm456x::Reset()
{
	uint8_t regaddr = ICM456X_REG_HOST_MSG_REG;

	Write8(&regaddr, 1, ICM456X_REG_MISC2_SOFT_RST);
}

void AgIcm456x::IntHandler()
{
	uint8_t regaddr = ICM456X_INT1_STATUS0_REG;
	uint16_t istatus, istatus2;

	istatus  = Read16(&regaddr, 1);

	//if (InterruptId() == 2)
	{
		regaddr = ICM456X_INT2_STATUS1_REG;
	}
	istatus2  = Read16(&regaddr, 1);

	//g_Uart.printf("%x %x\r\n", istatus, istatus2);

	if (istatus & ICM456X_INT_STATUS_DRDY)
	{
		//printf("Data Ready\r\n");
		UpdateData();

	}

	if (istatus & ICM456X_INT_STATUS_FIFO_THS)
	{
		printf("fifo ths\r\n");
	}
}

void AgIcm456x::ProcessFifo(uint8_t *pData, size_t Len)
{
	uint8_t *p = pData;
	uint8_t hdr = *p;
	uint8_t exhdr = 0;

	p++;

	if (hdr & ICM456X_FIFO_HDR_EXT_HDR_DATA)
	{
		exhdr = *p;
		p++;
	}

	if (hdr & ICM456X_FIFO_HDR_ACCEL_EN)
	{
		AccelSensor::vData.X = (((int8_t)p[1] << 8) | p[0]);
		AccelSensor::vData.Y = (((int8_t)p[3] << 8) | p[2]);
		AccelSensor::vData.Z = (((int8_t)p[5] << 8) | p[4]);

		p += 6;
	}

	if (hdr & ICM456X_FIFO_HDR_GYRO_EN)
	{
		GyroSensor::vData.X = (((int8_t)p[1] << 8) | p[0]);
		GyroSensor::vData.Y = (((int8_t)p[3] << 8) | p[2]);
		GyroSensor::vData.Z = (((int8_t)p[5] << 8) | p[4]);

		p += 6;
	}

	if (exhdr & ICM456X_FIFO_EXTHDR_ES0_EN)
	{
		if (exhdr & ICM456X_FIFO_EXTHDR_ES0_9BYTES)
		{
		}
		p += 9;
	}

	if (exhdr & ICM456X_FIFO_EXTHDR_ES1_EN)
	{
		p += 6;
	}

	if (hdr & 0x78)
	{
		if (hdr & ICM456X_FIFO_HDR_HIRES_EN)
		{
			// Fifo hires Temperature in Degrees Centigrade = (FIFO_TEMP_DATA / 128) + 25
			TempSensor::vData.Temperature = (((int16_t)(p[0] | (p[1] << 8)) * 100) >> 7) + 2500;
			p += 2;

			AccelSensor::vData.X <<= 4;
			AccelSensor::vData.X |= (p[2] >> 4) & 0xF;
			AccelSensor::vData.Y <<= 4;
			AccelSensor::vData.Y |= (p[3] >> 4) & 0xF;
			AccelSensor::vData.Z <<= 4;
			AccelSensor::vData.Z |= (p[4] >> 4) & 0xF;

	//		g_Uart.printf("%x %d %d %d\r\n", AccelSensor::vData.X, AccelSensor::vData.X, AccelSensor::vData.Y, AccelSensor::vData.Z);

			GyroSensor::vData.X <<= 4;
			GyroSensor::vData.X |= (p[2] & 0xF);
			GyroSensor::vData.Y <<= 4;
			GyroSensor::vData.Y |= (p[3] & 0xF);
			GyroSensor::vData.Z <<= 4;
			GyroSensor::vData.Z |= (p[4] & 0xF);
		}
		else
		{
			TempSensor::vData.Temperature = ((((int8_t)p[0]) * 100) >> 1) + 2500;
			p++;
		}
		uint16_t t1 = p[0] | (p[1] << 8);

		if (vPrevTime > t1)
		{
			// overflow
			vRollover += 0x10000;
		}

		vPrevTime = t1;

		uint64_t t = t1 + vRollover;

		AccelSensor::vData.Timestamp = t;
		AccelSensor::vData.Temp = TempSensor::vData.Temperature;
		GyroSensor::vData.Timestamp = t;
		GyroSensor::vData.Temp = TempSensor::vData.Temperature;
		TempSensor::vData.Timestamp = t;
	}
}

bool AgIcm456x::UpdateData()
{
	uint8_t regaddr = ICM456X_ACCEL_DATA_X1_UI_REG;
	uint8_t dd[2048];
	uint64_t t = vpTimer->uSecond();
	int cnt = 0;//Device::Read(&regaddr, 1, dd, 14);

	regaddr = ICM456X_FIFO_COUNT_0_REG;
	int fifocnt = Read16(&regaddr, 1);

	while (fifocnt > 0)
	{
		//g_Uart.printf("fifo cnt %d %d\r\n", fifocnt, EndianCvt16(fifocnt));

		regaddr = ICM456X_FIFO_DATA_REG;

		cnt = Read(&regaddr, 1, dd, 2);

		uint8_t hdr = dd[0];
		uint8_t hdr2 = dd[1];

		int pktlen = 0;
		switch (hdr & 0x70)
		{
			case ICM456X_FIFO_HDR_ACCEL_EN:
			case ICM456X_FIFO_HDR_GYRO_EN:
				pktlen = 8;
				break;
			case ICM456X_FIFO_HDR_ACCEL_EN | ICM456X_FIFO_HDR_GYRO_EN:
				pktlen = 16;
				break;
			case ICM456X_FIFO_HDR_ACCEL_EN | ICM456X_FIFO_HDR_GYRO_EN | ICM456X_FIFO_HDR_HIRES_EN:
				pktlen = 20;
				break;
		}

		if (hdr & ICM456X_FIFO_HDR_EXT_HDR_DATA)
		{
			if (pktlen > 0)
			{
				// with accel or gyro data
				pktlen = 32;
			}
			else
			{
				// No Accel & Gyro data
				switch (hdr2 & 0x3)
				{
					case ICM456X_FIFO_EXTHDR_ES0_EN:
					case ICM456X_FIFO_EXTHDR_ES1_EN:
						pktlen = 16;
						break;
					case ICM456X_FIFO_EXTHDR_ES0_EN | ICM456X_FIFO_EXTHDR_ES1_EN:
						pktlen = 20;
						break;
				}
			}
		}

		cnt = Read(&regaddr, 1, dd, pktlen);

		fifocnt--;

		if (cnt > 0 && fifocnt <= 0)
		{
			ProcessFifo(dd, pktlen);
		}

	}

	return true;
}

/**
 * @brief	Indirect read from mem or register
 *
 * @param	IRegAddr	: 16bits Reg/Mem address
 * @param	pBuff		: Pointer to receive buffer
 * @param	BuffLen		: Receive buffer length
 *
 * @return	Number of bytes read
 */
int AgIcm456x::Read(uint16_t IRegAddr, uint8_t *pBuff, int BuffLen)
{
	uint8_t reg = ICM456X_IREG_ADDR_15_8_REG;
	uint8_t cnt = 0;

	Write16(&reg, 1, EndianCvt16(IRegAddr));

	reg = ICM456X_IREG_DATA_REG;

	// NOTE: Must read one by one.  Burst read does not work here
	while (BuffLen > 0)
	{
		*pBuff = Read8(&reg, 1);
		pBuff++;
		BuffLen--;
		cnt++;
	}

	return cnt;
}

/**
 * @brief	Indirect write to mem or register
 *
 * NOTE: Require transfer in single write burst
 *
 * @param	IRegAddr	: 16bits Reg/Mem address
 * @param	pData		: Pointer to data to write
 * @param	DataLen		: Data length
 *
 * @return	Number of bytes written
 */
int AgIcm456x::Write(uint16_t IRegAddr, uint8_t *pData, int DataLen)
{
	uint8_t reg = ICM456X_IREG_ADDR_15_8_REG;
	uint8_t cnt = 0;
	uint8_t d[2 + DataLen] = { (uint8_t)(IRegAddr >> 8), (uint8_t)(IRegAddr & 0xFF), };

	memcpy(&d[2], pData, DataLen);

	cnt = Write(&reg, 1, d, 2 + DataLen);

	return cnt;
}

bool AuxIntrfIcm456x::Init(AgIcm456x * const pIcm)
{
	vIntrfData.pDevData = pIcm;

	uint8_t reg = ICM456X_IOC_PAD_SCENARIO_AUX_OVRD_REG;
	uint8_t d = ICM456X_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_OVRDEN | ICM456X_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_EN |
				ICM456X_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_MODE_OVRDEN | ICM456X_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_MODE_I2CM;
	pIcm->Write8(&reg, 1, d);

	vAuxCmdIdx = 0;

	return true;
}

void AuxIntrfIcm456x::Enable(void)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_IPREG_BAR_REG_60_REG;
	uint8_t d = ICM456X_IPREG_BAR_REG_60_PADS_AUX_SCLK_I2C|
				ICM456X_IPREG_BAR_REG_60_PADS_PIN10_AUX1_CS_PULLUP |
				ICM456X_IPREG_BAR_REG_60_PADS_PIN10_AUX1_CS_PULLRES_EN |
				ICM456X_IPREG_BAR_REG_60_PADS_PIN3_AUX1_SCLK_PULLUP |
				ICM456X_IPREG_BAR_REG_60_PADS_PIN3_AUX1_SCLK_PULLRES_EN;
	icm->Write(regaddr, &d, 1);

	regaddr = ICM456X_IPREG_BAR_REG_61_REG;
	d = ICM456X_IPREG_BAR_REG_61_PADS_PIN2_AUX1_SDI_PULLUP | ICM456X_IPREG_BAR_REG_61_PADS_PIN2_AUX1_SDI_PULLRES_EN;
	icm->Write(regaddr, &d, 1);
}

void AuxIntrfIcm456x::Disable(void)
{
}

bool AuxIntrfIcm456x::StartRx(uint32_t DevAddr)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_DEV0_DEVADDR_REG;
	uint8_t d = DevAddr;

	icm->Write(regaddr, &d, 1);

	return true;
}

// Receive Data only, no Start/Stop condition
int AuxIntrfIcm456x::RxData(uint8_t *pBuff, int BuffLen)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_COMMAND_0_REG + vAuxCmdIdx;
	uint8_t d = ICM456X_I2CM_COMMAND_CH_SEL_ID1 |
				ICM456X_I2CM_COMMAND_ENDFLAG;

	if (vAuxCmdIdx > 0)
	{
		d |= ICM456X_I2CM_COMMAND_R_W_RD_WO_AD;
	}
	else
	{
		d |= ICM456X_I2CM_COMMAND_R_W_RD_W_AD;
	}

	int len = BuffLen > 15 ? 15 : BuffLen;

	d |= len;

	icm->Write(regaddr, &d, 1);

	// Execute
	regaddr = ICM456X_I2CM_CONTROL_REG;
	d = ICM456X_I2CM_CONTROL_I2CM_GO;
	icm->Write(regaddr, &d, 1);

	int timout = 1000;

	while (timout > 0)
	{
		regaddr = ICM456X_I2CM_STATUS_REG;
		icm->Read(regaddr, &d, 1);

		//printf("Status %x\n", d);

		if (d & ICM456X_I2CM_STATUS_I2CM_DONE)
		{
			regaddr = ICM456X_I2CM_RD_DATA0_REG;
			icm->Read(regaddr, pBuff, len);
			break;
		}
	}

	vAuxCmdIdx = 0;

	return BuffLen;
}

void AuxIntrfIcm456x::StopRx(void)
{

}

bool AuxIntrfIcm456x::StartTx(uint32_t DevAddr)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_DEV0_DEVADDR_REG;
	uint8_t d = DevAddr;

	// Set device address
	icm->Write(regaddr, &d, 1);

	return true;
}

// Send Data only, no Start/Stop condition
int AuxIntrfIcm456x::TxData(uint8_t *pData, int DataLen)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_DEV0_ADDRCMD_REG;
	uint8_t d = *pData;

	if (DataLen > 1)
	{
		regaddr = ICM456X_I2CM_COMMAND_0_REG + vAuxCmdIdx;
		d = ICM456X_I2CM_COMMAND_CH_SEL_ID1 | ICM456X_I2CM_COMMAND_R_W_WR;

		int len = DataLen > 6 ? 6 : DataLen;

		d |= len;

		icm->Write(regaddr, &d, 1);

		regaddr = ICM456X_I2CM_WR_DATA0_REG;
		icm->Write(regaddr, pData, len);
		vAuxCmdIdx++;
	}

	return DataLen;
}

void AuxIntrfIcm456x::StopTx(void)
{
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;

	if (vAuxCmdIdx > 0)
	{
		uint16_t regaddr = ICM456X_I2CM_COMMAND_0_REG;
		uint8_t d;

		icm->Read(regaddr, &d, 1);

		d |= ICM456X_I2CM_COMMAND_ENDFLAG;
		icm->Write(regaddr, &d, 1);

		// Execute
		regaddr = ICM456X_I2CM_CONTROL_REG;
		d = ICM456X_I2CM_CONTROL_I2CM_GO;
		icm->Write(regaddr, &d, 1);

		vAuxCmdIdx = 0;
	}

}

int AuxIntrfIcm456x::Read(uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen)
{
	int cnt = 0;

#if 0
	if (pAdCmd)
	{
		StartTx(DevAddr);
		TxData(pAdCmd, AdCmdLen);
	}
	cnt = RxData(pBuff, BuffLen);
	StopRx();

#else
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_DEV_PROFILE1_REG;
	uint8_t d = DevAddr;
	int rlen = min(BuffLen, 15);

	// Set device address
	icm->Write(regaddr, &d, 1);

	// Set register address
	regaddr = ICM456X_I2CM_DEV_PROFILE0_REG;
	d = *pAdCmd;
	icm->Write(regaddr, &d, 1);

	// Set read command
	regaddr = ICM456X_I2CM_COMMAND_0_REG;

	if (AdCmdLen > 1)
	{
		int wlen = min(AdCmdLen, 6);
		d = wlen | ICM456X_I2CM_COMMAND_CH_SEL_ID1 | ICM456X_I2CM_COMMAND_R_W_WR;
		icm->Write(regaddr, &d, 1);

		regaddr = ICM456X_I2CM_WR_DATA0_REG;
		icm->Write(regaddr, pAdCmd, wlen);

		regaddr = ICM456X_I2CM_COMMAND_1_REG;
		d = rlen | ICM456X_I2CM_COMMAND_CH_SEL_ID1 |
			ICM456X_I2CM_COMMAND_R_W_RD_WO_AD | ICM456X_I2CM_COMMAND_ENDFLAG;
	}
	else
	{
		d = rlen | ICM456X_I2CM_COMMAND_CH_SEL_ID1 |
			ICM456X_I2CM_COMMAND_R_W_RD_W_AD | ICM456X_I2CM_COMMAND_ENDFLAG;
	}
	icm->Write(regaddr, &d, 1);

	// Execute
	regaddr = ICM456X_I2CM_CONTROL_REG;
	d = ICM456X_I2CM_CONTROL_I2CM_GO;// | ICM456X_I2CM_CONTROL_I2CM_SPEED_STD;
	icm->Write(regaddr, &d, 1);

	int timout = 1000;

	while (timout-- > 0)
	{
		regaddr = ICM456X_I2CM_STATUS_REG;
		icm->Read(regaddr, &d, 1);

		if (d & ICM456X_I2CM_STATUS_I2CM_DONE)
		{
			regaddr = ICM456X_I2CM_RD_DATA0_REG;
			cnt = icm->Read(regaddr, pBuff, rlen);
			break;
		}
	}
#endif

	return cnt;
}

int AuxIntrfIcm456x::Write(uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pData, int DataLen)
{
	int cnt = 0;

#if 0
	if (pAdCmd)
	{
		StartTx(DevAddr);
		TxData(pAdCmd, AdCmdLen);
		StopTx();
	}
#else
	AgIcm456x *icm = (AgIcm456x*)vIntrfData.pDevData;
	uint16_t regaddr = ICM456X_I2CM_DEV_PROFILE1_REG;
	uint8_t d = DevAddr;
	uint8_t dd[AdCmdLen + DataLen];
	int len = min(DataLen + AdCmdLen, 6);

	memcpy(dd, pAdCmd, AdCmdLen);

	if (pData)
	{
		memcpy(dd + AdCmdLen, pData, DataLen);
	}

	// Set device address
	icm->Write(regaddr, &d, 1);

	regaddr = ICM456X_I2CM_COMMAND_0_REG;
	d = len | ICM456X_I2CM_COMMAND_CH_SEL_ID1 | ICM456X_I2CM_COMMAND_R_W_WR | ICM456X_I2CM_COMMAND_ENDFLAG;
	icm->Write(regaddr, &d, 1);

	regaddr = ICM456X_I2CM_WR_DATA0_REG;
	cnt = 0;
//	cnt = icm->Write(regaddr, dd, len);
	for (int i = 0; i < len; i++)
	{
		cnt += icm->Write(regaddr + i, &dd[i], 1);
	}

	// Execute
	regaddr = ICM456X_I2CM_CONTROL_REG;
	d = ICM456X_I2CM_CONTROL_I2CM_GO;// | ICM456X_I2CM_CONTROL_I2CM_SPEED_STD;
	icm->Write(regaddr, &d, 1);

	int timout = 1000;

	while (timout-- > 0)
	{
		regaddr = ICM456X_I2CM_STATUS_REG;
		icm->Read(regaddr, &d, 1);

		if (d & ICM456X_I2CM_STATUS_I2CM_DONE)
		{
			break;
		}
	}

#endif

	return cnt;
}

#if 0
bool AgIcm456x::AuxIntrfInit(void)
{
	bool retval = false;

	vAuxIntrf.DevIntrf.pDevData = this;
	vAuxIntrf.DevIntrf.MaxTrxLen = 32;
	vAuxIntrf.DevIntrf.EnCnt = 1;
	vAuxIntrf.DevIntrf.Type = DEVINTRF_TYPE_I2C;
	vAuxIntrf.DevIntrf.bDma = false;
	vAuxIntrf.DevIntrf.bIntEn = false;
	vAuxIntrf.DevIntrf.bTxReady = true;
	vAuxIntrf.DevIntrf.bNoStop = false;
	vAuxIntrf.DevIntrf.Disable = AuxIntrfDisable;
	vAuxIntrf.DevIntrf.Enable = AuxIntrfEnable;
	vAuxIntrf.DevIntrf.PowerOff = AuxIntrfPowerOff;
	vAuxIntrf.DevIntrf.GetRate = AuxIntrfGetRate;
	vAuxIntrf.DevIntrf.SetRate = AuxIntrfSetRate;
	vAuxIntrf.DevIntrf.StartRx = AuxIntrfStartRx;
	vAuxIntrf.DevIntrf.StopRx = AuxIntrfStopRx;
	vAuxIntrf.DevIntrf.RxData = AuxIntrfRxData;
	vAuxIntrf.DevIntrf.StartTx = AuxIntrfStartTx;
	vAuxIntrf.DevIntrf.TxData = AuxIntrfTxData;
	vAuxIntrf.DevIntrf.StopTx = AuxIntrfStopTx;
	vAuxIntrf.DevIntrf.Reset = AuxIntrfReset;
	vAuxIntrf.DevIntrf.GetHandle = AuxIntrfGetHandle;
	vAuxIntrf.DevIntrf.IntPrio = 0;
	vAuxIntrf.DevIntrf.EvtCB = nullptr;
	vAuxIntrf.DevIntrf.MaxRetry = 5;

	atomic_flag_clear(&vAuxIntrf.DevIntrf.bBusy);

	return retval;
}

uint32_t AgIcm456x::AuxIntrfGetRate(DevIntrf_t * const pDev)
{

}

uint32_t AgIcm456x::AuxIntrfSetRate(DevIntrf_t * const pDev, uint32_t RateHz)
{

}

bool AgIcm456x::AuxIntrfStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{

}

int AgIcm456x::AuxIntrfRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{

}

void AgIcm456x::AuxIntrfStopRx(DevIntrf_t * const pDev)
{

}

bool AgIcm456x::AuxIntrfStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{

}

int AgIcm456x::AuxIntrfTxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{

}

void AgIcm456x::AuxIntrfStopTx(DevIntrf_t * const pDev)
{

}
#endif
