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

#include "convutil.h"
#include "sensors/ag_icm456x.h"


bool AccelIcm456x::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
	{
		return false;
	}

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

	return Freq;
}

uint8_t AccelIcm456x::Scale(uint8_t Value)
{
	uint8_t regaddr = ICM456X_ACCEL_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_ACCEL_CONFIG0_UI_FS_SEL_MASK;

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
	}

	Write8(&regaddr, 1, d);

	return Value;
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
	return 0;
}

bool AccelIcm456x::Enable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_ACCEL_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_ACCEL_MODE_LOW_NOISE;
	Write8(&regaddr, 1, d);

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

	return Freq;
}

uint32_t GyroIcm456x::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = ICM456X_GYRO_CONFIG0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_GYRO_CONFIG0_UI_FS_SEL_MASK;

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
	}

	Write8(&regaddr, 1, d);

	return Value;
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
	return 0;
}
bool GyroIcm456x::Enable()
{
	uint8_t regaddr = ICM456X_PWR_MGMT0_REG;
	uint8_t d = Read8(&regaddr, 1) & ~ICM456X_PWR_MGMT0_GYRO_MODE_MASK;

	d |= ICM456X_PWR_MGMT0_GYRO_MODE_LOW_NOISE;
	Write8(&regaddr, 1, d);

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
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool TempIcm456x::Enable()
{
	return false;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * @return	None
 */
void TempIcm456x::Disable()
{

}

AgIcm456x::AgIcm456x()
{
	memset(vbSensorEnabled, 0, sizeof(vbSensorEnabled));
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO;
}

bool AgIcm456x::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return true;
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

	Write16(&regaddr, 1, wm);

	regaddr = ICM456X_FIFO_CONFIG2_REG;
	Write8(&regaddr, 1, ICM456X_FIFO_CONFIG2_WR_WM_GT_TH);

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

	return true;
}

bool AgIcm456x::Enable()
{
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
	uint16_t istatus;

	if (InterruptId() == 2)
	{
		regaddr = ICM456X_INT2_STATUS0_REG;
	}

	istatus  = Read16(&regaddr, 1);

	if (istatus & ICM456X_INT_STATUS_DRDY)
	{
		printf("Data Ready\r\n");
	}

	if (istatus & ICM456X_INT_STATUS_FIFO_THS)
	{
		printf("fifo ths\r\n");
	}
}

bool AgIcm456x::UpdateData()
{
	return false;
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

	Write16(&reg, 1, EndianCvt16(IRegAddr));

	reg = ICM456X_IREG_DATA_REG;

	while (DataLen > 0)
	{
		Write8(&reg, 1, *pData);

		pData++;
		DataLen--;
		cnt++;
	}

	return cnt;
}

//int AgIcm45686::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
//{
//	return 0;
//}
