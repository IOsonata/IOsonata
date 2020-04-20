/**-------------------------------------------------------------------------
@file	am_lsm303agr.cpp

@brief	Implementation of ST LSM303AGR accel, mag sensor

This device is a combination of 2 independent entities.  Therefore the
implementation consists of 2 independent objects.


@author	Hoang Nguyen Hoan
@date	Sept. 18, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include <stdint.h>

#include "coredev/spi.h"
#include "sensors/am_lsm303agr.h"

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
bool AcceLsm303agr::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;///*MPU9250_AG_USER_CTRL_FIFO_EN | MPU9250_AG_USER_CTRL_DMP_EN |*/ MPU9250_AG_USER_CTRL_I2C_MST_EN;
	uint8_t mst = 0;

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || Cfg.DevAddr != LSM303AGR_ACCEL_I2C_DEVADDR)
	{
		regaddr = LSM303AGR_CTRL_REG4_A_REG;
		Write8(&regaddr, 1, LSM303AGR_CTRL_REG4_A_SPI_3WIRE_ENABLE);
	}

	// Read chip id
	regaddr = LSM303AGR_WHO_AM_I_A_REG;
	d = Read8(&regaddr, 1);

	if (d != LSM303AGR_WHO_AM_I_A_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	Scale(Cfg.Scale);
	SamplingFrequency(Cfg.Freq);

	return true;
}

bool AcceLsm303agr::Init(const TEMPSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return true;
}

uint32_t AcceLsm303agr::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr;
	uint32_t r = (1 << 12) - 1;
	uint32_t f = 0;
	uint8_t ctrl = 0;
	uint8_t ctrl4 = 0;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	ctrl = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG1_A_ODR_MASK | LSM303AGR_CTRL_REG1_A_LPEN);

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	ctrl4 = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG4_A_HR);


	if (Freq < 2000)
	{
		f = 1000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 15000)
	{
		f = 10000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_10HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 35000)
	{
		f = 25000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_25HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 70000)
	{
		f = 50000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_50HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 150000)
	{
		f = 100000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_100HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 300000)
	{
		f = 200000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_200HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 700000)
	{
		f = 400000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_400HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 1400000)
	{
		f = 1344000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1344_5376HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 2500000)
	{
		f = 1620000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1620HZ | LSM303AGR_CTRL_REG1_A_LPEN;
		r = 255;
	}
	else
	{
		f = 5376000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1344_5376HZ | LSM303AGR_CTRL_REG1_A_LPEN;
		r = 255;
	}

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	Write8(&regaddr, 1, ctrl4);

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	Write8(&regaddr, 1, ctrl);

	AccelSensor::Range(r);

	return AccelSensor::SamplingFrequency(f);
}

uint8_t AcceLsm303agr::Scale(uint8_t Value)
{
	uint8_t regaddr;
	uint8_t d;
	uint32_t f = 0;
	uint8_t ctrl = 0;
	uint8_t g = 0;

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	ctrl = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG4_A_FS_MASK);

	if (Value < 3)
	{
		g = 2;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_2G;
	}
	else if (Value < 6)
	{
		g = 4;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_4G;
	}
	else if (Value < 12)
	{
		g = 8;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_8G;
	}
	else
	{
		g = 16;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_16G;
	}

	Write8(&regaddr, 1, ctrl);

	return AccelSensor::Scale(g);
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
uint32_t AcceLsm303agr::FilterFreq(uint32_t Freq)
{
	return AccelSensor::FilterFreq(Freq);
}

bool AcceLsm303agr::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | LSM303AGR_CTRL_REG1_A_XEN | LSM303AGR_CTRL_REG1_A_YEN | LSM303AGR_CTRL_REG1_A_ZEN);

	return true;
}
void AcceLsm303agr::Disable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	d = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG1_A_XEN | LSM303AGR_CTRL_REG1_A_YEN | LSM303AGR_CTRL_REG1_A_ZEN);
	Write8(&regaddr, 1, d);
}

void AcceLsm303agr::Reset()
{

}

void AcceLsm303agr::PowerOff()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	Write8(&regaddr, 1, 0);
}

/**
 * @brief	Read device's register/memory block.
 *
 * This default implementation sets bit 7 of the Cmd/Addr byte for SPI read access as most
 * devices work this way on SPI interface. Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior reading data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pBuff		: Data buffer container
 * @param	BuffLen		: Data buffer size
 *
 * @return	Actual number of bytes read
 */
int AcceLsm303agr::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_3WIRE);
	}

	int retval = Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_NORMAL);
	}

	return retval;
}

/**
 * @brief	Write to device's register/memory block
 *
 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior writing data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pData		: Data buffer to be written to the device
 * @param	DataLen		: Size of data
 *
 * @return	Actual number of bytes written
 */
int AcceLsm303agr::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_3WIRE);
	}

	int retval = Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_NORMAL);
	}

	return retval;
}

void AcceLsm303agr::IntHandler()
{

}

/**
 * @brief	Initialize magnetometer sensor.
 *
 * NOTE : Accelerometer must be initialized first prior to this one.
 *
 * @param 	Cfg		: Accelerometer configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool MagLsm303agr::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer)
{

	return true;
}

