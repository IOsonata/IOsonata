/**-------------------------------------------------------------------------
@file	agm_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

@author	Hoang Nguyen Hoan
@date	Nov. 5, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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
#include "istddef.h"
#include "convutil.h"
#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_icm20948.h"

static const uint8_t dmp3_image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

bool AgmIcm20948::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	//if (vbInitialized)
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	uint16_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;//ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	uint8_t lpconfig = ICM20948_LP_CONFIG_ACCEL_CYCLE | ICM20948_LP_CONFIG_GYRO_CYCLE;

	Interface(pIntrf);
	DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN;

		//lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;
	}

	vCurrBank = -1;

	// Read chip id
	regaddr = ICM20948_WHO_AM_I;
	d = Read8((uint8_t*)&regaddr, 2);

	if (d != ICM20948_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	msDelay(500);

	regaddr = ICM20948_USER_CTRL;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_PWR_MGMT_1;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = ICM20948_PWR_MGMT_2;
	Write8((uint8_t*)&regaddr, 2, 0x3f);

	// Init master I2C interface

	//regaddr = ICM20948_FIFO_EN_1;
	//Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_1_SLV_0_FIFO_EN);

	//regaddr = ICM20948_LP_CONFIG;
	//Write8((uint8_t*)&regaddr, 2, lpconfig);

/*
	regaddr = ICM20948_I2C_MST_CTRL;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);
*/
	regaddr = ICM20948_ODR_ALIGN_EN;
	Write8((uint8_t*)&regaddr, 2, ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN);

	vbInitialized  = true;

	return true;
}

bool AccelIcm20948::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	AccelSensor::Range(ICM20948_ACC_MAX_RANGE);

	SamplingFrequency(CfgData.Freq);
	Scale(CfgData.Scale);
	FilterFreq(CfgData.FltrFreq);

	regaddr = ICM20948_PWR_MGMT_2;
	d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK;

	if (Value < 4)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G;
		Value = 2;
	}
	else if (Value < 8)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G;
		Value = 4;
	}
	else if (Value < 16)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_8G;
		Value = 8;
	}
	else
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_16G;
		Value = 16;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return AccelSensor::Scale(Value);
}

uint32_t AccelIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])

	// Find closes DIV value
	uint16_t div = 0;
	int diff = 1200;

	for (int i = 0; i < 0x1000; i++)
	{
		uint32_t f = 1125 / (1 + i);
		int df = Freq > f ? Freq - f : f - Freq;

		if (df < diff)
		{
			diff = df;
			div = i;
			AccelSensor::vSampFreq = f;
		}
	}

	uint16_t regaddr = ICM20948_ACCEL_SMPLRT_DIV_1;
	Write8((uint8_t*)&regaddr, 2, div >> 8);

	regaddr = ICM20948_ACCEL_SMPLRT_DIV_2;
	Write8((uint8_t*)&regaddr, 2, div & 0xFF);

	return AccelSensor::vSampFreq;
}

uint32_t AccelIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_MASK | ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE);

	if (Freq == 0)
	{
		Freq = 1248000;
	}
	else if (Freq < 11000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (6 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 8300;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (5 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 17000;
	}
	else if (Freq < 50000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (4 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 34400;
	}
	else if (Freq < 110000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (3 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 68800;
	}
	else if (Freq < 240000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (2 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 136000;
	}
	else if (Freq < 470000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (1 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 265000;
	}
	else if (Freq < 1000000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (7 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 499000;
	}
	else
	{
		Freq = 1248000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return AccelSensor::FilterFreq(Freq);
}

bool GyroIcm20948::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	SamplingFrequency(CfgData.Freq);

	Sensitivity(CfgData.Sensitivity);

	uint16_t regaddr = ICM20948_PWR_MGMT_2;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK;

	if (Value < 500)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_250DPS;
		Value = 250;
	}
	else if (Value < 1000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_500DPS;
		Value = 500;
	}
	else if (Value < 2000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS;
		Value = 1000;
	}
	else
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_2000DPS;
		Value = 2000;
	}

	return GyroSensor::Sensitivity(Value);
}

uint32_t GyroIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])

	// Find closes DIV value
	uint16_t div = 0;
	int diff = 1200;

	for (int i = 0; i < 0x100; i++)
	{
		uint32_t f = 1100 / (1 + i);
		int df = Freq > f ? Freq - f : f - Freq;

		if (df < diff)
		{
			diff = df;
			div = i;
			GyroSensor::vSampFreq = f;
		}
	}

	uint16_t regaddr = ICM20948_GYRO_SMPLRT_DIV;
	Write8((uint8_t*)&regaddr, 2, div);

	return GyroSensor::vSampFreq;
}

uint32_t GyroIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_MASK | ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE);

	if (Freq == 0)
	{
		Freq = 12316000;
	}
	else if (Freq < 11000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (6 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 8900;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (5 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 17800;
	}
	else if (Freq < 50000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (4 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 35900;
	}
	else if (Freq < 110000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (3 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 73300;
	}
	else if (Freq < 150000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (2 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 154300;
	}
	else if (Freq < 190000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (1 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 187600;
	}
	else if (Freq < 360000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (0 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 229800;
	}
	else if (Freq < 1000000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (7 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 376500;
	}
	else
	{
		Freq = 12316000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return GyroSensor::FilterFreq(Freq);
}

bool MagIcm20948::Init(const MAGSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	msDelay(200);

	regaddr = ICM20948_I2C_MST_CTRL;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);

	return MagAk09916::Init(CfgData, pIntrf, pTimer);
}

bool AgmIcm20948::Enable()
{
	return true;
}

void AgmIcm20948::Disable()
{
	uint16_t regaddr;
	uint8_t d;

	// Disable Accel & Gyro
	regaddr = ICM20948_PWR_MGMT_2;
	d = ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK | ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_LP_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_PWR_MGMT_1;
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::Reset()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1;

	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_DEVICE_RESET);
}

bool AgmIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr;

	if (bEnable == true)
	{
		Reset();

		msDelay(2000);
	}
	else
	{
//	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8((uint8_t*)&regaddr, 2, 0);

//	    regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8((uint8_t*)&regaddr, 2, 0);
	}

	return true;
}
/*
// Accel low pass frequency
uint32_t AgmIcm20948::FilterFreq(uint32_t Freq)
{
	return AccelSensor::FilterFreq();
}

// Accel scale
uint16_t AgmIcm20948::Scale(uint16_t Value)
{
	return AccelSensor::Scale();
}

// Gyro scale
uint32_t AgmIcm20948::Sensitivity(uint32_t Value)
{

	return GyroSensor::Sensitivity();
}
*/
bool AgmIcm20948::UpdateData()
{
	bool res;// = MagIcm20948::UpdateData();
	uint16_t regaddr = ICM20948_ACCEL_XOUT_H;
	int8_t d[20];

	regaddr = ICM20948_DATA_RDY_STATUS;
	d[0] = Read8((uint8_t*)&regaddr, 2);

	if (d[0] & 1)
	{
		regaddr = ICM20948_ACCEL_XOUT_H;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)d, 20);

		AccelSensor::vData.X = ((int16_t)d[0] << 8) | d[1];
		AccelSensor::vData.Y = ((int16_t)d[2] << 8) | d[3];
		AccelSensor::vData.Z = ((int16_t)d[4] << 8) | d[5];
		GyroSensor::vData.X = ((int16_t)d[6] << 8) | d[7];
		GyroSensor::vData.Y = ((int16_t)d[8] << 8) | d[9];
		GyroSensor::vData.Z = ((int16_t)d[10] << 8) | d[11];


		// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
		int16_t t = ((int16_t)d[12] << 8) | d[13];
		TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | d[13]) * 100 / 33387 + 2100;
		printf("Temp : %d %d\r\n", t, TempSensor::vData.Temperature);

		MagSensor::vData.X = ((int16_t)d[14] << 8) | d[15];
		MagSensor::vData.Y = ((int16_t)d[16] << 8) | d[17];
		MagSensor::vData.Z = ((int16_t)d[18] << 8) | d[19];
	}

	return res;
}

int AgmIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(pCmdAddr[1]);
		CmdAddrLen--;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(pCmdAddr[1]);
		CmdAddrLen--;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

int AgmIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = ICM20948_USER_CTRL;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

#if 1
		uint8_t d[4];

		regaddr = ICM20948_I2C_SLV0_ADDR;

		d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			d[2] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
			d[1] += cnt;
		}
#else
		regaddr = ICM20948_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

		regaddr = ICM20948_I2C_SLV0_REG;
		Write8((uint8_t*)&regaddr, 2, *pCmdAddr);

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (1 & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK));

		regaddr = ICM20948_USER_CTRL;
		Write8((uint8_t*)&regaddr, 2, userctrl);

		msDelay(100);

		Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);
		regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
		int cnt = Read((uint8_t*)&regaddr, 2, pBuff, 1);

		BuffLen -= cnt;

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, 0);


#endif
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int AgmIcm20948::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = ICM20948_USER_CTRL;
		uint8_t d[8];
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

		regaddr = ICM20948_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR);

		d[0] = *pCmdAddr;
		d[1] = 1 | ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN;	// Length : Write is done 1 byte at a time

		while (DataLen > 0)
		{
			d[2] = *pData;

			regaddr = ICM20948_I2C_SLV0_REG;
			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			d[0]++;
			pData++;
			DataLen--;
			retval++;
		}
	}
	else
	{
		retval = vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	return retval;
}

bool AgmIcm20948::SelectBank(uint8_t BankNo)
{
	if (BankNo > 3 || vCurrBank == BankNo)
		return false;

	vCurrBank = BankNo;

	uint8_t regaddr = ICM20948_REG_BANK_SEL;

	return Write8(&regaddr, 1, (BankNo << ICM20948_REG_BANK_SEL_USER_BANK_BITPOS) & ICM20948_REG_BANK_SEL_USER_BANK_MASK);
}

void AgmIcm20948::IntHandler()
{
	uint16_t regaddr = 0;
	uint8_t d;

	d = Read8((uint8_t*)&regaddr, 2);
//	if (d & MPU9250_AG_INT_STATUS_RAW_DATA_RDY_INT)
	{
		UpdateData();
	}
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
bool AgmIcm20948::Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	uint16_t regaddr = ICM20948_PWR_MGMT_1;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	// Enable temperature sensor
	d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}


