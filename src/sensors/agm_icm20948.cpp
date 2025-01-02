/**-------------------------------------------------------------------------
@file	agm_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

@author	Hoang Nguyen Hoan
@date	Nov. 5, 2018

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
#include "istddef.h"
#include "convutil.h"
#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_icm20948.h"

bool AgmIcm20948::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
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
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN;

		//lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;
	}

	vCurrBank = -1;

	// Read chip id
	regaddr = ICM20948_WHO_AM_I_REG;
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

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, userctrl);

/*
	regaddr = ICM20948_I2C_MST_CTRL;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);
*/
	regaddr = ICM20948_ODR_ALIGN_EN_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN);

	// ICM20948 has only 1 interrupt pin. Don't care the value
	if (Inter)
	{
		regaddr = ICM20948_INT_PIN_CFG_REG;

		if (IntPol == DEVINTR_POL_HIGH)
		{
			d = ICM20948_INT_PIN_CFG_INT_ANYRD_2CLEAR;
		}
		else
		{
			d = ICM20948_INT_PIN_CFG_INT_ANYRD_2CLEAR | ICM20948_INT_PIN_CFG_INT1_ACTL;
		}
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_REG;
		d = ICM20948_INT_ENABLE_WOM_INT_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_1_REG;
		d = ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		/*
		regaddr = ICM20948_INT_ENABLE_2;
		d = 1;
		Write8((uint8_t*)&regaddr, 2, d);
		regaddr = ICM20948_INT_ENABLE_3;
		d = 1;
		Write8((uint8_t*)&regaddr, 2, d);
		*/
	}

	vbInitialized  = true;

	return true;
}

bool AccelIcm20948::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_ACC_ADC_RANGE);

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
	Write8((uint8_t*)&regaddr, 2, d);

//	regaddr = ICM20948_FIFO_EN_2;
//	d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_2_ACCEL_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
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

	uint32_t div = (1125000 + (Freq >>1))/ Freq - 1;

	uint16_t regaddr = ICM20948_ACCEL_SMPLRT_DIV_1_REG;
	Write8((uint8_t*)&regaddr, 2, div >> 8);

	regaddr = ICM20948_ACCEL_SMPLRT_DIV_2_REG;
	Write8((uint8_t*)&regaddr, 2, div & 0xFF);

	return AccelSensor::SamplingFrequency(1125000 / (1 + div));
}

uint32_t AccelIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
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

bool GyroIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	SamplingFrequency(Cfg.Freq);

	vData.Range = Range(ICM20948_GYRO_ADC_RANGE);
	Sensitivity(Cfg.Sensitivity);

	uint16_t regaddr = ICM20948_PWR_MGMT_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;
	Write8((uint8_t*)&regaddr, 2, d);

//	regaddr = ICM20948_FIFO_EN_2;
//	d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_2_GYRO_X_FIFO_EN |
//		ICM20948_FIFO_EN_2_GYRO_Y_FIFO_EN | ICM20948_FIFO_EN_2_GYRO_Z_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
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

	uint32_t div = (1100000 + (Freq >> 1)) / Freq - 1;
	uint16_t regaddr = ICM20948_GYRO_SMPLRT_DIV_REG;
	Write8((uint8_t*)&regaddr, 2, div);

	return GyroSensor::SamplingFrequency(1100000 / (div + 1));
}

uint32_t GyroIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
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

bool MagIcm20948::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	msDelay(200);

	regaddr = ICM20948_I2C_MST_CTRL_REG;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	if (MagAk09916::Init(Cfg, pIntrf, pTimer) == false)
	{
		return false;
	}

//	regaddr = ICM20948_FIFO_EN_1;
//	d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_1_SLV_0_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);

	return true;
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
	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK | ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_LP_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_PWR_MGMT_1_REG;
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::Reset()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;

	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_DEVICE_RESET);
	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);

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
	bool res = MagIcm20948::UpdateData();
	uint16_t regaddr = ICM20948_ACCEL_XOUT_H_REG;
	uint8_t d[20];
	uint64_t t;
	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	regaddr = ICM20948_INT_STATUS_1_REG;//ICM20948_DATA_RDY_STATUS;
	d[0] = Read8((uint8_t*)&regaddr, 2);

	if (d[0] & 1)
	{
		regaddr = ICM20948_ACCEL_XOUT_H_REG;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)d, 14);

		AccelSensor::vData.Timestamp = t;
		AccelSensor::vData.X = ((int16_t)(d[0] << 8) | d[1]);
		AccelSensor::vData.Y = ((int16_t)(d[2] << 8) | d[3]);
		AccelSensor::vData.Z = ((int16_t)(d[4] << 8) | d[5]);
		GyroSensor::vData.Timestamp = t;
		GyroSensor::vData.X = ((int16_t)(d[6] << 8) | d[7]);
		GyroSensor::vData.Y = ((int16_t)(d[8] << 8) | d[9]);
		GyroSensor::vData.Z = ((int16_t)(d[10] << 8) | d[11]);


		// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
		int16_t t = ((int16_t)d[12] << 8) | d[13];
		TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | d[13]) * 100 / 33387 + 2100;
		//printf("Temp : %d %d\r\n", t, TempSensor::vData.Temperature);

		res = true;
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
		uint16_t regaddr = ICM20948_USER_CTRL_REG;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

#if 1
		uint8_t d[4];

		regaddr = ICM20948_I2C_SLV0_ADDR_REG;

		d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			d[2] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LEN_MASK);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL_REG;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			regaddr = ICM20948_EXT_SLV_SENS_DATA_00_REG;
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
		uint16_t regaddr = ICM20948_USER_CTRL_REG;
		uint8_t d[8];
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

		regaddr = ICM20948_I2C_SLV0_ADDR_REG;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR);

		d[0] = *pCmdAddr;
		d[1] = 1 | ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN;	// Length : Write is done 1 byte at a time

		while (DataLen > 0)
		{
			d[2] = *pData;

			regaddr = ICM20948_I2C_SLV0_REG_REG;
			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL_REG;
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

	uint8_t regaddr = ICM20948_REG_BANK_SEL_REG;

	return Write8(&regaddr, 1, (BankNo << ICM20948_REG_BANK_SEL_USER_BANK_BITPOS) & ICM20948_REG_BANK_SEL_USER_BANK_MASK);
}

void AgmIcm20948::IntHandler()
{
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t istatus[4];

	int cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);

	//printf("%x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);

	if (istatus[0] & ICM20948_INT_STATUS_I2C_MIST_INT)
	{

	}
	if (istatus[0] & ICM20948_INT_STATUS_DMP_INT1)
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus = Read8((uint8_t*)&regaddr, 2);
//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			printf("distatus %x\n", distatus);
		}
	}
	if (istatus[0] & ICM20948_INT_STATUS_PLL_RDY_INT)
	{

	}

	if (istatus[0] & ICM20948_INT_STATUS_WOM_INT)
	{

	}

	uint32_t drdy = (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT) |
			(istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK) |
			(istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK);
	if (drdy)
	{
		regaddr = ICM20948_FIFO_COUNTH_REG;
		uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		//printf("cnt = %d : ", cnt);
		cnt = EndianCvt16(cnt);

//		printf("%d\n", cnt);
		if (cnt > 2)
		{
			regaddr = ICM20948_FIFO_R_W_REG;
//			uint16_t h = Read16((uint8_t*)&regaddr, 2);
			uint8_t dd[16];
			dd [0] = Read8((uint8_t*)&regaddr, 2);
			dd [1] = Read8((uint8_t*)&regaddr, 2);

//			Read((uint8_t*)&regaddr, 2, dd, 16);
			regaddr = ICM20948_FIFO_COUNTH_REG;
			uint16_t cnt1 = Read16((uint8_t*)&regaddr, 2);
			//printf("cnt = %d : ", cnt);
			cnt1 = EndianCvt16(cnt1);
			printf("%d Fifo header=%x %x %d\n", cnt, dd[0], dd[1], cnt1);

		}
	}

#if 1
	if (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
	{
		UpdateData();
	}

	if (istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK)
	{
		printf("FIFO ovr Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

		printf("cnt = %d\r\n", cnt);
	}

	if (istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK)
	{
		printf("WM Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

		printf("cnt = %d\r\n", cnt);

#if 0
		for (int i = 0; i < cnt; i++)
		{
			regaddr = ICM20948_FIFO_R_W;
			uint8_t d = Read8((uint8_t*)&regaddr, 2);
			printf("Fifo d=%x\n", d);
		}
#else
		uint8_t dd[cnt];
		memset(dd, 0, cnt);
		regaddr = ICM20948_FIFO_R_W_REG;
		int l = Read((uint8_t*)&regaddr, 2, dd, cnt);
		printf("l:%d %x %x %x %x\n", l, dd[0], dd[1], dd[2], dd[3]);
#endif

	}
#endif

	regaddr = ICM20948_INT_STATUS_REG;
	cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);

//	printf("x- %x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);
	//istatus[0] = istatus[1] = istatus[2] = istatus[3] = 0;
	//cnt = Write((uint8_t*)&regaddr, 2, istatus, 4);
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
bool AgmIcm20948::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	// Enable temperature sensor
	d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

bool AgmIcm20948::InitDMP(uint16_t DmpStartAddr, const uint8_t * const pDmpImage, int Len)
{
	if (pDmpImage == NULL || Len == 0)
		return false;

	// Disable DMP & FIFO before FIFO can be reseted and DMP firmware loaded
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	Write8((uint8_t*)&regaddr,	2, d);

	// Reset FIFO
	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);

	// load external image
	bool res = UploadDMPImage(pDmpImage, Len);

	if (res)
	{
		DmpStartAddr = EndianCvt16(DmpStartAddr);

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH_REG;
		Write16((uint8_t*)&regaddr, 2, DmpStartAddr);

		regaddr = ICM20948_USER_CTRL_REG;
		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		return true;
	}

	return false;
}

bool AgmIcm20948::UploadDMPImage(const uint8_t * const pDmpImage, int Len)
{
	int len = Len, l = 0;
	uint8_t *p = (uint8_t*)pDmpImage;
	uint16_t regaddr;
	uint16_t memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;

	regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t pwrstate;

	pwrstate = Read8((uint8_t*)&regaddr, 2);

	// make sure it is on full power
	Write8((uint8_t*)&regaddr, 2, pwrstate & ~(ICM20948_PWR_MGMT_1_LP_EN | ICM20948_PWR_MGMT_1_SLEEP));

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

	l = min(len, ICM20948_DMP_MEM_BANK_SIZE - (memaddr % ICM20948_DMP_MEM_BANK_SIZE));

	while (len > 0)
	{
		regaddr = ICM20948_DMP_MEM_RW_REG;
		l = Write((uint8_t*)&regaddr, 2, p, l);

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_DMP_MEM_BANK_SIZE);
	}

	len = Len;
	p = (uint8_t*)pDmpImage;
	memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;

	// Verify

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);
	l = min(len, ICM20948_DMP_MEM_BANK_SIZE - (memaddr % ICM20948_DMP_MEM_BANK_SIZE));

	while (len > 0)
	{
		uint8_t m[ICM20948_DMP_MEM_BANK_SIZE];

		regaddr = ICM20948_DMP_MEM_RW_REG;
		Read((uint8_t*)&regaddr, 2, m, l);

		if (memcmp(p, m, l) != 0)
		{
			return false;
		}

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_DMP_MEM_BANK_SIZE);
	}

	// Restore power state
	Write8((uint8_t*)&regaddr, 2, pwrstate);

	return true;
}
