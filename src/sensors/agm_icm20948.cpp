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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"

#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"

#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_icm20948.h"

static inv_icm20948_t icm_device;

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
	usDelay(500000);

	regaddr = ICM20948_USER_CTRL;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_PWR_MGMT_1;
	Write8((uint8_t*)&regaddr, 2, 1);

	regaddr = ICM20948_PWR_MGMT_2;
	Write8((uint8_t*)&regaddr, 2, 0x7f);

	// Init master I2C interface

	regaddr = ICM20948_FIFO_EN_1;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_1_SLV_0_FIFO_EN);

	//regaddr = ICM20948_LP_CONFIG;
	//Write8((uint8_t*)&regaddr, 2, lpconfig);


	regaddr = ICM20948_I2C_MST_CTRL;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);

#if 0
	regaddr = ICM20948_AK09916_WIA1;
	uint8_t x[2];
	Read(AK09916_I2C_ADDR1, (uint8_t*)&regaddr, 1, x, 2);

	if (x[0] != ICM20948_AK09916_WIA1_ID)
	{
		return false;
	}
#endif

	vbInitialized  = true;

	return true;
}

bool AccelIcm20948::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	SamplingFrequency(CfgData.Freq);
	Scale(CfgData.Scale);
	FilterFreq(CfgData.FltrFreq);

	msDelay(100);

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	return 0;
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
	return Freq;
}

bool GyroIcm20948::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	SamplingFrequency(CfgData.Freq);

	Sensitivity(CfgData.Sensitivity);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	return 0;

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
	return Freq;
}

bool MagIcm20948::Init(const MAGSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t regaddr;
	uint8_t d[4];

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	msDelay(200);

	return MagAk09916::Init(CfgData, pIntrf, pTimer);
}

//uint32_t MagIcm20948::SamplingFrequency(uint32_t Freq)
//{
//	return Freq;
//}

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


#if 0
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_2;

Reset();
msDelay(2000);

	regaddr = MPU9250_AG_PWR_MGMT_1;
	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
						MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY);

	//return;

	regaddr = MPU9250_AG_USER_CTRL;
	Write8(&regaddr, 1, MPU9250_AG_USER_CTRL_I2C_MST_EN);

	// Disable Mag
	regaddr = MPU9250_MAG_CTRL1;
	uint8_t d = 0;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);

	// Disable Accel Gyro
	Write8(&regaddr, 1,
		 MPU9250_AG_PWR_MGMT_2_DIS_ZG | MPU9250_AG_PWR_MGMT_2_DIS_YG | MPU9250_AG_PWR_MGMT_2_DIS_XG |
		 MPU9250_AG_PWR_MGMT_2_DIS_ZA | MPU9250_AG_PWR_MGMT_2_DIS_YA | MPU9250_AG_PWR_MGMT_2_DIS_XA);

	regaddr = MPU9250_AG_PWR_MGMT_1;
	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
						MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY);
#endif
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

bool AgmIcm20948::UpdateData()
{
	MagIcm20948::UpdateData();
#if 0
	uint8_t regaddr = MPU9250_AG_FIFO_COUNT_H;//MPU9250_AG_ACCEL_XOUT_H;
	int8_t d[20];
	int32_t val;

	Read(&regaddr, 1, (uint8_t*)d, 2);
	val = ((d[0] & 0xF) << 8) | d[1];

	//printf("%d\r\n", val);

	if (val > 0)
	{
		int cnt = min(val, 18);
		regaddr = MPU9250_AG_FIFO_R_W;
	//	Read(&regaddr, 1, d, cnt);
	}

	vSampleCnt++;

	if (vpTimer)
	{
		vSampleTime = vpTimer->uSecond();
	}

	regaddr = MPU9250_AG_ACCEL_XOUT_H;
	Read(&regaddr, 1, (uint8_t*)d, 6);

	int32_t scale =  AccelSensor::Scale();
	val = (((((int32_t)d[0] << 8) | d[1]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.X = val;
	val = (((((int32_t)d[2] << 8) | d[3]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.Y = val;
	val = (((((int32_t)d[4] << 8) | d[5]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.Z = val;
	AccelSensor::vData.Timestamp = vSampleTime;

	regaddr = MPU9250_AG_GYRO_XOUT_H;

	Read(&regaddr, 1, (uint8_t*)d, 6);

	val = ((((int16_t)d[0] << 8) | d[1]) << 8) / GyroSensor::vSensitivity;
	GyroSensor::vData.X = val;
	val = ((((int16_t)d[2] << 8) | d[3]) << 8) / GyroSensor::vSensitivity;
	GyroSensor::vData.Y = val;
	val = ((((int32_t)d[4] << 8) | d[5]) << 8L) / GyroSensor::vSensitivity;
	GyroSensor::vData.Z = val;
	GyroSensor::vData.Timestamp = vSampleTime;

	regaddr = MPU9250_MAG_ST1;
	Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, (uint8_t*)d, 8);

	if (d[14] & MPU9250_MAG_ST1_DRDY)
	{
		val = (((int16_t)d[0]) << 8L) | d[1];
		val += (val * vMagSenAdj[0]) >> 8L;
		MagSensor::vData.X = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		val = (((int16_t)d[2]) << 8) | d[3];
		val += (val * vMagSenAdj[1]) >> 8L;
		MagSensor::vData.Y = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		val = (((int16_t)d[4]) << 8) | d[5];
		val += (val * vMagSenAdj[2]) >> 8L;
		MagSensor::vData.Z = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		MagSensor::vData.Timestamp = vSampleTime;
	}
#endif
	return true;
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
		uint16_t regaddr;
		uint8_t userctrl;
		uint8_t lpconfig;

		regaddr = ICM20948_USER_CTRL;
		//Write8(&regaddr, 1, ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN);
		userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

		regaddr = ICM20948_LP_CONFIG;
		lpconfig = Read8((uint8_t*)&regaddr, 2);

		//Write8((uint8_t*)&regaddr, 2, 0);

#if 1
		uint8_t d[8];


//		regaddr = ICM20948_I2C_SLV0_CTRL;
//		Write8(&regaddr, 1, 0);

		regaddr = ICM20948_I2C_SLV0_ADDR;

		//d[0] = ICM20948_I2C_SLV0_ADDR;
		d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			SelectBank(3);

			d[2] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			//usDelay(500 + (cnt << 4));
			msDelay(100);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
		}
		regaddr = ICM20948_LP_CONFIG;

		//Write8((uint8_t*)&regaddr, 2, lpconfig);
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
		uint16_t regaddr;
		uint8_t d[8];

		d[0] = ICM20948_I2C_SLV0_ADDR & 0xff;
		d[1] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR;
		d[2] = *pCmdAddr;
		d[3] = DataLen & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK;

		while (DataLen > 0)
		{
			regaddr = ICM20948_I2C_SLV0_DO;
			Write8((uint8_t*)&regaddr, 2, *pData);

			Write(d, 4, NULL, 0);

			d[2]++;
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
	uint16_t regaddr = 0;//MPU9250_AG_INT_STATUS;
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

	uint16_t regaddr = ICM20948_PWR_MGMT_1_DEVICE_RESET;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	// Enable temperature sensor
	d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}


