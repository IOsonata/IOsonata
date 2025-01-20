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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "istddef.h"
#include "convutil.h"
#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_icm20948DMP.h"

typedef struct {
	size_t Len;
} FifoDataLen_t;

static const size_t s_FifoDataLenLookup1[] = {
	ICM20948_FIFO_HEADER_ACCEL_SIZE, ICM20948_FIFO_HEADER_GYRO_SIZE,
	ICM20948_FIFO_HEADER_CPASS_SIZE, ICM20948_FIFO_HEADER_ALS_SIZE,
	ICM20948_FIFO_HEADER_QUAT6_SIZE, ICM20948_FIFO_HEADER_QUAT9_SIZE,
	ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE, ICM20948_FIFO_HEADER_GEOMAG_SIZE,
	ICM20948_FIFO_HEADER_PRESSURE_SIZE, ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE,
	ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE, ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE
};
static const size_t s_NbFifoDataLenLookup1 = sizeof(s_FifoDataLenLookup1) / sizeof(size_t);

static const size_t s_FifoDataLenLookup2[] = {
	ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE, ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE,
	ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE, ICM20948_FIFO_HEADER2_FSYNC_SIZE,
	ICM20948_FIFO_HEADER2_PICKUP_SIZE, 0, 0, ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE,
	2
};
static const size_t s_NbFifoDataLenLookup2 = sizeof(s_FifoDataLenLookup2) / sizeof(size_t);

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};
static const int s_DmpImageSize = sizeof(s_Dmp3Image);

bool AgmIcm20948::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);

	vCurrBank = -1;

	// Read chip id
	uint16_t regaddr = ICM20948_WHO_AM_I_REG;
	uint8_t d;

	if (DevAddr == ICM20948_I2C_DEV_ADDR0 || DevAddr == ICM20948_I2C_DEV_ADDR1)
	{
		if (pIntrf->Type() != DEVINTRF_TYPE_I2C)
		{
			// Device address is set to I2C but interface is not
			return false;
		}

		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
		{
			return false;
		}
	}
	else if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		// Interface is I2C but device address is not set.
		// Detect device
		DeviceAddress(ICM20948_I2C_DEV_ADDR0);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
		{
			// Try alternate address
			DeviceAddress(ICM20948_I2C_DEV_ADDR1);
			d = Read8((uint8_t*)&regaddr, 2);
			if (d != ICM20948_WHO_AM_I_ID)
			{
				return false;
			}
		}
	}
	else
	{
		// Not I2C interface
		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
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

	uint8_t userctrl = 0;//ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	uint8_t lpconfig = ICM20948_LP_CONFIG_ACCEL_CYCLE | ICM20948_LP_CONFIG_GYRO_CYCLE;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS;
		lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;
	}


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	msDelay(50);

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_2_DISABLE_ALL);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_ODR_ALIGN_EN_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN);

	// Upload DMP
	//InitDMP(ICM20948_DMP_START_ADDRESS, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);

	ResetDMPCtrlReg();

	// Fifo watermark 80%
	uint16_t val = EndianCvt16(800);
	WriteDMP(ICM20948_DMP_FIFO_WATERMARK, (uint8_t*)&val, 2);

	regaddr = ICM20948_FIFO_CFG_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_CFG_SINGLE);

	// Undocumented value
	regaddr = ICM20948_SINGLE_FIFO_PRIORITY_SEL;
	Write8((uint8_t*)&regaddr, 2, ICM20948_SINGLE_FIFO_PRIORITY_SEL_0XE4);



	// ICM20948 has only 1 interrupt pin. Don't care the value
	if (Inter)
	{
		regaddr = ICM20948_INT_PIN_CFG_REG;

		if (IntPol == DEVINTR_POL_HIGH)
		{
			d = 0;
		}
		else
		{
			d = 0 | ICM20948_INT_PIN_CFG_INT1_ACTL;
		}
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_REG;
		d = ICM20948_INT_ENABLE_DMP_INT1_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_1_REG;
		d = ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN;
		Write8((uint8_t*)&regaddr, 2, d);


		regaddr = ICM20948_INT_ENABLE_2_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN);

		regaddr = ICM20948_INT_ENABLE_3_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_3_FIFO_WM_EN);

	}


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

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK;

	if (Value < 3)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G;
		Value = 2;
	}
	else if (Value < 6)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G;
		Value = 4;
	}
	else if (Value < 12)
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

bool AccelIcm20948::Enable()
{
	uint16_t regaddr = ICM20948_FIFO_EN_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_2_ACCEL_FIFO_EN;

	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;

	Write8((uint8_t*)&regaddr, 2, d);


	return true;
}

bool GyroIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_GYRO_ADC_RANGE);
	Sensitivity(Cfg.Sensitivity);
	SamplingFrequency(Cfg.Freq);
	FilterFreq(Cfg.FltrFreq);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK;

	if (Value < 325)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_250DPS;
		Value = 250;
	}
	else if (Value < 750)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_500DPS;
		Value = 500;
	}
	else if (Value < 1500)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS;
		Value = 1000;
	}
	else
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_2000DPS;
		Value = 2000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

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
		d |= (6 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 8900;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= (5 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 17800;
	}
	else if (Freq < 50000)
	{
		d |= (4 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 35900;
	}
	else if (Freq < 110000)
	{
		d |= (3 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 73300;
	}
	else if (Freq < 150000)
	{
		d |= (2 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 154300;
	}
	else if (Freq < 190000)
	{
		d |= (1 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 187600;
	}
	else if (Freq < 360000)
	{
		d |= (0 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 229800;
	}
	else if (Freq < 1000000)
	{
		d |= (7 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 376500;
	}
	else
	{
		Freq = 12316000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return GyroSensor::FilterFreq(Freq);
}

bool GyroIcm20948::Enable()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
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

	float g[3][3] = {
		1.0, 0.0, 0.0,
		0.0, -1.0, 0.0,
		0.0, 0.0, -1.0 };
	float offs[3] = {0.0, 0.0, 0.0};

	SetCalibration(g, offs);

//	regaddr = ICM20948_FIFO_EN_1;
//	d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_1_SLV_0_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);


	return true;
}

AgmIcm20948::AgmIcm20948()
{
	vbDmpEnabled = false;
	vbSensorEnabled[0] = vbSensorEnabled[1] = vbSensorEnabled[2] = false;
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG;
	vFifoHdr = vFifoHdr2 = 0;
	vFifoDataLen = 0;
}

bool AgmIcm20948::Enable()
{
	uint8_t fifoen = 0;
	uint8_t d, userctrl;
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;

	regaddr = ICM20948_USER_CTRL_REG;
	userctrl = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, userctrl & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	ResetFifo();

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	msDelay(100);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2);

	if (vbSensorEnabled[ICM20948_ACCEL_IDX])
	{
//		AccelIcm20948::Enable();
		d &= ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
		fifoen |= ICM20948_FIFO_EN_2_ACCEL_FIFO_EN;
	}

	if (vbSensorEnabled[ICM20948_GYRO_IDX])
	{
		d &= ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;
		fifoen |= (ICM20948_FIFO_EN_2_GYRO_X_FIFO_EN | ICM20948_FIFO_EN_2_GYRO_Y_FIFO_EN |
				   ICM20948_FIFO_EN_2_GYRO_Z_FIFO_EN);
	}
	regaddr = ICM20948_FIFO_EN_2_REG;
	Write8((uint8_t*)&regaddr, 2, fifoen);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	Write8((uint8_t*)&regaddr, 2, d);

	if (vbSensorEnabled[ICM20948_MAG_IDX])
	{
		regaddr = ICM20948_FIFO_EN_1_REG;
//		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_1_SLV_0_FIFO_EN);
	}

	regaddr = ICM20948_USER_CTRL_REG;
	//d = Read8((uint8_t*)&regaddr, 2);
	userctrl |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	//Write8((uint8_t*)&regaddr, 2, userctrl);

	return true;
}

void AgmIcm20948::Disable()
{
	uint16_t regaddr;
	uint8_t d;

	// Disable Accel & Gyro
	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = ICM20948_PWR_MGMT_2_DISABLE_ALL;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_LP_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_PWR_MGMT_1_REG;
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_CLKSEL_STOP | ICM20948_PWR_MGMT_1_LP_EN;// | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::PowerOff()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_SLEEP | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::Reset()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;

	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_DEVICE_RESET);
//	regaddr = ICM20948_FIFO_RST_REG;
//	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);

}

void AgmIcm20948::ResetDMPCtrlReg()
{
	uint16_t regaddr = ICM20948_DMP_DATA_OUT_CTL1;
	uint8_t d[2] = {0, 0};

	WriteDMP(ICM20948_DMP_DATA_OUT_CTL1, d, 2);
	WriteDMP(ICM20948_DMP_DATA_OUT_CTL2, d, 2);
	WriteDMP(ICM20948_DMP_DATA_INTR_CTL, d, 2);
	WriteDMP(ICM20948_DMP_MOTION_EVENT_CTL, d, 2);
	WriteDMP(ICM20948_DMP_DATA_RDY_STATUS, d, 2);
}


bool AgmIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr = ICM20948_INT_ENABLE_REG;
    uint8_t d = Read8((uint8_t*)&regaddr, 2);

	if (bEnable == true)
	{
	    Write8((uint8_t*)&regaddr, 2, d | ICM20948_INT_ENABLE_WOM_INT_EN);

	    regaddr = ICM20948_ACCEL_WOM_THR_REG;
	    Write8((uint8_t*)&regaddr, 2, Threshold);

	    regaddr = ICM20948_ACCEL_INTEL_CTRL_REG;
	    Write8((uint8_t*)&regaddr, 2, ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN |
	    							  ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_CMPPREV);
	}
	else
	{
	    Write8((uint8_t*)&regaddr, 2, d & ~ICM20948_INT_ENABLE_WOM_INT_EN);

	    regaddr = ICM20948_ACCEL_INTEL_CTRL_REG;
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
void AgmIcm20948::ResetFifo(void)
{
	uint16_t regaddr;
	uint16_t cnt;

	regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, d & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	do {
		regaddr = ICM20948_FIFO_RST_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
		Write8((uint8_t*)&regaddr, 2, ~ICM20948_FIFO_RST_FIFO_RESET_MASK);//0x1e);
		msDelay(1);

		regaddr = ICM20948_FIFO_COUNTH_REG;
		cnt = EndianCvt16(Read16((uint8_t*)&regaddr, 2)) & 0x1FFF;
	} while (cnt != 0);

	regaddr = ICM20948_INT_STATUS_2_REG;
	Write16((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, d);
}

size_t AgmIcm20948::CalcFifoPacketSize(uint16_t Header, uint16_t Mask, const size_t *pLookup, size_t LookupSize)
{
	size_t cnt = 0;
	uint16_t bit = Mask;

	for (int i = 0; i < LookupSize && Mask != 0; i++)
	{
		if (Header & Mask)
		{
			cnt += pLookup[i];
		}
		Mask >>= 1;
	}

	return cnt;
}

size_t AgmIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
{
	bool retval = false;
	size_t cnt = 0;
	uint16_t regaddr = ICM20948_FIFO_R_W_REG;
	uint8_t *d = pFifo;//[ICM20948_FIFO_PAGE_SIZE];

	if (vFifoHdr & ICM20948_FIFO_HEADER_ACCEL)
	{
		if (Len < ICM20948_FIFO_HEADER_ACCEL_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_ACCEL_SIZE);

		AccelSensor::vData.Timestamp = Timestamp;
		AccelSensor::vData.X = ((d[0] << 8) | (d[1] & 0xFF));// << 15;
		AccelSensor::vData.Y = ((d[2] << 8) | (d[3] & 0xFF));// << 15;
		AccelSensor::vData.Z = ((d[4] << 8) | (d[5] & 0xFF));// << 15;
		d += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		cnt += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		Len -= ICM20948_FIFO_HEADER_ACCEL_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ACCEL; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GYRO)
	{
		if (Len < ICM20948_FIFO_HEADER_GYRO_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_GYRO_SIZE);

		GyroSensor::vData.Timestamp = Timestamp;
		GyroSensor::vData.X = ((uint16_t)d[0] << 8) | ((uint16_t)d[1] & 0xFF);
		GyroSensor::vData.Y = ((uint16_t)d[2] << 8) | ((uint16_t)d[3] & 0xFF);
		GyroSensor::vData.Z = ((uint16_t)d[4] << 8) | ((uint16_t)d[5] & 0xFF);

		// TODO : Process gyro bias
		uint16_t bias[3];

		bias[0] = ((uint16_t)d[6] << 8) | ((uint16_t)d[7] & 0xFF);
		bias[1] = ((uint16_t)d[8] << 8) | ((uint16_t)d[9] & 0xFF);
		bias[2] = ((uint16_t)d[10] << 8) | ((uint16_t)d[11] & 0xFF);

		GyroSensor::vData.X += bias[0] * (1<<4);
		GyroSensor::vData.Y += bias[1] * (1<<4);
		GyroSensor::vData.Z += bias[2] * (1<<4);

		d += ICM20948_FIFO_HEADER_GYRO_SIZE;
		cnt += ICM20948_FIFO_HEADER_GYRO_SIZE;
		Len -= ICM20948_FIFO_HEADER_GYRO_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GYRO; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CPASS_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_CPASS_SIZE);

		MagSensor::vData.Timestamp = Timestamp;
		MagSensor::vData.X = ((int16_t)d[0] << 8) | (d[1] & 0xFF);
		MagSensor::vData.Y = ((int16_t)d[2] << 8) | (d[3] & 0xFF);
		MagSensor::vData.Z = ((int16_t)d[4] << 8) | (d[5] & 0xFF);
		d += ICM20948_FIFO_HEADER_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CPASS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_ALS)
	{
		if (Len < ICM20948_FIFO_HEADER_ALS_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_ALS_SIZE);

		d += ICM20948_FIFO_HEADER_ALS_SIZE;
		cnt += ICM20948_FIFO_HEADER_ALS_SIZE;
		Len -= ICM20948_FIFO_HEADER_ALS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ALS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT6)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT6_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_QUAT6_SIZE);

		d += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT6_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT6; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT9)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT9_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_QUAT9_SIZE);

		d += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT9_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT9; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_STEP_DETECTOR)
	{
		if (Len < ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE);

		d += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		cnt += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		Len -= ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_STEP_DETECTOR; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GEOMAG)
	{
		if (Len < ICM20948_FIFO_HEADER_GEOMAG_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_GEOMAG_SIZE);

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESSURE)
	{
		if (Len < ICM20948_FIFO_HEADER_PRESSURE_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_PRESSURE_SIZE);
		d += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		cnt += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		Len -= ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PRESSURE; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE);
		d += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_CPASS; // Clear bit
	}

	if (vFifoHdr2 != 0)
	{
		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACCEL_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE);

			d += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACCEL_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_GYRO_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE);

			d += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_GYRO_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_CPASS_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE);

			d += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_CPASS_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_PICKUP)
		{
			if (Len < ICM20948_FIFO_HEADER2_PICKUP_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_PICKUP_SIZE);

			d += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			cnt += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			Len -= ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_PICKUP; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACTI_RECOG)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE);

			d += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACTI_RECOG; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
		vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;
	//printf("Footer size\n");
		return cnt;
	}
	//Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_FOOTER_SIZE);
	if (d[0] != 0 || d[1] != 0)
	{
		//printf("bad packet %x %x\n", d[0], d[1]);
	}

	cnt += ICM20948_FIFO_FOOTER_SIZE;
	Len -= ICM20948_FIFO_FOOTER_SIZE;
	vFifoHdr &= ~ICM20948_FIFO_HEADER_FOOTER; // Clear bit

//	printf("fh %x %x\n", vFifoHdr, vFifoHdr2);

	vFifoHdr = vFifoHdr2 = 0;
	//printf("h: %x %x : %x %x %x %x\n", header, header2, pFifo[0], pFifo[1], pFifo[2], pFifo[3]);

	return cnt;
}

void AgmIcm20948::UpdateData(SENSOR_TYPE Type, uint64_t Timestamp, uint8_t * const pData)
{
	switch (Type)
	{
		case SENSOR_TYPE_ACCEL:
			AccelSensor::vData.Timestamp = Timestamp;
			AccelSensor::vData.X = ((pData[0] << 8) | (pData[1] & 0xFF));
			AccelSensor::vData.Y = ((pData[2] << 8) | (pData[3] & 0xFF));
			AccelSensor::vData.Z = ((pData[4] << 8) | (pData[5] & 0xFF));
			break;
		case SENSOR_TYPE_GYRO:
			GyroSensor::vData.Timestamp = Timestamp;
			GyroSensor::vData.X = ((pData[0] << 8) | (pData[1] & 0xFF));
			GyroSensor::vData.Y = ((pData[2] << 8) | (pData[3] & 0xFF));
			GyroSensor::vData.Z = ((pData[4] << 8) | (pData[5] & 0xFF));
			break;
		case SENSOR_TYPE_MAG:
			MagSensor::vData.Timestamp = Timestamp;
			MagSensor::vData.X = ((int16_t)pData[0] << 8) | (pData[1] & 0xFF);
			MagSensor::vData.Y = ((int16_t)pData[2] << 8) | (pData[3] & 0xFF);
			MagSensor::vData.Z = ((int16_t)pData[4] << 8) | (pData[5] & 0xFF);
			break;
	}
}

void AgmIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	inv_sensor_event_t event;
	//uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensortype;
	event.timestamp = timestamp;
	switch (sensortype)
	{
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
		memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
		memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE:
		{
		//memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		//memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			float *p = (float*)data;
			GyroSensor::vData.X = p[0] * 256.0;
			GyroSensor::vData.Y = p[1] * 256.0;
			GyroSensor::vData.Z = p[2] * 256.0;
			GyroSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		{
			memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
			float *p = (float*)data;
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		{
			float *p = (float*)data;
			//memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			//memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		{
			float *p = (float*)data;
			//memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
			//memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
			MagSensor::vData.X = p[0] * 256.0;
			MagSensor::vData.Y = p[1] * 256.0;
			MagSensor::vData.Z = p[2] * 256.0;
			MagSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();

		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
	}
		break;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
		memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
		break;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
	case INV_ICM20948_SENSOR_STEP_DETECTOR:
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
		event.data.event = true;
		break;
	case INV_ICM20948_SENSOR_B2S:
		event.data.event = true;
		memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
		break;
	case INV_ICM20948_SENSOR_STEP_COUNTER:
		memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
		break;
	case INV_ICM20948_SENSOR_ORIENTATION:
		//we just want to copy x,y,z from orientation data
		memcpy(&(event.data.orientation), data, 3*sizeof(float));
		break;
	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;
	default:
		return;
	}
}

bool AgmIcm20948::UpdateData()
{
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t status[4];
	uint8_t d[20];
	uint64_t t;
	bool res = false;
	//uint8_t fifo[ICM20948_FIFO_PAGE_SIZE];
	//uint8_t *p = fifo;

	Read((uint8_t*)&regaddr, 2, status, 4);

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	if (status[0] & ICM20948_INT_STATUS_PLL_RDY_INT)
	{

	}

	if (status[0] & ICM20948_INT_STATUS_WOM_INT)
	{

	}
	if (status[0] & ICM20948_INT_STATUS_I2C_MIST_INT)
	{
		printf("ICM20948_INT_STATUS_I2C_MIST_INT\n");
	}
	if (status[0] & ICM20948_INT_STATUS_DMP_INT1 || status[2] || status[3])
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);
	//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			//printf("distatus %x\n", distatus);
		}
	}

#if 1
	if (vbDmpEnabled)
	{
		regaddr = ICM20948_FIFO_COUNTH_REG;
		size_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

		regaddr = ICM20948_FIFO_R_W_REG;
		uint8_t *p = &vFifo[vFifoDataLen];

		while (cnt > ICM20948_FIFO_PAGE_SIZE)
		{
//			int l = min((size_t)ICM20948_FIFO_PAGE_SIZE, min(cnt, (size_t)ICM20948_FIFO_SIZE_MAX - vFifoDataLen));

//			if (l == 0)
//			{
//				break;
//			}
			int l = Read((uint8_t*)&regaddr, 2, p, l);
			p += l;
			vFifoDataLen += l;
			cnt -= l;
		//}

		while (vFifoDataLen > 2)
		{
			if (vFifoHdr == 0 && vFifoHdr2 == 0)
			{
				int l = 0;
				// new packet
				vFifoHdr = ((uint16_t)vFifo[0] << 8U) | ((uint16_t)vFifo[1] & 0xFF);

				if (vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
				{
					ResetFifo();
					return false;
				}

				l = 2;

				if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
				{
					vFifoHdr2 = ((uint16_t)vFifo[2] << 8U) | ((uint16_t)vFifo[3] & 0xFF);

					if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
					{
						ResetFifo();
						return false;
					}

					l += 2;
				}
				vFifoDataLen -= l;

				if (vFifoDataLen > 0)
				{
					memmove(vFifo, &vFifo[l], vFifoDataLen);
				}
				//printf("Header %x %x\n", vFifoHdr, vFifoHdr2);
			}
			int l = ProcessDMPFifo(vFifo, vFifoDataLen, t);
			if (l == 0)
			{
				return false;
			}
			vFifoDataLen -= l;
			if (vFifoDataLen > 0)
			{
				memmove(vFifo, &vFifo[l], vFifoDataLen);
			}
		}
		}
	}
	else
#endif
	{
		//res = MagIcm20948::UpdateData();

		if (status[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
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

			//MagIcm20948::UpdateData();

			// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
			int16_t t = ((int16_t)d[12] << 8) | d[13];
			TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | d[13]) * 100 / 33387 + 2100;
			TempSensor::vData.Timestamp = t;

			res = true;
		}
	}

	return res;
}

int AgmIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(*(uint16_t*)pCmdAddr >> 7);
		CmdAddrLen--;
	}

	*(uint16_t*)pCmdAddr &= 0x7F;

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(*(uint16_t*)pCmdAddr >> 7);
		CmdAddrLen--;
	}

	*(uint16_t*)pCmdAddr &= 0x7F;

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

int AgmIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = ICM20948_USER_CTRL_REG;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;
		uint8_t d[4];

		regaddr = ICM20948_I2C_SLV0_ADDR_REG;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);
		//d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[0] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			d[1] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LEN_MASK);

			regaddr = ICM20948_I2C_SLV0_REG_REG;
			Write((uint8_t*)&regaddr, 2, d, 2);

			// Start transfer
			regaddr = ICM20948_USER_CTRL_REG;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(3);

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
			msDelay(2);

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
	uint64_t t = 0;
	UpdateData();

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}
	//status = Read8((uint8_t*)&regaddr, 2);
	//printf("- status %x\n", status);
	//Write8((uint8_t*)&regaddr, 2, status);

#if 0
	int cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);
	//printf("%x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);

	if (istatus[0] & ICM20948_INT_STATUS_I2C_MIST_INT)
	{
		printf("ICM20948_INT_STATUS_I2C_MIST_INT\n");
	}
	if (istatus[0] & ICM20948_INT_STATUS_DMP_INT1)
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);
//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			printf("distatus %x\n", distatus);
		}
	}
	uint32_t drdy = (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT) |
			(istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK) |
			(istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK);
	if (drdy)
	{
		regaddr = ICM20948_FIFO_COUNTH_REG;
		uint16_t cnt;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&cnt, 2);
		//printf("cnt = %d : ", cnt);
		cnt = EndianCvt16(cnt);

//		printf("%d\n", cnt);
		if (cnt > 2)
		{
			regaddr = ICM20948_FIFO_R_W_REG;
//			uint16_t h = Read16((uint8_t*)&regaddr, 2);
			uint8_t dd[16];
		//	dd [0] = Read8((uint8_t*)&regaddr, 2);
		//	dd [1] = Read8((uint8_t*)&regaddr, 2);

//			Read((uint8_t*)&regaddr, 2, dd, 16);
			regaddr = ICM20948_FIFO_COUNTH_REG;
			//uint16_t cnt1 = Read16((uint8_t*)&regaddr, 2);
			//printf("cnt = %d : ", cnt);
			//cnt1 = EndianCvt16(cnt1);
	//		printf("%d Fifo header=%x %x %d\n", cnt, dd[0], dd[1], cnt1);

		}
	}

#if 1
//	if (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
	{
//		printf("ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT\n");
		UpdateData();
	}

	if (istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK)
	{
	//	printf("FIFO ovr Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		//uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

	//	printf("cnt = %d\r\n", cnt);
	}

	if (istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK)
	{
		//printf("WM Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		//uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		//cnt = EndianCvt16(cnt);

		//printf("cnt = %d\r\n", cnt);

#if 0
		for (int i = 0; i < cnt; i++)
		{
			regaddr = ICM20948_FIFO_R_W;
			uint8_t d = Read8((uint8_t*)&regaddr, 2);
			//printf("Fifo d=%x\n", d);
		}
#else
		uint8_t dd[cnt];
		memset(dd, 0, cnt);
		regaddr = ICM20948_FIFO_R_W_REG;
	//	int l = Read((uint8_t*)&regaddr, 2, dd, cnt);
		//printf("l:%d %x %x %x %x\n", l, dd[0], dd[1], dd[2], dd[3]);
#endif

	}
#endif

	//regaddr = ICM20948_INT_STATUS_REG;
	//cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);

//	printf("x- %x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);
	//istatus[0] = istatus[1] = istatus[2] = istatus[3] = 0;
	//cnt = Write((uint8_t*)&regaddr, 2, istatus, 4);
#endif
}

int AgmIcm20948::ReadDMP(uint16_t MemAddr, uint8_t *pBuff, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW_REG;
	uint8_t *p = pBuff;

	while (Len > 0)
	{
		int l = min(16, Len);
		l = Read((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
	}

	return Len;
}

int AgmIcm20948::WriteDMP(uint16_t MemAddr, uint8_t *pData, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW_REG;
	uint8_t *p = pData;

	while (Len > 0)
	{
		int l = min(16, Len);
		l = Write((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
	}

	return Len;
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
#if 0
	// Reset FIFO
	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);
#else
	ResetFifo();
#endif

	// load external image
	bool res = UploadDMPImage(pDmpImage, Len);

	if (res)
	{
		DmpStartAddr = EndianCvt16(DmpStartAddr);

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH_REG;
		Write16((uint8_t*)&regaddr, 2, DmpStartAddr);

//		regaddr = ICM20948_USER_CTRL_REG;
//		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
//		Write8((uint8_t*)&regaddr, 2, d);

		vbDmpEnabled = true;

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

	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

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

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	len = Len;
	p = (uint8_t*)pDmpImage;
	memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;

	// Verify

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);
	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

	while (len > 0)
	{
		uint8_t m[ICM20948_FIFO_PAGE_SIZE];

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

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	// Restore power state
	Write8((uint8_t*)&regaddr, 2, pwrstate);

	return true;
}
