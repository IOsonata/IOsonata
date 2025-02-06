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
#include "sensors/agm_icm20948.h"

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

AgmIcm20948::AgmIcm20948()
{
	memset(vbSensorEnabled, 0, sizeof(vbSensorEnabled));
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG;
}

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

	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	msDelay(100);

	uint8_t userctrl = ICM20948_USER_CTRL_I2C_MST_EN;//ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	//uint8_t lpconfig = 0;//ICM20948_LP_CONFIG_ACCEL_CYCLE | ICM20948_LP_CONFIG_GYRO_CYCLE;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS;
		//regaddr = ICM20948_USER_CTRL_REG;
		//Write8((uint8_t*)&regaddr, 2, ICM20948_USER_CTRL_I2C_IF_DIS);

	}

//lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_2_DISABLE_ALL);

	regaddr = ICM20948_ODR_ALIGN_EN_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN);

	// Must be set here, otherwise won't work
	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_HWTEMP_FIX_DISABLE_REG;
	d = Read8((uint8_t*)&regaddr, 2) | 0x08;
	Write8((uint8_t*)&regaddr, 2, d);//ICM20948_HWTEMP_FIX_DISABLE_DIS);


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
		d = ICM20948_INT_ENABLE_WOM_INT_EN | ICM20948_INT_ENABLE_PLL_RDY_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_1_REG;
		d = ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN;
		Write8((uint8_t*)&regaddr, 2, d);
	}


	if (vbSensorEnabled[ICM20948_TEMP_IDX] == false)
	{
		// Enable temperature sensor by default
		uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
		uint8_t d = Read8((uint8_t*)&regaddr, 2);

		// Enable temperature sensor
		d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
		Write8((uint8_t*)&regaddr, 2, d);

		vbSensorEnabled[ICM20948_TEMP_IDX] = true;
	}


	return true;
}

bool AccelIcm20948::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d[10];

	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_ACC_ADC_RANGE);

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	// Read manufacture trim offset
	regaddr = ICM20948_XA_OFFS_H_REG;
	((AgmIcm20948*)this)->Read((uint8_t*)&regaddr, 2, d, 6);

	float offs[3];

	offs[0] = (float)(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;
	offs[1] = (float)(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;
	offs[2] = (float)(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;

	SetCalibrationOffset(offs);

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK;
	int32_t scale, scale2;

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

	uint32_t div = (1125000 + (Freq >> 1))/ Freq - 1;
	div = EndianCvt16(div);

	uint16_t regaddr = ICM20948_ACCEL_SMPLRT_DIV_1_REG;
	Write16((uint8_t*)&regaddr, 2, div);

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
	uint16_t regaddr = ICM20948_PWR_MGMT_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

bool GyroIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_GYRO_ADC_RANGE);
	SamplingFrequency(Cfg.Freq);
	Sensitivity(Cfg.Sensitivity);
	FilterFreq(Cfg.FltrFreq);

	// Read manufacture trim offset
	uint16_t regaddr = ICM20948_XG_OFFS_USRH_REG;
	uint8_t d[10];
	((AgmIcm20948*)this)->Read((uint8_t*)&regaddr, 2, d, 6);

	float offs[3];

	offs[0] = (float)(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;
	offs[1] = (float)(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;
	offs[2] = (float)(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;

	SetCalibrationOffset(offs);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK;
	int32_t scale;

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
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (6 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 8900;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (5 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 17800;
	}
	else if (Freq < 50000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (4 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 35900;
	}
	else if (Freq < 110000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (3 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 73300;
	}
	else if (Freq < 150000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (2 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 154300;
	}
	else if (Freq < 190000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (1 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 187600;
	}
	else if (Freq < 360000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (0 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 229800;
	}
	else if (Freq < 1000000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (7 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
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
	d = ICM20948_I2C_MST_CTRL_I2C_MST_P_NSR | 7;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 4);

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

	return true;
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

	if (vbSensorEnabled[ICM20948_TEMP_IDX] == false)
	{
		uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
		uint8_t d = Read8((uint8_t*)&regaddr, 2);

		// Enable temperature sensor
		d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
		Write8((uint8_t*)&regaddr, 2, d);

		vbSensorEnabled[ICM20948_TEMP_IDX] = true;
	}

	return true;
}

bool AgmIcm20948::Enable()
{
	uint8_t d;
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	msDelay(10);

	if (vbSensorEnabled[ICM20948_ACCEL_IDX])
	{
		AccelIcm20948::Enable();
	}

	if (vbSensorEnabled[ICM20948_GYRO_IDX])
	{
		GyroIcm20948::Enable();
	}

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_PRESSURE_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	if (vbSensorEnabled[ICM20948_MAG_IDX])
	{
		MagIcm20948::Enable();
	}

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
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_CLKSEL_STOP;// | ICM20948_PWR_MGMT_1_LP_EN;// | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	//Write8((uint8_t*)&regaddr, 2, d);
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
	Write8((uint8_t*)&regaddr, 2, 0);
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

bool AgmIcm20948::UpdateData()
{
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t status[4];
	uint8_t d[20];
	uint64_t t;
	bool res = false;

	Read((uint8_t*)&regaddr, 2, status, 2);

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
	}

	if (status[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
	{
		regaddr = ICM20948_ACCEL_XOUT_H_REG;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)d, 14);

		AccelSensor::vData.Timestamp = t;
		AccelSensor::vData.X = ((int16_t)(d[0] << 8) | ((int16_t)d[1] & 0xFF));
		AccelSensor::vData.Y = ((int16_t)(d[2] << 8) | ((int16_t)d[3] & 0xFF));
		AccelSensor::vData.Z = ((int16_t)(d[4] << 8) | ((int16_t)d[5] & 0xFF));
		GyroSensor::vData.Timestamp = t;
		GyroSensor::vData.X = ((int16_t)(d[6] << 8) | ((int16_t)d[7] & 0xFF));
		GyroSensor::vData.Y = ((int16_t)(d[8] << 8) | ((int16_t)d[9] & 0xFF));
		GyroSensor::vData.Z = ((int16_t)(d[10] << 8) | ((int16_t)d[11] & 0xFF));


		MagAk09916::UpdateData();

		// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
		int16_t t = ((int16_t)d[12] << 8) | d[13];
		TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | ((int16_t)d[13] & 0xFF)) * 100 / 33387 + 2100;
		TempSensor::vData.Timestamp = t;

		res = true;
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

	uint8_t regaddr = *(uint16_t*)pCmdAddr & 0x7F;

	return Device::Read(&regaddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(*(uint16_t*)pCmdAddr >> 7);
		CmdAddrLen--;
	}

	uint8_t regaddr = *(uint16_t*)pCmdAddr & 0x7F;

	return Device::Write(&regaddr, CmdAddrLen, pData, DataLen);
}

int AgmIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

#if 0
	// Use SLV4. This can read only 1 byte at a time
	uint8_t *p = pBuff;
	uint8_t d;
	uint16_t regaddr = ICM20948_I2C_SLV4_ADDR_REG;
	Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV4_ADDR_I2C_ID_4_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

	uint8_t addr = *pCmdAddr;

	regaddr = ICM20948_I2C_SLV4_REG_REG;
	Write8((uint8_t*)&regaddr, 2, addr);

	regaddr = ICM20948_I2C_SLV4_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV4_CTRL_I2C_SLV4_EN);

	while (BuffLen > 0)
	{

		regaddr = ICM20948_I2C_MST_STATUS_REG;

		do {
			d = Read8((uint8_t*)&regaddr, 2);

		} while ((d & ICM20948_I2C_MST_STATUS_I2C_SLV4_DONE) == 0);

		regaddr = ICM20948_I2C_SLV4_DI_REG;

		int l = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

		l = Read((uint8_t*)&regaddr, 2, p, 1);

		BuffLen -= l;
		p += l;
		retval += l;
		addr += l;

		regaddr = ICM20948_I2C_SLV4_CTRL_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV4_CTRL_I2C_SLV4_EN | ICM20948_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS);
	}

#else
	// Use SLV0 - Can read multi-bytes
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d[4];

	regaddr = ICM20948_I2C_SLV0_ADDR_REG;
	Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

	d[0] = *pCmdAddr;

	while (BuffLen > 0)
	{
		int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

		d[1] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LEN_MASK);

		regaddr = ICM20948_I2C_SLV0_REG_REG;
		Write((uint8_t*)&regaddr, 2, d, 2);


		// Delay require for transfer to complete
		msDelay(3);
		regaddr = ICM20948_I2C_MST_STATUS_REG;

		do {
			d[2] = Read8((uint8_t*)&regaddr, 2);

		} while (d[2] & ICM20948_I2C_MST_STATUS_I2C_SLV0_NACK);

		regaddr = ICM20948_EXT_SLV_SENS_DATA_00_REG;
		cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
		if (cnt <= 0)
			break;

		pBuff += cnt;
		BuffLen -= cnt;
		retval += cnt;
		d[1] += cnt;
	}
#endif
	return retval;
}

int AgmIcm20948::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d[8];
	uint8_t userctrl = Read8((uint8_t*)&regaddr, 2);// | ICM20948_USER_CTRL_I2C_MST_EN;

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

	//	Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

		d[0]++;
		pData++;
		DataLen--;
		retval++;
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

static uint64_t g_Dt = 0;
static uint64_t g_PrevT = 0;
static uint64_t g_IntDt = 0;

void AgmIcm20948::IntHandler()
{
	uint64_t t = vpTimer->uSecond();

	UpdateData();

	if (g_Dt == 0)
	{
		g_Dt = vpTimer->uSecond() - t;
	}
	else
	{
		g_Dt = (g_Dt + vpTimer->uSecond() - t) >> 1;
	}

	g_IntDt = t - g_PrevT;
	g_PrevT = t;
}

