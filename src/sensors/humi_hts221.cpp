/**-------------------------------------------------------------------------
@file	humi_hts221.cpp

@brief	ST HTS221 humidity and temperature sensor implementation

Init reads the calibration of the device once. UpdateData converts the raw
outputs by linear interpolation between the two calibration points of each
quantity, as ST TN1218 describes.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>

#include "sensors/humi_hts221.h"

// Bound of the reads that wait for the calibration reload to end
#define HTS221_BOOT_POLL_MAX			100

HumiHts221::HumiHts221()
{
	vOdr = HTS221_CTRL_REG1_ODR_1HZ;
	vH0x2 = vH1x2 = 0;
	vH0Out = vH1Out = 0;
	vT0x8 = vT1x8 = 0;
	vT0Out = vT1Out = 0;
	HumiSensor::vEvtHandler = NULL;
	TempSensor::vDataRdyHandler = NULL;
}

bool HumiHts221::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
	{
		return true;
	}

	if (pIntrf == NULL)
	{
		return false;
	}

	Interface(pIntrf);

	// Address on I2C, chip select index on SPI
	DeviceAddress(pIntrf->Type() == DEVINTRF_TYPE_I2C ? HTS221_I2C_DEV_ADDR : DevAddr);

	uint8_t regaddr = HTS221_WHO_AM_I_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d != HTS221_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	regaddr = HTS221_CTRL_REG2_REG;
	if (Read8(&regaddr, 1) & HTS221_CTRL_REG2_BOOT)
	{
		// Calibration reload did not end
		return false;
	}

	uint8_t c[HTS221_CALIB_SIZE];

	regaddr = HTS221_CALIB_REG;
	if (Read(&regaddr, 1, c, sizeof(c)) != (int)sizeof(c))
	{
		return false;
	}

	vH0x2 = c[HTS221_CALIB_H0_RH_X2];
	vH1x2 = c[HTS221_CALIB_H1_RH_X2];
	vT0x8 = (int16_t)(((c[HTS221_CALIB_T1_T0_MSB] & 3) << 8) | c[HTS221_CALIB_T0_DEGC_X8]);
	vT1x8 = (int16_t)((((c[HTS221_CALIB_T1_T0_MSB] >> 2) & 3) << 8) | c[HTS221_CALIB_T1_DEGC_X8]);
	vH0Out = (int16_t)(c[HTS221_CALIB_H0_T0_OUT] | (c[HTS221_CALIB_H0_T0_OUT + 1] << 8));
	vH1Out = (int16_t)(c[HTS221_CALIB_H1_T0_OUT] | (c[HTS221_CALIB_H1_T0_OUT + 1] << 8));
	vT0Out = (int16_t)(c[HTS221_CALIB_T0_OUT] | (c[HTS221_CALIB_T0_OUT + 1] << 8));
	vT1Out = (int16_t)(c[HTS221_CALIB_T1_OUT] | (c[HTS221_CALIB_T1_OUT + 1] << 8));

	if (vH0Out == vH1Out || vT0Out == vT1Out)
	{
		// Two equal points cannot be interpolated
		return false;
	}

	// Coherent 16 bits samples
	regaddr = HTS221_CTRL_REG1_REG;
	Write8(&regaddr, 1, HTS221_CTRL_REG1_BDU);

	Valid(true);

	return true;
}

bool HumiHts221::Init(const HumiSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	HumiSensor::vEvtHandler = Cfg.EvtHandler;
	HumiSensor::vOpMode = Cfg.OpMode;

	if (Cfg.HumOvrs > 0)
	{
		uint8_t avg = 0;

		while (avg < 7 && (4 << avg) < Cfg.HumOvrs)
		{
			avg++;
		}

		uint8_t regaddr = HTS221_AV_CONF_REG;
		uint8_t d = Read8(&regaddr, 1) & ~HTS221_AV_CONF_AVGH_MASK;

		Write8(&regaddr, 1, d | (avg << HTS221_AV_CONF_AVGH_POS));
	}

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE)
	{
		vOdr = HTS221_CTRL_REG1_ODR_ONE_SHOT;
		HumiSensor::SamplingFrequency(0);
		TempSensor::SamplingFrequency(0);
	}
	else
	{
		SamplingFrequency(Cfg.Freq);
	}

	return true;
}

bool HumiHts221::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
	{
		return false;
	}

	TempSensor::vDataRdyHandler = Cfg.DataRdyCB;
	TempSensor::vOpMode = Cfg.OpMode;

	uint8_t regaddr;

	if (Cfg.TempOvrs > 0)
	{
		uint8_t avg = 0;

		while (avg < 7 && (2 << avg) < Cfg.TempOvrs)
		{
			avg++;
		}

		regaddr = HTS221_AV_CONF_REG;
		uint8_t d = Read8(&regaddr, 1) & ~HTS221_AV_CONF_AVGT_MASK;

		Write8(&regaddr, 1, d | (avg << HTS221_AV_CONF_AVGT_POS));
	}

	regaddr = HTS221_CTRL_REG3_REG;

	if (Cfg.Inter)
	{
		// Push-pull DRDY pin, high while new data waits unless active low
		uint8_t d = HTS221_CTRL_REG3_DRDY;

		if (Cfg.IntPol == DEVINTR_POL_LOW)
		{
			d |= HTS221_CTRL_REG3_DRDY_H_L;
		}
		Write8(&regaddr, 1, d);
		InterruptId(Cfg.Inter);
		InterruptEnabled(true);
	}
	else
	{
		Write8(&regaddr, 1, 0);
	}

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE)
	{
		vOdr = HTS221_CTRL_REG1_ODR_ONE_SHOT;
		HumiSensor::SamplingFrequency(0);
		TempSensor::SamplingFrequency(0);
	}
	else
	{
		SamplingFrequency(Cfg.Freq);
	}

	return true;
}

uint32_t HumiHts221::SamplingFrequency(uint32_t Freq)
{
	if (Freq <= 1000)
	{
		vOdr = HTS221_CTRL_REG1_ODR_1HZ;
		Freq = 1000;
	}
	else if (Freq <= 7000)
	{
		vOdr = HTS221_CTRL_REG1_ODR_7HZ;
		Freq = 7000;
	}
	else
	{
		vOdr = HTS221_CTRL_REG1_ODR_12_5HZ;
		Freq = 12500;
	}

	uint8_t regaddr = HTS221_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1);

	// Change the rate now only when running, else Enable writes it
	if (d & HTS221_CTRL_REG1_PD)
	{
		Write8(&regaddr, 1, (d & ~HTS221_CTRL_REG1_ODR_MASK) | vOdr);
	}

	HumiSensor::SamplingFrequency(Freq);

	return TempSensor::SamplingFrequency(Freq);
}

bool HumiHts221::Enable()
{
	uint8_t regaddr = HTS221_CTRL_REG1_REG;

	return Write8(&regaddr, 1, HTS221_CTRL_REG1_PD | HTS221_CTRL_REG1_BDU | vOdr);
}

void HumiHts221::Disable()
{
	uint8_t regaddr = HTS221_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~HTS221_CTRL_REG1_PD;

	Write8(&regaddr, 1, d);
}

void HumiHts221::Reset()
{
	uint8_t regaddr = HTS221_CTRL_REG1_REG;

	// Power down, then reload the calibration and return the control
	// registers to their default values
	Write8(&regaddr, 1, 0);

	regaddr = HTS221_CTRL_REG2_REG;
	Write8(&regaddr, 1, HTS221_CTRL_REG2_BOOT);

	for (int i = 0; i < HTS221_BOOT_POLL_MAX; i++)
	{
		if ((Read8(&regaddr, 1) & HTS221_CTRL_REG2_BOOT) == 0)
		{
			break;
		}
	}

	regaddr = HTS221_CTRL_REG3_REG;
	Write8(&regaddr, 1, 0);

	regaddr = HTS221_AV_CONF_REG;
	Write8(&regaddr, 1, HTS221_AV_CONF_DEFAULT);
}

bool HumiHts221::StartSampling()
{
	if (vOdr != HTS221_CTRL_REG1_ODR_ONE_SHOT)
	{
		// Measures continuously
		return true;
	}

	uint8_t regaddr = HTS221_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d & HTS221_CTRL_REG2_ONE_SHOT)
	{
		// Previous measurement not done
		return false;
	}

	return Write8(&regaddr, 1, d | HTS221_CTRL_REG2_ONE_SHOT);
}

bool HumiHts221::UpdateData()
{
	uint8_t regaddr = HTS221_STATUS_REG;
	uint8_t status = Read8(&regaddr, 1);

	if ((status & (HTS221_STATUS_H_DA | HTS221_STATUS_T_DA)) == 0)
	{
		return false;
	}

	// Humidity then temperature, little endian
	uint8_t d[4];

	regaddr = HTS221_HUMIDITY_OUT_L_REG;
	if (Read(&regaddr, 1, d, sizeof(d)) != (int)sizeof(d))
	{
		return false;
	}

	uint64_t t = vpTimer != NULL ? vpTimer->uSecond() : 0;

	if (status & HTS221_STATUS_H_DA)
	{
		int32_t h = (int16_t)(d[0] | (d[1] << 8));

		// % x 100 from the % x 2 calibration
		int64_t hx200 = (int64_t)vH0x2 * 100 +
						(int64_t)(h - vH0Out) * (vH1x2 - vH0x2) * 100 / (vH1Out - vH0Out);
		int32_t hum = (int32_t)(hx200 / 2);

		if (hum < 0)
		{
			hum = 0;
		}
		else if (hum > 10000)
		{
			hum = 10000;
		}

		HumiSensor::vData.Timestamp = t;
		HumiSensor::vData.Humidity = (uint16_t)hum;
		HumiSensor::vSampleCnt++;

		if (HumiSensor::vEvtHandler != NULL)
		{
			HumiSensor::vEvtHandler(this, &(HumiSensor::vData));
		}
	}

	if (status & HTS221_STATUS_T_DA)
	{
		int32_t tout = (int16_t)(d[2] | (d[3] << 8));

		// Degree C x 100 from the degree C x 8 calibration
		int64_t tx800 = (int64_t)vT0x8 * 100 +
						(int64_t)(tout - vT0Out) * (vT1x8 - vT0x8) * 100 / (vT1Out - vT0Out);

		TempSensor::vData.Timestamp = t;
		TempSensor::vData.Temperature = (int32_t)(tx800 / 8);
		TempSensor::vSampleCnt++;

		if (TempSensor::vDataRdyHandler != NULL)
		{
			TempSensor::vDataRdyHandler(this, &(TempSensor::vData));
		}
	}

	return true;
}

void HumiHts221::IntHandler()
{
	if (UpdateData() == true && Device::vEvtHandler != NULL)
	{
		Device::vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

void HumiHts221::Heater(bool bOn)
{
	uint8_t regaddr = HTS221_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~HTS221_CTRL_REG2_HEATER;

	if (bOn)
	{
		d |= HTS221_CTRL_REG2_HEATER;
	}

	Write8(&regaddr, 1, d);
}

int HumiHts221::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen != 1)
	{
		// Register addresses are one byte
		return 0;
	}

	uint8_t cmd = *pCmdAddr;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		cmd |= HTS221_SPI_READ;
		if (BuffLen > 1)
		{
			cmd |= HTS221_SPI_ADDR_INC;
		}
	}
	else if (BuffLen > 1)
	{
		cmd |= HTS221_I2C_ADDR_INC;
	}

	return vpIntrf->Read(vDevAddr, &cmd, 1, pBuff, BuffLen);
}

int HumiHts221::Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen)
{
	if (CmdAddrLen != 1)
	{
		// Register addresses are one byte
		return 0;
	}

	uint8_t cmd = *pCmdAddr;

	if (DataLen > 1)
	{
		cmd |= vpIntrf->Type() == DEVINTRF_TYPE_SPI ? HTS221_SPI_ADDR_INC : HTS221_I2C_ADDR_INC;
	}

	return vpIntrf->Write(vDevAddr, &cmd, 1, pData, DataLen);
}
