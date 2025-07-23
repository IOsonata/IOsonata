/**-------------------------------------------------------------------------
@file	mag_ak09940.cpp

@brief	Implementation of Asahi Kasei AK09940x mag sensor


@author	Hoang Nguyen Hoan
@date	May 3, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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

#include "idelay.h"
#include "sensors/mag_ak09940.h"

#include "nrf_cli.h"

#define cli_printf(Format, ...) nrf_cli_fprintf(&s_Cli, NRF_CLI_DEFAULT, Format, ##__VA_ARGS__)

extern nrf_cli_t const s_Cli;

bool MagAk09940::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t regaddr = AK09940_WIA1_REG;
	uint16_t d = 0;

	MagSensor::Type(SENSOR_TYPE_MAG);
#if 1
	Interface(pIntrf);

	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		DeviceAddress(Cfg.DevAddr);
	}
	else
	{
		for (uint32_t ad = AK09940_I2C_7BITS_DEVADDR0; ad <= AK09940_I2C_7BITS_DEVADDR3; ad++)
		{
			DeviceAddress(AK09940_I2C_7BITS_DEVADDR0);
			Read(&regaddr, 1, (uint8_t*)&d, 2);

			if (d == AK09940_COMPANY_DEVICE_ID)
			{
				break;
			}
		}
	}

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}
#endif
//	Read(AK09940_I2C_7BITS_DEVADDR0, &regaddr, 1, (uint8_t*)&d, 2);
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	if (d != AK09940_COMPANY_DEVICE_ID)
	{
		return false;
	}

	DataReadyClear();

	Reset();

	regaddr = AK09940_CTRL1_REG;
	d = 0;

	Write(&regaddr, 1, (uint8_t*)&d, 1);

	Range(AK09940_ADC_RANGE);

	vSensitivity[0] = AK09940_SENSITIVITY;
	vSensitivity[1] = vSensitivity[0];
	vSensitivity[2] = vSensitivity[0];

	vData.Sensitivity[0] = vSensitivity[0];
	vData.Sensitivity[1] = vSensitivity[1];
	vData.Sensitivity[2] = vSensitivity[2];

	ClearCalibration();

	vCtrl3Val = 0;//AK09940_CTRL3_FIFO_EN;
	vOpMode = Cfg.OpMode;

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE || Cfg.OpMode == SENSOR_OPMODE_TIMER)
	{
		regaddr = AK09940_CTRL3_REG;
		vCtrl3Val = AK09940_CTRL3_MODE_SINGLE;

		Write(&regaddr, 1, &vCtrl3Val, 1);
	}
	else
	{
		vCtrl3Val = AK09940_CTRL3_FIFO_EN;

		if (Cfg.OpMode == SENSOR_OPMODE_LOW_POWER)
		{
			regaddr = AK09940_CTRL1_REG;
			d = AK09940_CTRL1_MT2_EN | 2;
			Write(&regaddr, 1, (uint8_t*)&d, 1);
		}
		else
		{
			vCtrl3Val |= AK09940_CTRL3_MT_LN1;
			regaddr = AK09940_CTRL1_REG;
			d = 2;
			Write(&regaddr, 1, (uint8_t*)&d, 1);
		}


		SamplingFrequency(Cfg.Freq);
	}

	if (Cfg.Inter != 0)
	{

	}
/*
	msDelay(1);

	regaddr = AK09940_ST_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	printf("ST %x\n", d);

	regaddr = AK09940_ST2_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);

	printf("ST2 %x\n", d & 0xFF);

//	regaddr = AK09940_ST_REG;
//	Read(&regaddr, 1, (uint8_t*)&d, 2);

//	printf("ST %x\n", d);

	uint8_t dd[16];

	regaddr = AK09940_HXL_REG;
	Read(&regaddr, 1, (uint8_t*)dd, 9);

*/
	return true;
}

uint32_t MagAk09940::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = AK09940_CTRL3_REG;
	uint8_t d = 0;

	//Read(&regaddr, 1, &vCtrl3Val, 1);

	vCtrl3Val &= ~AK09940_CTRL3_MODE_MASK;

	Write(&regaddr, 1, &d, 1);

	usDelay(100);

	if (Freq < 20000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_10HZ;
		Freq = 10000;
	}
	else if (Freq < 50000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_20HZ;
		Freq = 20000;
	}
	else if (Freq < 100000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_50HZ;
		Freq = 50000;
	}
	else if (Freq < 200000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_100HZ;
		Freq = 100000;
	}
	else if (Freq < 400000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_200HZ;
		Freq = 200000;
	}
	else if (Freq < 1000000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_400HZ;
		Freq = 400000;
	}
	else if (Freq < 2500000)
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_1000HZ;
		Freq = 1000000;
	}
	else
	{
		vCtrl3Val |= AK09940_CTRL3_MODE_CONTINUOUS_2500HZ;
		Freq = 2500000;
	}

	//d |= AK09940_CTRL3_MT_LN1;
	//Write(AK09940_I2C_7BITS_DEVADDR0, &regaddr, 1, &d, 1);
	//Write(&regaddr, 1, &d, 1);

	//d = 0;
	//Read(AK09940_I2C_7BITS_DEVADDR0, &regaddr, 1, &d, 1);
	//Read(&regaddr, 1, &d, 1);

	return MagSensor::SamplingFrequency(Freq);
}

/**
 * @brief	Power off the device completely.
 *
 * If supported, this will put the device in complete power down.
 * Full re-intialization is required to re-enable the device.
 */
void MagAk09940::PowerOff()
{
	uint8_t regaddr = AK09940_CTRL2_REG;
	uint8_t d = 0;

//	Write(AK09940_I2C_7BITS_DEVADDR0, &regaddr, 1, &d, 1);
	Write(&regaddr, 1, &d, 1);
}

bool MagAk09940::Enable()
{
	uint8_t regaddr = AK09940_ST_REG;
	uint16_t d;
/*
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	regaddr = AK09940_ST2_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);

	printf("ST %x\n", d);

	uint8_t dd[16];

	regaddr = AK09940_HXL_REG;
	Read(&regaddr, 1, (uint8_t*)dd, 9);

	//UpdateData();
	if (vOpMode == SENSOR_OPMODE_CONTINUOUS)
	{
		SamplingFrequency(Sensor::vSampFreq);
	}
	else
	{
		regaddr = AK09940_CTRL3_REG;
		d = AK09940_CTRL3_MODE_SINGLE;

		Write(&regaddr, 1, (uint8_t*)&d, 1);
	}
*/
	regaddr = AK09940_CTRL3_REG;
	Write(&regaddr, 1, &vCtrl3Val, 1);
	Read(&regaddr, 1, (uint8_t*)&d, 1);
	printf("vCtrl3Val %x, %x\n", vCtrl3Val, d);
/*
	regaddr = AK09940_ST_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 2);

	printf("ST %x\n", d);

	d = 0;
	Write(&regaddr, 1, (uint8_t*)&d, 1);
*/
	regaddr = AK09940_ST1_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);
	regaddr = AK09940_ST2_REG;
	Read(&regaddr, 1, (uint8_t*)&d, 1);

	State(SENSOR_STATE_SAMPLING);

	return true;
}

void MagAk09940::Disable()
{
	uint8_t regaddr = AK09940_CTRL3_REG;
	uint8_t d = 0;

	Read(&regaddr, 1, &d, 1);
	d &= AK09940_CTRL3_MODE_MASK;

	Write(&regaddr, 1, &d, 1);
}

void MagAk09940::Reset()
{
	uint8_t regaddr = AK09940_CTRL4_REG;
	uint8_t d = AK09940_CTRL4_SRST;

	Write(&regaddr, 1, (uint8_t*)&d, 1);
}

bool MagAk09940::UpdateData()
{
	uint8_t regaddr = AK09940_ST_REG;
	uint8_t d[2];

	Read(&regaddr, 1, d, 2);

cli_printf("st %x\n", d[0]);

	if (d[0] & AK09940_ST_DRDY)
	//if (isDataReady())
	{
		//regaddr = AK09940_ST1_REG;
		//Read(&regaddr, 1, &d[1], 1);

		int nb = (d[1] & AK09940_ST1_FNUM_MASK) >> 1;
		cli_printf("%x nb %d\n", d[1], nb);

		if (vpTimer)
		{
			vData.Timestamp = vpTimer->nSecond() * 1000ULL;
		}

		//regaddr = AK09940_ST1_REG;
		//Read(&regaddr, 1, d, 1);

		uint8_t dd[16];

		for (int i = 0; i < nb; i++)
		{
			regaddr = AK09940_HXL_REG;
			Read(&regaddr, 1, (uint8_t*)dd, 10);
		}
		regaddr = AK09940_ST2_REG;
		Read(&regaddr, 1, (uint8_t*)d, 1);
		cli_printf("st2 %x \n", dd[10]);

		//if ((dd[10] & AK09940_ST1_FNUM_MASK) == 0)
		{
			vData.X = dd[0] | (dd[1] << 8) | ((int8_t)dd[2] << 16);
			vData.Y = dd[3] | (dd[4] << 8) | ((int8_t)dd[5] << 16);
			vData.Z = dd[6] | (dd[7] << 8) | ((int8_t)dd[8] << 16);
		}

		DataReadyClear();

		return true;
	}

	return false;
}

/**
 * @brief	Interrupt handler (optional)
 *
 * Sensor that supports interrupt can implement this to handle interrupt.
 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
 */
void MagAk09940::IntHandler(void)
{
	uint8_t regaddr = AK09940_ST_REG;
	uint8_t st, st1, d[2];

	cli_printf("MagAk09940::IntHandler\n");
	//Read(&regaddr, 1, &st, 1);

	//regaddr = AK09940_ST1_REG;
	//Read(&regaddr, 1, &st1, 1);
//	Read(&regaddr, 1, d, 2);

	//if (st & AK09940_ST_DRDY)
	{
		DataReadySet();
		UpdateData();
	}
}

