/**-------------------------------------------------------------------------
@file	pm_mp2696a.cpp

@brief	Power management implementation of the MP2696A


@author	Hoang Nguyen Hoan
@date	Mar. 31, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include "pwrmgnt/pm_mp2696a.h"

bool PmMp2696a::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL || Cfg.DevAddr != MP2696A_I2C_DEVADDR)
	{
		return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);

	Reset();

	return true;
}

int32_t PmMp2696a::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit)
{
	if (VoutIdx != 0)
	{
		return 0;
	}

	uint8_t regaddr = MP2696A_REG04;
	uint8_t d = Read8(&regaddr, 1) & ~MP2696A_REG04_VBOOST_MASK;

	int32_t v = 0;

	if (mVolt < 5075)
	{
		d |= MP2696A_REG04_VBOOST_5V05;
		v = 5050;
	}
	else if (mVolt < 5100)
	{
		d |= MP2696A_REG04_VBOOST_5V075;
		v = 5075;
	}
	else if (mVolt < 5125)
	{
		d |= MP2696A_REG04_VBOOST_5V1;
		v = 5100;
	}
	else if (mVolt < 5150)
	{
		d |= MP2696A_REG04_VBOOST_5V125;
		v = 5125;
	}
	else if (mVolt < 5200)
	{
		d |= MP2696A_REG04_VBOOST_5V15;
		v = 5150;
	}
	else if (mVolt < 5225)
	{
		d |= MP2696A_REG04_VBOOST_5V2;
		v = 5200;
	}
	else
	{
		d |= MP2696A_REG04_VBOOST_5V225;
		v = 5225;
	}

	Write8(&regaddr, 1, d);

	regaddr = MP2696A_REG03;
	d = Read8(&regaddr, 1) & ~MP2696A_REG03_IOLIM_MASK;

	uint32_t c = (CurrLimit - 2100) / 100;

	d |= (c << MP2696A_REG03_IOLIM_POS);
	Write8(&regaddr, 1, d);

	return v;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool PmMp2696a::Enable()
{
	uint8_t regaddr = MP2696A_REG04;
	uint8_t d = Read8(&regaddr, 1);

	Write8(&regaddr, 1, d | MP2696A_REG04_Q2_EN | MP2696A_REG04_BST_EN);

	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void PmMp2696a::Disable()
{
	uint8_t regaddr = MP2696A_REG04;
	uint8_t d = Read8(&regaddr, 1) & ~(MP2696A_REG04_Q2_EN | MP2696A_REG04_BST_EN);

	Write8(&regaddr, 1, d);
}

/**
 * @brief	Reset device to it initial default state
 */
void PmMp2696a::Reset()
{
	uint8_t regaddr = MP2696A_REG00;
	uint8_t d = MP2696A_REG00_REG_RST;

	Write(&regaddr, 1, &d, 1);
}

void PmMp2696a::PowerOff()
{
	Disable();
}

uint32_t PmMp2696a::SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
{
	uint8_t regaddr = MP2696A_REG02;
	uint8_t d = Read(&regaddr, 1, &d, 1) & ~MP2696A_REG02_BATT_REG_MASK;


	if (mVoltEoC < 4100)
	{
		d |= MP2696A_REG02_BATT_REG_3V6;
	}
	else if (mVoltEoC < 4200)
	{
		d |= MP2696A_REG02_BATT_REG_4V1;
	}
	else if (mVoltEoC < 4300)
	{
		d |= MP2696A_REG02_BATT_REG_4V2;
	}
	else if (mVoltEoC < 4350)
	{
		d |= MP2696A_REG02_BATT_REG_4V3;
	}
	else if (mVoltEoC < 4400)
	{
		d |= MP2696A_REG02_BATT_REG_4V35;
	}
	else if (mVoltEoC < 4450)
	{
		d |= MP2696A_REG02_BATT_REG_4V4;
	}
	else
	{
		d |= MP2696A_REG02_BATT_REG_4V45;
	}

	Write(&regaddr, 1, &d, 1);

	regaddr = MP2696A_REG01;
	d = Read(&regaddr, 1, &d, 1) & ~MP2696A_REG01_ICC_MASK;

	uint32_t c = (mACurr - 500) / 100;
	d |= c << MP2696A_REG01_ICC_POS;
	Write(&regaddr, 1, &d, 1);

	return c + 500;
}

bool PmMp2696a::Charging()
{
	uint8_t regaddr = MP2696A_REG05;
	uint8_t d = Read(&regaddr, 1, &d, 1);

	return d & MP2696A_REG05_CHIP_STAT_CHARGE;
}

bool PmMp2696a::Battery()
{
	return true;
}

void PmMp2696a::IrqHandler()
{

}

