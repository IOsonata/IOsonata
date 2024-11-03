/**-------------------------------------------------------------------------
@file	pm_bq25120a.cpp

@brief	Power management implementation of the TI BQ25120A

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	July 25, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

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
#include <math.h>

#include "pwrmgnt/pm_bq25120a.h"

const int s_Bq25120aBaseVolt[] = { 12500, 13000, 15000, 18000 };
const int s_Bq25120aBaseVoltCnt = sizeof(s_Bq25120aBaseVolt) / sizeof(int);

bool PmBq25120a::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);

	if (Cfg.pVout)
	{
		SetVout(0, Cfg.pVout[0].mVout, Cfg.pVout[0].mAlimit);	// Vout
		SetVout(1, Cfg.pVout[1].mVout, Cfg.pVout[1].mAlimit);	// LS/LDO
	}

	if (Cfg.pBatProf != nullptr)
	{
		vChrgCurr = Cfg.ChrgCurr;
	}

	return true;
}

int32_t PmBq25120a::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit)
{
	uint8_t reg = BQ25120A_SYS_VOUT_CTRL_REG;
	uint8_t sel, out;
	int vout = 0;
	int diff = 5000;

	if (VoutIdx == 0)
	{
		if (mVolt < 1250)
		{
			sel = BQ25120A_SYS_SEL_1_1_1_2;
			out = ((mVolt - 1100 + 50) / 100) & 0xF;
			vout = 1100 + 100 * out;
		}
		else if (mVolt > 2800)
		{
			sel = BQ25120A_SYS_SEL_1_8_3_3;
			out = ((mVolt - 1800 + 50) / 100);
			if (out > 15)
			{
				out = 15;
			}
			vout = 1800 + 100 * out;
		}
		else
		{
			// Calculation here is performed in higher precision
			// Voltage is converted to 100 nV units. Therefore multiply everything by 10
			mVolt *= 10;
			for (uint8_t i = 0; i < s_Bq25120aBaseVoltCnt && diff != 0; i++)
			{
				int step = (i & 1) ? 1000 : 833;
				if (mVolt < s_Bq25120aBaseVolt[i])
				{
					continue;
				}
				int t = (mVolt - s_Bq25120aBaseVolt[i] + (step >> 1)) / step;

				if (t < 16)
				{
					int d = labs(mVolt - (s_Bq25120aBaseVolt[i] + (t * step)));
					if (d < diff)
					{
						diff = d;
						sel = i << 5;
						// NOTE : Special case when sel == 0, base voltage start was skip
						// 2 step count to handle 1.25V and above.
						out = i == 0 ? t + 2 : t;

						// Divide by 10 to get back to mV units for vout
						vout = (s_Bq25120aBaseVolt[i] + step * t + 5) / 10;
					}
				}
			}

		}

		Write8(&reg, 1, sel | (out << 1) | BQ25120A_SYS_SEL_EN_SYST_OUT);
	}
	else if (VoutIdx == 1)
	{
		out = (mVolt - 800 + 50) / 100;
		vout = 800 + out * 100;

		reg = BQ25120A_LOADSW_LDO_CTRL_REG;
		Write8(&reg, 1, (out << 2) | BQ25120A_LOADSW_LDO_CTRL_EN_LS_LDO);
	}
	return vout;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool PmBq25120a::Enable()
{
	bool retval = false;

	return retval;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void PmBq25120a::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void PmBq25120a::Reset()
{

}

void PmBq25120a::PowerOff()
{

}

uint32_t PmBq25120a::SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
{
	return 0;
}

bool PmBq25120a::Charging()
{
	uint8_t reg = BQ25120A_STATUS_SHIPMODE_CTRL_REG;
	uint8_t d = Read8(&reg, 1);

	return d & BQ25120A_STATUS_SHIPMODE_CTRL_CHARGING;
}

/**
 * @brief	Battery present status
 *
 * @return	true - Battery present
 */
bool PmBq25120a::Battery()
{
	bool retval = false;

	return retval;
}

void PmBq25120a::IrqHandler()
{

}
