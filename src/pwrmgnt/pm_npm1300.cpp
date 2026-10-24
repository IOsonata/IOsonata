/**-------------------------------------------------------------------------
@file	pm_npm1300.cpp

@brief	Power management implementation of the Nordic nPM1300

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	Apr. 10, 2024

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
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "convutil.h"
#include "pwrmgnt/pm_npm1300.h"

bool PmnPM1300::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	uint16_t addr;
	uint8_t info[4];

	if (pIntrf == NULL || (Cfg.NbVout > 0 && Cfg.pVout == NULL))
		return false;

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);
	vpEvtHandler = Cfg.pEvtHandler;
	vNtcBeta = Cfg.pBatProf != NULL ? Cfg.pBatProf->ThermBetaConst : 0;

	addr = EndianCvt16(NPM1300_DEVICE_INFO_REG);
	if (Read((uint8_t *)&addr, 2, info, sizeof(info)) != sizeof(info))
		return false;

	addr = EndianCvt16(NPM1300_ADC_ADCCONFIG_REG);
	if (!Write8((uint8_t *)&addr, 2, NPM1300_ADC_ADCCONFIG_AUTOENABLE))
		return false;

	if (Cfg.ChrgCurr == 0)
		Disable();
	else if (SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, Cfg.VEndChrg, Cfg.ChrgCurr) == 0)
		return false;

	for (size_t i = 0; i < Cfg.NbVout; i++)
	{
		if (SetVout(i, Cfg.pVout[i].mVout, Cfg.pVout[i].mAlimit) < 0)
			return false;
	}

	if (Cfg.bIntEn)
	{
		addr = EndianCvt16(NPM1300_GPIOS_GPIOMODE1_REG);
		if (!Write8((uint8_t *)&addr, 2, NPM1300_GPIOS_GPIOMODE_GPOIRQ))
			return false;
		addr = EndianCvt16(NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_REG);
		if (!Write8((uint8_t *)&addr, 2,
			NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_EVENTVBUSDETECTED |
			NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_EVENTVBUSREMOVED))
			return false;
	}
	return true;
}

bool PmnPM1300::Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf,
	PowerMgnt * const pPwrMnt)
{
	(void)pPwrMnt;
	if (pIntrf == NULL)
		return false;
	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);
	vNtcBeta = Cfg.BatProf.ThermBetaConst;
	return true;
}

/**
 * @brief	Get battery level
 *
 * Returns battery level in 1 digit fixed point decimal.
 *
 * ex. 123 => 12.3%
 *
 * @return	Battery level in (0-100) % in 1 digit fixed point
 */
uint16_t PmnPM1300::Level()
{
	return 0;
}

/**
 * @brief	Get battery temperature
 *
 * Returns battery temperature in 1 digit fixed point decimal.
 *
 * ex. 123 => 12.3 C
 *
 * @return	Battery level in (0-100) degree C in 1 digit fixed point
 */
int32_t PmnPM1300::Temperature()
{
	uint16_t addr;
	uint8_t msb;
	uint8_t lsb;
	uint16_t adc;
	float kelvin;

	if (vNtcBeta == 0)
		return INT32_MIN;
	addr = EndianCvt16(NPM1300_ADC_ADCNTC_RESULTMSB_REG);
	if (Read((uint8_t *)&addr, 2, &msb, 1) != 1)
		return INT32_MIN;
	addr = EndianCvt16(NPM1300_ADC_ADCGP0_RESULTLSBS_REG);
	if (Read((uint8_t *)&addr, 2, &lsb, 1) != 1)
		return INT32_MIN;
	adc = ((uint16_t)msb << 2) | ((lsb >> 2) & 3);
	if (adc == 0 || adc >= 1023)
		return INT32_MIN;
	kelvin = 1.0f / (1.0f / 298.15f - logf(1024.0f / adc - 1.0f) / vNtcBeta);
	return (int32_t)((kelvin - 273.15f) * 10.0f);
}

int32_t PmnPM1300::Voltage()
{
	uint16_t addr;
	uint8_t msb;
	uint8_t lsb;
	uint16_t adc;

	addr = EndianCvt16(NPM1300_ADC_ADCVBAT_RESULTMSB_REG);
	if (Read((uint8_t *)&addr, 2, &msb, 1) != 1)
		return -1;
	addr = EndianCvt16(NPM1300_ADC_ADCGP0_RESULTLSBS_REG);
	if (Read((uint8_t *)&addr, 2, &lsb, 1) != 1)
		return -1;
	adc = ((uint16_t)msb << 2) | (lsb & 3);
	return (int32_t)(((uint32_t)adc * 5000UL + 511UL) / 1023UL);
}

/**
 * @brief	Set output voltage
 *
 * If output voltage is zero, turn off the output.
 *
 * @param	VoutIdx : Zero based index of output source
 * @param	mVolt : Output voltage in mV
 * @param	mALimit : Output current limit in mA if available
 * 						set to zero for max capacity
 *
 * @return	Actual output voltage in mV
 *
 */
int32_t PmnPM1300::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit)
{
	uint16_t addr;
	uint16_t enableReg;
	uint8_t code;

	(void)CurrLimit;
	if (VoutIdx > 1 || (mVolt != 0 &&
		(mVolt < 1000 || mVolt > 3300 || (mVolt - 1000) % 100 != 0)))
		return -1;
	if (mVolt != 0)
	{
		addr = EndianCvt16(VoutIdx == 0 ?
			NPM1300_BUCK_BUCK1NORM_VOUT_REG : NPM1300_BUCK_BUCK2NORM_VOUT_REG);
		code = (uint8_t)((mVolt - 1000) / 100);
		if (!Write8((uint8_t *)&addr, 2, code))
			return -1;
	}
	enableReg = VoutIdx == 0 ?
		(mVolt ? NPM1300_BUCK_BUCK1ENA_SET_REG : NPM1300_BUCK_BUCK1ENA_CLR_REG) :
		(mVolt ? NPM1300_BUCK_BUCK2ENA_SET_REG : NPM1300_BUCK_BUCK2ENA_CLR_REG);
	addr = EndianCvt16(enableReg);
	return Write8((uint8_t *)&addr, 2, 1) ? mVolt : -1;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool PmnPM1300::Enable()
{
	uint16_t regaddr = EndianCvt16(NPM1300_BCHARGER_BCHGENABLE_SET_REG);
	uint8_t d = NPM1300_BCHARGER_BCHGENABLE_SET_ENABLECHARGING;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void PmnPM1300::Disable()
{
	uint16_t addr;

	addr = EndianCvt16(NPM1300_BCHARGER_BCHGENABLE_CLR_REG);
	Write8((uint8_t *)&addr, 2, NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING);
}

/**
 * @brief	Reset device to it initial default state
 */
void PmnPM1300::Reset()
{
	uint16_t addr;

	addr = EndianCvt16(NPM1300_SHIP_TASKRESET_CONFIG_REG);
	Write8((uint8_t *)&addr, 2, 1);
}

void PmnPM1300::PowerOff()
{
	uint16_t regaddr = EndianCvt16(NPM1300_VBUSIN_VBUSIN_STATUS_REG);
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	if (d & NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINPRESENT_DETECTED)
	{
		return;
	}

	regaddr = EndianCvt16(NPM1300_SHIP_TASKENTER_SHIPMODE_REG);
	d = 1;
	Write8((uint8_t*)&regaddr, 2, d);
}

/**
 * @brief	Set battery charging
 *
 * If charge current is set to zero, charging is turned off
 *
 * @param	Type : Charging type
 * @param	mVoltEoC : End of charge voltage in mV
 * @param	mACurr : Charge current limit
 * 					0 : Disable charge
 *
 * @return	Actual charge current set.
 */
uint32_t PmnPM1300::SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
{
	uint16_t addr;
	uint16_t raw;
	uint8_t code;

	if (mACurr == 0)
	{
		Disable();
		return 0;
	}
	if (Type != PWRMGNT_CHARGE_TYPE_NORMAL && Type != PWRMGNT_CHARGE_TYPE_AUTO)
		return 0;
	if (mVoltEoC < 3500 || mVoltEoC > 4450 || mVoltEoC % 50 != 0)
		return 0;
	if (mVoltEoC >= 3700 && mVoltEoC < 4000)
		return 0;
	if (mACurr < 32 || mACurr > 800)
		return 0;

	raw = (uint16_t)(mACurr / 2);
	addr = EndianCvt16(NPM1300_BCHARGER_BCHGISET_MSB_REG);
	if (!Write8((uint8_t *)&addr, 2, raw >> 1))
		return 0;
	addr = EndianCvt16(NPM1300_BCHARGER_BCHGISET_LSB_REG);
	if (!Write8((uint8_t *)&addr, 2, raw & 1))
		return 0;
	code = mVoltEoC < 3700 ? (mVoltEoC - 3500) / 50 : (mVoltEoC - 4000) / 50 + 4;
	addr = EndianCvt16(NPM1300_BCHARGER_BCHGVTERM_REG);
	if (!Write8((uint8_t *)&addr, 2, code))
		return 0;
	return Enable() ? raw * 2 : 0;
}

/**
 * @brief	Charging status
 *
 * @return	true - Charging
 */
bool PmnPM1300::Charging()
{
	uint16_t regaddr = EndianCvt16(NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG);
	uint8_t	d = 0;

	d = Read8((uint8_t*)&regaddr, 2);

	return d & (NPM1300_BCHARGER_BCHG_CHARGE_STATUS_TRICKLECHARGE |
				NPM1300_BCHARGER_BCHG_CHARGE_STATUS_CONSTANTCURRENT |
				NPM1300_BCHARGER_BCHG_CHARGE_STATUS_CONSTANTVOLTAGE |
				NPM1300_BCHARGER_BCHG_CHARGE_STATUS_RECHARGE);
}

/**
 * @brief	Battery present status
 *
 * @return	true - Battery present
 */
bool PmnPM1300::Battery()
{
	uint16_t regaddr = EndianCvt16(NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG);
	uint8_t	d = 0;

	d = Read8((uint8_t*)&regaddr, 2);

	return d & NPM1300_BCHARGER_BCHG_CHARGE_STATUS_BATTERYDETECTED;
}

/**
 * @brief	Interrupt handler
 *
 * Optional implementation to handle interrupt. This is device specific.
 *
 */
void PmnPM1300::IrqHandler()
{
	uint16_t addr;
	uint8_t events;

	addr = EndianCvt16(NPM1300_MAIN_EVENTS_VBUSIN0SET_REG);
	if (Read((uint8_t *)&addr, 2, &events, 1) != 1 || events == 0)
		return;
	addr = EndianCvt16(NPM1300_MAIN_EVENTS_VBUSIN0CLR_REG);
	Write8((uint8_t *)&addr, 2, events);
	if (vpEvtHandler != NULL &&
		(events & NPM1300_MAIN_EVENTS_VBUSIN0SET_EVENTVBUSDETECTED) != 0)
		vpEvtHandler(this, PWRMGNT_EVT_CHARGE_DETECTED);
}
