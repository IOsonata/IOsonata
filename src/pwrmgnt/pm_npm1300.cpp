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

#include "convutil.h"
#include "pwrmgnt/pm_npm1300.h"

bool PmnPM1300::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);

	uint16_t regaddr = EndianCvt16(NPM1300_DEVICE_INFO_REG);
	uint8_t buff[10];

	//Write((uint8_t*)&regaddr, 2, 0, 0);
	uint32_t d = 0xFF;

#if 1
	Read((uint8_t*)&regaddr, 2, (uint8_t*)buff, 9);

	for (int i = 0; i < 10; i++)
	{
		printf("%x ", buff[i]);
	}
	printf("\n\r");
#else
	Read((uint8_t*)&regaddr, 2, (uint8_t*)&d, 4);
	printf("%x\r\n", d);
	if (d != NPM1300_DEVICE_ID)
	{
		return false;
	}
#endif

	regaddr = EndianCvt16(NPM1300_VBUSIN_USBC_DETECT_STATUS_REG);
	d = Read8((uint8_t*)&regaddr, 2);
	printf("USBC_DETECT=%x\n", d);

	//if (d & (NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC1CMP_1A5HIGHPOWER | NPM1300_VBUSIN_USBC_DETECT_STATUS_VBUSINCC2CMP_1A5HIGHPOWER))
	{
		regaddr = EndianCvt16(NPM1300_VBUSIN_VBUSIN_ILIM0_REG);
		d = NPM1300_VBUSIN_VBUSIN_ILIM0_1500MA;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = EndianCvt16(NPM1300_VBUSIN_TASKUPDATE_ILIMSW_REG);
		d = NPM1300_VBUSIN_TASKUPDATE_ILIMSW_SELVBUSILIM0;
		Write8((uint8_t*)&regaddr, 2, d);
	}

	regaddr = EndianCvt16(NPM1300_VBUSIN_VBUSIN_STATUS_REG);
	d = Read8((uint8_t*)&regaddr, 2);
	printf("VBUSIN_STATUS=%x\n", d);

	// LED0
	regaddr = EndianCvt16(NPM1300_LEDDRV_LEDDRV0_MODESEL_REG);
	d = NPM1300_LEDDRV_LEDDRV0_MODESEL_CHARGING;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = EndianCvt16(NPM1300_BUCK_BUCK2ENA_SET_REG);
	d = NPM1300_BUCK_BUCK2ENA_SET_TASKBUCK2ENASET;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = EndianCvt16(NPM1300_BUCK_BUCK2NORM_VOUT_REG);
	d = Read8((uint8_t*)&regaddr, 2);
	printf("BUCK2NORM_VOUT=%d\n", d);

	regaddr = EndianCvt16(NPM1300_BUCK_BUCK2RET_VOUT_REG);
	d = NPM1300_BUCK_BUCK2RET_VOUT_3V3;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = EndianCvt16(NPM1300_BUCK_BUCK2RET_VOUT_REG);
	d = Read8((uint8_t*)&regaddr, 2);
	printf("BUCK2RET_VOUT=%d\n", d);

	SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, Cfg.VEndChrg, Cfg.ChrgCurr);

	if (Cfg.bIntEn == true)
	{
		// Configure GPIO1 as interrupt pin
		regaddr = EndianCvt16(NPM1300_GPIOS_GPIOPDEN1_REG);
		d = NPM1300_GPIOS_GPIOPDEN_ENABLE;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = EndianCvt16(NPM1300_GPIOS_GPIOMODE1_REG);
		d = NPM1300_GPIOS_GPIOMODE_GPOIRQ;
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = EndianCvt16(NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_REG);
		d = NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_EVENTVBUSDETECTED |
			NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_EVENTVBUSREMOVED;
		Write8((uint8_t*)&regaddr, 2, d);
	}
/*
	regaddr = NPM1300_BCHARGER_BCHGVTERM_REG;
	d = NPM1300_BCHARGER_BCHGVTERM_BCHGVTERMNORM_4V00;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = NPM1300_BCHARGER_BCHGISET_MSB_REG;
	d = 200;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = NPM1300_BCHARGER_BCHGENABLE_SET_REG;
	d = NPM1300_BCHARGER_BCHGENABLE_SET_ENABLECHARGING;

	Write8((uint8_t*)&regaddr, 2, d);
*/
	return true;
}

bool PmnPM1300::Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt)
{

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

}

int32_t PmnPM1300::Voltage()
{

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
	uint16_t regaddr = VoutIdx == 0 ? NPM1300_BUCK_BUCK1NORM_VOUT_REG : NPM1300_BUCK_BUCK2NORM_VOUT_REG;
	regaddr = EndianCvt16(regaddr);
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

}

/**
 * @brief	Reset device to it initial default state
 */
void PmnPM1300::Reset()
{

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
	uint16_t regaddr = EndianCvt16(NPM1300_BCHARGER_BCHGISET_MSB_REG);
	uint8_t msb = mACurr >> 2;
	uint8_t lsb = (mACurr >> 1) & 1;
	uint8_t v = 0;

	if (msb < 8)
	{
		// 32 mA
		msb = 8;
		lsb = 0;
	}
	else if (msb > 200)
	{
		msb = 200;
		lsb = 0;
	}

	printf("%d : msb %d, lsb %d\r\n", mACurr, msb, lsb);

	Write8((uint8_t*)&regaddr, 2, msb);

	regaddr = EndianCvt16(NPM1300_BCHARGER_BCHGISET_LSB_REG);
	Write8((uint8_t*)&regaddr, 2, lsb);

	if (mVoltEoC < 3500)
	{
		mVoltEoC = 3500;
	}
	else if (mVoltEoC > 4450)
	{
		mVoltEoC = 4450;
	}

	if (mVoltEoC < 4000)
	{
		v = (mVoltEoC - 3500) / 50;
	}
	else
	{
		v = (mVoltEoC - 4000) / 50 + 4;
	}

	printf("%d : %d\r\n", mVoltEoC, v);
	regaddr = EndianCvt16(NPM1300_BCHARGER_BCHGVTERM_REG);
	Write8((uint8_t*)&regaddr, 2, v);

	regaddr = EndianCvt16(NPM1300_BCHARGER_BCHG_ERR_REASON_REG);
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	printf("Reason %x\r\n", d);

	regaddr = EndianCvt16(NPM1300_BCHARGER_BCHGENABLE_SET_REG);
	d = NPM1300_BCHARGER_BCHGENABLE_SET_ENABLECHARGING | NPM1300_BCHARGER_BCHGENABLE_SET_ENABLEFULLCHGCOOL;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = EndianCvt16(NPM1300_BCHARGER_BCHG_ERR_REASON_REG);
	d = Read8((uint8_t*)&regaddr, 2);

	printf("Reason %x\r\n", d);

	return ((uint32_t)msb << 2UL) + ((uint32_t)lsb << 1UL);
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

}
