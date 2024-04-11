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

#include "pwrmgnt/pm_npm1300.h"

bool PmnPM1300::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	return true;
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

}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool PmnPM1300::Enable()
{

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

}

/**
 * @brief	Charging status
 *
 * @return	true - Charging
 */
bool PmnPM1300::Charging()
{

}

/**
 * @brief	Battery present status
 *
 * @return	true - Battery present
 */
bool PmnPM1300::Battery()
{

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
