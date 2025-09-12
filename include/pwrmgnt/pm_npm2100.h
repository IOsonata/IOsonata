/**-------------------------------------------------------------------------
@file	pm_npm2100.h

@brief	Power management implementation of the Nordic nPM1300

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	Sep. 12, 2025

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
#ifndef __PM_NPM2100_H__
#define __PM_NPM2100_H__

#include <stdint.h>

#include "pwrmgnt/pwrmgnt.h"

/** @addtogroup Power
  * @{
  */

#define NPM2100_DEVICE_ADDR				0x74	// 7 bits device address

#define NPM2100_EVENTS_SYSTEM_SET_REG				0
#define NPM2100_EVENTS_ADC_SET_REG					0x1
#define NPM2100_EVENTS_GPIO_SET_REG					0x2
#define NPM2100_EVENTS_BOOST_SET_REG				0x3
#define NPM2100_EVENTS_LDOSW_SET_REG				0x4
#define NPM2100_EVENTS_SYSTEM_CLR_REG				0x5
#define NPM2100_EVENTS_ADC_CLR_REG					0x6
#define NPM2100_EVENTS_GPIO_CLR_REG					0x7
#define NPM2100_EVENTS_BOOST_CLR_REG				0x8
#define NPM2100_EVENTS_LDOSW_CLR_REG				0x9
#define NPM2100_INTEN_SYSTEM_SET_REG				0xA
#define NPM2100_INTEN_ADC_SET_REG					0xB
#define NPM2100_INTEN_GPIO_SET_REG					0xC
#define NPM2100_INTEN_BOOST_SET_REG					0xD
#define NPM2100_INTEN_LDOSW_SET_REG					0xE
#define NPM2100_INTEN_SYSTEM_CLR_REG				0xF
#define NPM2100_INTEN_ADC_CLR_REG					0x10
#define NPM2100_INTEN_GPIO_CLR_REG					0x11
#define NPM2100_INTEN_BOOST_CLR_REG					0x12
#define NPM2100_INTEN_LDOSW_CLR_REG					0x13
#define NPM2100_REQUESTSET_REG						0x14
#define NPM2100_REQUESTCLR_REG						0x15
#define NPM2100_STATUS_REG							0x16

#define NPM2100_BOOST_TASKS_START_REG				0x20
#define NPM2100_BOOST_VOUT_REG						0x21
#define NPM2100_BOOST_VOUTSEL_REG					0x23
#define NPM2100_BOOST_OPER_REG						0x24
#define NPM2100_BOOST_COUNT_REG						0x25
#define NPM2100_BOOST_LIMIT_REG						0x26
#define NPM2100_BOOST_DPS_REG						0x27
#define NPM2100_BOOST_GPIO_REG						0x28
#define NPM2100_BOOST_PIN_REG						0x29
#define NPM2100_BOOST_CTRLSET_REG					0x2A
#define NPM2100_BOOST_CTRLCLR_REG					0x2B
#define NPM2100_BOOST_IBATLIM_REG					0x2D
#define NPM2100_BOOST_VBATMINLHSEL_REG				0x2E
#define NPM2100_BOOST_VBATMINL_REG					0x2F
#define NPM2100_BOOST_VBATMINH_REG					0x30
#define NPM2100_BOOST_VOUTMIN_REG					0x31
#define NPM2100_BOOST_VOUTMIN_WRN_REG				0x32
#define NPM2100_BOOST_VOUTDPS_REG					0x33
#define NPM2100_BOOST_STATUS0_REG					0x34
#define NPM2100_BOOST_STATUS1_REG					0x35

#define NPM2100_LDOSW_VOUT_REG						0x68
#define NPM2100_LDOSW_LDOSW_REG						0x69
#define NPM2100_LDOSW_SEL_REG						0x6A
#define NPM2100_LDOSW_GPIO_REG						0x6B
#define NPM2100_LDOSW_CONF_REG						0x6C
#define NPM2100_LDOSW_RAMP_REG						0x6D
#define NPM2100_LDOSW_STATUS_REG					0x6E
#define NPM2100_LDOSW_PRGOCP_REG					0x6F

#define NPM2100_GPIO_CONFIG0_REG					0x80
#define NPM2100_GPIO_CONFIG1_REG					0x81
#define NPM2100_GPIO_USAGE0_REG						0x83
#define NPM2100_GPIO_USAGE1_REG						0x84
#define NPM2100_GPIO_OUTPUT0_REG					0x86
#define NPM2100_GPIO_OUTPUT1_REG					0x87
#define NPM2100_GPIO_READ_REG						0x89

#define NPM2100_ADC_TASKS_ADC_REG					0x90
#define NPM2100_ADC_CONFIG_REG						0x91
#define NPM2100_ADC_DELAY_REG						0x92
#define NPM2100_ADC_OFFSETCFG_REG					0x93
#define NPM2100_ADC_CTRLSET_REG						0x94
#define NPM2100_ADC_CTRLCLR_REG						0x95
#define NPM2100_ADC_READVBAT_REG					0x96
#define NPM2100_ADC_READTEMP_REG					0x97
#define NPM2100_ADC_READDROOP_REG					0x98
#define NPM2100_ADC_READVOUT_REG					0x99
#define NPM2100_ADC_VOUTRECOV_REG					0x9A
#define NPM2100_ADC_AVERAGE_REG						0x9B
#define NPM2100_ADC_BOOST_REG						0x9C
#define NPM2100_ADC_STATUS_REG						0x9D
#define NPM2100_ADC_OFFSETFACTORY_REG				0x9E
#define NPM2100_ADC_OFFSETMEASURED_REG				0x9F

#define NPM2100_TIMER_TASKS_START_REG				0xB0
#define NPM2100_TIMER_TASKS_STOP_REG				0xB1
#define NPM2100_TIMER_TASKS_KICK_REG				0xB2
#define NPM2100_TIMER_CONFIG_REG					0xB3
#define NPM2100_TIMER_TARGETHI_REG					0xB4
#define NPM2100_TIMER_TARGETMID_REG					0xB5
#define NPM2100_TIMER_TARGETLO_REG					0xB6
#define NPM2100_TIMER_STATUS_REG					0xB7

#define NPM2100_SHIP_TASKS_SHIP_REG					0xC0
#define NPM2100_SHIP_WAKEUP_REG						0xC1
#define NPM2100_SHIP_SHPHLD_REG						0xC2

#define NPM2100_RESET_TASKS_RESET_REG				0xD0
#define NPM2100_RESET_TASKS_CLR_REG					0xD1
#define NPM2100_RESET_BUTTON_REG					0xD2
#define NPM2100_RESET_PIN_REG						0xD3
#define NPM2100_RESET_DEBOUNCE_REG					0xD4
#define NPM2100_RESET_RESET_REG						0xD5
#define NPM2100_RESET_ALTCONFIG_REG					0xD6
#define NPM2100_RESET_WRITE_REG						0xD7
#define NPM2100_RESET_STROBE_REG					0xD8
#define NPM2100_RESET_READ_REG						0xD9
#define NPM2100_RESET_SCRATCHB_REG					0xDA
#define NPM2100_RESET_WRITESTICKY_REG				0xDB
#define NPM2100_RESET_STROBESTICKY_REG				0xDC
#define NPM2100_RESET_READSTICKY_REG				0xDD
#define NPM2100_RESET_SYSGDENSTATUS_REG				0xDF




#ifdef __cplusplus

class PmnPM2100 : public PowerMgnt, public  FuelGauge {
public:
	bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf);
	bool Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt);

	/**
	 * @brief	Get battery level
	 *
	 * Returns battery level in 1 digit fixed point decimal.
	 *
	 * ex. 123 => 12.3%
	 *
	 * @return	Battery level in (0-100) % in 1 digit fixed point
	 */
	virtual uint16_t Level();

	/**
	 * @brief	Get battery temperature
	 *
	 * Returns battery temperature in 1 digit fixed point decimal.
	 *
	 * ex. 123 => 12.3 C
	 *
	 * @return	Battery level in (0-100) degree C in 1 digit fixed point
	 */
	virtual int32_t Temperature();

	virtual int32_t Voltage();

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
	int32_t SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	void Disable();

	/**
	 * @brief	Reset device to it initial default state
	 */
	void Reset();

	void PowerOff();

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
	uint32_t SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr);

	/**
	 * @brief	Charging status
	 *
	 * @return	true - Charging
	 */
	virtual bool Charging();

	/**
	 * @brief	Battery present status
	 *
	 * @return	true - Battery present
	 */
	virtual bool Battery();
	/**
	 * @brief	Interrupt handler
	 *
	 * Optional implementation to handle interrupt. This is device specific.
	 *
	 */
	virtual void IrqHandler();

private:
};

extern "C" {
#endif	// __cplusplus


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif // __PM_NPM2100_H__
