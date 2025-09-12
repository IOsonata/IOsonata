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

#define NPM2100_I2C_DEVICE_ADDR				0x74	// 7 bits device address

#define NPM2100_EVENTS_SYSTEM_SET_REG				0
#define NPM2100_EVENTS_SYSTEM_SET_DIETWARN							(1<<0)	//!< Die temp warning
#define NPM2100_EVENTS_SYSTEM_SET_SHPHLDFALL						(1<<1)	//!< SHPHLD falling edge
#define NPM2100_EVENTS_SYSTEM_SET_SHPHDLRISE						(1<<2)	//!< SHPHLD rising edge
#define NPM2100_EVENTS_SYSTEM_SET_PGRESETFALL						(1<<3)	//!< PG/RESET falling edge
#define NPM2100_EVENTS_SYSTEM_SET_PGRESETRISE						(1<<4)	//!< PG/RESET rising edge
#define NPM2100_EVENTS_SYSTEM_SET_TIMER								(1<<5)	//!< General purpose timer expired
#define NPM2100_EVENTS_SYSTEM_SET_TIMERPREWRN						(1<<6)	//!< Prewarning event before timer expires
#define NPM2100_EVENTS_SYSTEM_SET_TIMERFREE							(1<<7)	//!< Timer free event

#define NPM2100_EVENTS_ADC_SET_REG					0x1
#define NPM2100_EVENTS_ADC_SET_VBATRDY								(1<<0)	//!< ADC VBAT conversion ready
#define NPM2100_EVENTS_ADC_SET_DIETRDY								(1<<1)	//!< ADC die temperature ready
#define NPM2100_EVENTS_ADC_SET_DROOPDET								(1<<2)	//!< ADC droop detector conversion ready
#define NPM2100_EVENTS_ADC_SET_VOUTRDY								(1<<3)	//!< ADC VOUT conversion ready

#define NPM2100_EVENTS_GPIO_SET_REG					0x2
#define NPM2100_EVENTS_GPIO_SET_GPIO0FALL							(1<<0)	//!< Falling edge on GPIO0
#define NPM2100_EVENTS_GPIO_SET_GPIO0RISE							(1<<1)	//!< Rising edge on GPIO0
#define NPM2100_EVENTS_GPIO_SET_GPIO1FALL							(1<<2)	//!< Falling edge on GPIO1
#define NPM2100_EVENTS_GPIO_SET_GPIO1RISE							(1<<3)	//!< Rising edge on GPIO1
#define NPM2100_EVENTS_GPIO_SET_GPIO2FALL							(1<<4)	//!< Falling edge on GPIO2
#define NPM2100_EVENTS_GPIO_SET_GPIO2RISE							(1<<5)	//!< Rising edge on GPIO2

#define NPM2100_EVENTS_BOOST_SET_REG				0x3
#define NPM2100_EVENTS_BOOST_SET_VBATWRNF							(1<<0)	//!< VBAT dropped below VBATMINH threshold
#define NPM2100_EVENTS_BOOST_SET_VBATWRNR							(1<<1)	//!< VBAT rose above VBATMINH threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTMIN							(1<<2)	//!< VOUT dropped bellow VOUTMIN threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTWRNF							(1<<3)	//!< VOUT dropped bellow VOUTWRN threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTWRNR							(1<<4)	//!< VOUT rose above VOUTWRN threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTDPSF							(1<<5)	//!< VOUT dropped bellow VOUTDPS threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTDPSR							(1<<6)	//!< VOUT rose above VOUTDPS threshold
#define NPM2100_EVENTS_BOOST_SET_VOUTOK								(1<<7)	//!< VOUT reached target level after being low

#define NPM2100_EVENTS_LDOSW_SET_REG				0x4
#define NPM2100_EVENTS_LDOSW_SET_OCP								(1<<0)	//!< LDOSW over current
#define NPM2100_EVENTS_LDOSW_SET_VINTFAIL							(1<<1)	//!< VINT didn't recover during LDOSW powerup using digital stepper

#define NPM2100_EVENTS_SYSTEM_CLR_REG				0x5
#define NPM2100_EVENTS_SYSTEM_CLR_DIETWARN							(1<<0)	//!< Die temperature warning
#define NPM2100_EVENTS_SYSTEM_CLR_SHPHLDFALL						(1<<1)
#define NPM2100_EVENTS_SYSTEM_CLR_SHPHLDRISE						(1<<2)
#define NPM2100_EVENTS_SYSTEM_CLR_PGRESTFALL						(1<<3)
#define NPM2100_EVENTS_SYSTEM_CLR_PGRESETRISE						(1<<4)
#define NPM2100_EVENTS_SYSTEM_CLR_TIMER								(1<<5)
#define NPM2100_EVENTS_SYSTEM_CLR_TIMERPREWRN						(1<<6)
#define NPM2100_EVENTS_SYSTEM_CLR_TIMERFREE							(1<<7)

#define NPM2100_EVENTS_ADC_CLR_REG					0x6
#define NPM2100_EVENTS_ADC_CLR_VBATRDY								(1<<0)
#define NPM2100_EVENTS_ADC_CLR_DIETRDY								(1<<1)
#define NPM2100_EVENTS_ADC_CLR_DROOPDET								(1<<2)
#define NPM2100_EVENTS_ADC_CLR_VOUTRDY								(1<<3)

#define NPM2100_EVENTS_GPIO_CLR_REG					0x7
#define NPM2100_EVENTS_GPIO_CLR_GPIO0FALL							(1<<0)
#define NPM2100_EVENTS_GPIO_CLR_GPIO0RISE							(1<<1)
#define NPM2100_EVENTS_GPIO_CLR_GPIO1FALL							(1<<2)
#define NPM2100_EVENTS_GPIO_CLR_GPIO1RISE							(1<<3)
#define NPM2100_EVENTS_GPIO_CLR_GPIO2FALL							(1<<4)
#define NPM2100_EVENTS_GPIO_CLR_GPIO2RISE							(1<<5)

#define NPM2100_EVENTS_BOOST_CLR_REG				0x8
#define NPM2100_EVENTS_BOOST_CLR_VBATWRNF							(1<<0)
#define NPM2100_EVENTS_BOOST_CLR_VBATWRNR							(1<<1)
#define NPM2100_EVENTS_BOOST_CLR_VOUTMIN							(1<<2)
#define NPM2100_EVENTS_BOOST_CLR_VOUTWRNF							(1<<3)
#define NPM2100_EVENTS_BOOST_CLR_VOUTWRNR							(1<<4)
#define NPM2100_EVENTS_BOOST_CLR_VOUTDPSF							(1<<5)
#define NPM2100_EVENTS_BOOST_CLR_VOUTDPSR							(1<<6)
#define NPM2100_EVENTS_BOOST_CLR_VOUTOK								(1<<7)

#define NPM2100_EVENTS_LDOSW_CLR_REG				0x9
#define NPM2100_EVENTS_LDOSW_CLR_OCP								(1<<0)
#define NPM2100_EVENTS_LDOSW_CLR_VINTFAIL							(1<<1)

#define NPM2100_INTEN_SYSTEM_SET_REG				0xA
#define NPM2100_INTEN_SYSTEM_SET_DIETWARN							(1<<0)	//!< Die temp warning
#define NPM2100_INTEN_SYSTEM_SET_SHPHLDFALL							(1<<1)	//!< SHPHLD falling edge
#define NPM2100_INTEN_SYSTEM_SET_SHPHDLRISE							(1<<2)	//!< SHPHLD rising edge
#define NPM2100_INTEN_SYSTEM_SET_PGRESETFALL						(1<<3)	//!< PG/RESET falling edge
#define NPM2100_INTEN_SYSTEM_SET_PGRESETRISE						(1<<4)	//!< PG/RESET rising edge
#define NPM2100_INTEN_SYSTEM_SET_TIMER								(1<<5)	//!< General purpose timer expired
#define NPM2100_INTEN_SYSTEM_SET_TIMERPREWRN						(1<<6)	//!< Prewarning event before timer expires
#define NPM2100_INTEN_SYSTEM_SET_TIMERFREE							(1<<7)	//!< Timer free event

#define NPM2100_INTEN_ADC_SET_REG					0xB
#define NPM2100_INTEN_ADC_SET_VBATRDY								(1<<0)	//!< ADC VBAT conversion ready
#define NPM2100_INTEN_ADC_SET_DIETRDY								(1<<1)	//!< ADC die temperature ready
#define NPM2100_INTEN_ADC_SET_DROOPDET								(1<<2)	//!< ADC droop detector conversion ready
#define NPM2100_INTEN_ADC_SET_VOUTRDY								(1<<3)	//!< ADC VOUT conversion ready

#define NPM2100_INTEN_GPIO_SET_REG					0xC
#define NPM2100_INTEN_GPIO_SET_GPIO0FALL							(1<<0)	//!< Falling edge on GPIO0
#define NPM2100_INTEN_GPIO_SET_GPIO0RISE							(1<<1)	//!< Rising edge on GPIO0
#define NPM2100_INTEN_GPIO_SET_GPIO1FALL							(1<<2)	//!< Falling edge on GPIO1
#define NPM2100_INTEN_GPIO_SET_GPIO1RISE							(1<<3)	//!< Rising edge on GPIO1
#define NPM2100_INTEN_GPIO_SET_GPIO2FALL							(1<<4)	//!< Falling edge on GPIO2
#define NPM2100_INTEN_GPIO_SET_GPIO2RISE							(1<<5)	//!< Rising edge on GPIO2

#define NPM2100_INTEN_BOOST_SET_REG					0xD
#define NPM2100_INTEN_BOOST_SET_VBATWRNF							(1<<0)	//!< VBAT dropped below VBATMINH threshold
#define NPM2100_INTEN_BOOST_SET_VBATWRNR							(1<<1)	//!< VBAT rose above VBATMINH threshold
#define NPM2100_INTEN_BOOST_SET_VOUTMIN								(1<<2)	//!< VOUT dropped bellow VOUTMIN threshold
#define NPM2100_INTEN_BOOST_SET_VOUTWRNF							(1<<3)	//!< VOUT dropped bellow VOUTWRN threshold
#define NPM2100_INTEN_BOOST_SET_VOUTWRNR							(1<<4)	//!< VOUT rose above VOUTWRN threshold
#define NPM2100_INTEN_BOOST_SET_VOUTDPSF							(1<<5)	//!< VOUT dropped bellow VOUTDPS threshold
#define NPM2100_INTEN_BOOST_SET_VOUTDPSR							(1<<6)	//!< VOUT rose above VOUTDPS threshold
#define NPM2100_INTEN_BOOST_SET_VOUTOK								(1<<7)	//!< VOUT reached target level after being low

#define NPM2100_INTEN_LDOSW_SET_REG					0xE
#define NPM2100_INTEN_LDOSW_SET_OCP									(1<<0)	//!< LDOSW over current
#define NPM2100_INTEN_LDOSW_SET_VINTFAIL							(1<<1)	//!< VINT didn't recover during LDOSW powerup using digital stepper

#define NPM2100_INTEN_SYSTEM_CLR_REG				0xF
#define NPM2100_INTEN_SYSTEM_CLR_DIETWARN							(1<<0)	//!< Die temperature warning
#define NPM2100_INTEN_SYSTEM_CLR_SHPHLDFALL							(1<<1)
#define NPM2100_INTEN_SYSTEM_CLR_SHPHLDRISE							(1<<2)
#define NPM2100_INTEN_SYSTEM_CLR_PGRESTFALL							(1<<3)
#define NPM2100_INTEN_SYSTEM_CLR_PGRESETRISE						(1<<4)
#define NPM2100_INTEN_SYSTEM_CLR_TIMER								(1<<5)
#define NPM2100_INTEN_SYSTEM_CLR_TIMERPREWRN						(1<<6)
#define NPM2100_INTEN_SYSTEM_CLR_TIMERFREE							(1<<7)

#define NPM2100_INTEN_ADC_CLR_REG					0x10
#define NPM2100_INTEN_ADC_CLR_VBATRDY								(1<<0)	//!< ADC VBAT conversion ready
#define NPM2100_INTEN_ADC_CLR_DIETRDY								(1<<1)	//!< ADC die temperature ready
#define NPM2100_INTEN_ADC_CLR_DROOPDET								(1<<2)	//!< ADC droop detector conversion ready
#define NPM2100_INTEN_ADC_CLR_VOUTRDY								(1<<3)	//!< ADC VOUT conversion ready

#define NPM2100_INTEN_GPIO_CLR_REG					0x11
#define NPM2100_INTEN_GPIO_CLR_GPIO0FALL							(1<<0)
#define NPM2100_INTEN_GPIO_CLR_GPIO0RISE							(1<<1)
#define NPM2100_INTEN_GPIO_CLR_GPIO1FALL							(1<<2)
#define NPM2100_INTEN_GPIO_CLR_GPIO1RISE							(1<<3)
#define NPM2100_INTEN_GPIO_CLR_GPIO2FALL							(1<<4)
#define NPM2100_INTEN_GPIO_CLR_GPIO2RISE							(1<<5)

#define NPM2100_INTEN_BOOST_CLR_REG					0x12
#define NPM2100_INTEN_BOOST_CLR_VBATWRNF							(1<<0)
#define NPM2100_INTEN_BOOST_CLR_VBATWRNR							(1<<1)
#define NPM2100_INTEN_BOOST_CLR_VOUTMIN								(1<<2)
#define NPM2100_INTEN_BOOST_CLR_VOUTWRNF							(1<<3)
#define NPM2100_INTEN_BOOST_CLR_VOUTWRNR							(1<<4)
#define NPM2100_INTEN_BOOST_CLR_VOUTDPSF							(1<<5)
#define NPM2100_INTEN_BOOST_CLR_VOUTDPSR							(1<<6)
#define NPM2100_INTEN_BOOST_CLR_VOUTOK								(1<<7)

#define NPM2100_INTEN_LDOSW_CLR_REG					0x13
#define NPM2100_INTEN_LDOSW_CLR_OCP									(1<<0)
#define NPM2100_INTEN_LDOSW_CLR_VINTFAIL							(1<<1)

#define NPM2100_REQUESTSET_REG						0x14
#define NPM2100_REQUESTSET_DIETEMP									(1<<0)	//!< Start die temperature monitoring
#define NPM2100_REQUESTSET_DIETEMPENA								(1<<1)	//!< Enable die temperature monitoring when BOOST is in HP mode

#define NPM2100_REQUESTCLR_REG						0x15
#define NPM2100_REQUESTCLR_DIETEMP									(1<<0)	//!< Stop die temperature monitoring
#define NPM2100_REQUESTCLR_DIETEMPENA								(1<<1)	//!< Disable die temperature monitoring when BOOST is in HP mode

#define NPM2100_STATUS_REG							0x16
#define NPM2100_STATUS_SHPHLD										(1<<0)	//!< Status of SHPHLD pin
#define NPM2100_STATUS_PGRESET										(1<<1)	//!< Status of PG/RESET pin
#define NPM2100_STATUS_DIETEMP										(1<<2)	//!< Status of thermal warning

#define NPM2100_BOOST_TASKS_START_REG				0x20
#define NPM2100_BOOST_TASKS_START_PULSECNT							(1<<0)	//!< Start coil current pulse counter for DPS mode
#define NPM2100_BOOST_TASKS_START_DPSDUR							(1<<1)	//!< Start DPS mode duration measurement

#define NPM2100_BOOST_VOUT_REG						0x21
#define NPM2100_BOOST_VOUT_MASK										(0x1F<<0)
#define NPM2100_BOOST_VOUT_1V8										(0)
#define NPM2100_BOOST_VOUT_3V3										(30)

#define NPM2100_BOOST_VOUTSEL_REG					0x23
#define NPM2100_BOOST_VOUTSEL_PIN									(0<<0)	//!< Output voltage set by pin VSET
#define NPM2100_BOOST_VOUTSEL_REG									(1<<0)	//!< Output voltage set by BOOST.VOUT reg

#define NPM2100_BOOST_OPER_REG						0x24
#define NPM2100_BOOST_OPER_MODE_MASK								(7<<0)
#define NPM2100_BOOST_OPER_MODE_AUTO								(0<<0)	//!< Auto (HP/LP/ULP/PT) mode
#define NPM2100_BOOST_OPER_MODE_HP									(1<<0)	//!< Forced high power (HP) mode
#define NPM2100_BOOST_OPER_MODE_LP									(1<<0)	//!< Forced low power (LP) mode
#define NPM2100_BOOST_OPER_MODE_PT									(1<<0)	//!< Forced pass-through (PT) mode
#define NPM2100_BOOST_OPER_MODE_NOHP								(1<<0)	//!< Forced prevent high power mode
#define NPM2100_BOOST_OPER_DPS_MASK									(3<<3)
#define NPM2100_BOOST_OPER_DPS_DIS									(0<<3)	//!< Disable DPS
#define NPM2100_BOOST_OPER_DPS_ALLOW								(1<<3)	//!< Allow DPS mode
#define NPM2100_BOOST_OPER_DPS_ALLOWLP								(2<<3)	//!< Allow DPS in LP mode only
#define NPM2100_BOOST_OPER_DPSTIMER_MASK							(3<<5)
#define NPM2100_BOOST_OPER_DPSTIMER_100US							(0<<5)
#define NPM2100_BOOST_OPER_DPSTIMER_200US							(1<<5)
#define NPM2100_BOOST_OPER_DPSTIMER_400US							(2<<5)
#define NPM2100_BOOST_OPER_DPSTIMER_800US							(3<<5)

#define NPM2100_BOOST_COUNT_REG						0x25	//!< Number of counted coil current pulse per refresh period in DPS

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
