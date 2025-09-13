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
#define NPM2100_BOOST_LIMIT_PULSES_NOLIMIT							(0)		//!< No limit
#define NPM2100_BOOST_LIMIT_PULSES_3								(3)		//!< 3 pulses
#define NPM2100_BOOST_LIMIT_PULSES_4								(4)		//!< 4 pulses
#define NPM2100_BOOST_LIMIT_PULSES_255								(255)	//!< 255 pulses

#define NPM2100_BOOST_DPS_REG						0x27

#define NPM2100_BOOST_GPIO_REG						0x28
#define NPM2100_BOOST_GPIO_POL_NONE									(0)		//!< No GPIO controls BOOST
#define NPM2100_BOOST_GPIO_POL_GPIO0LO								(1)		//!< GPIO0 active low
#define NPM2100_BOOST_GPIO_POL_GPIO0HI								(2)		//!< GPIO0 active high
#define NPM2100_BOOST_GPIO_POL_GPIO1LO								(3)		//!< GPIO1 active low
#define NPM2100_BOOST_GPIO_POL_GPIO1HI								(4)		//!< GPIO1 active high
#define NPM2100_BOOST_GPIO_POL_GPIO2LO								(5)		//!< GPIO2 active low
#define NPM2100_BOOST_GPIO_POL_GPIO2HI								(6)		//!< GPIO2 active high


#define NPM2100_BOOST_PIN_REG						0x29
#define NPM2100_BOOST_PIN_FORCE_HP									(0)		//!< Active GPIO forces HP
#define NPM2100_BOOST_PIN_FORCE_LP									(1)		//!< Active GPIO forces LP
#define NPM2100_BOOST_PIN_FORCE_PT									(2)		//!< Active GPIO forces PT
#define NPM2100_BOOST_PIN_FORCE_NOHP								(3)		//!< Active GPIO forces NOHP

#define NPM2100_BOOST_CTRLSET_REG					0x2A
#define NPM2100_BOOST_CTRLSET_VOUTMIN								(1<<0)
#define NPM2100_BOOST_CTRLSET_VOUTWRN								(1<<1)
#define NPM2100_BOOST_CTRLSET_VOUTDPS								(1<<2)
#define NPM2100_BOOST_CTRLSET_OCP									(1<<3)
#define NPM2100_BOOST_CTRLSET_VBATMINSEL							(1<<4)

#define NPM2100_BOOST_CTRLCLR_REG					0x2B
#define NPM2100_BOOST_CTRLCLR_VOUTMIN								(1<<0)
#define NPM2100_BOOST_CTRLCLR_VOUTWRN								(1<<1)
#define NPM2100_BOOST_CTRLCLR_VOUTDPS								(1<<2)
#define NPM2100_BOOST_CTRLCLR_OCP									(1<<3)
#define NPM2100_BOOST_CTRLCLR_VBATMINSEL							(1<<4)

#define NPM2100_BOOST_IBATLIM_REG					0x2D
#define NPM2100_BOOST_IBATLIM_LVL_600MA								(0)
#define NPM2100_BOOST_IBATLIM_LVL_100MA								(1)
#define NPM2100_BOOST_IBATLIM_LVL_200MA								(2)
#define NPM2100_BOOST_IBATLIM_LVL_300MA								(3)
#define NPM2100_BOOST_IBATLIM_LVL_400MA								(4)
#define NPM2100_BOOST_IBATLIM_LVL_500MA								(5)
#define NPM2100_BOOST_IBATLIM_LVL_700MA								(6)
#define NPM2100_BOOST_IBATLIM_LVL_800MA								(7)

#define NPM2100_BOOST_VBATMINLHSEL_REG				0x2E
#define NPM2100_BOOST_VBATMINLHSEL_VBATMINLSEL						(1<<0)	//!< Enable register control for VBATMINL threshold setting
#define NPM2100_BOOST_VBATMINLHSEL_VBATMINHSEL						(1<<1)	//!< Enable register control for VBATMINH threshold setting

#define NPM2100_BOOST_VBATMINL_REG					0x2F	//!< VBATMINL comparator threshold setting (VBATMINL=0.65V+LVL*0.05V, legal legal range: 0-50)
#define NPM2100_BOOST_VBATMINL_LVL_0V65								(0)		//!< 0.65V
#define NPM2100_BOOST_VBATMINL_LVL_3V15								(50)	//!< 3.15V

#define NPM2100_BOOST_VBATMINH_REG					0x30	//!< VBATMINH comparator threshold setting (VBATMINL=0.65V+LVL*0.05V, legal legal range: 0-50)
#define NPM2100_BOOST_VBATMINH_LVL_0V65								(0)		//!< 0.65V
#define NPM2100_BOOST_VBATMINH_LVL_3V15								(50)	//!< 3.15V

#define NPM2100_BOOST_VOUTMIN_REG					0x31	//!< VOUTMIN comparator threshold setting (VOUTMIN=1.7V+LVL*0.05V, legal range: 0-31)
#define NPM2100_BOOST_VOUTMIN_LVL_1V70								(0)		//!< 1.70V
#define NPM2100_BOOST_VOUTMIN_LVL_2V20								(10)		//!< 2.20V Default
#define NPM2100_BOOST_VOUTMIN_LVL_3V25								(31)		//!< 3.25V

#define NPM2100_BOOST_VOUTMIN_WRN_REG				0x32	//!< VOUTMIN comparator threshold setting (VOUTMIN=1.7V+LVL*0.05V, legal range: 0-31)
#define NPM2100_BOOST_VOUTMIN_WRN_LVL_1V70							(0)		//!< 1.70V
#define NPM2100_BOOST_VOUTMIN_WRN_LVL_2V30							(12)	//!< 2.30V default
#define NPM2100_BOOST_VOUTMIN_WRN_LVL_3V25							(31)	//!< 3.25V

#define NPM2100_BOOST_VOUTDPS_REG					0x33	//!< VOUTDPS comparator threshold setting (VOUTDPS=1.9V+LVL*0.05V, legal range: 0-31)
#define NPM2100_BOOST_VOUTDPS_LVL_1V9								(0)		//!< 1.90V default
#define NPM2100_BOOST_VOUTDPS_LVL_3V45								(31)	//!< 3.45V

#define NPM2100_BOOST_STATUS0_REG					0x34
#define NPM2100_BOOST_STATUS0_MODE_HP								(0)		//!< High power mode (HP)
#define NPM2100_BOOST_STATUS0_MODE_LP								(1)		//!< Low power mode (LP)
#define NPM2100_BOOST_STATUS0_MODE_ULP								(2)		//!< Ultra-low power mode (ULP)
#define NPM2100_BOOST_STATUS0_MODE_PT								(3)		//!< Pass-through mode (PT)
#define NPM2100_BOOST_STATUS0_MODE_DPS								(4)		//!< Dynamic Power Smoothing mode (DPS)

#define NPM2100_BOOST_STATUS1_REG					0x35
#define NPM2100_BOOST_STATUS1_VOUTMIN_ACTIVE						(1<<0)
#define NPM2100_BOOST_STATUS1_VOUTWRN_ACTIVE						(1<<1)
#define NPM2100_BOOST_STATUS1_VOUTDPS_ACTIVE						(1<<2)
#define NPM2100_BOOST_STATUS1_VOUTLVL_UNDERVOLT						(0<<3)
#define NPM2100_BOOST_STATUS1_VOUTLVL_ATTARGET						(1<<3)
#define NPM2100_BOOST_STATUS1_CNTRDY_RDY							(1<<4)
#define NPM2100_BOOST_STATUS1_DURRDY_RDY							(1<<5)
#define NPM2100_BOOST_STATUS1_VSETCAPTURE_GND						(0<<6)	//!< Grounded (low)
#define NPM2100_BOOST_STATUS1_VSETCAPTURE_NC						(1<<6)	//!< not connected (high)

#define NPM2100_LDOSW_VOUT_REG						0x68	//!< Output voltage setting for LDO mode (VOUTLDO=0.4+LVL*0.05, legal range:8-52
#define NPM2100_LDOSW_VOUT_LVL_0V8									(8)
#define NPM2100_LDOSW_VOUT_LVL_3V0									(52)

#define NPM2100_LDOSW_LDOSW_REG						0x69
#define NPM2100_LDOSW_LDOSW_ENABLE									(1<<0)	//!< Enable LDOSW

#define NPM2100_LDOSW_SEL_REG						0x6A
#define NPM2100_LDOSW_SEL_MODE_LDO									(0<<0)	//!< LDO mode
#define NPM2100_LDOSW_SEL_MODE_LOADSW								(1<<0)	//!< Load switch mode
#define NPM2100_LDOSW_SEL_OPER_MASK									(3<<1)
#define NPM2100_LDOSW_SEL_OPER_AUTO									(0<<1)	//!< Auto (HP in Active mode/ULP in Hibernate mode) mode
#define NPM2100_LDOSW_SEL_OPER_ULP									(1<<1)
#define NPM2100_LDOSW_SEL_OPER_HP									(2<<1)
#define NPM2100_LDOSW_SEL_OPER_PINCTRL								(3<<1)	//!< GPIO controlled

#define NPM2100_LDOSW_GPIO_REG						0x6B
#define NPM2100_LDOSW_GPIO_POL_LOW									(0<<0)
#define NPM2100_LDOSW_GPIO_POL_HIGH									(1<<0)
#define NPM2100_LDOSW_GPIO_PIN_MASK									(3<<1)
#define NPM2100_LDOSW_GPIO_PIN_GPIO0								(0<<1)
#define NPM2100_LDOSW_GPIO_PIN_GPIO1								(1<<1)
#define NPM2100_LDOSW_GPIO_PIN_GPIO2								(2<<1)	//!< Reserved
#define NPM2100_LDOSW_GPIO_PIN_GPIO22								(3<<1)	//!< Reserved

#define NPM2100_LDOSW_CONF_REG						0x6C
#define NPM2100_LDOSW_CONF_OCP_EN									(1<<0)	//!< Enable OCP

#define NPM2100_LDOSW_RAMP_REG						0x6D
#define NPM2100_LDOSW_RAMP_EN										(1<<0)	//!< VOUTLDO ramps up step by step
#define NPM2100_LDOSW_RAMP_HALT										(1<<1)	//!< Halt VOUTLDO	ramping in case VINT droop

#define NPM2100_LDOSW_STATUS_REG					0x6E
#define NPM2100_LDOSW_STATUS_LDO									(1<<0)	//!< LDO mode enabled
#define NPM2100_LDOSW_STATUS_LOADSW									(1<<1)	//!< Load switch mode enabled
#define NPM2100_LDOSW_STATUS_HP										(1<<2)	//!< HP mode
#define NPM2100_LDOSW_STATUS_ULP									(1<<3)	//!< ULP mode
#define NPM2100_LDOSW_STATUS_OCP_ACTIVE								(1<<4)	//!< Over current detected

#define NPM2100_LDOSW_PRGOCP_REG					0x6F
#define NPM2100_LDOSW_PRGOCP_LDO_MASK								(0xf<<0)//!< Select OCP and soft start current limits for LDO operating mode (valid in HP mode)
#define NPM2100_LDOSW_PRGOCP_LDO_DNU								(0<<0)	//!< Do not use
#define NPM2100_LDOSW_PRGOCP_LDO_150MA								(1<<0)
#define NPM2100_LDOSW_PRGOCP_LDO_75MA								(4<<0)	//!< 75mA default
#define NPM2100_LDOSW_PRGOCP_LDO_50MA								(6<<0)
#define NPM2100_LDOSW_PRGOCP_LDO_38MA								(7<<0)
#define NPM2100_LDOSW_PRGOCP_LDO_25MA								(13<<0)
#define NPM2100_LDOSW_PRGOCP_LOADSW_MASK							(0xf<<4)//!< Select OCP and soft start current limits for Load switch operating mode (valid in HP mode)
#define NPM2100_LDOSW_PRGOCP_LOADSW_DNU								(0<<4)	//!< Do not use
#define NPM2100_LDOSW_PRGOCP_LOADSW_40MA							(1<<4)
#define NPM2100_LDOSW_PRGOCP_LOADSW_70MA							(1<<4)
#define NPM2100_LDOSW_PRGOCP_LOADSW_75MA							(1<<4)	//!< 75mA default
#define NPM2100_LDOSW_PRGOCP_LOADSW_80MA							(1<<4)
#define NPM2100_LDOSW_PRGOCP_LOADSW_110MA							(1<<4)

// GPIO0
#define NPM2100_GPIO_CONFIG0_REG					0x80
#define NPM2100_GPIO_CONFIG0_INPUT									(1<<0)	//!< Enable input buffer
#define NPM2100_GPIO_CONFIG0_OUTPUT									(1<<1)	//!< Enable output buffer
#define NPM2100_GPIO_CONFIG0_OPENDRAIN								(1<<2)	//!< Enable open-drain
#define NPM2100_GPIO_CONFIG0_PULLDOWN								(1<<3)	//!< Enable pulldown
#define NPM2100_GPIO_CONFIG0_PULLUP									(1<<4)	//!< Enable pullup
#define NPM2100_GPIO_CONFIG0_HIGH_DRIVE								(1<<5)	//!< Enable high drive strength
#define NPM2100_GPIO_CONFIG0_DEBOUNCE								(1<<6)	//!< Enable debounce

// GPIO1
#define NPM2100_GPIO_CONFIG1_REG					0x81
#define NPM2100_GPIO_CONFIG1_INPUT									(1<<0)	//!< Enable input buffer
#define NPM2100_GPIO_CONFIG1_OUTPUT									(1<<1)	//!< Enable output buffer
#define NPM2100_GPIO_CONFIG1_OPENDRAIN								(1<<2)	//!< Enable open-drain
#define NPM2100_GPIO_CONFIG1_PULLDOWN								(1<<3)	//!< Enable pulldown
#define NPM2100_GPIO_CONFIG1_PULLUP									(1<<4)	//!< Enable pullup
#define NPM2100_GPIO_CONFIG1_HIGH_DRIVE								(1<<5)	//!< Enable high drive strength
#define NPM2100_GPIO_CONFIG1_DEBOUNCE								(1<<6)	//!< Enable debounce

// GPIO0
#define NPM2100_GPIO_USAGE0_REG						0x83
#define NPM2100_GPIO_USAGE0_SEL_GPIO								(0<<0)	//!< GPIO mode
#define NPM2100_GPIO_USAGE0_SEL_INTLO								(1<<0)	//!< Interrupt output, active low
#define NPM2100_GPIO_USAGE0_SEL_INTHI								(2<<0)	//!< Interrupt output, active high

// GPIO1
#define NPM2100_GPIO_USAGE1_REG						0x84
#define NPM2100_GPIO_USAGE1_SEL_GPIO								(0<<0)	//!< GPIO mode
#define NPM2100_GPIO_USAGE1_SEL_INTLO								(1<<0)	//!< Interrupt output, active low
#define NPM2100_GPIO_USAGE1_SEL_INTHI								(2<<0)	//!< Interrupt output, active high

#define NPM2100_GPIO_OUTPUT0_REG					0x86
#define NPM2100_GPIO_OUTPUT1_REG					0x87
#define NPM2100_GPIO_READ_REG						0x89

#define NPM2100_ADC_TASKS_ADC_REG					0x90
#define NPM2100_ADC_TASKS_ADC_CONV_TRIG								(1<<0)	//!< Trigger conv
#define NPM2100_ADC_TASKS_ADC_ABORTDELAY_TIRG						(1<<1)	//!< Abort delay and trigger conv

#define NPM2100_ADC_CONFIG_REG						0x91
#define NPM2100_ADC_CONFIG_MODE_MASK								(7<<0)
#define NPM2100_ADC_CONFIG_MODE_INSBAT								(0<<0)	//!< instant VBAT measurement (READVBAT)
#define NPM2100_ADC_CONFIG_MODE_DELVBAT								(1<<0)	//!< Delayed VBAT measurement
#define NPM2100_ADC_CONFIG_MODE_DIETEMP								(2<<0)	//!< Die temperature measurement (READTEMP)
#define NPM2100_ADC_CONFIG_MODE_VOUTDROOP							(3<<0)	//!< VOUT droop measurement (READDROOP)
#define NPM2100_ADC_CONFIG_MODE_VOUT								(4<<0)	//!< VOUT measurement (READVOUT)
#define NPM2100_ADC_CONFIG_MODE_OFFSET								(5<<0) 	//!< ADC offset measurement (OFFSETMEASURED)
#define NPM2100_ADC_CONFIG_AVG_MASK									(7<<3)
#define NPM2100_ADC_CONFIG_AVG_DIS									(0<<3)	//!< Disable averaging
#define NPM2100_ADC_CONFIG_AVG_2									(1<<3)	//!< Averaging by 2
#define NPM2100_ADC_CONFIG_AVG_4
#define NPM2100_ADC_CONFIG_AVG_8
#define NPM2100_ADC_CONFIG_AVG_16
#define NPM2100_ADC_CONFIG_GPIO_MASK								(3<<6)
#define NPM2100_ADC_CONFIG_GPIO_NONE								(0<<6)	//!< No GPIO control VOUT DROOP detector
#define NPM2100_ADC_CONFIG_GPIO_0									(1<<6)	//!< Use GPIO0, active high
#define NPM2100_ADC_CONFIG_GPIO_1									(2<<6)
#define NPM2100_ADC_CONFIG_GPIO_2									(3<<6)

#define NPM2100_ADC_DELAY_REG						0x92	//!< Delay (ms)=5+TIME*4. Legal range is 0-255 or 5-1025 ms

#define NPM2100_ADC_OFFSETCFG_REG					0x93
#define NPM2100_ADC_OFFSETCFG_OFFSET_EN								(1<<0)	//!< Offset correction enabled
#define NPM2100_ADC_OFFSETCFG_SELOFFSET_FACTORY						(0<<1)	//!< Select factory offset register
#define NPM2100_ADC_OFFSETCFG_SELOFFSET_MEAS						(1<<1)	//!< Select measured offset register

#define NPM2100_ADC_CTRLSET_REG						0x94
#define NPM2100_ADC_CTRLSET_DROOP									(1<<0)	//!< Start VOUT droop detection
#define NPM2100_ADC_CTRLSET_RECOV									(1<<1)	//!< Start VOUT droop recovery time counter

#define NPM2100_ADC_CTRLCLR_REG						0x95
#define NPM2100_ADC_CTRLCLR_DROOP									(1<<0)	//!< Stop VOUT droop detection
#define NPM2100_ADC_CTRLCLR_RECOV									(1<<1)	//!< Stop VOUT droop recovery time counter

#define NPM2100_ADC_READVBAT_REG					0x96
#define NPM2100_ADC_READTEMP_REG					0x97
#define NPM2100_ADC_READDROOP_REG					0x98
#define NPM2100_ADC_READVOUT_REG					0x99
#define NPM2100_ADC_VOUTRECOV_REG					0x9A
#define NPM2100_ADC_AVERAGE_REG						0x9B

#define NPM2100_ADC_BOOST_REG						0x9C
#define NPM2100_ADC_BOOST_MODE_MASK									(7<<0)
#define NPM2100_ADC_BOOST_MODE_HP									(0<<0)
#define NPM2100_ADC_BOOST_MODE_LP									(1<<0)
#define NPM2100_ADC_BOOST_MODE_ULP									(2<<0)
#define NPM2100_ADC_BOOST_MODE_PT									(3<<0)
#define NPM2100_ADC_BOOST_MODE_DPS									(4<<0)


#define NPM2100_ADC_STATUS_REG						0x9D
#define NPM2100_ADC_STATUS_ADC_MASK									(3<<0)
#define NPM2100_ADC_STATUS_ADC_READY								(0<<0)
#define NPM2100_ADC_STATUS_ADC_DELAY								(1<<0)	//!< Busy waiting delay
#define NPM2100_ADC_STATUS_ADC_SINGLE								(2<<0)	//!< Busy instant measurement
#define NPM2100_ADC_STATUS_ADC_DROOP								(3<<0)	//!< Busy, VOUT droop measurement active
#define NPM2100_ADC_STATUS_DROOP									(1<<2)	//!< Droop detector powered on
#define NPM2100_ADC_STATUS_RECOV_STOPPED							(1<<3)	//!< Droop recovery counter stopped

#define NPM2100_ADC_OFFSETFACTORY_REG				0x9E
#define NPM2100_ADC_OFFSETMEASURED_REG				0x9F

#define NPM2100_TIMER_TASKS_START_REG				0xB0
#define NPM2100_TIMER_TASKS_START_TIMER								(1<<0)	//!< Start timer

#define NPM2100_TIMER_TASKS_STOP_REG				0xB1
#define NPM2100_TIMER_TASKS_STOP_TIMER								(1<<0)	//!< Stop timer (also boot monitor

#define NPM2100_TIMER_TASKS_KICK_REG				0xB2
#define NPM2100_TIMER_TASKS_KICK_WD									(1<<0)	//!< Kick watchdog

#define NPM2100_TIMER_CONFIG_REG					0xB3
#define NPM2100_TIMER_CONFIG_MODE_MASK								(3<<0)
#define NPM2100_TIMER_CONFIG_MODE_GENPURP							(0<<0)	//!< General purpose timer mode
#define NPM2100_TIMER_CONFIG_MODE_WDRST								(1<<0)	//!< Watch dog timer with reset
#define NPM2100_TIMER_CONFIG_MODE_WDPWRC							(2<<0)	//!< Watch dog timer with power cycle
#define NPM2100_TIMER_CONFIG_MODE_WKUP								(3<<0)	//!< Wakeup timer (for hibernate mode)

#define NPM2100_TIMER_TARGETHI_REG					0xB4
#define NPM2100_TIMER_TARGETMID_REG					0xB5
#define NPM2100_TIMER_TARGETLO_REG					0xB6

#define NPM2100_TIMER_STATUS_REG					0xB7
#define NPM2100_TIMER_STATUS_BUSY									(1<<0)

#define NPM2100_SHIP_TASKS_SHIP_REG					0xC0
#define NPM2100_SHIP_TASKS_SHIP_ENTER								(1<<0)	//!< Enter ship mode

#define NPM2100_SHIP_WAKEUP_REG						0xC1
#define NPM2100_SHIP_WAKEUP_EDGE_RISING								(1<<0)
#define NPM2100_SHIP_WAKEUP_HIBERNATE_PINDIS						(1<<1)	//!< Disable SHPHLD pin wakeup

#define NPM2100_SHIP_SHPHLD_REG						0xC2
#define NPM2100_SHIP_SHPHLD_RESISTOR_MASK							(3<<0)
#define NPM2100_SHIP_SHPHLD_RESISTOR_PULLUP							(0<<0)
#define NPM2100_SHIP_SHPHLD_RESISTOR_NONE							(1<<0)
#define NPM2100_SHIP_SHPHLD_RESISTOR_PULLDOWN						(2<<0)
#define NPM2100_SHIP_SHPHLD_CURR_MASK								(3<<2)
#define NPM2100_SHIP_SHPHLD_CURR_WEAK								(0<<2)
#define NPM2100_SHIP_SHPHLD_CURR_LOW								(1<<2)
#define NPM2100_SHIP_SHPHLD_CURR_MODERATE							(2<<2)
#define NPM2100_SHIP_SHPHLD_CURR_HIGH								(3<<2)
#define NPM2100_SHIP_SHPHLD_PULL_WEAK								(1<<4)	//!< Weak pullup enable

#define NPM2100_HIBERNATE_TASKS_HIBER_REG			0xC8
#define NPM2100_HIBERNATE_TASKS_HIBER_ENTER							(1<<0)	//!< Enter hibernate mode

#define NPM2100_HIBERNATE_TASKS_HIBERPT_REG			0xC9
#define NPM2100_HIBERNATE_TASKS_HIBERPT_ENTER						(1<<0)	//!< Enter hibernate PT mode

#define NPM2100_HIBERNATE_DEBOUNCE_REG				0xCA
#define NPM2100_HIBERNATE_DEBOUNCE_EN								(1<<0)	//!< Enable debounce filter
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_MASK						(7<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_10MS						(0<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_30MS						(1<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_60MS						(2<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_100MS						(3<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_300MS						(4<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_600MS						(5<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_1S							(6<<1)
#define NPM2100_HIBERNATE_DEBOUNCE_TIME_3S							(7<<1)


#define NPM2100_RESET_TASKS_RESET_REG				0xD0
#define NPM2100_RESET_TASKS_RESET_SWRST								(1<<0)	//!< Trigger power cycle

#define NPM2100_RESET_TASKS_CLR_REG					0xD1

#define NPM2100_RESET_BUTTON_REG					0xD2
#define NPM2100_RESET_BUTTON_LOGNPRESS_DIS							(1<<0)	//!< Disable long press reset

#define NPM2100_RESET_PIN_REG						0xD3
#define NPM2100_RESET_PIN_PGRESET									(0<<0)	//!< Reset button connected to PG/RESET
#define NPM2100_RESET_PIN_SHPHLD									(1<<0)	//!< Reset button connected to SHPHLD

#define NPM2100_RESET_DEBOUNCE_REG					0xD4
#define NPM2100_RESET_DEBOUNCE_TIME_MASK							(3<<0)
#define NPM2100_RESET_DEBOUNCE_TIME_10S								(0<<0)
#define NPM2100_RESET_DEBOUNCE_TIME_5S								(1<<0)
#define NPM2100_RESET_DEBOUNCE_TIME_20S								(2<<0)
#define NPM2100_RESET_DEBOUNCE_TIME_30S								(3<<0)

#define NPM2100_RESET_RESET_REG						0xD5
#define NPM2100_RESET_RESET_BOR										(1<<0)	//!< Borwn-out reset happened
#define NPM2100_RESET_RESET_REASON_MASK								(0xF<<1)
#define NPM2100_RESET_RESET_REASON_COLDPWRUP						(0<<1)	//!< Cold power up
#define NPM2100_RESET_RESET_REASON_TSD								(1<<1)	//!< Thermal shutdown
#define NPM2100_RESET_RESET_REASON_BOOTMONITOR						(2<<1)
#define NPM2100_RESET_RESET_REASON_BUTTON							(3<<1)
#define NPM2100_RESET_RESET_REASON_WDRST							(4<<1)
#define NPM2100_RESET_RESET_REASON_WDPWRCYCLE						(5<<1)
#define NPM2100_RESET_RESET_REASON_SWRESET							(6<<1)
#define NPM2100_RESET_RESET_REASON_HIBERPIN							(7<<1)
#define NPM2100_RESET_RESET_REASON_HIBERTIMER						(8<<1)
#define NPM2100_RESET_RESET_REASON_HIBERPTPIN						(9<<1)
#define NPM2100_RESET_RESET_REASON_HIBERPTTIMER						(10<<1)
#define NPM2100_RESET_RESET_REASON_PWROFFBUTTON						(11<<1)
#define NPM2100_RESET_RESET_REASON_SHIPEXIT							(12<<1)
#define NPM2100_RESET_RESET_REASON_OCP								(13<<1)

#define NPM2100_RESET_ALTCONFIG_REG					0xD6
#define NPM2100_RESET_ALTCONFIG_LDOSW_OFF							(1<<0)	//!< LDOSW disabled in WD reset state

#define NPM2100_RESET_WRITE_REG						0xD7
#define NPM2100_RESET_STROBE_REG					0xD8
#define NPM2100_RESET_STROBE_SCRATCHA								(1<<0)	//!< Srtobe vales into scratch a

#define NPM2100_RESET_READ_REG						0xD9	//!< Read scratch A register

#define NPM2100_RESET_SCRATCHB_REG					0xDA

#define NPM2100_RESET_WRITESTICKY_REG				0xDB
#define NPM2100_RESET_WRITESTICKY_BOOTMONSEL						(1<<0)	//!< Boot monitor is controlled by BOOTMODEEN register
#define NPM2100_RESET_WRITESTICKY_BOOTMON_EN						(1<<1)	//!< Boot monitor enabled
#define NPM2100_RESET_WRITESTICKY_PWRBUTTON_DIS						(1<<2)	//!< Disable power off button

#define NPM2100_RESET_STROBESTICKY_REG				0xDC
#define NPM2100_RESET_READSTICKY_REG				0xDD
#define NPM2100_RESET_READSTICKY_BOOTMONSEL							(1<<0)	//!< Boot monitor is controlled by BOOTMODEEN register
#define NPM2100_RESET_READSTICKY_BOOTMON_EN							(1<<1)	//!< Boot monitor enabled
#define NPM2100_RESET_READSTICKY_PWRBUTTON_DIS						(1<<2)	//!< Power off button disabled

#define NPM2100_RESET_SYSGDENSTATUS_REG				0xDF
#define NPM2100_RESET_SYSGDENSTATUS_BOOTMONSTATUS_SYSGDE			(1<<0)	//!< Boot monitor is active unless SYSGDENSTATE = 0
#define NPM2100_RESET_SYSGDENSTATUS_SYSGDENSTATE					(1<<1)	//!< SYSGDEN state at power up



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
