/**-------------------------------------------------------------------------
@file	pm_bq25120a.h

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
#ifndef __PM_BQ25120A_H__
#define __PM_BQ25120A_H__

#include <stdint.h>

#include "pwrmgnt/pwrmgnt.h"

/** @addtogroup Power
  * @{
  */

#define BQ25120A_I2C_7BITS_DEVADDR				0x6A

#define BQ25120A_STATUS_SHIPMODE_CTRL_REG			0
#define BQ25120A_STATUS_SHIPMODE_CTRL_SYS_EN_STAT							(1<<0)	//!< SW enabled
#define BQ25120A_STATUS_SHIPMODE_CTRL_CD_STAT								(1<<1)	//!< IC disabled
#define BQ25120A_STATUS_SHIPMODE_CTRL_VINDPM_STAT							(1<<2)	//!< VIN_DPM is active
#define BQ25120A_STATUS_SHIPMODE_CTRL_TIMER									(1<<3)	//!< Safety timer fault
#define BQ25120A_STATUS_SHIPMODE_CTRL_RESET_FAULT							(1<<4)	//!< Reset fault
#define BQ25120A_STATUS_SHIPMODE_CTRL_EN_SHIPMODE							(1<<5)	//!< Ship mode enable
#define BQ25120A_STATUS_SHIPMODE_CTRL_STAT_MASK								(3<<6)	//!<
#define BQ25120A_STATUS_SHIPMODE_CTRL_READY									(0<<6)	//!< Ready
#define BQ25120A_STATUS_SHIPMODE_CTRL_CHARGING 								(1<<6)	//!< Charge in progress
#define BQ25120A_STATUS_SHIPMODE_CTRL_CHARGE_DONE							(2<<6)	//!< Charge done
#define BQ25120A_STATUS_SHIPMODE_CTRL_FAULT									(3<<6)	//!< Fault

#define BQ25120A_FAULT_REG							1	// Fault & fault mask register
#define BQ25120A_FAULT_BAT_ICO_M											(1<<0)	//!< Mask BAT_OCP fault
#define BQ25120A_FAULT_BAT_UVLO_M											(1<<1)	//!< Mask BAT UVLO fault
#define BQ25120A_FAULT_VIN_UV_M												(1<<2)	//!< Mask Vin undervoltage fault
#define BQ25120A_FAULT_VIN_OV_M												(1<<3)	//!< Mask Vin overvoltage fault
#define BQ25120A_FAULT_BAT_OCP												(1<<4)	//!< BAT_OCP fault
#define BQ25120A_FAULT_BAT_UVLO												(1<<5) 	//!< BAT_UVLO fault
#define BQ25120A_FAULT_VIN_UV												(1<<6)	//!< Vin undervoltage fault
#define BQ25120A_FAULT_VIN_OV												(1<<7)	//!< Vin overvoltage fault

#define BQ25120A_TS_CTRL_FAULT_MASK_REG				2
#define BQ25120A_TS_CTRL_FAULT_MASK_TIMER_M									(1<<0)	//!< Mask timer fault interrupt (safety)
#define BQ25120A_TS_CTRL_FAULT_MASK_RESET_M									(1<<1)	//!< Mask reset interrupt from MR
#define BQ25120A_TS_CTRL_FAULT_MASK_WAKE_M									(1<<2)	//!< Mask interrupt from Wake condition from MR
#define BQ25120A_TS_CTRL_FAULT_MASK_EN_INT									(1<<3)	//!< Enable INT
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_FAULT_MASK							(3<<5)	//!<
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_FAULT_NORMAL							(0<<5)	//!< No fault
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_FAULT_TS_TEMP						(1<<5)	//!< Charging suspended
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_COOL									(2<<5)	//!< Chargin current reduced by half
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_WARM									(3<<5)	//!< Charging voltage reduced byt 140mmV
#define BQ25120A_TS_CTRL_FAULT_MASK_TS_EN									(1<<7)	//!< TS enable

#define BQ25120A_FAST_CHARGE_CTRL_REG				3
#define BQ25120A_FAST_CHARGE_CTRL_HZ_NODE									(1<<0)	//!< High impedance mode
#define BQ25120A_FAST_CHARGE_CTRL_CE										(1<<1)	//!< Charger disabled
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_0									(1<<2)	//!< Charge current 1mA or 10 mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_1									(1<<3)	//!< Charge current 2mA or 20 mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_2									(1<<4)	//!< Charge current 4mA or 40 mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_3									(1<<5)	//!< Charge current 8mA or 80 mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_4									(1<<6)	//!< Charge current 16mA or 160 mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_RANGE_0								(1<<7)	//!< Select charge range from 5mA to 35mA
#define BQ25120A_FAST_CHARGE_CTRL_ICHRG_RANGE_1								(1<<7)	//!< Select charge range from 40mA to 300mA

#define BQ25120A_TERM_PRE_CHRG_REG					4
#define BQ25120A_TERM_PRE_CHRG_TE											(1<<1)	//!< Enable charge current termination
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_0									(1<<2)	//!< Termination current 500uA or 1mA
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_1									(1<<3)	//!< Termination current 1mA or 2mA
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_2									(1<<4)	//!< Termination current 2mA or 4mA
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_3									(1<<5)	//!< Termination current 4mA or 8mA
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_4									(1<<6)	//!< Termination current 8mA or 16mA
#define BQ25120A_TERM_PRE_CHRG_IPRETERM_RANGE								(1<<7)	//!< Termination current 6mA or 37mA

#define BQ25120A_BAT_VOLTAGE_CTRL_REG				5
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG0									(1<<1)	//!< Battery regulation voltage : 10 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG1									(1<<2)	//!< Battery regulation voltage : 20 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG2									(1<<3)	//!< Battery regulation voltage : 40 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG3									(1<<4)	//!< Battery regulation voltage : 80 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG4									(1<<5)	//!< Battery regulation voltage : 160 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG5									(1<<6)	//!< Battery regulation voltage : 320 mV
#define BQ25120A_BAT_VOLTAGE_CTRL_VBREG6									(1<<7)	//!< Battery regulation voltage : 640 mV

#define BQ25120A_SYS_VOUT_CTRL_REG					6
#define BQ25120A_SYS_VOUT_CTRL_SYST_VOUT_MASK								(0xF<<1)//!< Step 100mV
//#define BQ25120A_SYS_VOUT_CTRL_SYST_VOUT0									(1<<1)	//!< OUT voltage : 100mV step if SYS_SEL is 01 or 11
//#define BQ25120A_SYS_VOUT_CTRL_SYST_VOUT1									(1<<2)	//!< OUT voltage : 200mV step if SYS_SEL is 01 or 11
//#define BQ25120A_SYS_VOUT_CTRL_SYST_VOUT2									(1<<3)	//!< OUT voltage : 400mV step if SYS_SEL is 01 or 11
//#define BQ25120A_SYS_VOUT_CTRL_SYST_VOUT3									(1<<4)	//!< OUT voltage : 800mV step if SYS_SEL is 01 or 11
#define BQ25120A_SYS_SEL_MASK												(3<<5)	//!<
#define BQ25120A_SYS_SEL_1_1_1_2											(0<<5)	//!< 1.1V and 1.2V selection
#define BQ25120A_SYS_SEL_1_3_2_8											(1<<5)	//!< 1.3V through 2.8V selection
#define BQ25120A_SYS_SEL_1_5_2_8											(2<<5)	//!< 1.5V through 2.75V selection
#define BQ25120A_SYS_SEL_1_8_3_3											(3<<5)	//!< 1.8V through 3.3V selection
#define BQ25120A_SYS_SEL_EN_SYST_OUT										(1<<7)	//!< Enable output

#define BQ25120A_LOADSW_LDO_CTRL_REG				7
#define BQ25120A_LOADSW_LDO_CTRL_MRRESET_VIN								(1<<0)	//!< Reset when MR
#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_MASK								(0x1F<<2)	//!< LDO out = 0.8V + 0.1 * LS_LDO
//#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_0									(1<<2)	//!< LS/LDO voltage : 100mV
//#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_1									(1<<3)	//!< LS/LDO voltage : 200mV
//#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_2									(1<<4)	//!< LS/LDO voltage : 400mV
//#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_3									(1<<5)	//!< LS/LDO voltage : 800mV
//#define BQ25120A_LOADSW_LDO_CTRL_LS_LDO_4									(1<<6)	//!< LS/LDO voltage : 1600mV
#define BQ25120A_LOADSW_LDO_CTRL_EN_LS_LDO									(1<<7)	//!< Enable LS/LDO

#define BQ25120A_BUT_CTRL_REG						8
#define BQ25120A_BUT_CTRL_WAKE2												(1<<0)	//!< WAKE2 status
#define BQ25120A_BUT_CTRL_WAKE1												(1<<1)	//!< WAKE1 status
#define BQ25120A_BUT_CTRL_PGB_MR											(1<<2)	//!< Output function as voltage shifted button MR input
#define BQ25120A_BUT_CTRL_MRRESET_MASK										(3<<3)
#define BQ25120A_BUT_CTRL_MRRESET_5S										(0<<3)	//!< MR reset 5s
#define BQ25120A_BUT_CTRL_MRRESET_9S										(1<<3)	//!< MR reset 9s
#define BQ25120A_BUT_CTRL_MRRESET_11S										(2<<3)	//!< MR reset 11s
#define BQ25120A_BUT_CTRL_MRRESET_15S										(3<<3)	//!< MR reset 15s
#define BQ25120A_BUT_CTRL_MRREC_SHIP										(0<<5)	//!< After reset, device enters ship mode
#define BQ25120A_BUT_CTRL_MRREC_HIZ											(1<<5)	//!< After reset, device enters Hi-Z mode
#define BQ25120A_BUT_CTRL_MRWAKE2_1000										(0<<6)	//!< WAKE2 1000 ms < MR
#define BQ25120A_BUT_CTRL_MRWAKE2_1500										(1<<6)	//!< WAKE2 1500 ms < MR
#define BQ25120A_BUT_CTRL_MRWAKE1_80										(0<<7)	//!< WAKE1 80 ms < MR
#define BQ25120A_BUT_CTRL_MRWAKE1_600										(1<<7)	//!< WAKE1 600 ms < MR

#define BQ25120A_ILIM_BAT_UVLO_CTRL_REG				9
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_MASK								(7<<0)
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_3V0								(2<<0)	//!< BUVLO = 3.0V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_2V8								(3<<0)	//!< BUVLO = 2.8V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_2V6								(4<<0)	//!< BUVLO = 2.6V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_2V4								(5<<0)	//!< BUVLO = 2.4V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_2V2								(6<<0)	//!< BUVLO = 2.2V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_BUVLO_2V2_1								(7<<0)	//!< BUVLO = 2.2V
#define BQ25120A_ILIM_BAT_UVLO_CTRL_ILIM_50									(1<<3)	//!< Input current limit 50mA
#define BQ25120A_ILIM_BAT_UVLO_CTRL_ILIM_100								(1<<4)	//!< Input current limit 100mA
#define BQ25120A_ILIM_BAT_UVLO_CTRL_ILIM_200								(1<<5)	//!< Input current limit 200mA
#define BQ25120A_ILIM_BAT_UVLO_CTRL_RESET									(1<<7)	//!< Reset all registers

#define BQ25120A_VBATMON_REG						0xA
#define BQ25120A_VBATMON_TH_MASK											(7<<2)
#define BQ25120A_VBATMON_TH_0												(1<<2)	//!< Above 0% VBMON_RANGE
#define BQ25120A_VBATMON_TH_2												(2<<2)	//!< Above 2% VBMON_RANGE
#define BQ25120A_VBATMON_TH_4												(3<<2)	//!< Above 4% VBMON_RANGE
#define BQ25120A_VBATMON_TH_6												(6<<2)	//!< Above 6% VBMON_RANGE
#define BQ25120A_VBATMON_TH_8												(7<<2)	//!< Above 8% VBMON_RANGE
#define BQ25120A_VBATMON_RANGE_MASK											(3<<5)
#define BQ25120A_VBATMON_RANGE_60_70										(0<<5)	//!< 60% to 70% of VBATREG
#define BQ25120A_VBATMON_RANGE_70_80										(1<<5)	//!< 70% to 80% of VBATREG
#define BQ25120A_VBATMON_RANGE_80_90										(2<<5)	//!< 80% to 90% of VBATREG
#define BQ25120A_VBATMON_RANGE_90_100										(3<<5)	//!< 90% to 100% of VBATREG
#define BQ25120A_VBATMON_READ												(1<<7)	//!< Initiate a new VBATREG reading

#define BQ25120A_VIN_DPM_TIMER_REG					0xB
#define BQ25120A_VIN_DPM_TIMER_TMR_MASK										(3<<1)	//!<
#define BQ25120A_VIN_DPM_TIMER_TMR_30M										(0<<1)	//!< 30 min fast charge
#define BQ25120A_VIN_DPM_TIMER_TMR_3H										(1<<1)	//!< 3 hours fast charge
#define BQ25120A_VIN_DPM_TIMER_TMR_9H										(2<<1)	//!< 9 hours fast charge
#define BQ25120A_VIN_DPM_TIMER_TMR_DIS										(3<<1)	//!< Disable safety timer

class PmBq25120a : public PowerMgnt {
public:

	/**
	 * @brief	Initialization
	 *
	 * @param 	Cfg 	: Reference to the configuration data
	 * @param	pIntrf	: Pointer to the communication interface.
	 * 					  Most programmable PMIC is I2C
	 *
	 * @return	true - success
	 */
	bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf);

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

	uint32_t SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr);

	bool Charging();
	bool Battery();

	void IrqHandler();

private:
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif // __PM_BQ25120A_H__
