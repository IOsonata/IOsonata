/**-------------------------------------------------------------------------
@file	pm_mp2696a.h

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

#ifndef __PM_MP2696A_H__
#define __PM_MP2696A_H__

#include <stdint.h>

#include "pwrmgnt/pwrmgnt.h"

/** @addtogroup Power
  * @{
  */

#define MP2696A_I2C_DEVADDR			0x6B

// Registers

/// Input voltage regulation setting and input current limit setting
#define MP2696A_REG00				0x00
#define MP2696A_REG00_IINLIM_MASK			(3<<0)
#define MP2696A_REG00_IINLIM_100MA			(0<<0)
#define MP2696A_REG00_IINLIM_500MA			(1<<0)
#define MP2696A_REG00_IINLIM_1000MA			(2<<0)
#define MP2696A_REG00_IINLIM_1500MA			(3<<0)
#define MP2696A_REG00_IINLIM_1800MA			(4<<0)
#define MP2696A_REG00_IINLIM_2100MA			(5<<0)
#define MP2696A_REG00_IINLIM_2400MA			(6<<0)
#define MP2696A_REG00_IINLIM_3000MA			(7<<0)
#define MP2696A_REG00_VINMIN_MASK			(3<<3)	//!< Input voltage regulation
#define MP2696A_REG00_VINMIN_4V45			(0<<3)	//!< 4.45V in 0.05V increments
#define MP2696A_REG00_VINMIN_4V50			(1<<3)	//!< 4.5V
#define MP2696A_REG00_VINMIN_4V55			(2<<3)	//!< 4.55V
#define MP2696A_REG00_VINMIN_4V60			(3<<3)	//!< 4.6V
#define MP2696A_REG00_VINMIN_4V65			(4<<3)	//!< 4.65V
#define MP2696A_REG00_VINMIN_4V70			(5<<3)	//!< 4.7V
#define MP2696A_REG00_VINMIN_4V75			(6<<3)	//!< 4.75V
#define MP2696A_REG00_VINMIN_4V80			(7<<3)	//!< 4.8V
#define MP2696A_REG00_EN_TIMER				(1<<6)
#define MP2696A_REG00_REG_RST				(1<<7)

/// Charge current setting and pre-charge current setting.
#define MP2696A_REG01				0x01
#define MP2696A_REG01_IPRE_MASK				(3<<0)	//!< Pre-charge current
#define MP2696A_REG01_IPRE_150MA			(1<<0)	//!< 150mA
#define MP2696A_REG01_IPRE_250MA			(2<<0)	//!< 250mA
#define MP2696A_REG01_IPRE_350MA			(3<<0)	//1< 350mA
#define MP2696A_REG01_EN_NTC				(1<<2)
#define MP2696A_REG01_ICC_MASK				(0x1F<<3)//!< Charge current
#define MP2696A_REG01_ICC_POS				3
#define MP2696A_REG01_ICC_500MA				(0<<3)	//!< 500mA in 100mA increments
#define MP2696A_REG01_ICC_600MA				(1<<3)	//!< 600mA
#define MP2696A_REG01_ICC_700MA				(2<<3)	//!< 700mA
#define MP2696A_REG01_ICC_800MA				(3<<3)	//!< 800mA
#define MP2696A_REG01_ICC_900MA				(4<<3)	//!< 900mA
#define MP2696A_REG01_ICC_1000MA			(5<<3)	//!< 1000mA
#define MP2696A_REG01_ICC_1100MA			(6<<3)	//!< 1100mA
#define MP2696A_REG01_ICC_1200MA			(7<<3)	//!< 1200mA
#define MP2696A_REG01_ICC_1300MA			(8<<3)	//!< 1300mA
#define MP2696A_REG01_ICC_1400MA			(9<<3)	//!< 1400mA
#define MP2696A_REG01_ICC_1500MA			(10<<3)	//!< 1500mA
#define MP2696A_REG01_ICC_1600MA			(11<<3)	//!< 1600mA
#define MP2696A_REG01_ICC_1700MA			(12<<3)	//!< 1700mA
#define MP2696A_REG01_ICC_1800MA			(13<<3)	//!< 1800mA
#define MP2696A_REG01_ICC_1900MA			(14<<3)	//!< 1900mA
#define MP2696A_REG01_ICC_2000MA			(15<<3)	//!< 2000mA

/// Battery regulation voltage and termination current setting.
#define MP2696A_REG02				0x02
#define MP2696A_REG02_CHG_EN				(1<<0)
#define MP2696A_REG02_ITERM_MASK			(3<<1)	//!< Charge termination current
#define MP2696A_REG02_ITERM_100MA			(0<<1)	//!< 100mA
#define MP2696A_REG02_ITERM_200MA			(1<<1)	//!< 200mA
#define MP2696A_REG02_ITERM_300MA			(2<<1)	//!< 300mA
#define MP2696A_REG02_ITERM_400MA			(3<<1)	//!< 400mA
#define MP2696A_REG02_JEITA_DIS				(1<<3)
#define MP2696A_REG02_BATT_REG_MASK			(7<<4)	//!< Charge voltage regulation
#define MP2696A_REG02_BATT_REG_3V6			(0<<4)	//!< 3.6V
#define MP2696A_REG02_BATT_REG_4V1			(1<<4)	//!< 4.1V
#define MP2696A_REG02_BATT_REG_4V2			(2<<4)	//!< 4.2V
#define MP2696A_REG02_BATT_REG_4V3			(3<<4)	//!< 4.3V
#define MP2696A_REG02_BATT_REG_4V35			(4<<4)	//!< 4.35V
#define MP2696A_REG02_BATT_REG_4V4			(5<<4)	//!< 4.4V
#define MP2696A_REG02_BATT_REG_4V45			(6<<4)	//!< 4.45V
#define MP2696A_REG02_BATT_OVP_DIS			(1<<7)

/// Boost output current limit setting and cable impedance compensation.
#define MP2696A_REG03				0x03
#define MP2696A_REG03_NO_LOAD				(1<<0)
#define MP2696A_REG03_RSYS_CMP_MASK			(7<<1)
#define MP2696A_REG03_RSYS_CMP_20MOHM		(1<<1)
#define MP2696A_REG03_RSYS_CMP_40MOHM		(2<<1)
#define MP2696A_REG03_RSYS_CMP_60MOHM		(3<<1)
#define MP2696A_REG03_RSYS_CMP_80MOHM		(4<<1)
#define MP2696A_REG03_IOLIM_MASK			(0xF<<4)//!< SYS output limits
#define MP2696A_REG03_IOLIM_POS				4
#define MP2696A_REG03_IOLIM_2A1				(0<<4)	//!< 2.1A in 100mA increments
#define MP2696A_REG03_IOLIM_2A2				(1<<4)	//!< 2.2A
#define MP2696A_REG03_IOLIM_2A3				(2<<4)	//!< 2.3A
#define MP2696A_REG03_IOLIM_2A4				(3<<4)	//!< 2.4A
#define MP2696A_REG03_IOLIM_2A5				(4<<4)	//!< 2.5A
#define MP2696A_REG03_IOLIM_2A6				(5<<4)	//!< 2.6A
#define MP2696A_REG03_IOLIM_2A7				(6<<4)	//!< 2.7A
#define MP2696A_REG03_IOLIM_2A8				(7<<4)	//!< 2.8A
#define MP2696A_REG03_IOLIM_2A9				(8<<4)	//!< 2.9A
#define MP2696A_REG03_IOLIM_3A0				(9<<4)	//!< 3.0A
#define MP2696A_REG03_IOLIM_3A1				(10<<4)	//!< 3.1A
#define MP2696A_REG03_IOLIM_3A2				(11<<4)	//!< 3.2A
#define MP2696A_REG03_IOLIM_3A3				(12<<4)	//!< 3.3A
#define MP2696A_REG03_IOLIM_3A4				(13<<4)	//!< 3.4A
#define MP2696A_REG03_IOLIM_3A5				(14<<4)	//!< 3.5A
#define MP2696A_REG03_IOLIM_3A6				(15<<4)	//!< 3.6A

/// Boost output voltage setting and boost control.
#define MP2696A_REG04				0x04
#define MP2696A_REG04_USB2_PLUG_IN			(1<<0)
#define MP2696A_REG04_USB2_EN_PLUG			(1<<1)
#define MP2696A_REG04_SYS_DSC_EN			(1<<2)	//!< Enable SYS discharge
#define MP2696A_REG04_Q2_EN					(1<<3)	//!< Q2 On
#define MP2696A_REG04_BST_EN				(1<<4)	//!< Enable Boost
#define MP2696A_REG04_VBOOST_MASK			(7<<5)
#define MP2696A_REG04_VBOOST_5V2			(0<<5)	//!< 5.2V
#define MP2696A_REG04_VBOOST_5V225			(1<<5)	//!< 5.2V + 0.025V
#define MP2696A_REG04_VBOOST_5V15			(2<<5)	//!< 5.2V - 0.05V
#define MP2696A_REG04_VBOOST_5V125			(3<<5)	//!< 5.2V - 0.075V
#define MP2696A_REG04_VBOOST_5V1			(4<<5)	//!< 5.2V - 0.1V
#define MP2696A_REG04_VBOOST_5V075			(5<<5)	//!< 5.2V - 0.125V
#define MP2696A_REG04_VBOOST_5V05			(6<<5)	//!< 5.2V - 0.15V

/// Status register.
#define MP2696A_REG05				0x05
#define MP2696A_REG05_USB1_PLUG_IN			(1<<1)
#define MP2696A_REG05_IPPM_STAT				(1<<2)
#define MP2696A_REG05_VPPM_STAT				(1<<3)
#define MP2696A_REG05_CHG_STAT_MASK			(3<<4)
#define MP2696A_REG05_CHG_STAT_NOTCHARGING	(0<<4)
#define MP2696A_REG05_CHG_STAT_PRECHARG		(1<<4)
#define MP2696A_REG05_CHG_STAT_CCVCHARG		(2<<4)
#define MP2696A_REG05_CHG_STAT_CHARGEDONE	(3<<4)
#define MP2696A_REG05_CHIP_STAT_MASK		(3<<6)
#define MP2696A_REG05_CHIP_STAT_IDLE		(0<<6)
#define MP2696A_REG05_CHIP_STAT_CHARGE		(1<<6)
#define MP2696A_REG05_CHIP_STAT_BOOST		(2<<6)
#define MP2696A_REG05_CHIP_STAT_PWRPATH		(3<<6)

/// Fault register.
#define MP2696A_REG06				0x06
#define MP2696A_REG06_NTC_FAULT_MASK		(7<<0)
#define MP2696A_REG06_NTC_FAULT_NORMAL		(0<<0)
#define MP2696A_REG06_NTC_FAULT_WARM		(1<<0)
#define MP2696A_REG06_NTC_FAULT_COOL		(2<<0)
#define MP2696A_REG06_NTC_FAULT_COLD		(3<<0)
#define MP2696A_REG06_NTC_FAULT_HOT			(4<<0)
#define MP2696A_REG06_CHG_FAULT_MASK		(3<<3)
#define MP2696A_REG06_CHG_FAULT_NORMAL		(0<<3)
#define MP2696A_REG06_CHG_FAULT_USB1_UV		(1<<3)
#define MP2696A_REG06_CHG_FAULT_USB1_OV		(2<<3)
#define MP2696A_REG06_CHG_FAULT_TIMEREXP	(3<<3)
#define MP2696A_REG06_BST_LMT				(1<<5)	//!< Boost works in Q2 curr limit
#define MP2696A_REG06_SYS_SHORT				(1<<6)	//!< SYS short circuit
#define MP2696A_REG06_BATT_UVLO				(1<<7)	//!< Battery

/// Boost no-load setting and miscellaneous control.
#define MP2696A_REG07				0x07
#define MP2696A_REG07_BST_IPK_MASK			(3<<0)
#define MP2696A_REG07_BST_IPK_5A			(0<<0)
#define MP2696A_REG07_BST_IPK_5A5			(1<<0)
#define MP2696A_REG07_BST_IPK_6A			(2<<0)
#define MP2696A_REG07_BST_IPK_6A5			(3<<0)
#define MP2696A_REG07_SW_FREQ_700KHZ		(0<<2)
#define MP2696A_REG07_SW_FREQ_1200KHZ		(1<<2)
#define MP2696A_REG07_VIN_OVP_6V			(0<<3)
#define MP2696A_REG07_VIN_OVP_11V			(1<<3)
#define MP2696A_REG07_NTC_STOP				(1<<4)	//!< Suspend charge and boost on NTC out of window
#define MP2696A_REG07_BATT_OVP				(1<<5)
#define MP2696A_REG07_NOLOAD_THR_MASK		(3<<6)
#define MP2696A_REG07_NOLOAD_THR_30MA		(0<<6)
#define MP2696A_REG07_NOLOAD_THR_50MA		(1<<6)
#define MP2696A_REG07_NOLOAD_THR_75MA		(2<<6)
#define MP2696A_REG07_NOLOAD_THR_100MA		(3<<6)

/// JEITA control.
#define MP2696A_REG08				0x08
#define MP2696A_REG08_VCOLD_72				(0<<0)	//!< Cold threshold 72%
#define MP2696A_REG08_VCOLD_68				(1<<0)	//!< Cold threshold 68%
#define MP2696A_REG08_VCOOL_MASK			(3<<1)
#define MP2696A_REG08_VCOOL_72				(0<<1)
#define MP2696A_REG08_VCOOL_68				(1<<1)
#define MP2696A_REG08_VCOOL_64				(2<<1)
#define MP2696A_REG08_VCOOL_60				(3<<1)
#define MP2696A_REG08_VWARM_MASK			(3<<3)
#define MP2696A_REG08_VWARM_44				(0<<3)
#define MP2696A_REG08_VWARM_40				(1<<3)
#define MP2696A_REG08_VWARM_38				(2<<3)
#define MP2696A_REG08_VWARM_36				(3<<3)
#define MP2696A_REG08_VHOT_34				(0<<5)	//!< Hot threshold 34%
#define MP2696A_REG08_VHOT_36				(1<<5)	//!< Hot threshold 36%
#define MP2696A_REG08_JEITA_ISET_143		(0<<6)	//!< 14.3% off Icc
#define MP2696A_REG08_JEITA_ISET_500		(1<<6)	//!< 50.0% off Icc
#define MP2696A_REG08_JEITA_VSET_100MV		(0<<7)	//!< Vbatfull - 100mV
#define MP2696A_REG08_JEITA_VSET_200MV		(1<<7)	//!< Vbatfull - 200mV


#ifdef __cplusplus

class PmMp2696a : public PowerMgnt {
public:
	bool Init(const PWRCFG &Cfg, DeviceIntrf * const pIntrf);

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
	uint32_t SetCharge(PWR_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr);

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
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif // __PM_MP2696A_H__

