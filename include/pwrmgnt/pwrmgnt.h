/**-------------------------------------------------------------------------
@file	pwrmgnt.h

@brief	Generic power management definition

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	July 25, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
#ifndef __PWRMGNT_H__
#define __PWRMGNT_H__

#include <stdint.h>

#include "device_intrf.h"
#include "device.h"
#include "miscdev/led.h"

/** @addtogroup Power
  * @{
  */


#define PWRMGNT_VOUT_MAXCNT			4	//!< Max number of Vout

class PowerMgnt;

#pragma pack(push, 1)
typedef enum __Charge_Type {
	PWRMGNT_CHARGE_TYPE_NORMAL,
	PWRMGNT_CHARGE_TYPE_TRICKLE,
	PWRMGNT_CHARGE_TYPE_AUTO			//!< Auto select optimum charge by driver implementation
} PWRMGNT_CHARGE_TYPE;

typedef struct __Vout_Cfg {
	int32_t mVout;						//!< Output voltage in mV
	uint32_t mAlimit;					//!< Output current limit in mA
} PwrMgntVoutCfg_t;

typedef enum __PowerMgnt_Evt {
	PWRMGNT_EVT_CHARGE_FULL,			//!< Battery full
	PWRMGNT_EVT_LOW_BAT,				//!< Battery low
	PWRMGNT_EVT_OVER_HEAT,				//!< Over heat detected
	PWRMGNT_EVT_CHARGE_DETECTED			//!< Charge source detected
} PWRMGNT_EVT;

typedef struct __Bat_Profile {
	uint32_t OpVolt;		//!< Battery nominal operating voltage in mV
	uint32_t ChrgVolt;		//!< Battery charge voltage in mV
	uint32_t Capacity;		//!< Battery capacity in mAh
	uint32_t ThermBetaConst;//!< Thermistor Beta constant
	uint32_t ThermResistor;	//!< Thermistor resistor value in KOhms
} BatProfile_t;

typedef void (*PwrMgntEvtHandler_t)(PowerMgnt *pSelf, PWRMGNT_EVT Evt);

typedef struct __PowerMgnt_Config {
	uint32_t DevAddr;					//!< Device address
	//uint32_t InCurLimit;				//!< Input current limit in mA, USB in current limit. Device dependent
	const PwrMgntVoutCfg_t * const pVout;		//!< Pointer to V out settings
	size_t NbVout;						//!< Number of V out
	int32_t VEndChrg;					//!< End of charge voltage level in mV
	uint32_t ChrgCurr;					//!< Charge current in mA
	uint32_t ChrgTimeout;				//!< Charge timeout in minutes
	const BatProfile_t * const pBatProf;	//!< Pointer to battery profile
	bool bIntEn;						//!< Interrupt enable
	int IntPrio;						//!< Interrupt priority
	int OffSwPin;						//!< Power switch button I/O pin assignment
	int OffSwHold;						//!< Power switch off hold time in seconds
	LedDev_t * const pLed;
	int NbLed;
	PwrMgntEvtHandler_t pEvtHandler;
} PwrMgntCfg_t;

#pragma pack(pop)

class PowerMgnt : virtual public Device {
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
	virtual bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf *pIntrf) = 0;

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
	virtual int32_t SetVout(size_t VoutIdx, int32_t mVolt, uint32_t mALimit) = 0;

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
	virtual uint32_t SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr) = 0;

	/**
	 * @brief	Charging status
	 *
	 * @return	true - Charging
	 */
	virtual bool Charging() { return false; }

	/**
	 * @brief	Battery present status
	 *
	 * @return	true - Battery present
	 */
	virtual bool Battery() { return false; }

	/**
	 * @brief	Interrupt handler
	 *
	 * Optional implementation to handle interrupt. This is device specific.
	 *
	 */
	virtual void IrqHandler() {}

protected:
	uint32_t vChrgCurr;	//!< Charge current
	PwrMgntEvtHandler_t vpEvtHandler;

private:
//	LED_DEV vLed;			//!< Led active level
};

// Low battery event handler
typedef void (*FGBatLowEvtHandler_t)(PowerMgnt * const pPwrMnt);

typedef struct __FuelGauge_Cfg {
	uint8_t DevAddr;
	const BatProfile_t &BatProf;//!< Reference to battery profile
	FGBatLowEvtHandler_t BatLowHandler;		//!< Battery low event handler
} FuelGaugeCfg_t;

class FuelGauge : virtual public Device {
public:
	virtual bool Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt) = 0;

	/**
	 * @brief	Get battery level
	 *
	 * Returns battery level in 1 digit fixed point decimal.
	 *
	 * ex. 123 => 12.3%
	 *
	 * @return	Battery level in (0-100) % in 1 digit fixed point
	 */
	virtual uint16_t Level() = 0;
	/**
	 * @brief	Get battery temperature
	 *
	 * Returns battery temperature in 1 digit fixed point decimal.
	 *
	 * ex. 123 => 12.3 C
	 *
	 * @return	Battery level in (0-100) degree C in 1 digit fixed point
	 */
	virtual int32_t Temperature() = 0;
	virtual int32_t Voltage() = 0;

protected:
	void LowBatAlert() { if (vBatLowHandler) vBatLowHandler(vpPwrMgnt); }

protected:
	FGBatLowEvtHandler_t	vBatLowHandler;		//!< Low battery event handler
	BatProfile_t vBatProfile;
	PowerMgnt *vpPwrMgnt;
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif //__PWRMGNT_H__
