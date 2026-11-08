/**-------------------------------------------------------------------------
@file	mcu_power.h

@brief	Generic MCU power control

Power functions of the MCU itself, not of an external power management chip
(see pwrmgnt/pwrmgnt.h for those):
- reset reason;
- power off, the lowest power mode, left only through a reset;
- retention registers, kept through power off and software resets;
- DC/DC regulator stages;
- RAM retention in power off.

Power off stops the MCU until a wake up source resets it. The wake up
sources are set with their own drivers before power off, for example
IOPinSetSense for a pin or the analog comparator. The reset reason of the
next start tells which one woke the MCU.

A function an MCU does not have returns false or 0.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __MCU_POWER_H__
#define __MCU_POWER_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup Coredev
  * @{
  */

/// Reset reasons, ORed, returned by McuResetReason
#define MCU_RESET_POWERON		(1UL << 0)	//!< Power on or brown out
#define MCU_RESET_PIN			(1UL << 1)	//!< Reset pin
#define MCU_RESET_WDT			(1UL << 2)	//!< Watchdog
#define MCU_RESET_SOFT			(1UL << 3)	//!< Software reset request
#define MCU_RESET_LOCKUP		(1UL << 4)	//!< CPU lockup
#define MCU_RESET_DEBUG			(1UL << 5)	//!< Debug interface
#define MCU_RESET_WAKE_PIN		(1UL << 8)	//!< Wake up from power off by a pin
#define MCU_RESET_WAKE_COMP		(1UL << 9)	//!< Wake up from power off by the analog comparator
#define MCU_RESET_WAKE_NFC		(1UL << 10)	//!< Wake up from power off by an NFC field
#define MCU_RESET_WAKE_USB		(1UL << 11)	//!< Wake up from power off by USB power
#define MCU_RESET_WAKE_MSK		(0xFUL << 8)	//!< All wake up from power off reasons

/// DC/DC regulator stages for McuDcDcEnable
#define MCU_DCDC_MAIN			0			//!< Stage supplying the core
#define MCU_DCDC_HIGHVOLT		1			//!< High voltage input stage, on MCU with one

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Reasons of the last reset
 *
 * Where the hardware adds the reasons of successive resets until they are
 * cleared, the first call reads and clears them, later calls return the same
 * value.
 *
 * @return	MCU_RESET_xxx reasons, ORed
 */
uint32_t McuResetReason(void);

/**
 * @brief	Power off
 *
 * Enters the lowest power mode. Only a reset leaves it: a wake up source set
 * before the call, the reset pin or a power cycle. RAM is lost except for
 * the parts kept with McuRamRetain. The function does not return.
 */
void McuPowerOff(void);

/**
 * @brief	Number of retention registers
 *
 * Retention registers keep their value through power off and software
 * resets, not through a power cycle.
 *
 * @return	Number of registers, 0 for none
 */
int McuRetRegCount(void);

/**
 * @brief	Read a retention register
 *
 * @param	Idx : Register index, 0 to McuRetRegCount() - 1
 *
 * @return	Register value, 0 for an invalid index
 */
uint32_t McuRetRegRead(int Idx);

/**
 * @brief	Write a retention register
 *
 * Bits above the register width are dropped.
 *
 * @param	Idx : Register index, 0 to McuRetRegCount() - 1
 * @param	Val : Value to keep
 *
 * @return	true - success
 */
bool McuRetRegWrite(int Idx, uint32_t Val);

/**
 * @brief	Select DC/DC or linear regulator for one stage
 *
 * The DC/DC regulator needs its inductor on the board. Enabling it on a
 * board without one stops the MCU from running properly.
 *
 * @param	Stage : MCU_DCDC_MAIN or MCU_DCDC_HIGHVOLT
 * @param	bEn	  : true - DC/DC, false - linear regulator
 *
 * @return	true - success, false - the MCU does not have this stage
 */
bool McuDcDcEnable(int Stage, bool bEn);

/**
 * @brief	Keep or drop a RAM range in power off
 *
 * The hardware works on RAM sections, every section the range touches is
 * changed.
 *
 * @param	Addr	: Start address
 * @param	Size	: Size in bytes
 * @param	bRetain	: true - keep the content in power off
 *
 * @return	true - success, false - the range is not RAM
 */
bool McuRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
