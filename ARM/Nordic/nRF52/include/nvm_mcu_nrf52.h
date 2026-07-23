/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf52.h

@brief	NvmMcu operations for the nRF52 internal memory.

		One operation set for every build. The memory work is the same in all
		of them: put the controller in write or erase mode, store the words or
		start the page erase, wait for ready, return the controller to read
		only. What changes is only who is allowed to touch the memory and how
		the wait is spent, and the driver works that out rather than asking the
		application to pick an implementation.

		  no stack running	 the controller is driven directly and the wait is
		 					 spent in the wait callback
		  link controller	 the same work runs inside an MPSL timeslot, and a
		 					 page erase is sliced across several of them
		  SoftDevice running the work is submitted to the SoftDevice and the
		 					 result arrives as a SoC event

		A SoftDevice that is present but stopped arbitrates nothing, so the
		controller is driven directly in that case as well.

		The long wait is handed to the application through NvmMcuNrf52SetWait,
		the same idea as the wait callback the flash and EEPROM drivers take in
		their config. That is what lets one operation set serve a bare metal
		loop, an event driven application and an RTOS without a variant for
		each.

		On a SoftDevice build the application must pass SoC events to
		NvmMcuNrf52SocEvt, from its own event dispatch.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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
#ifndef __NVM_MCU_NRF52_H__
#define __NVM_MCU_NRF52_H__

#include "storage/nvm_mcu.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// Called repeatedly while an operation is waiting, so the application can run
/// its event dispatch or yield to a scheduler. Returning false asks the driver
/// to give up on the wait.
typedef bool (*NvmMcuNrf52Wait_t)(void);

/// Counts of what the operations did, for a test to show which path ran.
typedef struct __Nvm_Mcu_Nrf52_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while the radio held the memory
	uint32_t	Evt;		//!< Results delivered as a SoC event
	uint32_t	Skipped;	//!< Page erases skipped, the page already read erased
} NvmMcuNrf52Stat_t;

/**
 * @brief	Bring up whatever the build needs before the first access.
 *
 * Optional. The operations do this on first use; call it only to choose when
 * it happens.
 *
 * @return	0 on success, negative errno on failure.
 */
int NvmMcuNrf52Init(void);

/**
 * @brief	Set the wait callback and the operation timeout.
 *
 * @param	pWait		: Called repeatedly while an operation waits. NULL to
 * \t\t\t\t\t\t  spend the wait in a short delay instead.
 * @param	TimeoutMs	: Give up on one operation after this many msec.
 * \t\t\t\t\t\t  0 keeps the current value.
 */
void NvmMcuNrf52SetWait(NvmMcuNrf52Wait_t pWait, uint32_t TimeoutMs);

/**
 * @brief	Pass a SoftDevice SoC event to the operations.
 *
 * Needed only on a build running a SoftDevice. Call it from the application
 * event dispatch for every SoC event; anything that is not a memory result is
 * ignored, so passing all of them is safe.
 *
 * @param	SysEvt : SoC event id
 */
void NvmMcuNrf52SocEvt(uint32_t SysEvt);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void NvmMcuNrf52GetStat(NvmMcuNrf52Stat_t *pStat);

/**
 * @brief	Get the operations to pass to NvmMcu::Init.
 *
 * @return	Reference to the operation set.
 */
const NvmMcuOp_t & NvmMcuNrf52Op(void);

/**
 * @brief	Fill the geometry fields of a config from the device itself.
 *
 * Reads the page size and the page count from the factory information
 * registers and sets BaseAddr, TotalSize, EraseSize and WriteGran. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmMcuNrf52Cfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_MCU_NRF52_H__
