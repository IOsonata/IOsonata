/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf54.h

@brief	NvmMcu operations for the nRF54L internal RRAM.

		One operation set for every build, as on the nRF52. The memory work is
		the same in all of them: enable write mode, store the words, wait for
		the write buffer to drain, restore. What changes is only who is allowed
		to touch the memory and how the wait is spent, and the driver works
		that out rather than asking the application to pick.

		  no stack running	 the controller is driven directly
		  link controller	 the same work runs inside an MPSL timeslot
		  SoftDevice running the work is submitted to the SoftDevice and the
		 					 result arrives as a SoC event

		RRAM has no erase command; it rewrites in place. That is a detail of
		this medium, so the erase operation here writes the erased pattern over
		the unit instead. Nothing above sees the difference, which is the point
		of putting the medium in the operation rather than in the driver.

		The geometry cannot be read from the device the way the nRF52 page size
		comes from FICR, so the total size is a build value. Override
		NVM_MCU_NRF54_TOTAL_SIZE for a part other than the nRF54L15.

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
#ifndef __NVM_MCU_NRF54_H__
#define __NVM_MCU_NRF54_H__

#include "storage/nvm_mcu.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// Called repeatedly while an operation is waiting, so the application can run
/// its event dispatch or yield to a scheduler. Returning false asks the driver
/// to give up on the wait.
typedef bool (*NvmMcuNrf54Wait_t)(void);

/// Counts of what the operations did, for a test to show which path ran.
typedef struct __Nvm_Mcu_Nrf54_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while the radio held the memory
	uint32_t	Evt;		//!< Results delivered as a SoC event
} NvmMcuNrf54Stat_t;

/**
 * @brief	Bring up whatever the build needs before the first access.
 *
 * Optional. The operations do this on first use.
 *
 * @return	0 on success, negative errno on failure.
 */
int NvmMcuNrf54Init(void);

/**
 * @brief	Set the wait callback and the operation timeout.
 *
 * @param	pWait		: Called repeatedly while an operation waits. NULL to
 * \t\t\t\t\t\t  spend the wait in a short delay instead.
 * @param	TimeoutMs	: Give up on one operation after this many msec.
 * \t\t\t\t\t\t  0 keeps the current value.
 */
void NvmMcuNrf54SetWait(NvmMcuNrf54Wait_t pWait, uint32_t TimeoutMs);

/**
 * @brief	Pass a SoftDevice SoC event to the operations.
 *
 * Needed only on a build running a SoftDevice.
 *
 * @param	SysEvt : SoC event id
 */
void NvmMcuNrf54SocEvt(uint32_t SysEvt);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void NvmMcuNrf54GetStat(NvmMcuNrf54Stat_t *pStat);

/**
 * @brief	Get the operations to pass to NvmMcu::Init.
 *
 * @return	Reference to the operation set.
 */
const NvmMcuOp_t & NvmMcuNrf54Op(void);

/**
 * @brief	Fill the geometry fields of a config.
 *
 * Sets BaseAddr, TotalSize, EraseSize and WriteGran for the RRAM. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmMcuNrf54Cfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_MCU_NRF54_H__
