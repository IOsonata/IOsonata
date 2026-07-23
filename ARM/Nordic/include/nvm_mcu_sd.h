/**-------------------------------------------------------------------------
@file	nvm_mcu_sd.h

@brief	NvmMcu operations over a full SoftDevice.

		The SoftDevice owns the memory controller and schedules the access
		around radio activity. A request is submitted with sd_flash_write or
		sd_flash_page_erase, is refused with a busy status while the radio
		holds the memory, and completes later through a SoC event.

		These operations wrap that into the plain behaviour NvmMcu expects:
		submit, retry while busy, wait for the completion event, return once
		the data is committed. The application sees no event model.

		Two things are needed from the application:

		  1. Every SoC event must be passed to NvmMcuSdSocEvt so the completion
			 can be observed. Wire it into whatever SoftDevice event dispatch
			 the application already runs.

		  2. If that dispatch does not run from an interrupt, supply an idle
			 callback with NvmMcuSdSetIdle so it can run while an operation is
			 pending. Without it the wait would never see the event.

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
#ifndef __NVM_MCU_SD_H__
#define __NVM_MCU_SD_H__

#include "storage/nvm_mcu.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// Idle callback run while an operation is pending, so an application whose
/// SoftDevice event dispatch is not interrupt driven can still deliver the
/// completion event.
typedef void (*NvmMcuSdIdle_t)(void);

/**
 * @brief	Pass a SoftDevice SoC event to the NVM operations.
 *
 * Call this from the application SoftDevice event dispatch for every SoC
 * event. Events other than the memory operation results are ignored, so it is
 * safe to pass all of them.
 *
 * @param	SysEvt : SoC event id
 */
void NvmMcuSdSocEvt(uint32_t SysEvt);

/**
 * @brief	Set the idle callback and the operation timeout.
 *
 * @param	pIdle		: Called repeatedly while an operation is pending. NULL
 * 					  when the SoftDevice event dispatch runs from an
 * 					  interrupt and needs no help.
 * @param	TimeoutMs	: Give up on one submitted operation after this many
 * 					  msec. 0 keeps the current value.
 */
void NvmMcuSdSetIdle(NvmMcuSdIdle_t pIdle, uint32_t TimeoutMs);

/**
 * @brief	Get the SoftDevice operations to pass to NvmMcu::Init.
 *
 * @return	Reference to the operation set.
 */
const NvmMcuOp_t & NvmMcuSdOp(void);

/**
 * @brief	Fill the geometry fields of a config from the device itself.
 *
 * Reads the page size and the page count from the factory information
 * registers and sets BaseAddr, TotalSize, EraseSize and WriteGran. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmMcuSdCfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_MCU_SD_H__
