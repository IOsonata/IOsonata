/**-------------------------------------------------------------------------
@file	nvm_mcu_nvmc.h

@brief	NvmMcu operations over the Nordic NVMC, with no stack arbitration.

		This is the plain path. The application owns the memory controller and
		nothing else uses it, so the write enable, the word stores, the page
		erase and the ready wait are performed directly and each operation
		returns once the data is committed.

		Use this on a build with no radio activity, or where the storage region
		is only written while the radio is idle. A build running a full
		SoftDevice, or one running the link layer controller with MPSL
		timeslots, needs operations that schedule the access around radio
		activity instead, because the memory controller stalls instruction
		fetch for the duration and a page erase is far longer than a radio
		event tolerates.

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
#ifndef __NVM_MCU_NVMC_H__
#define __NVM_MCU_NVMC_H__

#include "storage/nvm_mcu.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/**
 * @brief	Get the NVMC operations to pass to NvmMcu::Init.
 *
 * @return	Reference to the operation set.
 */
const NvmMcuOp_t & NvmMcuNvmcOp(void);

/**
 * @brief	Fill the geometry fields of a config from the device itself.
 *
 * Reads the page size and the page count from the factory information
 * registers and sets BaseAddr, TotalSize, EraseSize and WriteGran. The
 * remaining fields are left untouched, so the mode and the callbacks may be
 * set either before or after this call.
 *
 * @param	Cfg	: Config to fill
 */
void NvmMcuNvmcCfg(NvmCfg_t &Cfg);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_MCU_NVMC_H__
