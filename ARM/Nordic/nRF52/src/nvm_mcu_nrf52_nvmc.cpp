/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf52_nvmc.cpp

@brief	NvmMcu operations over the nRF52 NVMC, with no stack arbitration.

		The sequence is the one the memory controller requires: put the
		controller in write or erase mode, perform the word stores or start the
		page erase, wait for the ready flag, then return the controller to read
		only. This is the same shape a serial flash driver uses, with the write
		enable and the erase command being controller operations instead of bus
		commands.

		Nothing here arbitrates against radio activity. The controller stalls
		instruction fetch while it is busy, so a page erase, which takes on the
		order of tens of milliseconds, halts the core for that whole time. That
		is acceptable only when no radio event has to be met.

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
#include <stdint.h>
#include <errno.h>

#include "hal/nrf_nvmc.h"

#include "nvm_mcu_nvmc.h"

// The NVMC programs 32 bit words.
#define NVM_MCU_NVMC_WRITE_GRAN		4

static int NvmcProgram(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt,
					   void *pCtx)
{
	(void)pCtx;

	if ((Addr & (NVM_MCU_NVMC_WRITE_GRAN - 1)) != 0)
	{
		return -EINVAL;
	}
	if (WordCnt == 0)
	{
		return 0;
	}

	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_WRITE);

	volatile uint32_t *pDst = (volatile uint32_t *)Addr;
	for (uint32_t i = 0; i < WordCnt; i++)
	{
		pDst[i] = pSrc[i];
		while (!nrf_nvmc_ready_check(NRF_NVMC))
		{
			// A word write takes a few hundred usec.
		}
	}

	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);

	return 0;
}

static int NvmcErasePage(uintptr_t Addr, void *pCtx)
{
	(void)pCtx;

	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_ERASE);
	nrf_nvmc_page_erase_start(NRF_NVMC, (uint32_t)Addr);

	while (!nrf_nvmc_ready_check(NRF_NVMC))
	{
		// The core is stalled on instruction fetch for the whole erase.
	}

	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);

	return 0;
}

// Region write protect is left out. The nRF52 block protect is set once and
// only clears on reset, which does not match an operation that both protects
// and unprotects, so NvmMcu reports write protect as not supported here.
static const NvmMcuOp_t s_NvmMcuNvmcOp = {
	.Program = NvmcProgram,
	.ErasePage = NvmcErasePage,
	.SetProtect = nullptr,
	.MaxProgWords = 0,			// no limit, the controller takes any count
	.pCtx = nullptr
};

const NvmMcuOp_t & NvmMcuNvmcOp(void)
{
	return s_NvmMcuNvmcOp;
}

void NvmMcuNvmcCfg(NvmCfg_t &Cfg)
{
	// Geometry from the device itself rather than a build time constant.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.BaseAddr = 0;
	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
	Cfg.WriteGran = NVM_MCU_NVMC_WRITE_GRAN;
}
