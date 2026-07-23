/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf52_sd.cpp

@brief	NvmMcu operations over a full SoftDevice on nRF52.

		The SoftDevice arbitrates memory access against radio activity, so a
		request is submitted rather than performed. sd_flash_write and
		sd_flash_page_erase return a busy status while the radio holds the
		memory, and report the result later through a SoC event.

		Each operation here submits, retries while busy, and waits for the
		completion event before returning, so NvmMcu and the application see
		the plain behaviour of a memory that is written when the call returns.

		Note that sd_flash_page_erase takes a page number, not an address.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"

#include "idelay.h"

#include "nvm_mcu_sd.h"

// The memory programs 32 bit words.
#define NVM_MCU_SD_WRITE_GRAN		4

// Largest word count submitted in one request. This bounds how long a single
// request occupies the memory, which matters because the radio cannot use it
// meanwhile. Raise it for fewer, longer requests.
#ifndef NVM_MCU_SD_MAX_PROG_WORDS
#define NVM_MCU_SD_MAX_PROG_WORDS	256
#endif

// Default give up time for one submitted operation.
#ifndef NVM_MCU_SD_TIMEOUT_MS
#define NVM_MCU_SD_TIMEOUT_MS		5000
#endif

static volatile bool s_OpDone;
static volatile bool s_OpOk;
static NvmMcuSdIdle_t s_pIdle = nullptr;
static uint32_t s_TimeoutMs = NVM_MCU_SD_TIMEOUT_MS;
static uint32_t s_BusyCnt;
static uint32_t s_EvtCnt;
static uint32_t s_SyncCnt;

// True when the SoftDevice is running. When it is not, the memory calls
// complete before they return and no SoC event is produced.
static bool SdIsEnabled(void)
{
	uint8_t en = 0;

	if (sd_softdevice_is_enabled(&en) != NRF_SUCCESS)
	{
		return false;
	}

	return en != 0;
}

void NvmMcuSdSocEvt(uint32_t SysEvt)
{
	switch (SysEvt)
	{
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			s_OpOk = true;
			s_OpDone = true;
			s_EvtCnt++;
			break;

		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_OpOk = false;
			s_OpDone = true;
			s_EvtCnt++;
			break;

		default:
			break;
	}
}

void NvmMcuSdSetIdle(NvmMcuSdIdle_t pIdle, uint32_t TimeoutMs)
{
	s_pIdle = pIdle;
	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

/// Submit one request. Returns the SoftDevice status.
typedef uint32_t (*NvmMcuSdSubmit_t)(void *pArg);

/// Let the application run for about a msec, so its event dispatch can deliver
/// the completion event and the radio can release the memory.
static void SdIdleStep(void)
{
	if (s_pIdle != nullptr)
	{
		s_pIdle();
	}
	msDelay(1);
}

/// Submit, retry while the radio holds the memory, then wait for the result.
static int SdRun(NvmMcuSdSubmit_t Submit, void *pArg)
{
	uint32_t elapsed = 0;

	while (true)
	{
		s_OpDone = false;
		s_OpOk = false;
		__DMB();

		uint32_t status = Submit(pArg);

		if (status == NRF_SUCCESS)
		{
			// With the SoftDevice stopped the call is already finished and
			// sends no event, so there is nothing to wait for.
			if (SdIsEnabled() == false)
			{
				s_SyncCnt++;
				return 0;
			}
			break;
		}
		if (status != NRF_ERROR_BUSY)
		{
			return -EIO;
		}

		// The radio holds the memory. Let the application run and try again.
		s_BusyCnt++;
		SdIdleStep();
		if (++elapsed >= s_TimeoutMs)
		{
			return -ETIMEDOUT;
		}
	}

	// Accepted. The result arrives as a SoC event.
	while (elapsed < s_TimeoutMs)
	{
		if (s_OpDone)
		{
			return s_OpOk ? 0 : -EIO;
		}
		SdIdleStep();
		elapsed++;
	}

	return -ETIMEDOUT;
}

typedef struct {
	uint32_t		*pDst;
	const uint32_t	*pSrc;
	uint32_t		Words;
} NvmMcuSdWrArg_t;

static uint32_t SdWriteSubmit(void *pArg)
{
	NvmMcuSdWrArg_t *a = (NvmMcuSdWrArg_t *)pArg;

	return sd_flash_write(a->pDst, a->pSrc, a->Words);
}

static int SdProgram(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt,
					 void *pCtx)
{
	(void)pCtx;

	if ((Addr & (NVM_MCU_SD_WRITE_GRAN - 1)) != 0)
	{
		return -EINVAL;
	}
	if (WordCnt == 0)
	{
		return 0;
	}

	NvmMcuSdWrArg_t arg = { (uint32_t *)Addr, pSrc, WordCnt };

	return SdRun(SdWriteSubmit, &arg);
}

static uint32_t SdEraseSubmit(void *pArg)
{
	return sd_flash_page_erase(*(uint32_t *)pArg);
}

static int SdErasePage(uintptr_t Addr, void *pCtx)
{
	(void)pCtx;

	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	if (pagesize == 0)
	{
		return -EINVAL;
	}

	// The SoftDevice erase takes a page number, not an address.
	uint32_t page = (uint32_t)(Addr / pagesize);

	return SdRun(SdEraseSubmit, &page);
}

// Region write protect is left out. The nRF52 block protect only clears on
// reset, which does not match an operation that must also unprotect, so NvmMcu
// reports write protect as not supported here.
static const NvmMcuOp_t s_NvmMcuSdOp = {
	.Program = SdProgram,
	.ErasePage = SdErasePage,
	.SetProtect = nullptr,
	.MaxProgWords = NVM_MCU_SD_MAX_PROG_WORDS,
	.pCtx = nullptr
};

uint32_t NvmMcuSdBusyCount(void)
{
	return s_BusyCnt;
}

uint32_t NvmMcuSdEvtCount(void)
{
	return s_EvtCnt;
}

uint32_t NvmMcuSdSyncCount(void)
{
	return s_SyncCnt;
}

const NvmMcuOp_t & NvmMcuSdOp(void)
{
	return s_NvmMcuSdOp;
}

void NvmMcuSdCfg(NvmCfg_t &Cfg)
{
	// Geometry from the device itself rather than a build time constant.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.BaseAddr = 0;
	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
	Cfg.WriteGran = NVM_MCU_SD_WRITE_GRAN;
}
