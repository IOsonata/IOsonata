/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf52.cpp

@brief	NvmMcu operations for the nRF52 internal memory.

		The controller mechanics appear once. Everything else in this file
		decides where that work runs and how the wait is spent.

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
#include "idelay.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#endif

#ifdef NRFXLIB_SDC
#include "bt_pds_sdc.h"
#endif

#include "nvm_mcu_nrf52.h"

// The controller programs 32 bit words.
#define NVM_MCU_NRF52_WRITE_GRAN	4

// Words one program request may take. A word write is a few hundred usec, and
// on a timeslot build the whole request has to fit one slot, which is the
// tightest limit of the three cases. NvmMcu splits a longer write to match.
#ifndef NVM_MCU_NRF52_MAX_PROG_WORDS
#define NVM_MCU_NRF52_MAX_PROG_WORDS	8
#endif

// Erase slice per timeslot, in msec, and the worst case for one step.
#ifndef NVM_MCU_NRF52_ERASE_SLICE_MS
#define NVM_MCU_NRF52_ERASE_SLICE_MS	2
#endif

#ifndef NVM_MCU_NRF52_STEP_BUDGET_US
#define NVM_MCU_NRF52_STEP_BUDGET_US	((NVM_MCU_NRF52_ERASE_SLICE_MS + 1) * 1000UL)
#endif

#ifndef NVM_MCU_NRF52_TIMEOUT_MS
#define NVM_MCU_NRF52_TIMEOUT_MS	5000
#endif

static NvmMcuNrf52Wait_t s_pWait = nullptr;
static uint32_t s_TimeoutMs = NVM_MCU_NRF52_TIMEOUT_MS;
static NvmMcuNrf52Stat_t s_Stat;

// ---------------------------------------------------------------------------
// The controller mechanics. One copy, used by every path below.
// ---------------------------------------------------------------------------

static void NvmcProgramWords(uintptr_t Addr, const uint32_t *pSrc,
							 uint32_t WordCnt)
{
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
}

static void NvmcEraseBegin(uintptr_t Addr)
{
	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_ERASE);
	nrf_nvmc_page_erase_start(NRF_NVMC, (uint32_t)Addr);
}

static bool NvmcIsReady(void)
{
	return nrf_nvmc_ready_check(NRF_NVMC);
}

static void NvmcRelease(void)
{
	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
}

// True when the page already reads erased, so it can be left alone. Saves the
// erase and the wear that goes with it.
static bool NvmcPageIsErased(uintptr_t Addr, uint32_t PageSize)
{
	const uint32_t *p = (const uint32_t *)Addr;
	uint32_t words = PageSize / 4;

	for (uint32_t i = 0; i < words; i++)
	{
		if (p[i] != 0xFFFFFFFFUL)
		{
			return false;
		}
	}

	return true;
}

// Spend one unit of a long wait in the application, or in a short delay when
// it supplied nothing. Returns false when the application asked to give up.
static bool WaitStep(void)
{
	if (s_pWait != nullptr)
	{
		return s_pWait();
	}

	msDelay(1);

	return true;
}

// ---------------------------------------------------------------------------
// SoftDevice path. The memory is not ours to touch while it runs, so the work
// is submitted and the result arrives as a SoC event.
// ---------------------------------------------------------------------------
#ifdef SOFTDEVICE_PRESENT

static volatile bool s_OpDone;
static volatile bool s_OpOk;

static bool SdIsEnabled(void)
{
	uint8_t en = 0;

	if (sd_softdevice_is_enabled(&en) != NRF_SUCCESS)
	{
		return false;
	}

	return en != 0;
}

void NvmMcuNrf52SocEvt(uint32_t SysEvt)
{
	switch (SysEvt)
	{
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			s_OpOk = true;
			s_OpDone = true;
			s_Stat.Evt++;
			break;

		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_OpOk = false;
			s_OpDone = true;
			s_Stat.Evt++;
			break;

		default:
			break;
	}
}

typedef uint32_t (*SdSubmit_t)(void *pArg);

// Submit, retry while the radio holds the memory, then wait for the result.
static int SdRun(SdSubmit_t Submit, void *pArg)
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
			break;
		}
		if (status != NRF_ERROR_BUSY)
		{
			return -EIO;
		}

		s_Stat.Busy++;
		if (WaitStep() == false || ++elapsed >= s_TimeoutMs)
		{
			return -ETIMEDOUT;
		}
	}

	while (elapsed < s_TimeoutMs)
	{
		if (s_OpDone)
		{
			return s_OpOk ? 0 : -EIO;
		}
		if (WaitStep() == false)
		{
			return -ETIMEDOUT;
		}
		elapsed++;
	}

	return -ETIMEDOUT;
}

typedef struct {
	uint32_t		*pDst;
	const uint32_t	*pSrc;
	uint32_t		Words;
} SdWrArg_t;

static uint32_t SdWriteSubmit(void *pArg)
{
	SdWrArg_t *a = (SdWrArg_t *)pArg;

	return sd_flash_write(a->pDst, a->pSrc, a->Words);
}

static uint32_t SdEraseSubmit(void *pArg)
{
	// The SoftDevice erase takes a page number, not an address.
	return sd_flash_page_erase(*(uint32_t *)pArg);
}

#else

void NvmMcuNrf52SocEvt(uint32_t SysEvt)
{
	(void)SysEvt;
}

#endif	// SOFTDEVICE_PRESENT

// ---------------------------------------------------------------------------
// Timeslot path. The controller is ours, but the radio needs its slots, so the
// same mechanics run inside a timeslot and a page erase is sliced.
// ---------------------------------------------------------------------------
#ifdef NRFXLIB_SDC

static bool s_MpslUp = false;

static int MpslStart(void)
{
	if (s_MpslUp)
	{
		return 0;
	}

	int res = BtPdsMpslInit();
	if (res != 0)
	{
		return res;
	}

	s_MpslUp = true;

	return 0;
}

typedef struct {
	uintptr_t		Addr;
	const uint32_t	*pSrc;
	uint32_t		Words;
} MpslWrCtx_t;

static uint32_t MpslWriteStep(void *pv)
{
	MpslWrCtx_t *c = (MpslWrCtx_t *)pv;

	NvmcProgramWords(c->Addr, c->pSrc, c->Words);

	return 0;					// done in one step
}

typedef struct {
	uintptr_t	Addr;
	bool		Started;
} MpslErCtx_t;

static uint32_t MpslEraseStep(void *pv)
{
	MpslErCtx_t *c = (MpslErCtx_t *)pv;

	if (c->Started == false)
	{
		if (NvmcPageIsErased(c->Addr, NRF_FICR->CODEPAGESIZE))
		{
			s_Stat.Skipped++;
			return 0;
		}

		// Start it and hand the time back; the controller runs on while the
		// radio takes its slots.
		NvmcEraseBegin(c->Addr);
		c->Started = true;

		return NVM_MCU_NRF52_ERASE_SLICE_MS * 1000UL;
	}

	if (NvmcIsReady())
	{
		NvmcRelease();
		return 0;
	}

	return NVM_MCU_NRF52_ERASE_SLICE_MS * 1000UL;
}

#endif	// NRFXLIB_SDC

// ---------------------------------------------------------------------------
// The one operation set.
// ---------------------------------------------------------------------------

static int Nrf52Program(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt,
						void *pCtx)
{
	(void)pCtx;

	if ((Addr & (NVM_MCU_NRF52_WRITE_GRAN - 1)) != 0)
	{
		return -EINVAL;
	}
	if (WordCnt == 0)
	{
		return 0;
	}

#ifdef SOFTDEVICE_PRESENT
	if (SdIsEnabled())
	{
		SdWrArg_t arg = { (uint32_t *)Addr, pSrc, WordCnt };

		int res = SdRun(SdWriteSubmit, &arg);
		if (res == 0)
		{
			s_Stat.Ops++;
		}
		return res;
	}
	// Present but stopped: nothing arbitrates, so drive the controller.
#endif

#ifdef NRFXLIB_SDC
	int res = MpslStart();
	if (res != 0)
	{
		return res;
	}

	MpslWrCtx_t ctx = { Addr, pSrc, WordCnt };
	BtPdsMpslOp_t op;

	op.Step = MpslWriteStep;
	op.StepBudgetUs = NVM_MCU_NRF52_STEP_BUDGET_US;
	op.pCtx = &ctx;

	res = BtPdsMpslRun(&op);
	if (res == 0)
	{
		s_Stat.Ops++;
	}
	return res;
#else
	NvmcProgramWords(Addr, pSrc, WordCnt);
	s_Stat.Ops++;

	return 0;
#endif
}

static int Nrf52ErasePage(uintptr_t Addr, void *pCtx)
{
	(void)pCtx;

	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;

	if (pagesize == 0)
	{
		return -EINVAL;
	}

#ifdef SOFTDEVICE_PRESENT
	if (SdIsEnabled())
	{
		uint32_t page = (uint32_t)(Addr / pagesize);

		int res = SdRun(SdEraseSubmit, &page);
		if (res == 0)
		{
			s_Stat.Ops++;
		}
		return res;
	}
#endif

#ifdef NRFXLIB_SDC
	int res = MpslStart();
	if (res != 0)
	{
		return res;
	}

	MpslErCtx_t ctx = { Addr, false };
	BtPdsMpslOp_t op;

	op.Step = MpslEraseStep;
	op.StepBudgetUs = NVM_MCU_NRF52_STEP_BUDGET_US;
	op.pCtx = &ctx;

	res = BtPdsMpslRun(&op);
	if (res == 0)
	{
		s_Stat.Ops++;
	}
	return res;
#else
	if (NvmcPageIsErased(Addr, pagesize))
	{
		s_Stat.Skipped++;
		return 0;
	}

	NvmcEraseBegin(Addr);

	uint32_t elapsed = 0;
	while (!NvmcIsReady())
	{
		// A page erase is tens of msec and stalls instruction fetch. Let the
		// application spend the wait however it needs to.
		if (WaitStep() == false || ++elapsed >= s_TimeoutMs)
		{
			NvmcRelease();
			return -ETIMEDOUT;
		}
	}

	NvmcRelease();
	s_Stat.Ops++;

	return 0;
#endif
}

// Region write protect is left out. The nRF52 block protect only clears on
// reset, which cannot implement unprotect, so NvmMcu reports it unsupported.
static const NvmMcuOp_t s_NvmMcuNrf52Op = {
	.Program = Nrf52Program,
	.ErasePage = Nrf52ErasePage,
	.SetProtect = nullptr,
	.MaxProgWords = NVM_MCU_NRF52_MAX_PROG_WORDS,
	.pCtx = nullptr
};

int NvmMcuNrf52Init(void)
{
#ifdef NRFXLIB_SDC
	return MpslStart();
#else
	return 0;
#endif
}

void NvmMcuNrf52SetWait(NvmMcuNrf52Wait_t pWait, uint32_t TimeoutMs)
{
	s_pWait = pWait;

	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

void NvmMcuNrf52GetStat(NvmMcuNrf52Stat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}

const NvmMcuOp_t & NvmMcuNrf52Op(void)
{
	return s_NvmMcuNrf52Op;
}

void NvmMcuNrf52Cfg(NvmCfg_t &Cfg)
{
	// Geometry from the device itself rather than a build time constant.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.BaseAddr = 0;
	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
	Cfg.WriteGran = NVM_MCU_NRF52_WRITE_GRAN;
}
