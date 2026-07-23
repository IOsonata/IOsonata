/**-------------------------------------------------------------------------
@file	nvm_mcu_nrf54.cpp

@brief	NvmMcu operations for the nRF54L internal RRAM.

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
#include <string.h>

#include "hal/nrf_rramc.h"
#include "idelay.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#endif

#ifdef NRFXLIB_SDC
#include "bt_pds_sdc.h"
#endif

#include "nvm_mcu_nrf54.h"

// The controller writes 32 bit words.
#define NVM_MCU_NRF54_WRITE_GRAN	4

// The unit an erase covers. RRAM has no erase command, so the operation below
// writes the erased pattern over it, which is what erase means on this medium.
#ifndef NVM_MCU_NRF54_ERASE_SIZE
#define NVM_MCU_NRF54_ERASE_SIZE	0x00000400UL	// 1 KB
#endif

// Total RRAM. It cannot be read from the device the way the nRF52 page size
// comes from FICR, so override this for a part other than the nRF54L15.
#ifndef NVM_MCU_NRF54_TOTAL_SIZE
#define NVM_MCU_NRF54_TOTAL_SIZE	0x0017D000UL	// 1524 KB
#endif

// Words one request may take. A word write is microseconds, so this only has
// to keep a timeslot step short. NvmMcu splits a longer write to match.
#ifndef NVM_MCU_NRF54_MAX_PROG_WORDS
#define NVM_MCU_NRF54_MAX_PROG_WORDS	32
#endif

#ifndef NVM_MCU_NRF54_STEP_BUDGET_US
#define NVM_MCU_NRF54_STEP_BUDGET_US	2000
#endif

#ifndef NVM_MCU_NRF54_TIMEOUT_MS
#define NVM_MCU_NRF54_TIMEOUT_MS	5000
#endif

static NvmMcuNrf54Wait_t s_pWait = nullptr;
static uint32_t s_TimeoutMs = NVM_MCU_NRF54_TIMEOUT_MS;
static NvmMcuNrf54Stat_t s_Stat;

// ---------------------------------------------------------------------------
// The controller mechanics. One copy, used by every path below.
// ---------------------------------------------------------------------------

static void RramcWriteEnable(bool bEnable)
{
	nrf_rramc_config_t cfg;

	nrf_rramc_config_get(NRF_RRAMC, &cfg);
	cfg.mode_write = bEnable;
	nrf_rramc_config_set(NRF_RRAMC, &cfg);
}

static void RramcWriteWords(uintptr_t Addr, const uint32_t *pSrc,
							uint32_t WordCnt)
{
	volatile uint32_t *pDst = (volatile uint32_t *)Addr;

	RramcWriteEnable(true);

	for (uint32_t i = 0; i < WordCnt; i++)
	{
		pDst[i] = pSrc[i];
		while (!nrf_rramc_write_ready_check(NRF_RRAMC))
		{
			// Let the write buffer take the word before the next one.
		}
	}

	// The buffer has to be drained before the data is committed.
	while (!nrf_rramc_empty_buffer_check(NRF_RRAMC))
	{
	}

	RramcWriteEnable(false);
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

void NvmMcuNrf54SocEvt(uint32_t SysEvt)
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

// Submit, retry while the radio holds the memory, then wait for the result.
static int SdWrite(uint32_t *pDst, const uint32_t *pSrc, uint32_t WordCnt)
{
	uint32_t elapsed = 0;

	while (true)
	{
		s_OpDone = false;
		s_OpOk = false;
		__DMB();

		uint32_t status = sd_flash_write(pDst, pSrc, WordCnt);

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

#else

void NvmMcuNrf54SocEvt(uint32_t SysEvt)
{
	(void)SysEvt;
}

#endif	// SOFTDEVICE_PRESENT

// ---------------------------------------------------------------------------
// Timeslot path. The controller is ours, but the radio needs its slots, so the
// same mechanics run inside a timeslot. An RRAM write is short enough to fit.
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

	RramcWriteWords(c->Addr, c->pSrc, c->Words);

	return 0;					// done in one step
}

#endif	// NRFXLIB_SDC

// ---------------------------------------------------------------------------
// The one operation set.
// ---------------------------------------------------------------------------

static int Nrf54Program(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt,
						void *pCtx)
{
	(void)pCtx;

	if ((Addr & (NVM_MCU_NRF54_WRITE_GRAN - 1)) != 0)
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
		int res = SdWrite((uint32_t *)Addr, pSrc, WordCnt);
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
	op.StepBudgetUs = NVM_MCU_NRF54_STEP_BUDGET_US;
	op.pCtx = &ctx;

	res = BtPdsMpslRun(&op);
	if (res == 0)
	{
		s_Stat.Ops++;
	}
	return res;
#else
	RramcWriteWords(Addr, pSrc, WordCnt);
	s_Stat.Ops++;

	return 0;
#endif
}

static int Nrf54ErasePage(uintptr_t Addr, void *pCtx)
{
	uint32_t ones[NVM_MCU_NRF54_MAX_PROG_WORDS];

	memset(ones, 0xFF, sizeof(ones));

	uint32_t words = NVM_MCU_NRF54_ERASE_SIZE / NVM_MCU_NRF54_WRITE_GRAN;

	while (words > 0)
	{
		uint32_t n = words < NVM_MCU_NRF54_MAX_PROG_WORDS ?
					 words : NVM_MCU_NRF54_MAX_PROG_WORDS;

		int res = Nrf54Program(Addr, ones, n, pCtx);
		if (res < 0)
		{
			return res;
		}
		Addr += n * NVM_MCU_NRF54_WRITE_GRAN;
		words -= n;
	}

	return 0;
}

// Region write protect is left out for now. The RRAM controller has region
// protection, but it is set once and does not clear until reset, which cannot
// implement unprotect, so NvmMcu reports it unsupported.
static const NvmMcuOp_t s_NvmMcuNrf54Op = {
	.Program = Nrf54Program,
	.ErasePage = Nrf54ErasePage,
	.SetProtect = nullptr,
	.MaxProgWords = NVM_MCU_NRF54_MAX_PROG_WORDS,
	.pCtx = nullptr
};

int NvmMcuNrf54Init(void)
{
#ifdef NRFXLIB_SDC
	return MpslStart();
#else
	return 0;
#endif
}

void NvmMcuNrf54SetWait(NvmMcuNrf54Wait_t pWait, uint32_t TimeoutMs)
{
	s_pWait = pWait;

	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

void NvmMcuNrf54GetStat(NvmMcuNrf54Stat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}

const NvmMcuOp_t & NvmMcuNrf54Op(void)
{
	return s_NvmMcuNrf54Op;
}

void NvmMcuNrf54Cfg(NvmCfg_t &Cfg)
{
	Cfg.BaseAddr = 0;
	Cfg.TotalSize = NVM_MCU_NRF54_TOTAL_SIZE;
	Cfg.EraseSize = NVM_MCU_NRF54_ERASE_SIZE;
	Cfg.WriteGran = NVM_MCU_NRF54_WRITE_GRAN;
}
