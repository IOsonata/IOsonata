/**-------------------------------------------------------------------------
@file	nvm_nrfx.cpp

@brief	The nRF on die memory controller as a DeviceIntrf.

		The frame, the read and the arbitration are the same whatever the
		memory is made of. Only the controller access and what erase means on
		it follow from the MCU model.

@author	Hoang Nguyen Hoan
@date	July 24, 2026

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
#include <string.h>
#include <errno.h>

#include "idelay.h"

#if defined(NRF52_SERIES)
#include "hal/nrf_nvmc.h"
#elif defined(NRF54L_SERIES) || defined(NRF54L15_XXAA)
#include "hal/nrf_rramc.h"
#else
#error "nvm_nrfx: unsupported MCU model"
#endif

// A SoftDevice build does not always define SOFTDEVICE_PRESENT; a bare metal
// SDK project may name the SoftDevice alone. Getting this wrong is not a
// compile error, it drives the controller directly while the SoftDevice owns
// the memory, so the test is made once here and used everywhere below.
#if defined(SOFTDEVICE_PRESENT) || defined(S112) || defined(S113) || \
	defined(S132) || defined(S140) || defined(S145)
#define NVM_INTRF_SOFTDEVICE		1
#elif defined(NRF54L_SERIES) || defined(NRF54L15_XXAA)
// The nRF54L library is built with no SoftDevice define at all, because one
// archive serves SoftDevice, link controller and plain bare metal
// applications alike. A compile time test can therefore never be right here:
// it compiled this path out and the demo drove the controller directly under
// a live S145. The path is compiled in instead and the choice is made at run
// time through NvmIntrfSdRunning(): weak below, answering false, overridden
// in nrf_sdh.c, which only an application that enables a SoftDevice pulls
// from the archive.
#define NVM_INTRF_SOFTDEVICE		1
#define NVM_INTRF_SD_RUNTIME		1
#endif

// The same reasoning holds for the link controller: the nRF54L library does
// not define NRFXLIB_SDC either, so the timeslot path is compiled in and the
// choice is made at run time on a weak reference to the MPSL wrapper. An
// application with the link controller in it defines the wrapper through its
// own chain; one without links nothing extra, the reference resolves null,
// and the operation goes to the controller. The flash arbitration therefore
// lives entirely here; the timeslot mechanics stay MPSL's, in bt_pds_sdc.
#if !defined(NRFXLIB_SDC) && (defined(NRF54L_SERIES) || defined(NRF54L15_XXAA))
#define NVM_INTRF_SLOT_RUNTIME		1
#endif

#ifdef NVM_INTRF_SOFTDEVICE
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#if defined(NRF52_SERIES)
#include "nrf_sdh_soc.h"
#else
#include "bm/softdevice_handler/nrf_sdh_soc.h"
#endif
#endif

#if defined(NRFXLIB_SDC) || defined(NVM_INTRF_SLOT_RUNTIME)
#include "bt_pds_sdc.h"
#endif

#include "storage/nvm_intrf.h"

// The command set, using the opcodes a serial flash uses so a config for
// internal memory reads like one for a flash chip.
#define NVM_INTRF_CMD_READ		0x03		//!< Read
#define NVM_INTRF_CMD_WRITE		0x02		//!< Program words
#define NVM_INTRF_CMD_ERASE		0x20		//!< Erase or clear one unit

// Address bytes on the frame. Internal memory is 32 bit addressed.
#define NVM_INTRF_ADDR_SIZE		4

// The controller writes 32 bit words.
#define NVM_INTRF_WRITE_GRAN	4

// ---------------------------------------------------------------------------
// What follows from the MCU model
// ---------------------------------------------------------------------------
#if defined(NRF52_SERIES)

// Largest bytes one transfer may take. A word write is a few hundred usec and
// on a timeslot build the whole transfer has to fit one slot, which is the
// tightest of the three cases. The Nvm driver splits at this.
#ifndef NVM_INTRF_MAX_XFER
#define NVM_INTRF_MAX_XFER			32
#endif

// Erase slice per timeslot, in msec, and the worst case for one step.
#ifndef NVM_INTRF_ERASE_SLICE_MS
#define NVM_INTRF_ERASE_SLICE_MS	2
#endif

#else	// nRF54L

// An RRAM word write is microseconds, so a transfer only has to keep a
// timeslot step short.
#ifndef NVM_INTRF_MAX_XFER
#define NVM_INTRF_MAX_XFER			64
#endif

// What an erase covers. RRAM has no erase command, so an erase writes the
// erased pattern over this much, which is what erase means on this medium.
#ifndef NVM_INTRF_ERASE_SIZE
#define NVM_INTRF_ERASE_SIZE		0x00000400UL	// 1 KB
#endif

// Total RRAM. It cannot be read from the device the way the nRF52 page size
// comes from FICR, so override this for a part other than the nRF54L15.
#ifndef NVM_INTRF_TOTAL_SIZE
#define NVM_INTRF_TOTAL_SIZE		0x0017D000UL	// 1524 KB
#endif

#ifndef NVM_INTRF_ERASE_SLICE_MS
#define NVM_INTRF_ERASE_SLICE_MS	2
#endif

#endif

#ifndef NVM_INTRF_STEP_BUDGET_US
#define NVM_INTRF_STEP_BUDGET_US	((NVM_INTRF_ERASE_SLICE_MS + 1) * 1000UL)
#endif

#ifndef NVM_INTRF_TIMEOUT_MS
#define NVM_INTRF_TIMEOUT_MS		5000
#endif

// The command and address are collected apart from the data, so the data
// buffer stays word aligned. Reading it as words from an odd frame offset
// would be an unaligned access, and the SoftDevice rejects an unaligned source
// outright.
#define NVM_INTRF_FRAME_HDR			(1 + NVM_INTRF_ADDR_SIZE)

static uint8_t s_Hdr[NVM_INTRF_FRAME_HDR];
static int s_HdrLen;
alignas(4) static uint8_t s_Data[NVM_INTRF_MAX_XFER];
static int s_DataLen;

static NvmIntrfWait_t s_pWait = nullptr;
static uint32_t s_TimeoutMs = NVM_INTRF_TIMEOUT_MS;
static NvmIntrfStat_t s_Stat;

static int NvmSubmit(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt);

// ---------------------------------------------------------------------------
// The controller. One of these two, chosen by the MCU model.
// ---------------------------------------------------------------------------
#if defined(NRF52_SERIES)

static void CtrlWriteWords(uintptr_t Addr, const uint32_t *pSrc,
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

static void CtrlEraseBegin(uintptr_t Addr)
{
	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_ERASE);
	nrf_nvmc_page_erase_start(NRF_NVMC, (uint32_t)Addr);
}

static bool CtrlIsReady(void)
{
	return nrf_nvmc_ready_check(NRF_NVMC);
}

static void CtrlRelease(void)
{
	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
}

static uint32_t CtrlEraseSize(void)
{
	return NRF_FICR->CODEPAGESIZE;
}

#else	// nRF54L

static void RramcWriteEnable(bool bEnable)
{
	nrf_rramc_config_t cfg;

	nrf_rramc_config_get(NRF_RRAMC, &cfg);
	cfg.mode_write = bEnable;
	nrf_rramc_config_set(NRF_RRAMC, &cfg);
}

static void CtrlWriteWords(uintptr_t Addr, const uint32_t *pSrc,
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

	// The buffer has to drain before the data is committed.
	while (!nrf_rramc_empty_buffer_check(NRF_RRAMC))
	{
	}

	RramcWriteEnable(false);
}

static uint32_t CtrlEraseSize(void)
{
	return NVM_INTRF_ERASE_SIZE;
}

#endif

// True when the unit already reads erased, so it can be left alone. Saves the
// work and the wear that goes with it.
static bool CtrlIsErased(uintptr_t Addr, uint32_t Len)
{
	const uint32_t *p = (const uint32_t *)Addr;
	uint32_t words = Len / 4;

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
// SoftDevice path. The memory is not ours to touch while one runs, so the work
// is submitted and the result arrives as a SoC event.
// ---------------------------------------------------------------------------
#ifdef NVM_INTRF_SOFTDEVICE

static volatile bool s_OpDone;
static volatile bool s_OpOk;
// Set while a request of ours is outstanding. The SoC flash event is broadcast
// to every observer and has nothing identifying in it, so this is how the
// handler knows whether the event it was handed belongs to it.
static volatile bool s_OpPending;

#ifdef NVM_INTRF_SD_RUNTIME

// Weak: an application with no SoftDevice support linked runs on this answer
// and drives the controller itself. nrf_sdh.c has the real answer.
extern "C" __attribute__((weak)) bool NvmIntrfSdRunning(void)
{
	return false;
}

static bool SdRunning(void)
{
	return NvmIntrfSdRunning();
}

#else

// One library serves SoftDevice and link controller applications, so the
// SoftDevice is not always in the flash. Its info structure magic says
// whether it is; calling into one that is not there faults.
static bool SdPresent(void)
{
#if defined(SD_MAGIC_NUMBER) && defined(MBR_SIZE)
	return SD_MAGIC_NUMBER_GET(MBR_SIZE) == SD_MAGIC_NUMBER;
#else
	return true;
#endif
}

static bool SdRunning(void)
{
	uint8_t en = 0;

	if (SdPresent() == false)
	{
		return false;
	}

	if (sd_softdevice_is_enabled(&en) != NRF_SUCCESS)
	{
		return false;
	}

	return en != 0;
}

#endif	// NVM_INTRF_SD_RUNTIME

static void NvmIntrfSocEvt(uint32_t SysEvt)
{
	if (s_OpPending == false)
	{
		// Every observer sees every event; this one is not ours.
		return;
	}

	switch (SysEvt)
	{
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			s_OpOk = true;
			s_OpPending = false;
			s_OpDone = true;
			s_Stat.Evt++;
			break;

		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_OpOk = false;
			s_OpPending = false;
			s_OpDone = true;
			s_Stat.Evt++;
			break;

		default:
			break;
	}
}

// The SoC flash events are broadcast to every observer, so the interface
// registers its own and the application has nothing to do. NvmIntrfSocEvt
// stays public for an application with its own event dispatch.
static void NvmIntrfSocObserver(uint32_t SysEvt, void *pCtx)
{
	(void)pCtx;

	NvmIntrfSocEvt(SysEvt);
}

#if defined(NRF52_SERIES)
NRF_SDH_SOC_OBSERVER(s_NvmIntrfSocObs, 0, NvmIntrfSocObserver, NULL);
#else
// The bare metal SDK orders the arguments differently and takes a symbolic
// priority level rather than a number.
NRF_SDH_SOC_OBSERVER(s_NvmIntrfSocObs, NvmIntrfSocObserver, NULL, HIGH);
#endif

typedef uint32_t (*SdSubmit_t)(void *pArg);

// Submit, retry while the radio holds the memory, then wait for the result.
static int SdRun(SdSubmit_t Submit, void *pArg)
{
	uint32_t elapsed = 0;

	while (true)
	{
		s_OpDone = false;
		s_OpOk = false;
		s_OpPending = true;
		__DMB();

		uint32_t status = Submit(pArg);

		if (status == NRF_SUCCESS)
		{
			break;
		}

		s_OpPending = false;

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
			s_OpPending = false;
			return -ETIMEDOUT;
		}
		elapsed++;
	}

	s_OpPending = false;

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

#if defined(NRF52_SERIES)
static uint32_t SdEraseSubmit(void *pArg)
{
	// The SoftDevice erase takes a page number, not an address.
	return sd_flash_page_erase(*(uint32_t *)pArg);
}
#endif

#else

static bool SdRunning(void) { return false; }

#endif	// NVM_INTRF_SOFTDEVICE

// ---------------------------------------------------------------------------
// Timeslot path. The controller is ours, but the radio needs its slots.
// ---------------------------------------------------------------------------
#if defined(NRFXLIB_SDC) || defined(NVM_INTRF_SLOT_RUNTIME)

typedef struct {
	uintptr_t		Addr;
	const uint32_t	*pSrc;
	uint32_t		Words;
} SlotWrCtx_t;

static uint32_t SlotWriteStep(void *pv)
{
	SlotWrCtx_t *c = (SlotWrCtx_t *)pv;

	CtrlWriteWords(c->Addr, c->pSrc, c->Words);

	return 0;					// done in one step
}

#if defined(NRF52_SERIES)
typedef struct {
	uintptr_t	Addr;
	bool		Started;
} SlotErCtx_t;

// A page erase is far longer than a timeslot, so it is started in one and
// polled in later ones while the radio keeps its slots.
static uint32_t SlotEraseStep(void *pv)
{
	SlotErCtx_t *c = (SlotErCtx_t *)pv;

	if (c->Started == false)
	{
		if (CtrlIsErased(c->Addr, CtrlEraseSize()))
		{
			s_Stat.Skipped++;
			return 0;
		}

		CtrlEraseBegin(c->Addr);
		c->Started = true;

		return NVM_INTRF_ERASE_SLICE_MS * 1000UL;
	}

	if (CtrlIsReady())
	{
		CtrlRelease();
		return 0;
	}

	return NVM_INTRF_ERASE_SLICE_MS * 1000UL;
}
#endif

#endif	// NRFXLIB_SDC || NVM_INTRF_SLOT_RUNTIME

#ifdef NRFXLIB_SDC

static bool s_SlotUp = false;

static int SlotStart(void)
{
	if (s_SlotUp)
	{
		return 0;
	}

	int res = BtPdsMpslInit();
	if (res != 0)
	{
		return res;
	}

	s_SlotUp = true;

	return 0;
}

#else

static int SlotStart(void) { return 0; }

#endif	// NRFXLIB_SDC

#ifdef NVM_INTRF_SLOT_RUNTIME

// Weak references to the MPSL wrapper. A weak undefined reference extracts
// nothing from the archive: with no link controller in the application both
// resolve null and the operation goes straight at the controller.
extern "C" __attribute__((weak)) int BtPdsMpslInit(void);
extern "C" __attribute__((weak)) int BtPdsMpslRun(BtPdsMpslOp_t *pOp);

#endif	// NVM_INTRF_SLOT_RUNTIME

// ---------------------------------------------------------------------------
// The one place that knows which of the three cases we are in.
// ---------------------------------------------------------------------------

static int NvmSubmit(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt)
{
	if (WordCnt == 0)
	{
		return 0;
	}

#ifdef NVM_INTRF_SOFTDEVICE
#ifdef NVM_INTRF_SD_RUNTIME
	if (SdRunning())
	{
		SdWrArg_t arg = { (uint32_t *)Addr, pSrc, WordCnt };

		return SdRun(SdWriteSubmit, &arg);
	}
	// Not running, or no SoftDevice support linked: fall through.
#else
	if (SdPresent())
	{
		if (SdRunning())
		{
			SdWrArg_t arg = { (uint32_t *)Addr, pSrc, WordCnt };

			return SdRun(SdWriteSubmit, &arg);
		}

		// Present but stopped: neither the stack nor the radio runs, so
		// drive the controller.
		CtrlWriteWords(Addr, pSrc, WordCnt);

		return 0;
	}
	// No SoftDevice in the flash: a link controller build arbitrates below.
#endif
#endif

#if defined(NRFXLIB_SDC)
	int res = SlotStart();
	if (res != 0)
	{
		return res;
	}

	SlotWrCtx_t ctx = { Addr, pSrc, WordCnt };
	BtPdsMpslOp_t op;

	op.Step = SlotWriteStep;
	op.StepBudgetUs = NVM_INTRF_STEP_BUDGET_US;
	op.pCtx = &ctx;

	return BtPdsMpslRun(&op);
#elif defined(NVM_INTRF_SLOT_RUNTIME)
	if (BtPdsMpslRun != nullptr && BtPdsMpslInit != nullptr)
	{
		int res = BtPdsMpslInit();
		if (res != 0)
		{
			return res;
		}

		SlotWrCtx_t ctx = { Addr, pSrc, WordCnt };
		BtPdsMpslOp_t op;

		op.Step = SlotWriteStep;
		op.StepBudgetUs = NVM_INTRF_STEP_BUDGET_US;
		op.pCtx = &ctx;

		return BtPdsMpslRun(&op);
	}
	// No link controller in this application: the memory is ours.
	CtrlWriteWords(Addr, pSrc, WordCnt);

	return 0;
#else
	CtrlWriteWords(Addr, pSrc, WordCnt);

	return 0;
#endif
}

#if defined(NRF52_SERIES)
// Erase with the memory all ours: start the page erase and wait it out. The
// wait is handed to the application, since the erase stalls instruction fetch.
static int NvmEraseBare(uintptr_t Addr, uint32_t page)
{
	(void)page;

	if (CtrlIsErased(Addr, CtrlEraseSize()))
	{
		s_Stat.Skipped++;
		return 0;
	}

	CtrlEraseBegin(Addr);

	uint32_t elapsed = 0;
	while (!CtrlIsReady())
	{
		// A page erase is tens of msec and stalls instruction fetch. Let the
		// application spend the wait however it needs to.
		if (WaitStep() == false || ++elapsed >= s_TimeoutMs)
		{
			CtrlRelease();
			return -ETIMEDOUT;
		}
	}

	CtrlRelease();

	return 0;
}
#endif

// What erase means differs with the memory, so this is the second thing the
// MCU model decides.
static int NvmEraseUnit(uintptr_t Addr)
{
#if defined(NRF52_SERIES)
	uint32_t page = CtrlEraseSize();

	if (page == 0)
	{
		return -EINVAL;
	}

#ifdef NVM_INTRF_SOFTDEVICE
	if (SdPresent())
	{
		if (SdRunning())
		{
			uint32_t no = (uint32_t)(Addr / page);

			return SdRun(SdEraseSubmit, &no);
		}

		// Present but stopped: neither the stack nor the radio runs, so
		// erase in place.
		return NvmEraseBare(Addr, page);
	}
	// No SoftDevice in the flash: a link controller build arbitrates below.
#endif

#ifdef NRFXLIB_SDC
	int res = SlotStart();
	if (res != 0)
	{
		return res;
	}

	SlotErCtx_t ctx = { Addr, false };
	BtPdsMpslOp_t op;

	op.Step = SlotEraseStep;
	op.StepBudgetUs = NVM_INTRF_STEP_BUDGET_US;
	op.pCtx = &ctx;

	return BtPdsMpslRun(&op);
#else
	return NvmEraseBare(Addr, page);
#endif

#else	// nRF54L

	// RRAM has no erase command and rewrites in place, so the erased pattern
	// is written over the unit. It goes the same way as any other write.
	alignas(4) static uint8_t ones[NVM_INTRF_MAX_XFER];
	uint32_t left = CtrlEraseSize();

	if (CtrlIsErased(Addr, left))
	{
		s_Stat.Skipped++;
		return 0;
	}

	memset(ones, 0xFF, sizeof(ones));

	while (left > 0)
	{
		uint32_t n = left < sizeof(ones) ? left : (uint32_t)sizeof(ones);

		int res = NvmSubmit(Addr, (const uint32_t *)ones,
							n / NVM_INTRF_WRITE_GRAN);
		if (res != 0)
		{
			return res;
		}
		Addr += n;
		left -= n;
	}

	return 0;
#endif
}

// ---------------------------------------------------------------------------
// The interface
// ---------------------------------------------------------------------------

static uintptr_t FrameAddr(void)
{
	uintptr_t a = 0;

	for (int i = 0; i < NVM_INTRF_ADDR_SIZE; i++)
	{
		a = (a << 8) | s_Hdr[1 + i];
	}

	return a;
}

// Put whatever whole words have been collected into the memory.
static int NvmFlush(void)
{
	if (s_DataLen < (int)NVM_INTRF_WRITE_GRAN)
	{
		return 0;
	}

	uint32_t words = (uint32_t)s_DataLen / NVM_INTRF_WRITE_GRAN;
	int res = NvmSubmit(FrameAddr(), (const uint32_t *)s_Data, words);

	if (res == 0)
	{
		s_Stat.Ops++;
	}

	s_DataLen = 0;

	return res;
}

extern "C" {

static bool NvmStartRx(DevIntrf_t *, uint32_t)
{
	// A restart after the command and address were sent, so keep the frame.
	return true;
}

static bool NvmStartTx(DevIntrf_t *, uint32_t)
{
	s_HdrLen = 0;
	s_DataLen = 0;

	return true;
}

// The work is done here rather than at the end of the transfer, because this
// is the only place a failure can be reported: returning short tells the
// driver the memory did not take the data.
static int NvmTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	int n = 0;

	while (n < Len && s_HdrLen < NVM_INTRF_FRAME_HDR)
	{
		s_Hdr[s_HdrLen++] = pData[n++];
	}

	if (s_HdrLen < NVM_INTRF_FRAME_HDR)
	{
		return n;
	}

	// An erase needs the address only.
	if (s_Hdr[0] == NVM_INTRF_CMD_ERASE)
	{
		if (NvmEraseUnit(FrameAddr()) != 0)
		{
			return 0;
		}
		s_Stat.Ops++;
		return Len;
	}

	if (s_Hdr[0] != NVM_INTRF_CMD_WRITE)
	{
		return Len;				// a read frame, served by RxData
	}

	while (n < Len)
	{
		s_Data[s_DataLen++] = pData[n++];

		if (s_DataLen == (int)sizeof(s_Data))
		{
			if (NvmFlush() != 0)
			{
				return n - (int)sizeof(s_Data);
			}
		}
	}

	// The driver bounds a transfer to the page size, so the whole frame
	// arrives together and this is where it completes.
	if (NvmFlush() != 0)
	{
		return 0;
	}

	return n;
}

static int NvmTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	return NvmTxData(pDev, pData, Len);
}

// The memory is mapped, so a read is a copy from the address in the frame.
static int NvmRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_HdrLen < NVM_INTRF_FRAME_HDR || s_Hdr[0] != NVM_INTRF_CMD_READ)
	{
		return 0;
	}

	memcpy(pBuff, (const void *)FrameAddr(), (size_t)Len);

	return Len;
}

static void NvmStopRx(DevIntrf_t *)
{
	s_HdrLen = 0;
}

// Everything was done in TxData, where a failure could be reported.
static void NvmStopTx(DevIntrf_t *)
{
	s_HdrLen = 0;
	s_DataLen = 0;
}

static void NvmIntrfDisable(DevIntrf_t *) {}
static void NvmIntrfEnable(DevIntrf_t *) {}
static uint32_t NvmGetRate(DevIntrf_t *) { return 0; }
static uint32_t NvmSetRate(DevIntrf_t *, uint32_t Rate) { return Rate; }
static void NvmPowerOff(DevIntrf_t *) {}
static void *NvmGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

}	// extern "C"

bool NvmIntrf::Init(void)
{
	memset(&vDevIntrf, 0, sizeof(vDevIntrf));
	memset(&s_Stat, 0, sizeof(s_Stat));

	vDevIntrf.pDevData = this;
	vDevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	vDevIntrf.Disable = NvmIntrfDisable;
	vDevIntrf.Enable = NvmIntrfEnable;
	vDevIntrf.GetRate = NvmGetRate;
	vDevIntrf.SetRate = NvmSetRate;
	vDevIntrf.StartRx = NvmStartRx;
	vDevIntrf.RxData = NvmRxData;
	vDevIntrf.StopRx = NvmStopRx;
	vDevIntrf.StartTx = NvmStartTx;
	vDevIntrf.TxData = NvmTxData;
	vDevIntrf.TxSrData = NvmTxSrData;
	vDevIntrf.StopTx = NvmStopTx;
	vDevIntrf.PowerOff = NvmPowerOff;
	vDevIntrf.GetHandle = NvmGetHandle;
	vDevIntrf.MaxRetry = 1;
	vDevIntrf.MaxTrxLen = NVM_INTRF_FRAME_HDR + NVM_INTRF_MAX_XFER;
	vDevIntrf.EnCnt = 1;
	atomic_flag_clear(&vDevIntrf.bBusy);

	s_HdrLen = 0;
	s_DataLen = 0;

#if defined(NRFXLIB_SDC) && defined(NVM_INTRF_SOFTDEVICE) && !defined(NVM_INTRF_SD_RUNTIME)
	if (SdPresent())
	{
		// The SoftDevice arbitrates; the timeslot session is not needed and
		// MPSL must not be brought up beside it.
		return true;
	}
#endif

	return SlotStart() == 0;
}

void NvmIntrfSetWait(NvmIntrfWait_t pWait, uint32_t TimeoutMs)
{
	s_pWait = pWait;

	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

void NvmIntrfGetStat(NvmIntrfStat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}

void NvmIntrfCfg(NvmCfg_t &Cfg)
{
	Cfg.DevNo = 0;
#if defined(NRF52_SERIES)
	// Geometry from the device itself.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
#else
	Cfg.TotalSize = NVM_INTRF_TOTAL_SIZE;
	Cfg.EraseSize = NVM_INTRF_ERASE_SIZE;
#endif
	Cfg.PageSize = NVM_INTRF_MAX_XFER;		// largest bytes per transfer
	Cfg.WriteGran = NVM_INTRF_WRITE_GRAN;
	Cfg.AddrSize = NVM_INTRF_ADDR_SIZE;
	Cfg.RdCmd = { NVM_INTRF_CMD_READ, 0 };
	Cfg.WrCmd = { NVM_INTRF_CMD_WRITE, 0 };
	Cfg.EraseCmd = { NVM_INTRF_CMD_ERASE, 0 };
	Cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

// Where the memory the application owns ends. The default is the whole
// device. Weak: an S145 application overrides it from nrf_sdh.c, because the
// SoftDevice image sits at the top of the RRAM there and the application
// slot ends at the storage partition.
extern "C" __attribute__((weak)) uint64_t NvmIntrfCeiling(void)
{
#if defined(NRF52_SERIES)
	return (uint64_t)NRF_FICR->CODEPAGESIZE * (uint64_t)NRF_FICR->CODESIZE;
#else
	return NVM_INTRF_TOTAL_SIZE;
#endif
}
