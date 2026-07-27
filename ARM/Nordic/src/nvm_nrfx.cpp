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
// lives entirely here; who may touch the memory is registered by the stack.
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

// NvmSubmit and NvmEraseUnit answer this when the operation was started and
// the flash event will report it. Positive, so the existing checks for a
// negative errno keep working unchanged.
#define NVM_INTRF_STARTED			1

// The staging buffer is declared in the header, where the class is, so the
// per target transfer size cannot exceed it.
static_assert(NVM_INTRF_MAX_XFER <= NVM_INTRF_XFER_SIZE,
			  "NVM_INTRF_MAX_XFER exceeds the staging buffer");

// How long the non interrupt path spins on the flash event flag. A bound so a
// lost event cannot hang; not a timeout the application configures, because
// that is the memory's business and lives in Nvm.
#ifndef NVM_INTRF_SD_SPIN
#define NVM_INTRF_SD_SPIN			0x02000000UL
#endif

//---------------------------------------------------------------------------
// Per controller state. There is one memory controller and one stack holding
// it, so these describe the hardware rather than a transaction.
//---------------------------------------------------------------------------

// Who decides when the memory may be touched. Registered by the stack that
// owns the radio; null means the memory is the application's alone.
static NvmIntrfArb_t s_pArbiter = nullptr;

static NvmIntrfStat_t s_Stat;

static int NvmSubmit(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt);

// The instance a transfer is running on. Set by StartTx and StartRx, read by
// the flash event, which arrives with no interface argument of its own.
static NvmIntrfXfer_t *s_pXferDev = nullptr;

// Whether to submit and return rather than submit and wait. This is the
// bIntEn the application put in NvmCfg_t, handed down by the driver, not a
// build option. Only the SoftDevice path can honour it: the flash event
// already arrives by interrupt, so the wait in SdRun is the only thing in the
// way. The timeslot and direct controller paths finish inside their call
// whatever this says and report done at once, which the driver copes with.
static bool AsyncWanted(void)
{
	return s_pXferDev != nullptr && s_pXferDev->pDevIntrf->bIntEn &&
		   s_pXferDev->pDevIntrf->EvtCB != nullptr;
}

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

#if defined(NRF52_SERIES)

// True when the page already reads erased, so it can be left alone. Saves the
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

#endif	// NRF52_SERIES

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

// Tell the driver the transfer it started has finished. Runs in the SoC event
// context, so it does nothing but hand over the result.
static void AsyncReport(bool bOk)
{
	if (AsyncWanted() == false || s_pXferDev->bAsyncPending == false)
	{
		return;
	}

	s_pXferDev->bAsyncPending = false;

	// End the transfer here. The driver returned as soon as the operation was
	// under way and never sent a stop, because on this interface the stop is
	// what the completion does: it clears the frame and releases the busy
	// latch that DeviceIntrfStartTx set. Without it the next StartTx is
	// refused and nothing works again.
	DeviceIntrfStopTx(s_pXferDev->pDevIntrf);

	s_pXferDev->pDevIntrf->EvtCB(s_pXferDev->pDevIntrf,
					bOk ? DEVINTRF_EVT_COMPLETED : DEVINTRF_EVT_TX_TIMEOUT,
					nullptr, 0);
}

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
			AsyncReport(true);
			break;

		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_OpOk = false;
			s_OpPending = false;
			s_OpDone = true;
			s_Stat.Evt++;
			AsyncReport(false);
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
// Submit only. Returns 0 when the operation is under way and the flash event
// will report it, or a negative errno when it could not be started.
static int SdStart(SdSubmit_t Submit, void *pArg)
{
	s_OpDone = false;
	s_OpOk = false;
	s_OpPending = true;
	__DMB();

	uint32_t status = Submit(pArg);

	if (status == NRF_SUCCESS)
	{
		if (s_pXferDev != nullptr)
		{
			s_pXferDev->bAsyncPending = true;
		}

		return 0;
	}

	s_OpPending = false;

	if (status != NRF_ERROR_BUSY)
	{
		return -EIO;
	}

	// Busy is the stack holding the memory. That is a refusal, not a wait:
	// report it and let the driver come back, rather than spinning here on
	// the caller's time.
	s_Stat.Busy++;

	return -EBUSY;
}

// Submit and report the result once the flash event has been seen. Kept for
// the case where the driver did not ask for interrupt completion: the wait is
// a tight spin on the event flag, not a callback into the application, because
// how long to wait and what to do while waiting are the memory's business and
// live in Nvm.
static int SdRun(SdSubmit_t Submit, void *pArg)
{
	int r = SdStart(Submit, pArg);

	if (r != 0)
	{
		return r;
	}

	for (uint32_t i = 0; i < NVM_INTRF_SD_SPIN; i++)
	{
		if (s_OpDone)
		{
			if (s_pXferDev != nullptr)
			{
				s_pXferDev->bAsyncPending = false;
			}

			return s_OpOk ? 0 : -EIO;
		}
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
// One bounded unit of an operation, for an arbiter to run where the memory is
// safe to touch. Nothing here knows what a timeslot is, and none of it is
// conditional: whether an arbiter exists is answered at run time, so the steps
// have to be built whatever the stack is.
// ---------------------------------------------------------------------------
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
#endif	// NRF52_SERIES

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
		static SdWrArg_t arg;
		arg = { (uint32_t *)Addr, pSrc, WordCnt };

		s_Stat.Sd++;

		if (AsyncWanted())
		{
			int r = SdStart(SdWriteSubmit, &arg);

			return r != 0 ? r : NVM_INTRF_STARTED;
		}

		return SdRun(SdWriteSubmit, &arg);
	}
	// Not running, or no SoftDevice support linked: fall through.
#else
	if (SdPresent())
	{
		if (SdRunning())
		{
			static SdWrArg_t arg;
			arg = { (uint32_t *)Addr, pSrc, WordCnt };

			s_Stat.Sd++;

			if (AsyncWanted())
			{
				int r = SdStart(SdWriteSubmit, &arg);

				return r != 0 ? r : NVM_INTRF_STARTED;
			}

			return SdRun(SdWriteSubmit, &arg);
		}

		// Present but stopped: neither the stack nor the radio runs, so
		// drive the controller.
		s_Stat.Direct++;
		CtrlWriteWords(Addr, pSrc, WordCnt);

		return 0;
	}
	// No SoftDevice in the flash: a link controller build arbitrates below.
#endif
#endif

	if (s_pArbiter != nullptr)
	{
		SlotWrCtx_t ctx = { Addr, pSrc, WordCnt };
		NvmIntrfOp_t op;

		op.Step = SlotWriteStep;
		op.StepBudgetUs = NVM_INTRF_STEP_BUDGET_US;
		op.pCtx = &ctx;

		s_Stat.Slot++;

		return s_pArbiter(&op);
	}

	// Nobody claimed the memory, so it is ours.
	s_Stat.Direct++;
	CtrlWriteWords(Addr, pSrc, WordCnt);

	return 0;
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
		// A page erase is tens of msec and stalls instruction fetch anyway,
		// so there is nothing useful to do here. Bounded so a controller that
		// never reports ready cannot hang.
		if (++elapsed >= NVM_INTRF_SD_SPIN)
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
			static uint32_t no;
			no = (uint32_t)(Addr / page);

			s_Stat.Sd++;

			if (AsyncWanted())
			{
				int r = SdStart(SdEraseSubmit, &no);

				return r != 0 ? r : NVM_INTRF_STARTED;
			}

			return SdRun(SdEraseSubmit, &no);
		}

		// Present but stopped: neither the stack nor the radio runs, so
		// erase in place.
		s_Stat.Direct++;

		return NvmEraseBare(Addr, page);
	}
	// No SoftDevice in the flash: a link controller build arbitrates below.
#endif

	if (s_pArbiter != nullptr)
	{
		SlotErCtx_t ctx = { Addr, false };
		NvmIntrfOp_t op;

		op.Step = SlotEraseStep;
		op.StepBudgetUs = NVM_INTRF_STEP_BUDGET_US;
		op.pCtx = &ctx;

		s_Stat.Slot++;

		return s_pArbiter(&op);
	}

	s_Stat.Direct++;

	return NvmEraseBare(Addr, page);

#else	// nRF54L

	// RRAM rewrites in place, so it has no erase and the config reports none.
	// Nvm answers an erase on such a medium itself and never frames one, so
	// this is unreachable. An application that wants the region to read as
	// ones writes ones, the way it would on an EEPROM or an FRAM.
	(void)Addr;

	return -ENOTSUP;
#endif
}

// ---------------------------------------------------------------------------
// The interface
// ---------------------------------------------------------------------------

static uintptr_t FrameAddr(NvmIntrfXfer_t *pXfer)
{
	uintptr_t a = 0;

	for (int i = 0; i < NVM_INTRF_ADDR_SIZE; i++)
	{
		a = (a << 8) | pXfer->Hdr[1 + i];
	}

	return a;
}

// Put whatever whole words have been collected into the memory.
static int NvmFlush(NvmIntrfXfer_t *pXfer)
{
	if (pXfer->DataLen == 0)
	{
		return 0;
	}

	// A payload that is not whole words cannot be programmed; trimming it
	// silently would report bytes written that never were.
	if ((pXfer->DataLen % (int)NVM_INTRF_WRITE_GRAN) != 0)
	{
		pXfer->DataLen = 0;
		return -EINVAL;
	}

	uint32_t words = (uint32_t)pXfer->DataLen / NVM_INTRF_WRITE_GRAN;
	int res = NvmSubmit(FrameAddr(pXfer), (const uint32_t *)pXfer->Data, words);

	// Started counts as issued. Ops is how many operations the memory was
	// asked for, not how many finished inside the call.
	if (res == 0 || res == NVM_INTRF_STARTED)
	{
		s_Stat.Ops++;
	}

	pXfer->DataLen = 0;

	return res;
}

extern "C" {

static bool NvmStartRx(DevIntrf_t *pIntrf, uint32_t)
{
	// A restart after the command and address were sent, so keep the frame.
	s_pXferDev = (NvmIntrfXfer_t *)pIntrf->pDevData;

	return true;
}

static bool NvmStartTx(DevIntrf_t *pIntrf, uint32_t)
{
	NvmIntrfXfer_t *dev = (NvmIntrfXfer_t *)pIntrf->pDevData;

	s_pXferDev = dev;
	dev->HdrLen = 0;
	dev->DataLen = 0;

	return true;
}

// The work is done here rather than at the end of the transfer, because this
// is the only place a failure can be reported: returning short tells the
// driver the memory did not take the data.
static int NvmTxData(DevIntrf_t *pIntrf, const uint8_t *pData, int Len)
{
	NvmIntrfXfer_t *dev = (NvmIntrfXfer_t *)pIntrf->pDevData;
	int n = 0;

	while (n < Len && dev->HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		dev->Hdr[dev->HdrLen++] = pData[n++];
	}

	if (dev->HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		return n;
	}

	// An erase needs the address only.
	if (dev->Hdr[0] == NVM_INTRF_CMD_ERASE)
	{
		int res = NvmEraseUnit(FrameAddr(dev));

		if (res == NVM_INTRF_STARTED)
		{
			// Under way. The driver reads -1 as started and waits for the
			// event rather than treating it as a short transfer.
			s_Stat.Ops++;

			return -1;
		}
		if (res != 0)
		{
			return 0;
		}
		s_Stat.Ops++;
		return Len;
	}

	if (dev->Hdr[0] != NVM_INTRF_CMD_WRITE)
	{
		return Len;				// a read frame, served by RxData
	}

	while (n < Len)
	{
		dev->Data[dev->DataLen++] = pData[n++];

		if (dev->DataLen == (int)sizeof(dev->Data))
		{
			int res = NvmFlush(dev);

			if (res == NVM_INTRF_STARTED)
			{
				// The driver bounds a transfer to the page size, so this
				// only fires on the last of the buffer.
				return -1;
			}
			if (res != 0)
			{
				return n - (int)sizeof(dev->Data);
			}
		}
	}

	// The driver bounds a transfer to the page size, so the whole frame
	// arrives together and this is where it completes.
	int res = NvmFlush(dev);

	if (res == NVM_INTRF_STARTED)
	{
		// Under way. The driver reads -1 as started.
		return -1;
	}
	if (res != 0)
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
static int NvmRxData(DevIntrf_t *pIntrf, uint8_t *pBuff, int Len)
{
	NvmIntrfXfer_t *dev = (NvmIntrfXfer_t *)pIntrf->pDevData;

	if (dev->HdrLen < NVM_INTRF_FRAME_SIZE ||
		dev->Hdr[0] != NVM_INTRF_CMD_READ)
	{
		return 0;
	}

	memcpy(pBuff, (const void *)FrameAddr(dev), (size_t)Len);

	return Len;
}

static void NvmStopRx(DevIntrf_t *pIntrf)
{
	((NvmIntrfXfer_t *)pIntrf->pDevData)->HdrLen = 0;
}

// Everything was done in TxData, where a failure could be reported.
static void NvmStopTx(DevIntrf_t *pIntrf)
{
	NvmIntrfXfer_t *dev = (NvmIntrfXfer_t *)pIntrf->pDevData;

	dev->HdrLen = 0;
	dev->DataLen = 0;
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

	memset(&vXfer, 0, sizeof(vXfer));

	// pDevData is the transaction state, so a transfer callback reaches it
	// without knowing anything about this class.
	vDevIntrf.pDevData = &vXfer;
	vXfer.pDevIntrf = &vDevIntrf;
	vDevIntrf.Type = DEVINTRF_TYPE_MEMCTRL;
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
	vDevIntrf.EnCnt = 1;

	// bIntEn is left for the driver to set from NvmCfg_t; the flash event is
	// reported through EvtCB, which Nvm::Init hooks up.
	atomic_flag_clear(&vDevIntrf.bBusy);

#if defined(NRFXLIB_SDC) && defined(NVM_INTRF_SOFTDEVICE) && !defined(NVM_INTRF_SD_RUNTIME)
	if (SdPresent())
	{
		// The SoftDevice arbitrates; the timeslot session is not needed and
		// MPSL must not be brought up beside it.
		return true;
	}
#endif

	return true;
}

void NvmIntrfSetArbiter(NvmIntrfArb_t pArb)
{
	s_pArbiter = pArb;
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

	// Both parts map their memory from 0, so the frame address and the offset
	// within the device are the same number here. A part mapped elsewhere
	// puts its base in here and the driver adds it.
	Cfg.BaseAddr = 0;

#if defined(NRF52_SERIES)
	// Geometry from the device itself.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
#else
	// RRAM overwrites in place: a direct read write medium with no erase.
	// The logical sector is what a block consumer replaces at once.
	Cfg.TotalSize = NVM_INTRF_TOTAL_SIZE;
	Cfg.EraseSize = 0;
	Cfg.SectorSize = NVM_INTRF_ERASE_SIZE;
#endif
	Cfg.PageSize = NVM_INTRF_MAX_XFER;		// largest bytes per transfer
	Cfg.WriteGran = NVM_INTRF_WRITE_GRAN;
	Cfg.AddrSize = NVM_INTRF_ADDR_SIZE;

	// The opcode says which operation the frame is, the way FlashEraseSector
	// puts one in front of the address. The driver fills these in for a medium
	// with a command protocol and leaves them alone for one without, and this
	// medium is the second kind, so state them here. Uniform frames matter:
	// an erase always has its opcode, so a read and a write must too or
	// this end cannot tell them apart.
	Cfg.RdCmd = { NVM_INTRF_CMD_READ, 0 };
	Cfg.WrCmd = { NVM_INTRF_CMD_WRITE, 0 };

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
