/**-------------------------------------------------------------------------
@file	nvmc_intrf.cpp

@brief	The nRF52 internal memory controller as a DeviceIntrf.

		A frame arrives as command, address and data, exactly as it would on a
		wire. The frame is collected during the transfer and acted on when it
		completes, which is where the controller work and its arbitration live.

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
#include <string.h>
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

#include "nvmc_intrf.h"

// The controller programs 32 bit words.
#define NVMC_WRITE_GRAN				4

// Largest bytes one transfer may take. A word write is a few hundred usec, and
// on a timeslot build the whole transfer has to fit one slot, which is the
// tightest limit of the three cases. The Nvm driver splits at this.
#ifndef NVMC_INTRF_MAX_XFER
#define NVMC_INTRF_MAX_XFER			32
#endif

// Erase slice per timeslot, in msec, and the worst case for one step.
#ifndef NVMC_INTRF_ERASE_SLICE_MS
#define NVMC_INTRF_ERASE_SLICE_MS	2
#endif

#ifndef NVMC_INTRF_STEP_BUDGET_US
#define NVMC_INTRF_STEP_BUDGET_US	((NVMC_INTRF_ERASE_SLICE_MS + 1) * 1000UL)
#endif

#ifndef NVMC_INTRF_TIMEOUT_MS
#define NVMC_INTRF_TIMEOUT_MS		5000
#endif

// The command and address are collected apart from the data, so the data
// buffer stays word aligned. Reading it as words from an odd frame offset
// would be an unaligned access, and the SoftDevice rejects an unaligned
// source outright.
#define NVMC_FRAME_HDR				(1 + NVMC_ADDR_SIZE)
#define NVMC_FRAME_MAX				(NVMC_FRAME_HDR + NVMC_INTRF_MAX_XFER)

static uint8_t s_Hdr[NVMC_FRAME_HDR];
static int s_HdrLen;
alignas(4) static uint8_t s_Data[NVMC_INTRF_MAX_XFER];
static int s_DataLen;
static bool s_Rx;

static NvmcIntrfWait_t s_pWait = nullptr;
static uint32_t s_TimeoutMs = NVMC_INTRF_TIMEOUT_MS;
static NvmcIntrfStat_t s_Stat;

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

void NvmcIntrfSocEvt(uint32_t SysEvt)
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

void NvmcIntrfSocEvt(uint32_t SysEvt)
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

		return NVMC_INTRF_ERASE_SLICE_MS * 1000UL;
	}

	if (NvmcIsReady())
	{
		NvmcRelease();
		return 0;
	}

	return NVMC_INTRF_ERASE_SLICE_MS * 1000UL;
}

#endif	// NRFXLIB_SDC

// ---------------------------------------------------------------------------
// The work, chosen by what is running rather than by the application.
// ---------------------------------------------------------------------------

static int NvmcWrite(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt)
{
#ifdef SOFTDEVICE_PRESENT
	if (SdIsEnabled())
	{
		SdWrArg_t arg = { (uint32_t *)Addr, pSrc, WordCnt };

		return SdRun(SdWriteSubmit, &arg);
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
	op.StepBudgetUs = NVMC_INTRF_STEP_BUDGET_US;
	op.pCtx = &ctx;

	return BtPdsMpslRun(&op);
#else
	NvmcProgramWords(Addr, pSrc, WordCnt);

	return 0;
#endif
}

static int NvmcErase(uintptr_t Addr)
{
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;

	if (pagesize == 0)
	{
		return -EINVAL;
	}

#ifdef SOFTDEVICE_PRESENT
	if (SdIsEnabled())
	{
		uint32_t page = (uint32_t)(Addr / pagesize);

		return SdRun(SdEraseSubmit, &page);
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
	op.StepBudgetUs = NVMC_INTRF_STEP_BUDGET_US;
	op.pCtx = &ctx;

	return BtPdsMpslRun(&op);
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

	return 0;
#endif
}

// ---------------------------------------------------------------------------
// The interface. A frame is collected, then acted on when the transfer ends.
// ---------------------------------------------------------------------------

static uintptr_t FrameAddr(void)
{
	uintptr_t a = 0;

	for (int i = 0; i < NVMC_ADDR_SIZE; i++)
	{
		a = (a << 8) | s_Hdr[1 + i];
	}

	return a;
}

// Put whatever whole words have been collected into the memory.
static int NvmcFlush(void)
{
	if (s_DataLen < (int)NVMC_WRITE_GRAN)
	{
		return 0;
	}

	uint32_t words = (uint32_t)s_DataLen / NVMC_WRITE_GRAN;
	int res = NvmcWrite(FrameAddr(), (const uint32_t *)s_Data, words);

	if (res == 0)
	{
		s_Stat.Ops++;
	}

	s_DataLen = 0;

	return res;
}

extern "C" {

static bool NvmcStartRx(DevIntrf_t *, uint32_t)
{
	// A restart after the command and address were sent, so keep the frame.
	s_Rx = true;

	return true;
}

static bool NvmcStartTx(DevIntrf_t *, uint32_t)
{
	s_HdrLen = 0;
	s_DataLen = 0;
	s_Rx = false;

	return true;
}

// The work is done here rather than at the end of the transfer, because this
// is the only place a failure can be reported: returning short tells the
// driver the memory did not take the data.
static int NvmcTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	int n = 0;

	while (n < Len && s_HdrLen < NVMC_FRAME_HDR)
	{
		s_Hdr[s_HdrLen++] = pData[n++];
	}

	if (s_HdrLen < NVMC_FRAME_HDR)
	{
		return n;
	}

	// An erase needs the address only.
	if (s_Hdr[0] == NVMC_CMD_ERASE)
	{
		if (NvmcErase(FrameAddr()) != 0)
		{
			return 0;
		}
		s_Stat.Ops++;
		return Len;
	}

	if (s_Hdr[0] != NVMC_CMD_WRITE)
	{
		return Len;				// a read frame, served by RxData
	}

	while (n < Len)
	{
		s_Data[s_DataLen++] = pData[n++];

		if (s_DataLen == (int)sizeof(s_Data))
		{
			if (NvmcFlush() != 0)
			{
				return n - (int)sizeof(s_Data);
			}
		}
	}

	// The driver bounds a transfer to the page size, so the whole frame
	// arrives together and this is where it completes.
	if (NvmcFlush() != 0)
	{
		return 0;
	}

	return n;
}

static int NvmcTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	return NvmcTxData(pDev, pData, Len);
}

// The memory is mapped, so a read is a copy from the address in the frame.
static int NvmcRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_HdrLen < NVMC_FRAME_HDR || s_Hdr[0] != NVMC_CMD_READ)
	{
		return 0;
	}

	memcpy(pBuff, (const void *)FrameAddr(), (size_t)Len);

	return Len;
}

static void NvmcStopRx(DevIntrf_t *)
{
	s_Rx = false;
	s_HdrLen = 0;
}

// Everything was done in TxData, where a failure could be reported.
static void NvmcStopTx(DevIntrf_t *)
{
	s_HdrLen = 0;
	s_DataLen = 0;
}

static void NvmcIntrfDisable(DevIntrf_t *) {}
static void NvmcIntrfEnable(DevIntrf_t *) {}
static uint32_t NvmcGetRate(DevIntrf_t *) { return 0; }
static uint32_t NvmcSetRate(DevIntrf_t *, uint32_t Rate) { return Rate; }
static void NvmcPowerOff(DevIntrf_t *) {}
static void *NvmcGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

}	// extern "C"

bool NvmcIntrf::Init(void)
{
	memset(&vDevIntrf, 0, sizeof(vDevIntrf));

	vDevIntrf.pDevData = this;
	vDevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	vDevIntrf.Disable = NvmcIntrfDisable;
	vDevIntrf.Enable = NvmcIntrfEnable;
	vDevIntrf.GetRate = NvmcGetRate;
	vDevIntrf.SetRate = NvmcSetRate;
	vDevIntrf.StartRx = NvmcStartRx;
	vDevIntrf.RxData = NvmcRxData;
	vDevIntrf.StopRx = NvmcStopRx;
	vDevIntrf.StartTx = NvmcStartTx;
	vDevIntrf.TxData = NvmcTxData;
	vDevIntrf.TxSrData = NvmcTxSrData;
	vDevIntrf.StopTx = NvmcStopTx;
	vDevIntrf.PowerOff = NvmcPowerOff;
	vDevIntrf.GetHandle = NvmcGetHandle;
	vDevIntrf.MaxRetry = 1;
	vDevIntrf.MaxTrxLen = NVMC_FRAME_MAX;
	vDevIntrf.EnCnt = 1;
	atomic_flag_clear(&vDevIntrf.bBusy);

	s_HdrLen = 0;
	s_DataLen = 0;
	s_Rx = false;

#ifdef NRFXLIB_SDC
	return MpslStart() == 0;
#else
	return true;
#endif
}

void NvmcIntrfSetWait(NvmcIntrfWait_t pWait, uint32_t TimeoutMs)
{
	s_pWait = pWait;

	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

void NvmcIntrfGetStat(NvmcIntrfStat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}

void NvmcIntrfCfg(NvmCfg_t &Cfg)
{
	// Geometry from the device itself rather than a build time constant.
	uint32_t pagesize = NRF_FICR->CODEPAGESIZE;
	uint32_t pagecnt = NRF_FICR->CODESIZE;

	Cfg.DevNo = 0;
	Cfg.TotalSize = (uint64_t)pagesize * (uint64_t)pagecnt;
	Cfg.EraseSize = pagesize;
	Cfg.PageSize = NVMC_INTRF_MAX_XFER;		// largest bytes per transfer
	Cfg.WriteGran = NVMC_WRITE_GRAN;
	Cfg.AddrSize = NVMC_ADDR_SIZE;
	Cfg.RdCmd = { NVMC_CMD_READ, 0 };
	Cfg.WrCmd = { NVMC_CMD_WRITE, 0 };
	Cfg.EraseCmd = { NVMC_CMD_ERASE, 0 };
}
