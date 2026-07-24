/**-------------------------------------------------------------------------
@file	rramc_intrf.cpp

@brief	The nRF54L internal RRAM controller as a DeviceIntrf.

		A frame arrives as command, address and data, exactly as it would on a
		wire. The command and the address are collected apart from the data so
		the data buffer stays word aligned, and the work is done as the data
		arrives, which is the only place a failure can be reported back.

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

#include "rramc_intrf.h"

// The controller writes 32 bit words.
#define RRAMC_WRITE_GRAN			4

// What an erase covers. RRAM has no erase command, so the erase below writes
// the erased pattern over this much, which is what erase means on this medium.
#ifndef RRAMC_INTRF_ERASE_SIZE
#define RRAMC_INTRF_ERASE_SIZE		0x00000400UL	// 1 KB
#endif

// Total RRAM. It cannot be read from the device the way the nRF52 page size
// comes from FICR, so override this for a part other than the nRF54L15.
#ifndef RRAMC_INTRF_TOTAL_SIZE
#define RRAMC_INTRF_TOTAL_SIZE		0x0017D000UL	// 1524 KB
#endif

// Largest bytes one transfer may take. A word write is microseconds, so this
// only has to keep a timeslot step short. The Nvm driver splits at this.
#ifndef RRAMC_INTRF_MAX_XFER
#define RRAMC_INTRF_MAX_XFER		64
#endif

#ifndef RRAMC_INTRF_STEP_BUDGET_US
#define RRAMC_INTRF_STEP_BUDGET_US	2000
#endif

#ifndef RRAMC_INTRF_TIMEOUT_MS
#define RRAMC_INTRF_TIMEOUT_MS		5000
#endif

// The command and address are collected apart from the data, so the data
// buffer stays word aligned. Reading it as words from an odd frame offset
// would be an unaligned access, and the SoftDevice rejects an unaligned
// source outright.
#define RRAMC_FRAME_HDR				(1 + RRAMC_ADDR_SIZE)

static uint8_t s_Hdr[RRAMC_FRAME_HDR];
static int s_HdrLen;
alignas(4) static uint8_t s_Data[RRAMC_INTRF_MAX_XFER];
static int s_DataLen;
static bool s_Rx;

static RramcIntrfWait_t s_pWait = nullptr;
static uint32_t s_TimeoutMs = RRAMC_INTRF_TIMEOUT_MS;
static RramcIntrfStat_t s_Stat;

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

	// The buffer has to drain before the data is committed.
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

void RramcIntrfSocEvt(uint32_t SysEvt)
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

void RramcIntrfSocEvt(uint32_t SysEvt)
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
// The work, chosen by what is running rather than by the application.
// ---------------------------------------------------------------------------

static int RramcPut(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt)
{
	if (WordCnt == 0)
	{
		return 0;
	}

#ifdef SOFTDEVICE_PRESENT
	if (SdIsEnabled())
	{
		return SdWrite((uint32_t *)Addr, pSrc, WordCnt);
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
	op.StepBudgetUs = RRAMC_INTRF_STEP_BUDGET_US;
	op.pCtx = &ctx;

	return BtPdsMpslRun(&op);
#else
	RramcWriteWords(Addr, pSrc, WordCnt);

	return 0;
#endif
}

// RRAM has no erase command, so an erase is the erased pattern written over
// the unit. It goes the same way as any other write.
static int RramcClear(uintptr_t Addr)
{
	alignas(4) static uint8_t ones[RRAMC_INTRF_MAX_XFER];
	uint32_t left = RRAMC_INTRF_ERASE_SIZE;

	memset(ones, 0xFF, sizeof(ones));

	while (left > 0)
	{
		uint32_t n = left < sizeof(ones) ? left : (uint32_t)sizeof(ones);

		int res = RramcPut(Addr, (const uint32_t *)ones, n / RRAMC_WRITE_GRAN);
		if (res != 0)
		{
			return res;
		}
		Addr += n;
		left -= n;
	}

	return 0;
}

// ---------------------------------------------------------------------------
// The interface.
// ---------------------------------------------------------------------------

static uintptr_t FrameAddr(void)
{
	uintptr_t a = 0;

	for (int i = 0; i < RRAMC_ADDR_SIZE; i++)
	{
		a = (a << 8) | s_Hdr[1 + i];
	}

	return a;
}

// Put whatever whole words have been collected into the memory.
static int RramcFlush(void)
{
	if (s_DataLen < (int)RRAMC_WRITE_GRAN)
	{
		return 0;
	}

	uint32_t words = (uint32_t)s_DataLen / RRAMC_WRITE_GRAN;
	int res = RramcPut(FrameAddr(), (const uint32_t *)s_Data, words);

	if (res == 0)
	{
		s_Stat.Ops++;
	}

	s_DataLen = 0;

	return res;
}

extern "C" {

static bool RramcStartRx(DevIntrf_t *, uint32_t)
{
	// A restart after the command and address were sent, so keep the frame.
	s_Rx = true;

	return true;
}

static bool RramcStartTx(DevIntrf_t *, uint32_t)
{
	s_HdrLen = 0;
	s_DataLen = 0;
	s_Rx = false;

	return true;
}

// The work is done here rather than at the end of the transfer, because this
// is the only place a failure can be reported: returning short tells the
// driver the memory did not take the data.
static int RramcTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	int n = 0;

	while (n < Len && s_HdrLen < RRAMC_FRAME_HDR)
	{
		s_Hdr[s_HdrLen++] = pData[n++];
	}

	if (s_HdrLen < RRAMC_FRAME_HDR)
	{
		return n;
	}

	// An erase needs the address only.
	if (s_Hdr[0] == RRAMC_CMD_ERASE)
	{
		if (RramcClear(FrameAddr()) != 0)
		{
			return 0;
		}
		s_Stat.Ops++;
		return Len;
	}

	if (s_Hdr[0] != RRAMC_CMD_WRITE)
	{
		return Len;				// a read frame, served by RxData
	}

	while (n < Len)
	{
		s_Data[s_DataLen++] = pData[n++];

		if (s_DataLen == (int)sizeof(s_Data))
		{
			if (RramcFlush() != 0)
			{
				return n - (int)sizeof(s_Data);
			}
		}
	}

	// The driver bounds a transfer to the page size, so the whole frame
	// arrives together and this is where it completes.
	if (RramcFlush() != 0)
	{
		return 0;
	}

	return n;
}

static int RramcTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	return RramcTxData(pDev, pData, Len);
}

// The memory is mapped, so a read is a copy from the address in the frame.
static int RramcRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_HdrLen < RRAMC_FRAME_HDR || s_Hdr[0] != RRAMC_CMD_READ)
	{
		return 0;
	}

	memcpy(pBuff, (const void *)FrameAddr(), (size_t)Len);

	return Len;
}

static void RramcStopRx(DevIntrf_t *)
{
	s_Rx = false;
	s_HdrLen = 0;
}

// Everything was done in TxData, where a failure could be reported.
static void RramcStopTx(DevIntrf_t *)
{
	s_HdrLen = 0;
	s_DataLen = 0;
}

static void RramcIntrfDisable(DevIntrf_t *) {}
static void RramcIntrfEnable(DevIntrf_t *) {}
static uint32_t RramcGetRate(DevIntrf_t *) { return 0; }
static uint32_t RramcSetRate(DevIntrf_t *, uint32_t Rate) { return Rate; }
static void RramcPowerOff(DevIntrf_t *) {}
static void *RramcGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

}	// extern "C"

bool RramcIntrf::Init(void)
{
	memset(&vDevIntrf, 0, sizeof(vDevIntrf));

	vDevIntrf.pDevData = this;
	vDevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	vDevIntrf.Disable = RramcIntrfDisable;
	vDevIntrf.Enable = RramcIntrfEnable;
	vDevIntrf.GetRate = RramcGetRate;
	vDevIntrf.SetRate = RramcSetRate;
	vDevIntrf.StartRx = RramcStartRx;
	vDevIntrf.RxData = RramcRxData;
	vDevIntrf.StopRx = RramcStopRx;
	vDevIntrf.StartTx = RramcStartTx;
	vDevIntrf.TxData = RramcTxData;
	vDevIntrf.TxSrData = RramcTxSrData;
	vDevIntrf.StopTx = RramcStopTx;
	vDevIntrf.PowerOff = RramcPowerOff;
	vDevIntrf.GetHandle = RramcGetHandle;
	vDevIntrf.MaxRetry = 1;
	vDevIntrf.MaxTrxLen = RRAMC_FRAME_HDR + RRAMC_INTRF_MAX_XFER;
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

void RramcIntrfSetWait(RramcIntrfWait_t pWait, uint32_t TimeoutMs)
{
	s_pWait = pWait;

	if (TimeoutMs != 0)
	{
		s_TimeoutMs = TimeoutMs;
	}
}

void RramcIntrfGetStat(RramcIntrfStat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}

void RramcIntrfCfg(NvmCfg_t &Cfg)
{
	Cfg.DevNo = 0;
	Cfg.TotalSize = RRAMC_INTRF_TOTAL_SIZE;
	Cfg.EraseSize = RRAMC_INTRF_ERASE_SIZE;
	Cfg.PageSize = RRAMC_INTRF_MAX_XFER;	// largest bytes per transfer
	Cfg.WriteGran = RRAMC_WRITE_GRAN;
	Cfg.AddrSize = RRAMC_ADDR_SIZE;
	Cfg.RdCmd = { RRAMC_CMD_READ, 0 };
	Cfg.WrCmd = { RRAMC_CMD_WRITE, 0 };
	Cfg.EraseCmd = { RRAMC_CMD_ERASE, 0 };
	Cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}
