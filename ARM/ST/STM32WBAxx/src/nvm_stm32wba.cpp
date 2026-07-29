/**-------------------------------------------------------------------------
@file	nvm_stm32wba.cpp

@brief	The STM32WBA on die flash controller as a DeviceIntrf.

		The frame, the read and the staging are the same as the Nordic port.
		Only the controller access and the geometry follow from the MCU
		model: WBA2x programs a 64 bit double word into 4 KB pages, WBA5x a
		128 bit quad word into 8 KB pages, and the device header says which
		through FLASH_DOUBLEWORD_SUPPORT. One driver serves the family; the
		difference is a parameter, not a shape.

		Every operation finishes inside its call. The flash on this part is
		the application's alone: the ST link layer keeps its own state in
		RAM and registers no arbiter over the memory, so there is no
		SoftDevice or timeslot path here. What a program or erase does cost
		is core stall: fetching from the bank being written halts the CPU
		for the operation, up to the page erase time, which delays every
		interrupt including the link layer's. Place the storage region in
		the other bank on a dual bank part, or accept the stall on a single
		bank one; that placement is the linker script's business.

@author	Hoang Nguyen Hoan
@date	July 29, 2026

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

#include "stm32wbaxx.h"

#include "coredev/interrupt.h"
#include "storage/nvm_intrf.h"

// The command set, using the opcodes a serial flash uses so a config for
// internal memory reads like one for a flash chip.
#define NVM_INTRF_CMD_READ		0x03		//!< Read
#define NVM_INTRF_CMD_WRITE		0x02		//!< Program words
#define NVM_INTRF_CMD_ERASE		0x20		//!< Erase one page

// 32 bit addresses, the flash bus address as mapped.
#define NVM_INTRF_ADDR_SIZE		4

// ---------------------------------------------------------------------------
// What follows from the MCU model
// ---------------------------------------------------------------------------

// The flash word: what one program operation takes, whole, aligned. The
// device header says which of the two this part is. Nothing narrower and
// nothing unaligned programs; the driver above pads to WriteGran, so a
// transfer arrives as whole flash words.
#if defined(FLASH_DOUBLEWORD_SUPPORT)
#define NVM_INTRF_WRITE_GRAN	8			//!< 64 bit double word
#define NVM_INTRF_PAGE_SIZE		0x1000UL	//!< 4 KB
#else
#define NVM_INTRF_WRITE_GRAN	16			//!< 128 bit quad word
#define NVM_INTRF_PAGE_SIZE		0x2000UL	//!< 8 KB
#endif

// Total flash, from the device's own size register, in the same shape the
// HAL reads it: KB count at FLASHSIZE_BASE, all ones or zero meaning the
// full 1 MB.
static inline uint32_t WbaFlashSize(void)
{
	uint32_t kb = *(const volatile uint16_t *)FLASHSIZE_BASE;

	if (kb == 0xFFFFUL || kb == 0UL)
	{
		return 0x100000UL;
	}

	return kb << 10;
}

#if defined(FLASH_DBANK_SUPPORT)
#define NVM_INTRF_BANK_SIZE()	(WbaFlashSize() / 2UL)
#endif

// Largest bytes one transfer stages. A multiple of both flash words so one
// number serves the family.
#ifndef NVM_INTRF_MAX_XFER
#define NVM_INTRF_MAX_XFER		64
#endif

static_assert(NVM_INTRF_MAX_XFER <= NVM_INTRF_XFER_SIZE,
			  "NVM_INTRF_MAX_XFER exceeds the staging buffer");
static_assert((NVM_INTRF_MAX_XFER % NVM_INTRF_WRITE_GRAN) == 0,
			  "NVM_INTRF_MAX_XFER is not whole flash words");

// A bound so a stuck controller cannot hang the caller. A page erase is
// milliseconds; this is loop passes, not a duration, the way the Nordic
// port bounds its spin.
#ifndef NVM_INTRF_SPIN
#define NVM_INTRF_SPIN			0x00800000UL
#endif

// Every register this port touches is the non secure bank, the same access
// the HAL makes for a non CMSE build. A secure build owns SECCR1/SECSR and
// is not this port's business.
#define WBA_FLASH				FLASH_NS

// The error bits one failed operation can raise, write one to clear.
#define WBA_FLASH_ERR			(FLASH_NSSR_OPERR | FLASH_NSSR_PROGERR | \
								 FLASH_NSSR_WRPERR | FLASH_NSSR_PGAERR | \
								 FLASH_NSSR_SIZERR | FLASH_NSSR_PGSERR | \
								 FLASH_NSSR_OPTWERR)

// ---------------------------------------------------------------------------
// Per controller state. One memory controller, so these describe the
// hardware rather than a transaction, the same reasoning as the Nordic port.
// ---------------------------------------------------------------------------

static NvmIntrfStat_t s_Stat;

static NvmDevIntrf_t s_NvmDev;

DevIntrf_t * const NvmMcuDevIntrf(void)
{
	return &s_NvmDev.DevIntrf;
}

// ---------------------------------------------------------------------------
// The controller
// ---------------------------------------------------------------------------

// Busy covers the operation and the write buffer both; the HAL waits on the
// same pair. Bounded, so a controller that never settles reports instead of
// hanging.
static bool WbaFlashWait(void)
{
	uint32_t spin = NVM_INTRF_SPIN;

	while (WBA_FLASH->NSSR & (FLASH_NSSR_BSY | FLASH_NSSR_WDW))
	{
		if (spin-- == 0)
		{
			return false;
		}
	}

	return true;
}

// Take and report the outcome of the operation that just ran: errors are
// cleared by writing them back, EOP likewise when it was raised.
static int WbaFlashResult(void)
{
	uint32_t sr = WBA_FLASH->NSSR;

	if (sr & FLASH_NSSR_EOP)
	{
		WBA_FLASH->NSSR = FLASH_NSSR_EOP;
	}

	if (sr & WBA_FLASH_ERR)
	{
		WBA_FLASH->NSSR = sr & WBA_FLASH_ERR;

		return -EIO;
	}

	return 0;
}

static void WbaFlashUnlock(void)
{
	if (WBA_FLASH->NSCR1 & FLASH_NSCR1_LOCK)
	{
		WBA_FLASH->NSKEYR = 0x45670123UL;
		WBA_FLASH->NSKEYR = 0xCDEF89ABUL;
	}
}

static void WbaFlashLock(void)
{
	WBA_FLASH->NSCR1 |= FLASH_NSCR1_LOCK;
}

// Program whole flash words at Addr. Addr is the mapped bus address, whole
// flash word aligned; the driver splits and pads above, so anything else
// here is refused rather than trimmed.
//
// The word's 32 bit writes go out back to back with interrupts held, the
// way the HAL programs the same word: a fetch from this bank between them
// stalls anyway, and an interrupt writing flash mid word raises PGSERR.
static int WbaFlashProgram(uintptr_t Addr, const uint8_t *pData, uint32_t Len)
{
	if ((Addr % NVM_INTRF_WRITE_GRAN) != 0 ||
		(Len % NVM_INTRF_WRITE_GRAN) != 0)
	{
		return -EINVAL;
	}

	if (WbaFlashWait() == false)
	{
		return -ETIMEDOUT;
	}

	// Stale error bits fail the sequence check of the operation being
	// started, so they go before PG is set.
	WBA_FLASH->NSSR = WBA_FLASH->NSSR & WBA_FLASH_ERR;

	WbaFlashUnlock();
	WBA_FLASH->NSCR1 |= FLASH_NSCR1_PG;

	int res = 0;
	volatile uint32_t *dst = (volatile uint32_t *)Addr;
	const uint8_t *src = pData;

	for (uint32_t left = Len; left > 0; left -= NVM_INTRF_WRITE_GRAN)
	{
		// The controller collects the word's bus writes and wants the flash
		// word whole; an interrupt touching the flash between them raises
		// PGSERR or commits a partial word. Held only for the word's few
		// stores; the busy wait below runs with interrupts live, which is
		// where the link layer gets serviced between words.
		uint32_t state = DisableInterrupt();

		for (int i = 0; i < NVM_INTRF_WRITE_GRAN / 4; i++)
		{
			uint32_t w;

			memcpy(&w, src, sizeof(w));
			*dst++ = w;
			src += sizeof(w);
		}

		EnableInterrupt(state);

		if (WbaFlashWait() == false)
		{
			res = -ETIMEDOUT;
			break;
		}

		res = WbaFlashResult();
		if (res != 0)
		{
			break;
		}
	}

	WBA_FLASH->NSCR1 &= ~FLASH_NSCR1_PG;
	WbaFlashLock();

	return res;
}

// ---------------------------------------------------------------------------
// What the port knows about the memory itself
// ---------------------------------------------------------------------------

// The C linkage of NvmMcuErase, NvmMcuIsReady and NvmMcuCeiling comes from
// their declarations in nvm_intrf.h; the definitions inherit it, so nothing
// is marked here.

// Erase the page holding Addr. Addr is the mapped bus address, page aligned;
// the driver only asks at the erase unit it was configured with.
int NvmMcuErase(uintptr_t Addr)
{
	uint32_t size = WbaFlashSize();

	if (Addr < FLASH_BASE || (Addr % NVM_INTRF_PAGE_SIZE) != 0 ||
		Addr + NVM_INTRF_PAGE_SIZE > FLASH_BASE + size)
	{
		return -EINVAL;
	}

	if (WbaFlashWait() == false)
	{
		return -ETIMEDOUT;
	}

	WBA_FLASH->NSSR = WBA_FLASH->NSSR & WBA_FLASH_ERR;

	uint32_t page = (uint32_t)((Addr - FLASH_BASE) / NVM_INTRF_PAGE_SIZE);
	uint32_t cr = WBA_FLASH->NSCR1;

	cr &= ~(FLASH_NSCR1_PNB_Msk | FLASH_NSCR1_PG | FLASH_NSCR1_MER);

#if defined(FLASH_DBANK_SUPPORT)
	// The page number counts within the bank; which bank is a bit of its
	// own.
	uint32_t bank = NVM_INTRF_BANK_SIZE();

	if ((Addr - FLASH_BASE) >= bank)
	{
		page -= bank / NVM_INTRF_PAGE_SIZE;
		cr |= FLASH_NSCR1_BKER;
	}
	else
	{
		cr &= ~FLASH_NSCR1_BKER;
	}
#endif

	cr |= (page << FLASH_NSCR1_PNB_Pos) | FLASH_NSCR1_PER;

	WbaFlashUnlock();
	WBA_FLASH->NSCR1 = cr;
	WBA_FLASH->NSCR1 = cr | FLASH_NSCR1_STRT;

	int res = WbaFlashWait() ? WbaFlashResult() : -ETIMEDOUT;

	WBA_FLASH->NSCR1 &= ~(FLASH_NSCR1_PER | FLASH_NSCR1_PNB_Msk);
	WbaFlashLock();

	if (res == 0)
	{
		s_Stat.Ops++;
		s_Stat.Direct++;
		s_Stat.RepDone++;
	}

	return res;
}

bool NvmMcuIsReady(void)
{
	return (WBA_FLASH->NSSR & (FLASH_NSSR_BSY | FLASH_NSSR_WDW)) == 0;
}

void NvmMcuCfg(NvmCfg_t &Cfg)
{
	Cfg.DevNo = 0;

	// The flash is mapped at FLASH_BASE, not at 0, so the base goes in here
	// and every address this port sees is the mapped bus address. A creator
	// that reserves a region through the linker script puts that region's
	// base and size in instead, the way the Nordic bond stores do.
	Cfg.BaseAddr = FLASH_BASE;
	Cfg.TotalSize = WbaFlashSize();
	Cfg.EraseSize = NVM_INTRF_PAGE_SIZE;
	Cfg.PageSize = NVM_INTRF_MAX_XFER;		// largest bytes per transfer
	Cfg.WriteGran = NVM_INTRF_WRITE_GRAN;
	Cfg.AddrSize = NVM_INTRF_ADDR_SIZE;

	// The opcode says which operation the frame is, uniform across the
	// three, the same reasoning as the Nordic port.
	Cfg.RdCmd = { NVM_INTRF_CMD_READ, 0 };
	Cfg.WrCmd = { NVM_INTRF_CMD_WRITE, 0 };

	Cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

// Where the memory the application owns ends. The default is the whole
// device; weak so a build whose upper flash belongs to something else can
// answer with that boundary instead.
__attribute__((weak)) uint64_t NvmMcuCeiling(void)
{
	return (uint64_t)FLASH_BASE + WbaFlashSize();
}

// ---------------------------------------------------------------------------
// The interface
// ---------------------------------------------------------------------------

static uintptr_t FrameAddr(NvmDevIntrf_t *pXfer)
{
	uintptr_t a = 0;

	for (int i = 0; i < NVM_INTRF_ADDR_SIZE; i++)
	{
		a = (a << 8) | pXfer->Hdr[1 + i];
	}

	return a;
}

// Put whatever whole flash words have been collected into the memory.
static int NvmFlush(NvmDevIntrf_t *pXfer)
{
	if (pXfer->DataLen == 0)
	{
		return 0;
	}

	// A payload that is not whole flash words cannot be programmed; trimming
	// it silently would report bytes written that never were.
	if ((pXfer->DataLen % NVM_INTRF_WRITE_GRAN) != 0)
	{
		pXfer->DataLen = 0;

		return -EINVAL;
	}

	int res = WbaFlashProgram(FrameAddr(pXfer), pXfer->Data,
							  (uint32_t)pXfer->DataLen);

	s_Stat.Ops++;
	s_Stat.Direct++;

	if (res == 0)
	{
		s_Stat.RepDone++;
	}

	pXfer->DataLen = 0;

	return res;
}

static bool NvmStartRx(DevIntrf_t *, uint32_t)
{
	// A restart after the command and address were sent, so keep the frame.
	return true;
}

static bool NvmStartTx(DevIntrf_t *pIntrf, uint32_t)
{
	NvmDevIntrf_t *dev = (NvmDevIntrf_t *)pIntrf->pDevData;

	dev->HdrLen = 0;
	dev->DataLen = 0;

	return true;
}

// The work is done here rather than at the end of the transfer, because this
// is the only place a failure can be reported: returning short tells the
// driver the memory did not take the data.
static int NvmTxData(DevIntrf_t *pIntrf, const uint8_t *pData, int Len)
{
	NvmDevIntrf_t *dev = (NvmDevIntrf_t *)pIntrf->pDevData;
	int n = 0;

	while (n < Len && dev->HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		dev->Hdr[dev->HdrLen++] = pData[n++];
	}

	if (dev->HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		return n;
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
			if (NvmFlush(dev) != 0)
			{
				return n - dev->DataLen;
			}
		}
	}

	if (NvmFlush(dev) != 0)
	{
		return 0;
	}

	// The frame address moves with what was taken, so a driver that streams
	// a transfer in pieces lands each where the last one ended.
	uintptr_t next = FrameAddr(dev) + (uintptr_t)(n - NVM_INTRF_FRAME_SIZE);

	for (int i = NVM_INTRF_ADDR_SIZE; i > 0; i--)
	{
		dev->Hdr[i] = (uint8_t)(next & 0xFF);
		next >>= 8;
	}

	return Len;
}

static int NvmTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	return NvmTxData(pDev, pData, Len);
}

static int NvmRxData(DevIntrf_t *pIntrf, uint8_t *pBuff, int Len)
{
	NvmDevIntrf_t *dev = (NvmDevIntrf_t *)pIntrf->pDevData;

	if (dev->HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		return 0;
	}

	// The flash is memory mapped, so a read is a copy from the bus address.
	memcpy(pBuff, (const void *)FrameAddr(dev), (size_t)Len);

	// Advance for a continued read, the same way the write side does.
	uintptr_t next = FrameAddr(dev) + (uintptr_t)Len;

	for (int i = NVM_INTRF_ADDR_SIZE; i > 0; i--)
	{
		dev->Hdr[i] = (uint8_t)(next & 0xFF);
		next >>= 8;
	}

	return Len;
}

static void NvmStopRx(DevIntrf_t *) {}
static void NvmStopTx(DevIntrf_t *) {}
static void NvmIntrfDisable(DevIntrf_t *) {}
static void NvmIntrfEnable(DevIntrf_t *) {}
static uint32_t NvmGetRate(DevIntrf_t *) { return 0; }
static uint32_t NvmSetRate(DevIntrf_t *, uint32_t Rate) { return Rate; }
static void NvmPowerOff(DevIntrf_t *) {}
static void *NvmGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

bool NvmIntrf::Init(DevIntrfEvtHandler_t EvtCB, bool bIntEn)
{
	memset(&s_NvmDev, 0, sizeof(s_NvmDev));
	memset(&s_Stat, 0, sizeof(s_Stat));

	// The whole object is the handle: DevIntrf_t is its first member, so a
	// transfer callback casts pDevData back to it and reaches the state
	// without knowing anything about this class.
	s_NvmDev.DevIntrf.pDevData = &s_NvmDev;
	s_NvmDev.DevIntrf.Type = DEVINTRF_TYPE_MEMCTRL;
	s_NvmDev.DevIntrf.Disable = NvmIntrfDisable;
	s_NvmDev.DevIntrf.Enable = NvmIntrfEnable;
	s_NvmDev.DevIntrf.GetRate = NvmGetRate;
	s_NvmDev.DevIntrf.SetRate = NvmSetRate;
	s_NvmDev.DevIntrf.StartRx = NvmStartRx;
	s_NvmDev.DevIntrf.RxData = NvmRxData;
	s_NvmDev.DevIntrf.StopRx = NvmStopRx;
	s_NvmDev.DevIntrf.StartTx = NvmStartTx;
	s_NvmDev.DevIntrf.TxData = NvmTxData;
	s_NvmDev.DevIntrf.TxSrData = NvmTxSrData;
	s_NvmDev.DevIntrf.StopTx = NvmStopTx;
	s_NvmDev.DevIntrf.PowerOff = NvmPowerOff;
	s_NvmDev.DevIntrf.GetHandle = NvmGetHandle;
	s_NvmDev.DevIntrf.MaxRetry = 1;
	s_NvmDev.DevIntrf.EnCnt = 1;

	// Both come from whoever built this, which is what owns the interface
	// configuration. Every operation on this controller finishes inside its
	// call, so bIntEn true only means the driver is told through EvtCB as
	// well; nothing is deferred.
	s_NvmDev.DevIntrf.EvtCB = EvtCB;
	s_NvmDev.DevIntrf.bIntEn = bIntEn;
	atomic_flag_clear(&s_NvmDev.DevIntrf.bBusy);

	return true;
}

// The arbiter hook the header declares. Nothing on this part registers one:
// the ST link layer keeps its state in RAM and does not own the flash. Held
// so a stack that ever does can, and read nowhere until then.
static NvmIntrfArb_t s_pArbiter = nullptr;

void NvmIntrfSetArbiter(NvmIntrfArb_t pArb)
{
	s_pArbiter = pArb;
	(void)s_pArbiter;
}

void NvmIntrfGetStat(NvmIntrfStat_t *pStat)
{
	if (pStat != nullptr)
	{
		*pStat = s_Stat;
	}
}
