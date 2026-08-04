/**-------------------------------------------------------------------------
@file	nvm_stm32.cpp

@brief	The STM32 on die flash controller as a DeviceIntrf.

		One file serves the families the way rng_stm32.cpp serves the RNG:
		stm32.h selects the device header, and the family differences live
		in a small set of aliases and islands. The frame, the read and the
		staging are the same everywhere; only the controller access and the
		geometry follow from the MCU model.

		Families served and their models, all read from the device headers:

		  WBA5x : 128 bit quad word program, 8 KB pages, NSCR1/NSSR/NSKEYR
		  WBA2x : 64 bit double word program, 4 KB pages, same registers
		  L4    : 64 bit double word program, 2 KB pages, CR/SR/KEYR
		  L4+   : 64 bit double word program, 4 KB pages, dual bank parts

		Not served, deliberately:

		  WB    : the same controller as L4, but the flash is shared with
		          the M0+ radio core behind HSEM arbitration, which is a
		          stack coordination problem this port does not own.
		  F4    : sector erase over non uniform sectors; a store would sit
		          in 128 KB units, parked until a need shows up.
		  F0/F3 : parts too small for a flash store to be worth its pages.

		Every operation finishes inside its call; nothing on these parts
		arbitrates the memory. What an operation does cost is core stall
		when fetching from the bank being written, up to the page erase
		time, which delays every interrupt including a link layer's on the
		wireless parts. Place the storage region in the other bank on a
		dual bank part; the placement is the linker script's business.

		On L4 the flash data and instruction caches are flushed after a
		program or erase, the way the HAL flushes them: a memory mapped
		read through a stale cache line answers with the bytes from before
		the operation, which for a store means reading back data it never
		wrote.

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

#include "stm32.h"

#include "coredev/interrupt.h"
#include "storage/nvm_intrf.h"

#if !defined(IOSONATA_STM32_WBA) && !defined(IOSONATA_STM32_L4)
#error "nvm_stm32: this STM32 family has no NVM controller support"
#endif

// The command set, using the opcodes a serial flash uses so a config for
// internal memory reads like one for a flash chip.
#define NVM_INTRF_CMD_READ		0x03		//!< Read
#define NVM_INTRF_CMD_WRITE		0x02		//!< Program words
#define NVM_INTRF_CMD_ERASE		0x20		//!< Erase one page

// 32 bit addresses, the flash bus address as mapped.
#define NVM_INTRF_ADDR_SIZE		4

// ---------------------------------------------------------------------------
// What follows from the MCU model: the register aliases, the flash word, the
// page, and the family islands the shared bodies below stand on.
// ---------------------------------------------------------------------------
#if defined(IOSONATA_STM32_WBA)

// Every register this port touches is the non secure bank, the same access
// the HAL makes for a non CMSE build. A secure build owns SECCR1/SECSR and
// is not this port's business.
#define ST_FLASH_CR				(FLASH_NS->NSCR1)
#define ST_FLASH_SR				(FLASH_NS->NSSR)
#define ST_FLASH_KEYR			(FLASH_NS->NSKEYR)

#define ST_FLASH_CR_PG			FLASH_NSCR1_PG
#define ST_FLASH_CR_PER			FLASH_NSCR1_PER
#define ST_FLASH_CR_MER			FLASH_NSCR1_MER
#define ST_FLASH_CR_PNB_Pos		FLASH_NSCR1_PNB_Pos
#define ST_FLASH_CR_PNB_Msk		FLASH_NSCR1_PNB_Msk
#define ST_FLASH_CR_STRT		FLASH_NSCR1_STRT
#define ST_FLASH_CR_LOCK		FLASH_NSCR1_LOCK
#ifdef FLASH_NSCR1_BKER
#define ST_FLASH_CR_BKER		FLASH_NSCR1_BKER
#endif

#define ST_FLASH_SR_EOP			FLASH_NSSR_EOP

// Busy covers the operation and the write buffer both; the HAL waits on the
// same pair.
#define ST_FLASH_SR_BUSY		(FLASH_NSSR_BSY | FLASH_NSSR_WDW)

// The error bits one failed operation can raise, write one to clear.
#define ST_FLASH_ERR			(FLASH_NSSR_OPERR | FLASH_NSSR_PROGERR | \
								 FLASH_NSSR_WRPERR | FLASH_NSSR_PGAERR | \
								 FLASH_NSSR_SIZERR | FLASH_NSSR_PGSERR | \
								 FLASH_NSSR_OPTWERR)

// The flash word: what one program operation takes, whole, aligned. The
// device header says which of the two this part is. The driver above pads
// to WriteGran, so a transfer arrives as whole flash words.
#if defined(FLASH_DOUBLEWORD_SUPPORT)
#define NVM_INTRF_WRITE_GRAN	8			//!< 64 bit double word
#define NVM_INTRF_PAGE_SIZE		0x1000UL	//!< 4 KB
#else
#define NVM_INTRF_WRITE_GRAN	16			//!< 128 bit quad word
#define NVM_INTRF_PAGE_SIZE		0x2000UL	//!< 8 KB
#endif

#else	// IOSONATA_STM32_L4

#define ST_FLASH_CR				(FLASH->CR)
#define ST_FLASH_SR				(FLASH->SR)
#define ST_FLASH_KEYR			(FLASH->KEYR)

#define ST_FLASH_CR_PG			FLASH_CR_PG
#define ST_FLASH_CR_PER			FLASH_CR_PER
#define ST_FLASH_CR_MER			(FLASH_CR_MER1)
#define ST_FLASH_CR_PNB_Pos		FLASH_CR_PNB_Pos
#define ST_FLASH_CR_PNB_Msk		FLASH_CR_PNB_Msk
#define ST_FLASH_CR_STRT		FLASH_CR_STRT
#define ST_FLASH_CR_LOCK		FLASH_CR_LOCK
#ifdef FLASH_CR_BKER
#define ST_FLASH_CR_BKER		FLASH_CR_BKER
#endif

#define ST_FLASH_SR_EOP			FLASH_SR_EOP
#define ST_FLASH_SR_BUSY		(FLASH_SR_BSY)

// Every error bit this family's header defines. RDERR only exists where
// PCROP does, so it joins under its own test.
#ifdef FLASH_SR_RDERR
#define ST_FLASH_ERR_RD			FLASH_SR_RDERR
#else
#define ST_FLASH_ERR_RD			0
#endif
#define ST_FLASH_ERR			(FLASH_SR_OPERR | FLASH_SR_PROGERR | \
								 FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
								 FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
								 FLASH_SR_MISERR | FLASH_SR_FASTERR | \
								 ST_FLASH_ERR_RD)

#define NVM_INTRF_WRITE_GRAN	8			//!< 64 bit double word

// 2 KB pages on classic L4, 4 KB on the L4+ parts, the same part list the
// HAL keys its FLASH_PAGE_SIZE on. An L4+ part with the DBANK option off
// runs 8 KB pages in 128 bit mode, which this first cut does not read from
// the option bytes; note it at bring up if such a part turns up.
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx) || defined(STM32L4R5xx) || \
	defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || \
	defined(STM32L4S7xx) || defined(STM32L4S9xx)
#define NVM_INTRF_PAGE_SIZE		0x1000UL	//!< 4 KB
#else
#define NVM_INTRF_PAGE_SIZE		0x0800UL	//!< 2 KB
#endif

#endif	// family

// Total flash, from the device's own size register: a KB count, present on
// both families at their FLASHSIZE_BASE.
static inline uint32_t StFlashSize(void)
{
	uint32_t kb = *(const volatile uint16_t *)FLASHSIZE_BASE;

#if defined(IOSONATA_STM32_WBA)
	// All ones or zero means the full 1 MB, the same reading the HAL makes.
	if (kb == 0xFFFFUL || kb == 0UL)
	{
		return 0x100000UL;
	}
#endif

	return kb << 10;
}

#ifdef ST_FLASH_CR_BKER
// Bank size on a dual bank part. On L4 the classic dual bank parts split
// the device in half, the same arithmetic the HAL uses.
#define ST_FLASH_BANK_SIZE()	(StFlashSize() / 2UL)
#endif

// Largest bytes one transfer stages. A multiple of both flash words so one
// number serves every family here.
#ifndef NVM_INTRF_MAX_XFER
#define NVM_INTRF_MAX_XFER		64
#endif

static_assert(NVM_INTRF_MAX_XFER <= NVM_INTRF_XFER_SIZE,
			  "NVM_INTRF_MAX_XFER exceeds the staging buffer");
static_assert((NVM_INTRF_MAX_XFER % NVM_INTRF_WRITE_GRAN) == 0,
			  "NVM_INTRF_MAX_XFER is not whole flash words");

// A bound so a stuck controller cannot hang the caller. Loop passes, not a
// duration, the way the Nordic port bounds its spin.
#ifndef NVM_INTRF_SPIN
#define NVM_INTRF_SPIN			0x00800000UL
#endif

// ---------------------------------------------------------------------------
// Per controller state. One memory controller, so these describe the
// hardware rather than a transaction, the same reasoning as the Nordic port.
// ---------------------------------------------------------------------------

static NvmIntrfStat_t s_Stat;

static NvmDevIntrf_t s_NvmDev;

DevIntrf_t *NvmMcuDevIntrf(void)
{
	return &s_NvmDev.DevIntrf;
}

// ---------------------------------------------------------------------------
// The controller
// ---------------------------------------------------------------------------

// Bounded, so a controller that never settles reports instead of hanging.
static bool StFlashWait(void)
{
	uint32_t spin = NVM_INTRF_SPIN;

	while (ST_FLASH_SR & ST_FLASH_SR_BUSY)
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
static int StFlashResult(void)
{
	uint32_t sr = ST_FLASH_SR;

	if (sr & ST_FLASH_SR_EOP)
	{
		ST_FLASH_SR = ST_FLASH_SR_EOP;
	}

	if (sr & ST_FLASH_ERR)
	{
		ST_FLASH_SR = sr & ST_FLASH_ERR;

		return -EIO;
	}

	return 0;
}

static void StFlashUnlock(void)
{
	if (ST_FLASH_CR & ST_FLASH_CR_LOCK)
	{
		ST_FLASH_KEYR = 0x45670123UL;
		ST_FLASH_KEYR = 0xCDEF89ABUL;
	}
}

static void StFlashLock(void)
{
	ST_FLASH_CR |= ST_FLASH_CR_LOCK;
}

// What runs after the medium changed under the caches. A memory mapped read
// through a stale line answers with the bytes from before the operation,
// which for a store means reading back data it never wrote. WBA fetches
// through its ICACHE peripheral, which snoops nothing either, but data
// reads there do not cache flash, so only L4 has work to do here.
static void StFlashCacheFlush(void)
{
#if defined(IOSONATA_STM32_L4)
	uint32_t acr = FLASH->ACR;

	if (acr & FLASH_ACR_DCEN)
	{
		FLASH->ACR = acr & ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;
	}
	if (acr & FLASH_ACR_ICEN)
	{
		FLASH->ACR = FLASH->ACR & ~FLASH_ACR_ICEN;
		FLASH->ACR |= FLASH_ACR_ICRST;
		FLASH->ACR &= ~FLASH_ACR_ICRST;
		FLASH->ACR |= FLASH_ACR_ICEN;
	}
#endif
}

// Program whole flash words at Addr. Addr is the mapped bus address, whole
// flash word aligned; the driver splits and pads above, so anything else
// here is refused rather than trimmed.
//
// The word's 32 bit writes go out back to back with interrupts held, the
// way the HAL programs the same word: a fetch from this bank between them
// stalls anyway, and an interrupt writing flash mid word raises PGSERR.
static int StFlashProgram(uintptr_t Addr, const uint8_t *pData, uint32_t Len)
{
	if ((Addr % NVM_INTRF_WRITE_GRAN) != 0 ||
		(Len % NVM_INTRF_WRITE_GRAN) != 0)
	{
		return -EINVAL;
	}

	if (StFlashWait() == false)
	{
		return -ETIMEDOUT;
	}

	// Stale error bits fail the sequence check of the operation being
	// started, so they go before PG is set.
	ST_FLASH_SR = ST_FLASH_SR & ST_FLASH_ERR;

	StFlashUnlock();
	ST_FLASH_CR |= ST_FLASH_CR_PG;

	int res = 0;
	volatile uint32_t *dst = (volatile uint32_t *)Addr;
	const uint8_t *src = pData;

	for (uint32_t left = Len; left > 0; left -= NVM_INTRF_WRITE_GRAN)
	{
		// The controller collects the word's bus writes and wants the flash
		// word whole; an interrupt touching the flash between them raises
		// PGSERR or commits a partial word. Held only for the word's few
		// stores; the busy wait below runs with interrupts live.
		uint32_t state = DisableInterrupt();

		for (int i = 0; i < NVM_INTRF_WRITE_GRAN / 4; i++)
		{
			uint32_t w;

			memcpy(&w, src, sizeof(w));
			*dst++ = w;
			src += sizeof(w);
		}

		EnableInterrupt(state);

		if (StFlashWait() == false)
		{
			res = -ETIMEDOUT;
			break;
		}

		res = StFlashResult();
		if (res != 0)
		{
			break;
		}
	}

	ST_FLASH_CR &= ~ST_FLASH_CR_PG;
	StFlashLock();

	StFlashCacheFlush();

	return res;
}

// ---------------------------------------------------------------------------
// What the port knows about the memory itself. The C linkage of NvmMcuErase,
// NvmMcuIsReady and NvmMcuCeiling comes from their declarations in
// nvm_intrf.h; the definitions inherit it, so nothing is marked here.
// ---------------------------------------------------------------------------

// Erase the page holding Addr. Addr is the mapped bus address, page aligned;
// the driver only asks at the erase unit it was configured with.
int NvmMcuErase(uintptr_t Addr)
{
	uint32_t size = StFlashSize();

	if (Addr < FLASH_BASE || (Addr % NVM_INTRF_PAGE_SIZE) != 0 ||
		Addr + NVM_INTRF_PAGE_SIZE > FLASH_BASE + size)
	{
		return -EINVAL;
	}

	if (StFlashWait() == false)
	{
		return -ETIMEDOUT;
	}

	ST_FLASH_SR = ST_FLASH_SR & ST_FLASH_ERR;

	uint32_t page = (uint32_t)((Addr - FLASH_BASE) / NVM_INTRF_PAGE_SIZE);
	uint32_t cr = ST_FLASH_CR;

	cr &= ~(ST_FLASH_CR_PNB_Msk | ST_FLASH_CR_PG | ST_FLASH_CR_MER);

#ifdef ST_FLASH_CR_BKER
	// The page number counts within the bank; which bank is a bit of its
	// own.
	uint32_t bank = ST_FLASH_BANK_SIZE();

	if ((Addr - FLASH_BASE) >= bank)
	{
		page -= bank / NVM_INTRF_PAGE_SIZE;
		cr |= ST_FLASH_CR_BKER;
	}
	else
	{
		cr &= ~ST_FLASH_CR_BKER;
	}
#endif

	cr |= (page << ST_FLASH_CR_PNB_Pos) | ST_FLASH_CR_PER;

	StFlashUnlock();
	ST_FLASH_CR = cr;
	ST_FLASH_CR = cr | ST_FLASH_CR_STRT;

	int res = StFlashWait() ? StFlashResult() : -ETIMEDOUT;

	ST_FLASH_CR &= ~(ST_FLASH_CR_PER | ST_FLASH_CR_PNB_Msk);
	StFlashLock();

	StFlashCacheFlush();

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
	return (ST_FLASH_SR & ST_FLASH_SR_BUSY) == 0;
}

void NvmMcuCfg(NvmCfg_t &Cfg)
{
	Cfg.DevNo = 0;

	// The flash is mapped at FLASH_BASE, not at 0, so the base goes in here
	// and every address this port sees is the mapped bus address. A creator
	// that reserves a region through the linker script puts that region's
	// base and size in instead, the way the Nordic bond stores do.
	Cfg.BaseAddr = FLASH_BASE;
	Cfg.TotalSize = StFlashSize();
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
	return (uint64_t)FLASH_BASE + StFlashSize();
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

	int res = StFlashProgram(FrameAddr(pXfer), pXfer->Data,
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

// The arbiter hook the header declares. Nothing on these parts registers
// one; held so a stack that ever does can, and read nowhere until then.
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
