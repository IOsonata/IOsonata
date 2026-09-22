/**-------------------------------------------------------------------------
@file	dfu_stm32.cpp

@brief	DFU target layer for STM32: F0, F4, L4, L4+ and WBA on die flash.

One file serves the families the way nvm_stm32.cpp does: stm32.h selects
the device header and the family differences are a few aliases and the
erase and program sequences, all taken from the device headers.

	family	program unit				erase unit
	F0		16 bit half word			1 KB page (2 KB on F030xC, F07x, F09x)
	F4		32 bit word, PSIZE x32		sector: 16, 16, 16, 16, 64, 128... KB
	L4		64 bit double word, ECC		2 KB page
	L4+		64 bit double word, ECC		4 KB page, 8 KB with DBANK off
	WBA5x	128 bit quad word, ECC		8 KB page
	WBA2x	64 bit double word, ECC		4 KB page

On the ECC parts a program unit takes one program between two erases; the
generic layer never programs one twice. A unit whose data is all ones is
not programmed at all, so it stays erased.

F4 programs 32 bits at a time, which needs VDD from 2.7 V to 3.6 V. A board
running the part lower needs PSIZE x8 or x16, which this file does not do.

The core stalls on a flash fetch while the controller works, on every part
here, so the code runs from flash: nothing needs to be in RAM. The source
of a write may be in flash; each unit is read into registers before the
program starts.

Cortex-M0 (F0) has no VTOR: the application vector table is copied to the
start of SRAM and SRAM is remapped at 0 (SYSCFG MEM_MODE). Its linker script
leaves those bytes alone.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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

#include "stm32.h"

#include "coredev/interrupt.h"
#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

#if !defined(IOSONATA_STM32_F0) && !defined(IOSONATA_STM32_F4) && \
	!defined(IOSONATA_STM32_L4) && !defined(IOSONATA_STM32_WBA)
#error "dfu_stm32: this STM32 family has no DFU target support"
#endif

// ---------------------------------------------------------------------------
// What follows from the MCU model: register aliases, status bits, program
// unit and page size. The L4 and WBA sets are those of nvm_stm32.cpp.
// ---------------------------------------------------------------------------

#if defined(IOSONATA_STM32_WBA)

// Non secure bank registers, the access a non CMSE build makes.
#define ST_FLASH_CR				(FLASH_NS->NSCR1)
#define ST_FLASH_SR				(FLASH_NS->NSSR)
#define ST_FLASH_KEYR			(FLASH_NS->NSKEYR)

#define ST_FLASH_CR_PG			FLASH_NSCR1_PG
#define ST_FLASH_CR_PER			FLASH_NSCR1_PER
#ifdef FLASH_NSCR1_MER
#define ST_FLASH_CR_MER			FLASH_NSCR1_MER
#else
#define ST_FLASH_CR_MER			(FLASH_NSCR1_MER1 | FLASH_NSCR1_MER2)
#endif
#define ST_FLASH_CR_PNB_Pos		FLASH_NSCR1_PNB_Pos
#define ST_FLASH_CR_PNB_Msk		FLASH_NSCR1_PNB_Msk
#define ST_FLASH_CR_STRT		FLASH_NSCR1_STRT
#define ST_FLASH_CR_LOCK		FLASH_NSCR1_LOCK
#ifdef FLASH_NSCR1_BKER
#define ST_FLASH_CR_BKER		FLASH_NSCR1_BKER
#endif

#define ST_FLASH_SR_EOP			FLASH_NSSR_EOP
#define ST_FLASH_SR_BUSY		(FLASH_NSSR_BSY | FLASH_NSSR_WDW)
#define ST_FLASH_ERR			(FLASH_NSSR_OPERR | FLASH_NSSR_PROGERR | \
								 FLASH_NSSR_WRPERR | FLASH_NSSR_PGAERR | \
								 FLASH_NSSR_SIZERR | FLASH_NSSR_PGSERR | \
								 FLASH_NSSR_OPTWERR)
#define ST_FLASH_CLR			ST_FLASH_ERR

#if defined(FLASH_DOUBLEWORD_SUPPORT)
#define DFU_TGT_WR_UNIT			8U			//!< 64 bit double word
#define DFU_TGT_PAGE			0x1000UL	//!< 4 KB
#else
#define DFU_TGT_WR_UNIT			16U			//!< 128 bit quad word
#define DFU_TGT_PAGE			0x2000UL	//!< 8 KB
#endif

#elif defined(IOSONATA_STM32_L4)

#define ST_FLASH_CR				(FLASH->CR)
#define ST_FLASH_SR				(FLASH->SR)
#define ST_FLASH_KEYR			(FLASH->KEYR)

#define ST_FLASH_CR_PG			FLASH_CR_PG
#define ST_FLASH_CR_PER			FLASH_CR_PER
#define ST_FLASH_CR_MER			FLASH_CR_MER1
#define ST_FLASH_CR_PNB_Pos		FLASH_CR_PNB_Pos
#define ST_FLASH_CR_PNB_Msk		FLASH_CR_PNB_Msk
#define ST_FLASH_CR_STRT		FLASH_CR_STRT
#define ST_FLASH_CR_LOCK		FLASH_CR_LOCK
#ifdef FLASH_CR_BKER
#define ST_FLASH_CR_BKER		FLASH_CR_BKER
#endif

#define ST_FLASH_SR_EOP			FLASH_SR_EOP
#define ST_FLASH_SR_BUSY		FLASH_SR_BSY
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
// OPTVERR is cleared with the others before an operation, as the HAL does,
// but it says nothing about the operation itself.
#define ST_FLASH_CLR			(ST_FLASH_ERR | FLASH_SR_OPTVERR)

#define DFU_TGT_WR_UNIT			8U			//!< 64 bit double word

// L4+ pages are 4 KB with DBANK set, 8 KB in single bank mode.
#if defined(FLASH_OPTR_DBANK)
#define DFU_TGT_PAGE			(FLASH->OPTR & FLASH_OPTR_DBANK ? \
								 0x1000UL : 0x2000UL)
#else
#define DFU_TGT_PAGE			0x0800UL	//!< 2 KB
#endif

#elif defined(IOSONATA_STM32_F4)

#define ST_FLASH_CR				(FLASH->CR)
#define ST_FLASH_SR				(FLASH->SR)
#define ST_FLASH_KEYR			(FLASH->KEYR)

#define ST_FLASH_CR_PG			FLASH_CR_PG
#define ST_FLASH_CR_STRT		FLASH_CR_STRT
#define ST_FLASH_CR_LOCK		FLASH_CR_LOCK

#define ST_FLASH_SR_EOP			FLASH_SR_EOP
#define ST_FLASH_SR_BUSY		FLASH_SR_BSY
#ifdef FLASH_SR_RDERR
#define ST_FLASH_ERR_RD			FLASH_SR_RDERR
#else
#define ST_FLASH_ERR_RD			0
#endif
#define ST_FLASH_ERR			(FLASH_SR_OPERR | FLASH_SR_WRPERR | \
								 FLASH_SR_PGAERR | FLASH_SR_PGPERR | \
								 FLASH_SR_PGSERR | ST_FLASH_ERR_RD)
#define ST_FLASH_CLR			ST_FLASH_ERR

// 32 bit parallelism: word program, VDD 2.7 V to 3.6 V.
#define ST_FLASH_CR_PSIZE_X32	FLASH_CR_PSIZE_1

#define DFU_TGT_WR_UNIT			4U

#else	// IOSONATA_STM32_F0

#define ST_FLASH_CR				(FLASH->CR)
#define ST_FLASH_SR				(FLASH->SR)
#define ST_FLASH_KEYR			(FLASH->KEYR)

#define ST_FLASH_CR_PG			FLASH_CR_PG
#define ST_FLASH_CR_PER			FLASH_CR_PER
#define ST_FLASH_CR_STRT		FLASH_CR_STRT
#define ST_FLASH_CR_LOCK		FLASH_CR_LOCK

#define ST_FLASH_SR_EOP			FLASH_SR_EOP
#define ST_FLASH_SR_BUSY		FLASH_SR_BSY
#define ST_FLASH_ERR			(FLASH_SR_PGERR | FLASH_SR_WRPRTERR)
#define ST_FLASH_CLR			ST_FLASH_ERR

#define DFU_TGT_WR_UNIT			2U			//!< 16 bit half word

// The HAL's FLASH_PAGE_SIZE part list.
#if defined(STM32F030xC) || defined(STM32F070xB) || defined(STM32F071xB) || \
	defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
	defined(STM32F098xx)
#define DFU_TGT_PAGE			0x0800UL	//!< 2 KB
#else
#define DFU_TGT_PAGE			0x0400UL	//!< 1 KB
#endif

// Every STM32F0 vector table: 16 core entries and 32 interrupt entries. The
// application script leaves this much at the start of SRAM.
#define DFU_TGT_F0_VEC_SIZE		0xC0U

#endif

static_assert(DFU_TGT_WR_UNIT <= DFU_TGT_WRITE_UNIT_MAX, "program unit");

// A bound so a stuck controller cannot hang the boot. Loop passes, sized for
// the longest operation here, a 128 KB F4 sector erase (2 s at x32), at the
// fastest core clock.
#ifndef DFU_TGT_SPIN
#define DFU_TGT_SPIN			0x10000000UL
#endif

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

// Total flash from the device's size register, a KB count.
static inline uint32_t StFlashSize(void)
{
#if defined(IOSONATA_STM32_WBA)
	// The header's reading: all ones or zero means the part's full size.
	return FLASH_SIZE;
#else
	return (uint32_t)(*(const volatile uint16_t *)FLASHSIZE_BASE) << 10;
#endif
}

static inline bool StFlashIn(uintptr_t Addr, uint32_t Len)
{
	uint32_t size = StFlashSize();

	return Addr >= FLASH_BASE && Addr - FLASH_BASE <= size &&
		   Len <= size - (Addr - FLASH_BASE);
}

#if defined(IOSONATA_STM32_F4)

// F4 sectors, from the start of a bank: four of 16 KB, one of 64 KB, then
// 128 KB ones. A 2 MB part repeats the pattern in bank 2 from 1 MB, whose
// sectors the controller numbers from 16 (SNB bit 4). The DB1M option of
// the 1 MB F42x/F43x parts, which splits them in two banks, is not read.
static uint32_t StF4Sector(uintptr_t Addr, uint32_t *pSnb)
{
	if (StFlashIn(Addr, 1) == false)
	{
		return 0;
	}

	uint32_t off = (uint32_t)(Addr - FLASH_BASE);
	uint32_t snb = 0;

	if (off >= 0x100000UL)
	{
		off -= 0x100000UL;
		snb = 16;
	}

	uint32_t size;

	if (off < 0x10000UL)
	{
		snb += off / 0x4000UL;
		size = 0x4000UL;
	}
	else if (off < 0x20000UL)
	{
		snb += 4;
		size = 0x10000UL;
	}
	else
	{
		snb += 5 + (off - 0x20000UL) / 0x20000UL;
		size = 0x20000UL;
	}

	if (pSnb != nullptr)
	{
		*pSnb = snb;
	}

	return size;
}

#else

#ifdef ST_FLASH_CR_BKER
// Bank size, 0 on a part or in a mode with one bank.
static uint32_t StBankSize(void)
{
#if defined(FLASH_OPTR_DBANK)
	// L4+: two banks only with DBANK set.
	return (FLASH->OPTR & FLASH_OPTR_DBANK) ? StFlashSize() / 2 : 0;
#elif defined(FLASH_OPTR_DUALBANK)
	// L47x, L48x, L49x, L4Ax: 1 MB parts always have two banks, smaller ones
	// when DUALBANK is set.
	uint32_t size = StFlashSize();

	return size >= 0x100000UL || (FLASH->OPTR & FLASH_OPTR_DUALBANK) ?
		   size / 2 : 0;
#else
	return StFlashSize() / 2;
#endif
}

// Whether the banks are swapped: the second physical bank is mapped first.
static inline bool StBankSwapped(void)
{
#if defined(SYSCFG_MEMRMP_FB_MODE)
	return (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) != 0;
#elif defined(FLASH_OPTR_SWAP_BANK)
	return (FLASH->OPTR & FLASH_OPTR_SWAP_BANK) != 0;
#else
	return false;
#endif
}
#endif

#endif

// ---------------------------------------------------------------------------
// The controller
// ---------------------------------------------------------------------------

static bool StFlashWait(void)
{
	uint32_t spin = DFU_TGT_SPIN;

	while (ST_FLASH_SR & ST_FLASH_SR_BUSY)
	{
		if (--spin == 0)
		{
			return false;
		}
	}

	return true;
}

// Outcome of the operation that just ran. Status bits are write one to
// clear, EOP included.
static bool StFlashResult(void)
{
	uint32_t sr = ST_FLASH_SR;

	if (sr & ST_FLASH_SR_EOP)
	{
		ST_FLASH_SR = ST_FLASH_SR_EOP;
	}

	if (sr & ST_FLASH_ERR)
	{
		ST_FLASH_SR = sr & ST_FLASH_ERR;

		return false;
	}

	return true;
}

// Ready for an operation: idle, stale errors cleared (they fail the sequence
// check of the next operation), unlocked.
static bool StFlashBegin(void)
{
	if (StFlashWait() == false)
	{
		return false;
	}

	ST_FLASH_SR = ST_FLASH_SR & ST_FLASH_CLR;

	if (ST_FLASH_CR & ST_FLASH_CR_LOCK)
	{
		ST_FLASH_KEYR = 0x45670123UL;
		ST_FLASH_KEYR = 0xCDEF89ABUL;
	}

	return (ST_FLASH_CR & ST_FLASH_CR_LOCK) == 0;
}

static void StFlashEnd(void)
{
	ST_FLASH_CR |= ST_FLASH_CR_LOCK;
}

// The mapped view after the memory changed under it. F4 and L4 flash
// instruction and data caches are reset the way the HAL's FLASH_FlushCaches
// does it, each disabled while it is reset. WBA fetches and reads flash
// through ICACHE, which is invalidated. F0 has only a prefetch buffer.
static void StFlashSync(void)
{
#if defined(IOSONATA_STM32_F4) || defined(IOSONATA_STM32_L4)
	uint32_t acr = FLASH->ACR;

	if (acr & FLASH_ACR_ICEN)
	{
		FLASH->ACR &= ~FLASH_ACR_ICEN;
		FLASH->ACR |= FLASH_ACR_ICRST;
		FLASH->ACR &= ~FLASH_ACR_ICRST;
		FLASH->ACR |= FLASH_ACR_ICEN;
	}
	if (acr & FLASH_ACR_DCEN)
	{
		FLASH->ACR &= ~FLASH_ACR_DCEN;
		FLASH->ACR |= FLASH_ACR_DCRST;
		FLASH->ACR &= ~FLASH_ACR_DCRST;
		FLASH->ACR |= FLASH_ACR_DCEN;
	}
#elif defined(IOSONATA_STM32_WBA)
	if (ICACHE->CR & ICACHE_CR_EN)
	{
		uint32_t spin = DFU_TGT_SPIN;

		ICACHE->CR |= ICACHE_CR_CACHEINV;
		while ((ICACHE->SR & ICACHE_SR_BUSYF) && --spin != 0)
		{
		}
	}
#endif
	__DSB();
	__ISB();
}

// ---------------------------------------------------------------------------
// Target API
// ---------------------------------------------------------------------------

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
#if defined(IOSONATA_STM32_F4)
	return StF4Sector(Addr, nullptr);
#else
	return StFlashIn(Addr, 1) ? (uint32_t)DFU_TGT_PAGE : 0;
#endif
}

bool DfuTgtErase(uintptr_t Addr)
{
	uint32_t unit = DfuTgtEraseUnit(Addr);

	if (unit == 0 || ((Addr - FLASH_BASE) % unit) != 0 ||
		StFlashBegin() == false)
	{
		return false;
	}

#if defined(IOSONATA_STM32_F0)
	ST_FLASH_CR |= ST_FLASH_CR_PER;
	FLASH->AR = (uint32_t)Addr;
	ST_FLASH_CR |= ST_FLASH_CR_STRT;

	bool ok = StFlashWait() && StFlashResult();

	ST_FLASH_CR &= ~ST_FLASH_CR_PER;
#elif defined(IOSONATA_STM32_F4)
	uint32_t snb;

	(void)StF4Sector(Addr, &snb);

	uint32_t cr = ST_FLASH_CR & ~(FLASH_CR_PSIZE | FLASH_CR_SNB | FLASH_CR_PG |
								  FLASH_CR_MER);

	cr |= ST_FLASH_CR_PSIZE_X32 | FLASH_CR_SER | (snb << FLASH_CR_SNB_Pos);
	ST_FLASH_CR = cr;
	ST_FLASH_CR = cr | ST_FLASH_CR_STRT;

	bool ok = StFlashWait() && StFlashResult();

	ST_FLASH_CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
#else	// L4, WBA
	uint32_t off = (uint32_t)(Addr - FLASH_BASE);
	uint32_t cr = ST_FLASH_CR & ~(ST_FLASH_CR_PNB_Msk | ST_FLASH_CR_PG |
								  ST_FLASH_CR_MER);

#ifdef ST_FLASH_CR_BKER
	// The page number counts within its bank, the bank is a bit of its own,
	// physical: with the banks swapped the first mapped half is bank 2.
	uint32_t bank = StBankSize();
	bool second = false;

	if (bank != 0)
	{
		second = off >= bank;
		if (second)
		{
			off -= bank;
		}
		if (StBankSwapped())
		{
			second = !second;
		}
	}
	if (second)
	{
		cr |= ST_FLASH_CR_BKER;
	}
	else
	{
		cr &= ~ST_FLASH_CR_BKER;
	}
#endif

	cr |= ((off / unit) << ST_FLASH_CR_PNB_Pos) | ST_FLASH_CR_PER;
	ST_FLASH_CR = cr;
	ST_FLASH_CR = cr | ST_FLASH_CR_STRT;

	bool ok = StFlashWait() && StFlashResult();

	ST_FLASH_CR &= ~(ST_FLASH_CR_PER | ST_FLASH_CR_PNB_Msk);
#endif

	StFlashEnd();
	StFlashSync();

	return ok;
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr % DFU_TGT_WR_UNIT) != 0 || (Len % DFU_TGT_WR_UNIT) != 0 ||
		pData == nullptr || StFlashIn(Addr, Len) == false)
	{
		return false;
	}

	if (Len == 0)
	{
		return true;
	}

	if (StFlashBegin() == false)
	{
		return false;
	}

#if defined(IOSONATA_STM32_F4)
	ST_FLASH_CR = (ST_FLASH_CR & ~FLASH_CR_PSIZE) | ST_FLASH_CR_PSIZE_X32;
#endif
	ST_FLASH_CR |= ST_FLASH_CR_PG;

	const uint8_t *src = (const uint8_t *)pData;
	bool ok = true;

	for (uint32_t off = 0; off < Len && ok; off += DFU_TGT_WR_UNIT)
	{
		// Into registers first: the source may be unaligned, or in flash,
		// which must not be read between the stores of one unit.
		uint32_t w[(DFU_TGT_WR_UNIT + 3) / 4];
		bool ones = true;

		w[0] = 0xFFFFFFFFUL;
		memcpy(w, src + off, DFU_TGT_WR_UNIT);
		for (uint32_t i = 0; i < sizeof(w) / 4; i++)
		{
			ones &= w[i] == 0xFFFFFFFFUL;
		}
		if (ones)
		{
			// Erased already. Left unprogrammed, so an ECC unit stays open.
			continue;
		}

#if DFU_TGT_WR_UNIT == 2
		*(volatile uint16_t *)(Addr + off) = (uint16_t)w[0];
#elif DFU_TGT_WR_UNIT == 4
		*(volatile uint32_t *)(Addr + off) = w[0];
#else
		// The controller takes the unit as back to back word stores; an
		// interrupt touching flash between them raises PGSERR. Held for the
		// stores only, the wait below runs with interrupts live.
		volatile uint32_t *dst = (volatile uint32_t *)(Addr + off);
		uint32_t state = DisableInterrupt();

		for (uint32_t i = 0; i < DFU_TGT_WR_UNIT / 4; i++)
		{
			dst[i] = w[i];
		}

		EnableInterrupt(state);
#endif

		ok = StFlashWait() && StFlashResult();
	}

	ST_FLASH_CR &= ~ST_FLASH_CR_PG;
	StFlashEnd();
	StFlashSync();

	return ok && memcmp((const void *)Addr, pData, Len) == 0;
}

// ---------------------------------------------------------------------------
// Start and reset
// ---------------------------------------------------------------------------

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	if (DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_CM_SRAM_START,
						DFU_CM_SRAM_END))
	{
		return true;
	}

#if defined(IOSONATA_STM32_L4)
	// L4 SRAM2 has its own address, where an application may put its stack.
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, SRAM2_BASE,
						   SRAM2_BASE + SRAM2_SIZE);
#else
	return false;
#endif
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();

#if DFU_CM_HAS_VTOR
	DfuCmStart((const uint32_t *)RunAddr, true);
#else
	// The vectors at 0 are fetched from wherever MEM_MODE maps there: the
	// application table goes to the start of SRAM, and SRAM goes to 0. What
	// the boot had in that RAM is not used again.
	const volatile uint32_t *vec = (const volatile uint32_t *)RunAddr;
	volatile uint32_t *ram = (volatile uint32_t *)SRAM_BASE;

	for (uint32_t i = 0; i < DFU_TGT_F0_VEC_SIZE / 4; i++)
	{
		ram[i] = vec[i];
	}

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	(void)RCC->APB2ENR;
	SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) |
					SYSCFG_CFGR1_MEM_MODE;

	DfuCmStart((const uint32_t *)RunAddr, false);
#endif
}

/** @} */
