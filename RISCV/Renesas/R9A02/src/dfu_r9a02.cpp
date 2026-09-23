/**-------------------------------------------------------------------------
@file	dfu_r9a02.cpp

@brief	DFU target layer for Renesas R9A02G021 (RISC-V): code flash through the
		low power flash sequencer.

The code flash is programmed and erased with the sequencer in code flash
P/E mode, during which it cannot be read: that code runs from RAM, the data
word is read before the mode change and machine interrupts are off. The
application starts at its reset entry, the first word of slot 0.

Not confirmed from the repository: r9a02g021.h gives only the base of the
flash sequencer, so its register map and sequence are taken from the RA2
low power flash (MF4) at the same base; the 2 KB block, the 4 byte program
unit, the sequencer clock being the core clock, and the watchdog reset.
Every status error or timeout makes the call fail.

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
#include "r9a02g021.h"

#include "dfu/dfu_target.h"
#include "dfu_rv.h"

extern "C" uint32_t SystemCoreClock;

/** @addtogroup DFU
  * @{
  */

// ---------------------------------------------------------------------------
// Code flash through the low power flash sequencer (FACI_LP, R9A02_FACI_LP_BASE)
//
// r9a02g021.h gives the block base only. The register offsets, key values,
// mode sequence, command codes and status bits below are those of the RA2
// low power flash of version MF4 (RA2E1, which r9a02g021.h names as the
// closest RA sibling), where the same block sits at the same base. The
// only offset the repository itself uses is FLDWAITR at 0x3FC4
// (system_r9a02.c), which matches that map.
// ---------------------------------------------------------------------------

#define DFU_TGT_FLP_REG8(Off)		(*(volatile uint8_t *)(R9A02_FACI_LP_BASE + (Off)))
#define DFU_TGT_FLP_REG16(Off)		(*(volatile uint16_t *)(R9A02_FACI_LP_BASE + (Off)))

#define DFU_TGT_FPMCR				DFU_TGT_FLP_REG8(0x100)
#define DFU_TGT_FASR				DFU_TGT_FLP_REG8(0x104)
#define DFU_TGT_FSARL				DFU_TGT_FLP_REG16(0x108)
#define DFU_TGT_FSARH				DFU_TGT_FLP_REG16(0x110)
#define DFU_TGT_FCR					DFU_TGT_FLP_REG8(0x114)
#define DFU_TGT_FEARL				DFU_TGT_FLP_REG16(0x118)
#define DFU_TGT_FEARH				DFU_TGT_FLP_REG16(0x120)
#define DFU_TGT_FRESETR				DFU_TGT_FLP_REG8(0x124)
#define DFU_TGT_FSTATR1				DFU_TGT_FLP_REG8(0x12C)
#define DFU_TGT_FWBL0				DFU_TGT_FLP_REG16(0x130)
#define DFU_TGT_FWBH0				DFU_TGT_FLP_REG16(0x138)
#define DFU_TGT_FPR					DFU_TGT_FLP_REG8(0x180)
#define DFU_TGT_FISR				DFU_TGT_FLP_REG8(0x1D8)
#define DFU_TGT_FSTATR2				DFU_TGT_FLP_REG16(0x1F0)
#define DFU_TGT_FENTRYR				DFU_TGT_FLP_REG16(0x3FB0)
#define DFU_TGT_PFBER				DFU_TGT_FLP_REG8(0x3FC8)

#define DFU_TGT_FENTRYR_CF_PE		0xAA01U		// Key, code flash P/E mode
#define DFU_TGT_FENTRYR_READ		0xAA00U		// Key, read mode
#define DFU_TGT_FPR_UNLOCK			0xA5U
#define DFU_TGT_FPMCR_CF_PE			0x02U
#define DFU_TGT_FPMCR_READ			0x08U

#define DFU_TGT_FCR_OPST			0x80U
#define DFU_TGT_FCR_PROGRAM			0x81U
#define DFU_TGT_FCR_ERASE			0x84U

#define DFU_TGT_FSTATR1_FRDY		0x40U
#define DFU_TGT_FSTATR2_ERERR		0x01U
#define DFU_TGT_FSTATR2_PRGERR		0x02U
#define DFU_TGT_FSTATR2_ILGLERR		0x10U

// 2 KB blocks, programmed 4 bytes at a time, from 0.
#define DFU_TGT_WR_UNIT				4U
#define DFU_TGT_ERASE_UNIT			0x800U
#define DFU_TGT_CF_END				R9A02_FLASH_SIZE_MAX

// Waits, in microseconds: mode change settling (tDIS, tMS), then the
// command limits with margin (block erase 355 ms, 4 byte program 1.4 ms).
#define DFU_TGT_TDIS_US				3U
#define DFU_TGT_TMS_US				16U
#define DFU_TGT_ERASE_US			710000UL
#define DFU_TGT_PROGRAM_US			3000UL
#define DFU_TGT_MODE_US				1000UL

// Highest core clock of the part (HOCO 48 MHz).
#define DFU_TGT_MHZ_MAX				48U

// Everything that runs while the code flash is in P/E mode is in RAM, in
// .iram.text, which ResetEntry copies before main.
#define DFU_TGT_RAMFUNC		__attribute__((section(".iram.text.dfu"), noinline))

// Busy wait, about one loop per 4 clocks or more.
DFU_TGT_RAMFUNC static void DfuTgtDelay(uint32_t Us, uint32_t Mhz)
{
	for (uint32_t n = Us * Mhz / 4U + 1U; n > 0; n--)
	{
		__asm volatile("" ::: "memory");
	}
}

DFU_TGT_RAMFUNC static void DfuTgtFpmcr(uint8_t Val)
{
	DFU_TGT_FPR = DFU_TGT_FPR_UNLOCK;
	DFU_TGT_FPMCR = Val;
	DFU_TGT_FPMCR = (uint8_t)~Val;
	DFU_TGT_FPMCR = Val;
}

// Wait for FRDY to read State, bounded. Loop count is per MHz, one loop
// taken as 4 clocks.
DFU_TGT_RAMFUNC static bool DfuTgtWaitFrdy(uint8_t State, uint32_t Us,
										   uint32_t Mhz)
{
	uint32_t n = Us * (Mhz / 4U + 1U);

	while ((DFU_TGT_FSTATR1 & DFU_TGT_FSTATR1_FRDY) != State)
	{
		if (n-- == 0)
		{
			return false;
		}
	}

	return true;
}

DFU_TGT_RAMFUNC static bool DfuTgtPeEnter(uint32_t Mhz)
{
	// Prefetch buffer off while the contents change.
	DFU_TGT_PFBER = 0;

	DFU_TGT_FENTRYR = DFU_TGT_FENTRYR_CF_PE;
	DfuTgtFpmcr(DFU_TGT_FPMCR_CF_PE);
	DfuTgtDelay(DFU_TGT_TDIS_US, Mhz);

	if (DFU_TGT_FENTRYR != (DFU_TGT_FENTRYR_CF_PE & 0xFFU))
	{
		return false;
	}

	// Sequencer clock: MHz - 1 up to 32 MHz, then one step per 2 MHz.
	DFU_TGT_FISR = (uint8_t)(Mhz >= 32U ? (0x1FU + ((Mhz - 32U) >> 1)) & 0x3FU :
								(Mhz - 1U) & 0x1FU);
	DFU_TGT_FASR = 0;	// User area

	return true;
}

DFU_TGT_RAMFUNC static bool DfuTgtPeExit(uint32_t Mhz)
{
	DfuTgtFpmcr(DFU_TGT_FPMCR_READ);
	DfuTgtDelay(DFU_TGT_TMS_US, Mhz);

	DFU_TGT_FENTRYR = DFU_TGT_FENTRYR_READ;

	uint32_t n = DFU_TGT_MODE_US * (Mhz / 4U + 1U);
	while (DFU_TGT_FENTRYR != 0)
	{
		if (n-- == 0)
		{
			return false;
		}
	}

	DFU_TGT_PFBER = 1;

	return true;
}

// Command started through FCR: wait for the end, stop it, check. On a
// timeout or an error the sequencer is reset.
DFU_TGT_RAMFUNC static bool DfuTgtCmd(uint8_t Cmd, uint16_t ErrMask,
									  uint32_t Us, uint32_t Mhz)
{
	DFU_TGT_FCR = Cmd;

	bool ok = DfuTgtWaitFrdy(DFU_TGT_FSTATR1_FRDY, Us, Mhz);

	DFU_TGT_FCR = DFU_TGT_FCR & (uint8_t)~DFU_TGT_FCR_OPST;
	DFU_TGT_FCR = 0;

	ok = DfuTgtWaitFrdy(0, DFU_TGT_MODE_US, Mhz) && ok &&
		 (DFU_TGT_FSTATR2 & ErrMask) == 0;

	if (ok == false)
	{
		DFU_TGT_FRESETR = 1;
		DFU_TGT_FRESETR = 0;
	}

	return ok;
}

// Erase the 2 KB block at Addr.
DFU_TGT_RAMFUNC static bool DfuTgtRamErase(uint32_t Addr, uint32_t Data,
										   uint32_t Mhz)
{
	(void)Data;

	bool ok = DfuTgtPeEnter(Mhz);

	if (ok)
	{
		uint32_t end = Addr + DFU_TGT_ERASE_UNIT - 1U;

		DFU_TGT_FSARH = (uint16_t)(Addr >> 16);
		DFU_TGT_FSARL = (uint16_t)Addr;
		DFU_TGT_FEARH = (uint16_t)(end >> 16);
		DFU_TGT_FEARL = (uint16_t)end;
		ok = DfuTgtCmd(DFU_TGT_FCR_ERASE,
					   DFU_TGT_FSTATR2_ERERR | DFU_TGT_FSTATR2_ILGLERR,
					   DFU_TGT_ERASE_US, Mhz);
	}

	return DfuTgtPeExit(Mhz) && ok;
}

// Program the word Data at Addr.
DFU_TGT_RAMFUNC static bool DfuTgtRamProgram(uint32_t Addr, uint32_t Data,
											 uint32_t Mhz)
{
	bool ok = DfuTgtPeEnter(Mhz);

	if (ok)
	{
		DFU_TGT_FSARH = (uint16_t)(Addr >> 16);
		DFU_TGT_FSARL = (uint16_t)Addr;
		DFU_TGT_FWBL0 = (uint16_t)Data;
		DFU_TGT_FWBH0 = (uint16_t)(Data >> 16);
		ok = DfuTgtCmd(DFU_TGT_FCR_PROGRAM,
					   DFU_TGT_FSTATR2_PRGERR | DFU_TGT_FSTATR2_ILGLERR,
					   DFU_TGT_PROGRAM_US, Mhz);
	}

	return DfuTgtPeExit(Mhz) && ok;
}

// The trap vector and every handler are in the code flash, so machine
// interrupts are off while it is in P/E mode.
static bool DfuTgtRun(bool (*pFunc)(uint32_t, uint32_t, uint32_t),
					  uint32_t Addr, uint32_t Data)
{
	uint32_t mhz = (SystemCoreClock + 999999UL) / 1000000UL;
	uint32_t mstatus;

	// The sequencer clock setting and the waits come from it: refuse a core
	// clock the part cannot run at rather than program with a wrong one.
	if (mhz == 0 || mhz > DFU_TGT_MHZ_MAX)
	{
		return false;
	}

	__asm volatile("csrrci %0, mstatus, 8" : "=r" (mstatus) :: "memory");

	bool ok = pFunc(Addr, Data, mhz);

	__asm volatile("fence rw, rw" ::: "memory");
	if (mstatus & 8U)
	{
		__asm volatile("csrsi mstatus, 8" ::: "memory");
	}

	return ok;
}

// ---------------------------------------------------------------------------
// Internal memory
// ---------------------------------------------------------------------------

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	return Addr < DFU_TGT_CF_END ? DFU_TGT_ERASE_UNIT : 0;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if ((Addr % DFU_TGT_ERASE_UNIT) != 0 || Addr >= DFU_TGT_CF_END)
	{
		return false;
	}

	return DfuTgtRun(DfuTgtRamErase, (uint32_t)Addr, 0);
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr % DFU_TGT_WR_UNIT) != 0 || (Len % DFU_TGT_WR_UNIT) != 0 ||
		Addr >= DFU_TGT_CF_END || Len > DFU_TGT_CF_END - Addr)
	{
		return false;
	}

	const uint8_t *p = (const uint8_t *)pData;

	for (uint32_t o = 0; o < Len; o += 4, p += 4)
	{
		// Read here, before P/E mode: the source may be anywhere, the code
		// flash included, and unaligned. The word then goes in a register.
		uint32_t w = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
					 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);

		if (DfuTgtRun(DfuTgtRamProgram, (uint32_t)(Addr + o), w) == false)
		{
			return false;
		}
	}

	return true;
}

// ---------------------------------------------------------------------------
// Start and reset
// ---------------------------------------------------------------------------

// A RISC-V application here starts at its .reset section, which its linker
// script places at RESET_VECTOR, the start of slot 0: the entry is the first
// word of the payload.
bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return RunAddr < DFU_TGT_CF_END && ImgSize <= DFU_TGT_CF_END - RunAddr &&
		   DfuRvEntryValid(pImg, ImgSize, 0);
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuRvStart(RunAddr);
}

// WDT, RA2 register layout at R9A02_WDT_BASE.
#define DFU_TGT_WDTRR		(*(volatile uint8_t *)(R9A02_WDT_BASE + 0x00))
#define DFU_TGT_WDTCR		(*(volatile uint16_t *)(R9A02_WDT_BASE + 0x02))
#define DFU_TGT_WDTRCR		(*(volatile uint8_t *)(R9A02_WDT_BASE + 0x06))

// No software reset request is known on this core, so the watchdog makes
// one: register start mode (OFS0 as erased), shortest timeout, no window,
// reset on expiry. If the watchdog already runs, from OFS0 or the
// application, it resets the part the same way once it stops being fed.
void DfuTgtReset(void)
{
	DfuRvQuiesce();

	DFU_TGT_WDTCR = 0x3310U;	// Window 0 - 100 %, PCLKB / 4, 1024 cycles
	DFU_TGT_WDTRCR = 0x80U;		// Reset on underflow or refresh error
	DFU_TGT_WDTRR = 0x00U;		// Start
	DFU_TGT_WDTRR = 0xFFU;

	for (;;)
	{
	}
}

/** @} */
