/**-------------------------------------------------------------------------
@file	dfu_sam4e.cpp

@brief	DFU target layer for Microchip SAM4E: EEFC.

One 1 MB flash plane, 512 byte pages. A page is programmed through the
latch buffer: the page is written word by word at its own addresses, then
the write page command programs it. Erase is by groups of 16 pages (8 KB,
one lock region), which is the erase unit here: the 4 page group only
exists in the 8 KB sectors and the 32 page group not in them.

The flash cannot be read while a command runs and the code runs from it,
so the latch load and the command run from RAM (.fastrun, which the
startup copies with .data), interrupts off. The source of a write may be
the flash itself, so each page goes through a RAM buffer first.

The chip starts from the flash at 0x400000 when GPNVM bit 1 is set, which
is where the boot is linked.

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
#include <string.h>

#include "sam4e.h"
#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_TGT_PAGE			IFLASH_PAGE_SIZE			//!< 512
#define DFU_TGT_EPA_PAGES		16
#define DFU_TGT_ERASE_UNIT		(DFU_TGT_PAGE * DFU_TGT_EPA_PAGES)	//!< 8 KB
#define DFU_TGT_EPA_16			2		//!< EPA FARG[1:0], 16 pages

// Status of a command that went through.
#define DFU_TGT_FSR_ERR			(EEFC_FSR_FCMDE | EEFC_FSR_FLOCKE | EEFC_FSR_FLERR)

// Polls of FRDY before a command is given up. A 16 page erase is the
// longest command used, some tens of ms; this is seconds at any clock.
#define DFU_TGT_TIMEOUT			0x4000000UL

// One page, word aligned, what the latch buffer is loaded from.
static uint32_t s_DfuTgtPage[DFU_TGT_PAGE / 4];

// ---------------------------------------------------------------------------
// Internal memory
// ---------------------------------------------------------------------------

/**
 * @brief	Run one EEFC command, from RAM.
 *
 * Nothing here may touch the flash: no call out, no literal read from it.
 * The code cache is off meanwhile and invalidated before it goes back on,
 * so what it holds of the pages changed is dropped. The wait states are
 * raised to what the part needs for a program operation for the time of
 * the command.
 *
 * @param	Fcr  : EEFC_FCR value, key included.
 * @param	pDst : Page to load the latch buffer at, with pSrc.
 * @param	pSrc : One page of words, or null for a command with no data.
 *
 * @return	EEFC_FSR bits seen while the command ran, FRDY clear on timeout.
 */
__attribute__((section(".fastrun"), noinline, long_call))
static uint32_t DfuTgtCmd(uint32_t Fcr, volatile uint32_t *pDst,
						  const uint32_t *pSrc)
{
	Sam4eEfc *efc = SAM4E_EFC;
	Sam4eCmcc *cmcc = SAM4E_CMCC;
	uint32_t pm = __get_PRIMASK();
	uint32_t fsr = 0;
	uint32_t n = DFU_TGT_TIMEOUT;

	__disable_irq();

	bool cache = (cmcc->CMCC_SR & CMCC_SR_CSTS) != 0;
	if (cache)
	{
		cmcc->CMCC_CTRL = 0;
		while ((cmcc->CMCC_SR & CMCC_SR_CSTS) != 0)
		{
		}
	}

	uint32_t fmr = efc->EEFC_FMR;
	efc->EEFC_FMR = (fmr & ~EEFC_FMR_FWS_Msk) |
					EEFC_FMR_FWS(CHIP_FLASH_WRITE_WAIT_STATE);

	// Ready, and the error bits of an earlier command cleared by the read.
	while ((efc->EEFC_FSR & EEFC_FSR_FRDY) == 0 && --n != 0)
	{
	}

	if (n != 0)
	{
		if (pSrc != nullptr)
		{
			for (uint32_t i = 0; i < DFU_TGT_PAGE / 4; i++)
			{
				pDst[i] = pSrc[i];
			}
		}
		__DSB();

		efc->EEFC_FCR = Fcr;

		// The error bits clear on read, so keep every read.
		n = DFU_TGT_TIMEOUT;
		uint32_t s;
		do {
			s = efc->EEFC_FSR;
			fsr |= s;
		} while ((s & EEFC_FSR_FRDY) == 0 && --n != 0);

		if (n == 0)
		{
			fsr &= ~EEFC_FSR_FRDY;
		}
	}

	efc->EEFC_FMR = fmr;

	if (cache)
	{
		cmcc->CMCC_MAINT0 = CMCC_MAINT0_INVALL;
		cmcc->CMCC_CTRL = CMCC_CTRL_CEN;
	}

	__DSB();
	__ISB();
	__set_PRIMASK(pm);

	return fsr;
}

static inline bool DfuTgtInFlash(uintptr_t Addr, uint32_t Len)
{
	return Addr >= IFLASH_ADDR && Addr - IFLASH_ADDR < IFLASH_SIZE &&
		   Len <= IFLASH_SIZE - (Addr - IFLASH_ADDR);
}

static inline bool DfuTgtOk(uint32_t Fsr)
{
	return (Fsr & (EEFC_FSR_FRDY | DFU_TGT_FSR_ERR)) == EEFC_FSR_FRDY;
}

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_PAGE;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	return DfuTgtInFlash(Addr, 1) ? DFU_TGT_ERASE_UNIT : 0;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if (DfuTgtInFlash(Addr, DFU_TGT_ERASE_UNIT) == false ||
		(Addr % DFU_TGT_ERASE_UNIT) != 0)
	{
		return false;
	}

	uint32_t page = (uint32_t)(Addr - IFLASH_ADDR) / DFU_TGT_PAGE;

	// Erase pages: the first page, a multiple of 16, with the group size in
	// the two low bits.
	return DfuTgtOk(DfuTgtCmd(EEFC_FCR_FKEY_PASSWD |
							  EEFC_FCR_FARG(page | DFU_TGT_EPA_16) |
							  EEFC_FCR_FCMD_EPA, nullptr, nullptr));
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if (pData == nullptr || (Addr % DFU_TGT_PAGE) != 0 ||
		(Len % DFU_TGT_PAGE) != 0 || DfuTgtInFlash(Addr, Len) == false)
	{
		return false;
	}

	const uint8_t *p = (const uint8_t *)pData;

	for (uint32_t off = 0; off < Len; off += DFU_TGT_PAGE)
	{
		volatile uint32_t *dst = (volatile uint32_t *)(Addr + off);
		uint32_t page = (uint32_t)(Addr + off - IFLASH_ADDR) / DFU_TGT_PAGE;

		// Read the source while the flash is idle, any alignment.
		memcpy(s_DfuTgtPage, p + off, DFU_TGT_PAGE);

		if (DfuTgtOk(DfuTgtCmd(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(page) |
							   EEFC_FCR_FCMD_WP, dst, s_DfuTgtPage)) == false ||
			memcmp((const void *)dst, s_DfuTgtPage, DFU_TGT_PAGE) != 0)
		{
			return false;
		}
	}

	return true;
}

// ---------------------------------------------------------------------------
// Start and reset
// ---------------------------------------------------------------------------

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, IRAM_ADDR,
						   IRAM_ADDR + IRAM_SIZE);
}

void DfuTgtReset(void)
{
	// Processor and peripherals, as the reset controller's software reset
	// does. RAM is kept.
	__DSB();
	SAM4E_RSTC->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST |
						  RSTC_CR_PERRST;
	for (;;)
	{
	}
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();
	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
