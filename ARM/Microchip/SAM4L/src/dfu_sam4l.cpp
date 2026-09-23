/**-------------------------------------------------------------------------
@file	dfu_sam4l.cpp

@brief	DFU target layer for Microchip SAM4L (SAM4LC, SAM4LS): FLASHCALW.

One flash plane of 128, 256 or 512 KB, 512 byte pages, erased and
programmed a page at a time. A page is programmed through the page
buffer: cleared, loaded word by word at the page's own addresses, then
the write page command programs it. The flash size is read from CHIPID,
so one library serves the three sizes.

The flash cannot be read while a command runs and the code runs from it,
so the page buffer load and the command run from RAM (.fastrun, which the
startup copies with .data), interrupts off. The source of a write may be
the flash itself, so each page goes through a RAM buffer first. HCACHE,
which SystemInit turns on, is off during the command and invalidated
after it.

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

#include "sam4lxxx.h"
#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_TGT_PAGE			FLASH_PAGE_SIZE			//!< 512

// Status of a command that went through.
#define DFU_TGT_FSR_ERR			(FLASHCALW_FSR_LOCKE | FLASHCALW_FSR_PROGE)

// Polls of FRDY before a command is given up. A page erase or write takes a
// few ms; this is seconds at any clock.
#define DFU_TGT_TIMEOUT			0x4000000UL

// One page, word aligned, what the page buffer is loaded from.
static uint32_t s_DfuTgtPage[DFU_TGT_PAGE / 4];

// ---------------------------------------------------------------------------
// Internal memory
// ---------------------------------------------------------------------------

// Poll FRDY, keeping every status read: LOCKE and PROGE clear on read.
// Returns the bits seen, FRDY clear on timeout. RAM only, inlined in
// DfuTgtCmd.
__attribute__((always_inline))
static inline uint32_t DfuTgtWait(Flashcalw *pFc)
{
	uint32_t fsr = 0;
	uint32_t s;
	uint32_t n = DFU_TGT_TIMEOUT;

	do {
		s = pFc->FLASHCALW_FSR;
		fsr |= s;
	} while ((s & FLASHCALW_FSR_FRDY) == 0 && --n != 0);

	return n != 0 ? fsr : fsr & ~FLASHCALW_FSR_FRDY;
}

/**
 * @brief	Run one FLASHCALW page command, from RAM.
 *
 * Nothing here may touch the flash: no call out, no literal read from it.
 *
 * @param	Cmd  : FLASHCALW_FCMD_CMD_EP or FLASHCALW_FCMD_CMD_WP.
 * @param	Page : Page number.
 * @param	pDst : Page to load the page buffer at, with pSrc.
 * @param	pSrc : One page of words for a write, null for an erase.
 *
 * @return	FLASHCALW_FSR bits seen while the command ran, FRDY clear on
 * 			timeout.
 */
__attribute__((section(".fastrun"), noinline, long_call))
static uint32_t DfuTgtCmd(uint32_t Cmd, uint32_t Page,
						  volatile uint32_t *pDst, const uint32_t *pSrc)
{
	Flashcalw *fc = SAM4L_HFLASHC;
	Hcache *hc = SAM4L_HCACHE;
	uint32_t pm = __get_PRIMASK();

	__disable_irq();

	// The HCACHE registers only answer with its bus clock on.
	bool cache = (SAM4L_PM->PM_PBBMASK & PM_PBBMASK_HCACHE) != 0 &&
				 (hc->HCACHE_SR & HCACHE_SR_CSTS_EN) != 0;
	if (cache)
	{
		hc->HCACHE_CTRL = HCACHE_CTRL_CEN_NO;
		while ((hc->HCACHE_SR & HCACHE_SR_CSTS_EN) != 0)
		{
		}
	}

	// Ready, and the error bits of an earlier command cleared by the read.
	uint32_t fsr = DfuTgtWait(fc) & FLASHCALW_FSR_FRDY;

	if ((fsr & FLASHCALW_FSR_FRDY) != 0 && pSrc != nullptr)
	{
		fc->FLASHCALW_FCMD = FLASHCALW_FCMD_KEY_KEY | FLASHCALW_FCMD_CMD_CPB;
		fsr = DfuTgtWait(fc);

		if ((fsr & FLASHCALW_FSR_FRDY) != 0)
		{
			for (uint32_t i = 0; i < DFU_TGT_PAGE / 4; i++)
			{
				pDst[i] = pSrc[i];
			}
			__DSB();
		}
	}

	if ((fsr & FLASHCALW_FSR_FRDY) != 0 && (fsr & DFU_TGT_FSR_ERR) == 0)
	{
		fc->FLASHCALW_FCMD = FLASHCALW_FCMD_KEY_KEY |
							 FLASHCALW_FCMD_PAGEN(Page) | Cmd;
		fsr = DfuTgtWait(fc);
	}

	if (cache)
	{
		hc->HCACHE_MAINT0 = HCACHE_MAINT0_INVALL_YES;
		hc->HCACHE_CTRL = HCACHE_CTRL_CEN_YES;
		while ((hc->HCACHE_SR & HCACHE_SR_CSTS_EN) == 0)
		{
		}
	}

	__DSB();
	__ISB();
	__set_PRIMASK(pm);

	return fsr;
}

// Flash size of the part, from CHIPID. The encoding is the one of every
// SAM CHIPID; SAM4L comes in the three sizes below.
static uint32_t DfuTgtFlashSize(void)
{
	switch (SAM4L_CHIPID->CHIPID_CIDR & CHIPID_CIDR_NVPSIZ_Msk)
	{
		case CHIPID_CIDR_NVPSIZ_128K:
			return 0x20000;
		case CHIPID_CIDR_NVPSIZ_256K:
			return 0x40000;
		case CHIPID_CIDR_NVPSIZ_512K:
			return 0x80000;
		default:
			return FLASH_SIZE;
	}
}

static inline bool DfuTgtInFlash(uintptr_t Addr, uint32_t Len)
{
	uint32_t size = DfuTgtFlashSize();

	// The flash starts at 0.
	return Addr < size && Len <= size - Addr;
}

static inline bool DfuTgtOk(uint32_t Fsr)
{
	return (Fsr & (FLASHCALW_FSR_FRDY | DFU_TGT_FSR_ERR)) ==
		   FLASHCALW_FSR_FRDY;
}

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_PAGE;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	return DfuTgtInFlash(Addr, 1) ? DFU_TGT_PAGE : 0;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if (DfuTgtInFlash(Addr, DFU_TGT_PAGE) == false ||
		(Addr % DFU_TGT_PAGE) != 0)
	{
		return false;
	}

	uint32_t page = (uint32_t)Addr / DFU_TGT_PAGE;

	return DfuTgtOk(DfuTgtCmd(FLASHCALW_FCMD_CMD_EP, page, nullptr, nullptr));
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
		uint32_t page = (uint32_t)(Addr + off) / DFU_TGT_PAGE;

		// Read the source while the flash is idle, any alignment.
		memcpy(s_DfuTgtPage, p + off, DFU_TGT_PAGE);

		if (DfuTgtOk(DfuTgtCmd(FLASHCALW_FCMD_CMD_WP, page, dst,
							   s_DfuTgtPage)) == false ||
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
	// HRAMC0, whose size depends on the part: the generic SRAM range.
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_CM_SRAM_START,
						   DFU_CM_SRAM_END);
}

void DfuTgtReset(void)
{
	// SYSRESETREQ resets the core and the peripherals; RAM is kept.
	NVIC_SystemReset();
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();
	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
