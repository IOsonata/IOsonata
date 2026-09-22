/**-------------------------------------------------------------------------
@file	dfu_lpc17xx.cpp

@brief	DFU target layer for NXP LPC17xx (Cortex-M3): flash through the IAP
		ROM, start through VTOR.

The LPC1769 flash is 16 sectors of 4 KB, then 14 of 32 KB, erased and
programmed by the IAP ROM: prepare, then erase or copy RAM to flash, 256
bytes at a time. The ROM needs the source in RAM, the top 32 bytes of the
local SRAM for itself, and no flash read while it runs, so interrupts are
off around each call and the data goes through a RAM buffer.

The library helpers in lpc_iap.c do not build for this family (no IAP
entry for it there), so the ROM is called here directly, with the command
codes of lpc_iap.h and SystemCoreClock as its clock, the way lpc_iap.c
passes it.

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

#include "LPC17xx.h"
#include "lpc_iap.h"

#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_TGT_FLASH_SIZE		0x80000UL	//!< LPC1769, 512 KB
#define DFU_TGT_SMALL_SIZE		0x1000UL	//!< Sectors 0 to 15
#define DFU_TGT_SMALL_END		0x10000UL
#define DFU_TGT_LARGE_SIZE		0x8000UL	//!< Sectors 16 to 29
#define DFU_TGT_WR_UNIT			256U		//!< Smallest IAP copy RAM to flash

// RAM blocks a stack pointer may point in: the local SRAM and the two AHB
// SRAM banks, which follow each other.
#define DFU_TGT_SRAM_START		0x10000000UL
#define DFU_TGT_SRAM_END		0x10008000UL
#define DFU_TGT_AHBRAM_START	0x2007C000UL
#define DFU_TGT_AHBRAM_END		0x20084000UL

// IAP ROM entry, a Thumb address. Command and result, 5 words each.
#define DFU_TGT_IAP_ENTRY		0x1FFF1FF1UL

// PLL0STAT bits and a loop bound on the waits for them, a few cycles on
// the silicon.
#define DFU_TGT_PLL0_ENABLED	(1UL << 24)
#define DFU_TGT_PLL0_CONNECTED	(1UL << 25)
#define DFU_TGT_CLK_WAIT		100000

typedef void (*DfuTgtIap_t)(uint32_t *pCmd, uint32_t *pRes);

extern "C" uint32_t SystemCoreClock;

static uint32_t DfuTgtIap(uint32_t Cmd, uint32_t P0, uint32_t P1, uint32_t P2,
						  uint32_t P3)
{
	uint32_t cmd[5] = { Cmd, P0, P1, P2, P3 };
	uint32_t res[5] = { IAPSTATUS_INVALID_COMMAND, 0, 0, 0, 0 };

	((DfuTgtIap_t)DFU_TGT_IAP_ENTRY)(cmd, res);

	return res[0];
}

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	if (Addr < DFU_TGT_SMALL_END)
	{
		return DFU_TGT_SMALL_SIZE;
	}

	return Addr < DFU_TGT_FLASH_SIZE ? DFU_TGT_LARGE_SIZE : 0;
}

static uint32_t DfuTgtSector(uintptr_t Addr)
{
	if (Addr < DFU_TGT_SMALL_END)
	{
		return Addr / DFU_TGT_SMALL_SIZE;
	}

	return DFU_TGT_SMALL_END / DFU_TGT_SMALL_SIZE +
		   (Addr - DFU_TGT_SMALL_END) / DFU_TGT_LARGE_SIZE;
}

// The ROM runs with the flash busy: nothing may fetch from it meanwhile.
static uint32_t DfuTgtIrqOff(void)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();

	return pm;
}

static void DfuTgtIrqRestore(uint32_t Pm)
{
	if (Pm == 0)
	{
		__enable_irq();
	}
}

bool DfuTgtErase(uintptr_t Addr)
{
	uint32_t unit = DfuTgtEraseUnit(Addr);

	if (unit == 0 || (Addr % unit) != 0)
	{
		return false;
	}

	uint32_t sect = DfuTgtSector(Addr);
	uint32_t pm = DfuTgtIrqOff();
	uint32_t res = DfuTgtIap(IAPCMD_PREP_SECTOR, sect, sect, 0, 0);

	if (res == IAPSTATUS_SUCCESS)
	{
		res = DfuTgtIap(IAPCMD_ERASE_SECT, sect, sect, SystemCoreClock / 1000,
						0);
	}
	DfuTgtIrqRestore(pm);

	return res == IAPSTATUS_SUCCESS;
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr % DFU_TGT_WR_UNIT) != 0 || (Len % DFU_TGT_WR_UNIT) != 0 ||
		Addr >= DFU_TGT_FLASH_SIZE || Len > DFU_TGT_FLASH_SIZE - Addr)
	{
		return false;
	}

	// On the stack, for the time of the write only. Word aligned for the
	// ROM, which reads its source from RAM only.
	uint32_t buf[DFU_TGT_WR_UNIT / 4];
	const uint8_t *p = (const uint8_t *)pData;

	for (uint32_t off = 0; off < Len; off += DFU_TGT_WR_UNIT)
	{
		uintptr_t a = Addr + off;
		uint32_t sect = DfuTgtSector(a);

		// The source may be anywhere, flash included: copied while the
		// flash still reads.
		memcpy(buf, p + off, DFU_TGT_WR_UNIT);

		uint32_t pm = DfuTgtIrqOff();
		uint32_t res = DfuTgtIap(IAPCMD_PREP_SECTOR, sect, sect, 0, 0);

		if (res == IAPSTATUS_SUCCESS)
		{
			res = DfuTgtIap(IAPCMD_COPY_RAM_TO_FLASH, (uint32_t)a,
							(uint32_t)(uintptr_t)buf, DFU_TGT_WR_UNIT,
							SystemCoreClock / 1000);
		}
		DfuTgtIrqRestore(pm);

		if (res != IAPSTATUS_SUCCESS)
		{
			return false;
		}
	}

	return true;
}

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_SRAM_START,
						   DFU_TGT_SRAM_END) ||
		   DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_AHBRAM_START,
						   DFU_TGT_AHBRAM_END);
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

static void DfuTgtPll0Feed(void)
{
	LPC_SC->PLL0FEED = 0xAA;
	LPC_SC->PLL0FEED = 0x55;
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();

	// PLL0 off and the IRC back as the clock, as after a reset: the
	// application SystemInit programs PLL0 without disconnecting it first.
	// Disconnect, then disable, each with its feed, then the dividers and
	// the source, which may only change with PLL0 out of the way.
	LPC_SC->PLL0CON &= ~2UL;
	DfuTgtPll0Feed();
	for (int i = 0; i < DFU_TGT_CLK_WAIT &&
		 (LPC_SC->PLL0STAT & DFU_TGT_PLL0_CONNECTED); i++)
	{
	}
	LPC_SC->PLL0CON = 0;
	DfuTgtPll0Feed();
	for (int i = 0; i < DFU_TGT_CLK_WAIT &&
		 (LPC_SC->PLL0STAT & DFU_TGT_PLL0_ENABLED); i++)
	{
	}
	LPC_SC->CCLKCFG = 0;
	LPC_SC->CLKSRCSEL = 0;

	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
