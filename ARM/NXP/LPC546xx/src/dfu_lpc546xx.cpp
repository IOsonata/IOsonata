/**-------------------------------------------------------------------------
@file	dfu_lpc546xx.cpp

@brief	DFU target layer for NXP LPC546xx (Cortex-M4F): flash through the IAP
		ROM, start through VTOR.

The flash is 32 KB sectors of 256 byte pages, erased and programmed by the
IAP ROM at 0x03000205: prepare, then erase or copy RAM to flash, 256 bytes
at a time. The erase unit is the sector: slots are erased whole, and one
IAP erase per 32 KB is far fewer erase operations than one per page. The ROM needs the source in RAM and no flash read while it
runs, so interrupts are off around each call and the data goes through a
RAM buffer. The IAP helpers are those of the library, lpc_iap.c, which
pass SystemCoreClock to the ROM as its clock.

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

#include "LPC546xx.h"
#include "lpc_iap.h"

#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

// The part number define (CPU_LPC54605J512...) sets the size in the
// features header; without one, the 512 KB part.
#ifdef FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES
#define DFU_TGT_FLASH_SIZE		FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES
#else
#define DFU_TGT_FLASH_SIZE		0x80000UL
#endif
#define DFU_TGT_SECT_SIZE		0x8000UL	//!< 32 KB sectors
#define DFU_TGT_WR_UNIT			256U		//!< One page, smallest IAP copy

// RAM blocks a stack pointer may point in: SRAMX and SRAM0 to SRAM3,
// which follow each other.
#define DFU_TGT_SRAMX_START		0x04000000UL
#define DFU_TGT_SRAMX_END		0x04008000UL
#define DFU_TGT_SRAM_START		0x20000000UL
#define DFU_TGT_SRAM_END		0x20028000UL

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	return Addr < DFU_TGT_FLASH_SIZE ? DFU_TGT_SECT_SIZE : 0;
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
	if (Addr >= DFU_TGT_FLASH_SIZE || (Addr % DFU_TGT_SECT_SIZE) != 0)
	{
		return false;
	}

	int sect = (int)(Addr / DFU_TGT_SECT_SIZE);
	uint32_t pm = DfuTgtIrqOff();
	IAPSTATUS res = IAPPrepSectWrite(sect, sect);

	if (res == IAPSTATUS_SUCCESS)
	{
		res = IAPEraseSector(sect, sect);
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
		int sect = (int)(a / DFU_TGT_SECT_SIZE);

		// The source may be anywhere, flash included: copied while the
		// flash still reads.
		memcpy(buf, p + off, DFU_TGT_WR_UNIT);

		uint32_t pm = DfuTgtIrqOff();
		IAPSTATUS res = IAPPrepSectWrite(sect, sect);

		if (res == IAPSTATUS_SUCCESS)
		{
			res = IAPCopyRamToFlash((uint8_t *)a, (uint8_t *)buf,
									DFU_TGT_WR_UNIT);
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
		   DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_SRAMX_START,
						   DFU_TGT_SRAMX_END);
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

void DfuTgtStart(uintptr_t RunAddr)
{
	// The boot leaves the clocks as SystemInit set them, on the FRO, which
	// the application SystemInit takes over as it does after a reset.
	DfuCmQuiesce();
	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
