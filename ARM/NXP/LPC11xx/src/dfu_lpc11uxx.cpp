/**-------------------------------------------------------------------------
@file	dfu_lpc11uxx.cpp

@brief	DFU target layer for NXP LPC11Uxx (Cortex-M0): flash through the IAP
		ROM, start through SYSMEMREMAP.

The flash is 4 KB sectors, erased and programmed by the IAP ROM: prepare,
then erase or copy RAM to flash, 256 bytes at a time. The ROM needs the
source in RAM, the top 32 bytes of RAM for itself, and no flash read while
it runs, so interrupts are off around each call and the data goes through
a RAM buffer. The IAP helpers are those of the library, iap_lpc11uxx.c,
which pass SystemCoreClock to the ROM as its clock.

The Cortex-M0 has no VTOR. The application vector table is copied to the
start of RAM and SYSMEMREMAP maps RAM at address 0, so the application
script keeps the first DFU_TGT_VECT_SIZE bytes of RAM free. The remap
covers the first 512 bytes of the map, so DfuTgtStart has to sit above
them; dfu_boot_lpc11u35.ld checks it.

The 8 KB of SRAM0 do not hold the boot with its stack, so the boot script
puts some of its buffers in the 2 KB USB SRAM, a section of its own that
ResetEntry does not clear and whose clock is off at reset. An entry of the
C library preinit array turns that clock on and clears the section, before
any constructor and before main. Where no script defines the section it
does nothing.


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

#include "LPC11Uxx.h"
#include "system_LPC11Uxx.h"
#include "iap_lpc11uxx.h"

#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_TGT_FLASH_SIZE		0x10000UL	//!< LPC11U35, 64 KB
#define DFU_TGT_SECT_SIZE		0x1000UL	//!< Uniform 4 KB sectors
#define DFU_TGT_WR_UNIT			256U		//!< Smallest IAP copy RAM to flash

// RAM blocks a stack pointer may point in: SRAM0 and the USB SRAM.
#define DFU_TGT_SRAM0_START		0x10000000UL
#define DFU_TGT_SRAM0_END		0x10002000UL
#define DFU_TGT_USBRAM_START	0x20004000UL
#define DFU_TGT_USBRAM_END		0x20004800UL

// Vector table: 16 core entries and 32 interrupts. SYSMEMREMAP MAP value 1,
// user RAM mode, fetches the vectors from the start of SRAM0.
#define DFU_TGT_VECT_SIZE		(48U * 4U)
#define DFU_TGT_REMAP_RAM		1U

// Loop bound on clock switch waits, a few cycles on the silicon.
#define DFU_TGT_CLK_WAIT		100000

// USB SRAM section of the boot script, left out elsewhere.
extern "C" char __dfu_usbram_bss_start[] __attribute__((weak));
extern "C" char __dfu_usbram_bss_end[] __attribute__((weak));

#define DFU_TGT_USBRAM_CLK		(1UL << 27)	//!< SYSAHBCLKCTRL USBSRAM

static void DfuTgtUsbRamInit(void)
{
	// Through integers, so the compiler does not take the weak addresses for
	// never null.
	uintptr_t s = (uintptr_t)__dfu_usbram_bss_start;
	uintptr_t e = (uintptr_t)__dfu_usbram_bss_end;

	if (e > s)
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= DFU_TGT_USBRAM_CLK;
		memset((void *)s, 0, e - s);
	}
}

// Kept by the section scripts (KEEP .preinit_array), run by
// __libc_init_array ahead of the constructors.
__attribute__((section(".preinit_array"), used))
static void (* const s_DfuTgtUsbRamInit)(void) = DfuTgtUsbRamInit;

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
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_SRAM0_START,
						   DFU_TGT_SRAM0_END) ||
		   DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_USBRAM_START,
						   DFU_TGT_USBRAM_END);
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();

	// Main clock back to the IRC, as after a reset: the application
	// SystemInit sets the system PLL up again, which it may only do while
	// the PLL does not clock the core.
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_IRC;
	LPC_SYSCON->MAINCLKUEN = 0;
	LPC_SYSCON->MAINCLKUEN = 1;
	for (int i = 0; i < DFU_TGT_CLK_WAIT && (LPC_SYSCON->MAINCLKUEN & 1) == 0;
		 i++)
	{
	}

	// Vector table to the start of RAM.
	const uint32_t *src = (const uint32_t *)RunAddr;
	volatile uint32_t *dst = (volatile uint32_t *)DFU_TGT_SRAM0_START;

	for (unsigned i = 0; i < DFU_TGT_VECT_SIZE / 4; i++)
	{
		dst[i] = src[i];
	}

	uint32_t sp = src[0];
	uint32_t pc = src[1];

	__set_CONTROL(0);
	__DSB();
	__ISB();

	// Then RAM mapped at 0 and the jump. The remap covers the first 512
	// bytes of the map, boot code included, so nothing of the boot below
	// 0x200 may run after it: the last instructions are these, in this
	// function, which the boot script checks sits above 0x200.
	__asm volatile(
		"str	%[map], [%[reg]]	\n"
		"dsb						\n"
		"isb						\n"
		"msr	msp, %[sp]			\n"
		"bx		%[pc]				\n"
		:
		: [map] "r" (DFU_TGT_REMAP_RAM),
		  [reg] "r" (&LPC_SYSCON->SYSMEMREMAP),
		  [sp] "r" (sp), [pc] "r" (pc)
		: "memory"
	);

	for (;;)
	{
	}
}

/** @} */
