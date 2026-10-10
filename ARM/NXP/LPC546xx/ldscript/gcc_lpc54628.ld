/**-------------------------------------------------------------------------
@file	gcc_lpc54628.ld

@brief	GCC linker script for the LPC54628J512

		512 KB flash in 32 KB sectors, SRAMX 32 KB, SRAM0 64 KB, SRAM1 to
		SRAM3 96 KB.

		RAM is SRAM0 without its last 32 bytes, kept free for the IAP ROM.
		SRAM1 to SRAM3 are clocked only once SystemInit has run, after
		ResetEntry has set up .data and .bss, so those stay in SRAM0. RAM2
		(SRAM1 to SRAM3, again without the last 32 bytes) and SRAMX are for
		data placed there explicitly.

		The boot ROM starts the flash image only when the first 8 words of
		the vector table sum to zero. __valid_user_code_checksum is that 8th
		word, computed here from the stack top and the 6 handler addresses
		(Thumb bit included).

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lstdc++ -lgcc -lc -lnosys)

MEMORY
{
	FLASH (rx)	: ORIGIN = 0x00000000, LENGTH = 512K
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 64K - 32
	RAM2 (rwx)	: ORIGIN = 0x20010000, LENGTH = 96K - 32
	SRAMX (rwx)	: ORIGIN = 0x04000000, LENGTH = 32K
}

INCLUDE "gcc_arm_flash.ld"

__valid_user_code_checksum = 0 - (__StackTop + (ResetEntry + 1) + (NMI_Handler + 1) +
								  (HardFault_Handler + 1) + (MemManage_Handler + 1) +
								  (BusFault_Handler + 1) + (UsageFault_Handler + 1));
