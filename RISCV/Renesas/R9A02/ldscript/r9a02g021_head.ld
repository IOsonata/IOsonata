/*--------------------------------------------------------------------------
File   : r9a02g021_head.ld

Author : Hoang Nguyen Hoan          Oct. 12, 2026

Desc   : Start of the R9A02G021 code flash, shared by gcc_R9A02G021.ld and
         dfu_boot_r9a02g021.ld. The including script defines the HEAD
         memory region at 0, 0x810 bytes long.

The CPU starts at address 0 (R01UH1036EJ0110, 2.7.2.2). The option-setting
memory is inside the code flash (6.2, 13.6): OFS0 at 0x400, OFS1 at 0x404,
the flash read protection words from 0x408 to 0x43B and OSIS from 0x800 to
0x80F. HEAD places the reset entry and the CLIC vector table below 0x400
and the trap entries between 0x43C and 0x800.

Option-setting values. Override on the link command line, for example
-Wl,--defsym=R9A02_OFS1=0xFFFFCEFF:

    R9A02_OFS0      default 0xFFFFFFFF  IWDT and WDT stopped after reset
    R9A02_OFS1      default 0xFFFFCFFF  HOCO 48 MHz, HOCO off after reset,
                                        voltage monitor 0 reset off,
                                        internal clock supply type A
    R9A02_OSIS0..3  default 0xFFFFFFFF  ID code protection off

The flash read protection words are always 0xFFFFFFFF: FLRPROTAC must be
0xFFFFFFFF for the debugger to connect (13.7.1).

Copyright (c) 2026, I-SYST, all rights reserved
----------------------------------------------------------------------------*/

PROVIDE(R9A02_OFS0 = 0xFFFFFFFF);
PROVIDE(R9A02_OFS1 = 0xFFFFCFFF);
PROVIDE(R9A02_OSIS0 = 0xFFFFFFFF);
PROVIDE(R9A02_OSIS1 = 0xFFFFFFFF);
PROVIDE(R9A02_OSIS2 = 0xFFFFFFFF);
PROVIDE(R9A02_OSIS3 = 0xFFFFFFFF);

SECTIONS
{
	.head ORIGIN(HEAD) :
	{
		KEEP(*(.reset))
		. = ALIGN(64);
		KEEP(*(.vectors.table))
		__head_low_end__ = .;
		. = 0x400;
		LONG(R9A02_OFS0)
		LONG(R9A02_OFS1)
		. = 0x43C;
		. = ALIGN(64);
		KEEP(*(.vectors .vectors.*))
		__head_high_end__ = .;
		. = 0x800;
		LONG(R9A02_OSIS0)
		LONG(R9A02_OSIS1)
		LONG(R9A02_OSIS2)
		LONG(R9A02_OSIS3)
	} > HEAD =0xFFFFFFFF
}

ASSERT(ORIGIN(HEAD) == 0 && LENGTH(HEAD) == 0x810, "HEAD must be 0 to 0x80F")
ASSERT(__head_low_end__ <= 0x400, "reset entry and vector table run into OFS0 at 0x400")
ASSERT(__head_high_end__ <= 0x800, "trap entries run into OSIS at 0x800")
