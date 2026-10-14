/*--------------------------------------------------------------------------
File   : gcc_R9A02G021.ld

Author : Hoang Nguyen Hoan          Aug. 2025

Desc   : Renesas R9A02G021 linker script, normal boot.
         Includes r9a02g021_head.ld for the start of the code flash and the
         shared gcc_riscv_flash.ld for the standard sections.

Memory map, R9A02G021 User's Manual R01UH1036EJ0110, 4.1 and 34.1. Every
part has the same memories:

    HEAD        0x00000000  2064 B   reset entry, CLIC vector table, trap
                                     entries, option-setting memory, OSIS.
                                     See r9a02g021_head.ld
    FLASH       0x00000810  126 KB   code, rodata, .data load image
    DATA_FLASH  0x40100000  4 KB     data flash
    RAM         0x20004000  12 KB    SRAM0 parity area: .data, .bss, heap,
                                     stack
    IRAM        0x20000000  4 KB     SRAM0 ECC area, ECC off after reset:
                                     DTC vector table, .iram.text

Copyright (c) 2025, I-SYST, all rights reserved
----------------------------------------------------------------------------*/

OUTPUT_ARCH( "riscv" )

SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/RISCV/ldscript")
GROUP( -lstdc++ -lgcc -lc)

MEMORY
{
	HEAD          (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00000810
	FLASH         (rx)  : ORIGIN = 0x00000810, LENGTH = 0x00020000 - 0x00000810
	DATA_FLASH    (r)   : ORIGIN = 0x40100000, LENGTH = 0x00001000
	RAM           (rwx) : ORIGIN = 0x20004000, LENGTH = 0x00003000
	IRAM          (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00001000
}

/* The .reset input section goes in HEAD, the shared script's .reset output
 * section stays empty at the start of FLASH. */
RESET_VECTOR = ORIGIN(FLASH);

__global_pointer$ = ORIGIN(RAM) + 0x800;

INCLUDE "r9a02g021_head.ld"

/* DTC vector table of dtc_r9a02.cpp, 1 KB aligned. First in IRAM, ahead of
 * the .iram.text of gcc_riscv_flash.ld, the alignment then costs nothing. */
SECTIONS
{
	.dtc_vector (NOLOAD) : ALIGN(1024)
	{
		*(.bss.dtc_vector)
	} > IRAM
}

INCLUDE "gcc_riscv_flash.ld"
