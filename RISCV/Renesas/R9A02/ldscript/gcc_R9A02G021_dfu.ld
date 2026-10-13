/*--------------------------------------------------------------------------
Application with DFU, Renesas R9A02G021.

Same memories as gcc_R9A02G021.ld with FLASH cut down to slot 0 and RAM
stopped short of the recovery request word, from the layout the stage 0
boot shares. An image that outgrows slot 0 fails to link.

RESET_VECTOR is the start of slot 0: the .reset section, ResetEntry, is the
first thing in the image, and that is where the boot jumps (DfuTgtStart).
The CLIC vector table and the trap entries follow it in .ivector.

The option-setting memory belongs to the boot image (r9a02g021_head.ld), so
nothing here is placed at 0x400 or 0x800.
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

INCLUDE "dfu_layout_r9a02g021.ld"

RESET_VECTOR = __dfu_slot0_start;

MEMORY
{
	FLASH         (rx)  : ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	DATA_FLASH    (r)   : ORIGIN = 0x40100000, LENGTH = 0x00001000
	RAM           (rwx) : ORIGIN = 0x20004000, LENGTH = __dfu_flag - 0x20004000
	IRAM          (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00001000
}

__global_pointer$ = ORIGIN(RAM) + 0x800;

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

ASSERT(ADDR(.reset) == __dfu_slot0_start, "reset entry not at the start of slot 0")
