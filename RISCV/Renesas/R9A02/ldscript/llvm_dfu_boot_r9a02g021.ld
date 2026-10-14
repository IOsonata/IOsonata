/*--------------------------------------------------------------------------
Stage 0 DFU boot, Renesas R9A02G021, LLVM linker.

The core starts at address 0, the start of the boot. The start of the code
flash, with the reset entry, the CLIC vector table, the trap entries, the
option-setting memory and OSIS, is laid out by r9a02g021_head.ld, as in
gcc_R9A02G021.ld. The application image leaves it out.

RAM: the 12 KB SRAM0 parity area, stopped at the recovery request word.
IRAM: the 4 KB SRAM0 ECC area.
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

INCLUDE "dfu_layout_r9a02g021.ld"

MEMORY
{
	HEAD          (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00000810
	FLASH         (rx)  : ORIGIN = 0x00000810, LENGTH = __dfu_boot_end - 0x00000810
	RAM           (rwx) : ORIGIN = 0x20004000, LENGTH = __dfu_flag - 0x20004000
	IRAM          (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00001000
}

/* The .reset input section goes in HEAD, the shared script's .reset output
 * section stays empty at the start of FLASH. */
RESET_VECTOR = ORIGIN(FLASH);

__global_pointer$ = ORIGIN(RAM) + 0x800;

INCLUDE "r9a02g021_head.ld"

INCLUDE "llvm_riscv_flash.ld"

ASSERT(ADDR(.head) == __dfu_boot_start, "reset entry not at the start of the boot")
