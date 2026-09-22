/*--------------------------------------------------------------------------
Stage 0 DFU boot, Renesas R9A02G021.

The core starts at the .reset section at 0 (RESET_VECTOR), the start of
the boot.

The option setting memory (OFS0, OFS1, BPS0..3 at 0x0100A150) is part of
the boot image, filled the way gcc_R9A02G021.ld fills it: from the .ofs.*
input sections of system_r9a02.c, 0xFF in any hole. The application image
leaves it out.

RAM: the first 1 KB is left to the e2studio trace buffer, as in
gcc_R9A02G021.ld, and RAM stops at the recovery request word.
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

RESET_VECTOR = __dfu_boot_start;

MEMORY
{
    FLASH         (rx)  : ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
    OFS           (r)   : ORIGIN = 0x0100A150, LENGTH = 0x00000018   /* 24 B    */
    E2S_TRACE_BUF (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00000400   /* 1 KiB   */
    RAM           (rwx) : ORIGIN = 0x20000400, LENGTH = __dfu_flag - 0x20000400
}

__global_pointer$ = ORIGIN(RAM) + LENGTH(RAM) / 2;

REGION_ALIAS("IRAM", RAM);

SECTIONS
{
    .ofs ORIGIN(OFS) :
    {
        FILL(0xFFFFFFFF);
        KEEP(*(.ofs.0))
        . = ORIGIN(OFS) + 0x04;
        KEEP(*(.ofs.1))
        . = ORIGIN(OFS) + 0x08;
        KEEP(*(.ofs.bps0))
        . = ORIGIN(OFS) + 0x0C;
        KEEP(*(.ofs.bps1))
        . = ORIGIN(OFS) + 0x10;
        KEEP(*(.ofs.bps2))
        . = ORIGIN(OFS) + 0x14;
        KEEP(*(.ofs.bps3))
        . = ORIGIN(OFS) + 0x18;
    } > OFS = 0xFFFFFFFF
}

INCLUDE "gcc_riscv_flash.ld"

ASSERT(ADDR(.reset) == __dfu_boot_start, "reset entry not at the start of the boot")
