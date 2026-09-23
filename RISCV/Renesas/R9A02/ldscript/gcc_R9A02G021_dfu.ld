/*--------------------------------------------------------------------------
Application with DFU, Renesas R9A02G021.

Same as gcc_R9A02G021.ld with FLASH cut down to slot 0 and RAM stopped
short of the recovery request word, from the layout the stage 0 boot
shares. An image that outgrows slot 0 fails to link.

RESET_VECTOR is the start of slot 0: the .reset section, ResetEntry, is the
first thing in the image, and that is where the boot jumps (DfuTgtStart).

The option setting memory belongs to the boot image, so the .ofs.* words
of system_r9a02.c are dropped here: they would also put 0x0100A150 in the
binary that gets signed.
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
    DATA_FLASH    (r)   : ORIGIN = 0x40100000, LENGTH = 0x00001000   /* 4 KiB   */
    E2S_TRACE_BUF (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00000400   /* 1 KiB   */

    /* The e2studio trace buffer requires 0x400 bytes of RAM starting at
     * address 0x20000000.  Those have been allocated above.  If trace is
     * not used, the application can reclaim those 0x400 bytes by changing
     * the RAM ORIGIN below to 0x20000000, increasing LENGTH by 0x400, and
     * removing the E2S_TRACE_BUF definition above.
     */
    RAM           (rwx) : ORIGIN = 0x20000400, LENGTH = __dfu_flag - 0x20000400
    QSPI_FLASH    (rx)  : ORIGIN = 0x60000000, LENGTH = 0x08000000   /* 128 MiB */
}

__global_pointer$ = ORIGIN(RAM) + LENGTH(RAM) / 2;

REGION_ALIAS("IRAM", RAM);

SECTIONS
{
    /DISCARD/ : { *(.ofs.*) }
}

INCLUDE "gcc_riscv_flash.ld"

ASSERT(ADDR(.reset) == __dfu_slot0_start, "reset entry not at the start of slot 0")
