/*--------------------------------------------------------------------------
File   : gcc_R9A02G021.ld

Author : Hoang Nguyen Hoan          Aug. 2025

Desc   : Renesas R9A02G021 (RISC-V Andes N22) linker script.
         Includes the shared gcc_riscv_flash.ld for the standard sections.

Memory map:
    FLASH         0x00000000  1.5 MiB    code + rodata + .data LMA
    OFS           0x0100A150  24 B       option-setting memory (read at every
                                         reset by the BSC ROM): OFS0, OFS1,
                                         BPS0..BPS3
    E2S_TRACE_BUF 0x20000000  1 KiB      e2studio trace buffer (SAU window)
    RAM           0x20000400  255 KiB    .data + .bss + heap + stack
    QSPI_FLASH    0x60000000  128 MiB    external XIP, unused unless app maps it

The OFS section MUST be populated.  Renesas tooling and most JLink/OpenOCD
profiles erase OFS along with the rest of flash, so a missing section reads
back as 0xFFFFFFFF and the chip boots with all option features disabled —
fine.  But some flash workflows (incremental flashing, sector-mask erase)
leave OFS untouched.  If a previous program wrote OFS0.IWDTSTRT = 0 (IWDT
auto-start), every reset re-arms the IWDT and the chip resets again ~2 s
later.  Explicit values here make the build deterministic.

The OFS data symbols themselves are defined in system_r9a02.c so they can
be tuned per project (override at link time by providing a non-weak
g_R9A02_Ofs0 / g_R9A02_Ofs1 / g_R9A02_Bps[N]).

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

RESET_VECTOR = 0;

MEMORY
{
    FLASH         (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00180000   /* 1.5 MiB */
    OFS           (r)   : ORIGIN = 0x0100A150, LENGTH = 0x00000018   /* 24 B    */
    E2S_TRACE_BUF (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00000400   /* 1 KiB   */

    /* The e2studio trace buffer requires 0x400 bytes of RAM starting at
     * address 0x20000000.  Those have been allocated above.  If trace is
     * not used, the application can reclaim those 0x400 bytes by changing
     * the RAM ORIGIN below to 0x20000000, increasing LENGTH by 0x400, and
     * removing the E2S_TRACE_BUF definition above.
     */
    RAM           (rwx) : ORIGIN = 0x20000400, LENGTH = 0x0003FC00   /* 255 KiB */
    QSPI_FLASH    (rx)  : ORIGIN = 0x60000000, LENGTH = 0x08000000   /* 128 MiB */
}

__global_pointer$ = ORIGIN(RAM) + LENGTH(RAM) / 2;

/* R9A02 has no separate executable SRAM — RAM is already (rwx).  The
 * shared gcc_riscv_flash.ld places .iram.text in IRAM; alias it here
 * so the section lands in the regular RAM region. */
REGION_ALIAS("IRAM", RAM);

/*--------------------------------------------------------------------------
 * Option-setting memory.
 *
 * Each of the six 32-bit slots gets its own input section so a project can
 * override individual slots without touching the rest.  The ".ofs.*"
 * inputs come from system_r9a02.c (defined as weak so an application can
 * provide its own non-weak versions to override).
 *
 * FILL is required for any holes inside the region: the linker emits no
 * data for symbols not yet defined, and unfilled holes default to 0x00,
 * which on R9A02G021 is "feature ENABLED".  We want unfilled holes to be
 * 0xFF (= disabled).  Each slot section is exactly 4 B so there should
 * be no holes in practice, but FILL guards against accidental gaps.
 *--------------------------------------------------------------------------*/
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
