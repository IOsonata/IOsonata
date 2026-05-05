/**-------------------------------------------------------------------------
@file	gcc_esp32c3.ld

@brief	ESP32-C3 chip-specific linker script.

Defines memory map, RESET_VECTOR, __global_pointer$, __StackTop, and
the chip-specific .iram.text placement.  All standard sections (.text,
.rodata, .data, .bss, .heap, .reset, etc.) and ResetEntry symbols are
provided by the shared gcc_riscv_flash.ld INCLUDEd at the bottom.

Memory map (ESP32-C3 TRM Rev 0.4 §3):
  IRAM       : 0x4037C000  80 KB    Internal SRAM (instruction-side view)
  RAM        : 0x3FC80000  320 KB   Internal SRAM (data-side view)
  FLASH      : 0x42000000  4 MB     IROM XIP cache window
  FLASH_DROM : 0x3C000000  4 MB     DROM cache window (read-only data)

Direct Boot is used: the ROM bootloader maps the application image
placed at flash offset 0 and jumps to the entry symbol provided by
ENTRY(ResetEntry) in gcc_riscv_flash.ld.  No second-stage bootloader.

Trap controller: CLINT in direct mode (mtvec[1:0] = 0b00).  Trap handler
needs only 4-byte alignment.

@author	Hoang Nguyen Hoan
@date	Mar. 2026

@license	MIT
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

GROUP( -lstdc++ -lgcc -lc )

MEMORY
{
    IRAM       (RWX) : ORIGIN = 0x4037C000, LENGTH = 0x13C00
    RAM        (RW)  : ORIGIN = 0x3FC80000, LENGTH = 0x50000
    FLASH      (RX)  : ORIGIN = 0x42000000, LENGTH = 0x400000
    FLASH_DROM (R)   : ORIGIN = 0x3C000000, LENGTH = 0x400000
}

/*
 * RESET_VECTOR: virtual address of ResetEntry inside the IROM XIP window.
 * 0x42000020 leaves room for the 24-byte ESP image header + 8-byte
 * segment header at the start of flash, after which the segment data
 * (containing ResetEntry placed in .reset by attribute) begins.
 */
RESET_VECTOR = 0x42000020;

/*
 * __global_pointer$: ABI small-data anchor for RISC-V GP-relative
 * addressing.  ResetEntry.c loads this into gp before SystemInit().
 * Placed 0x800 bytes into RAM so the +/-2 KB GP window covers the
 * most-accessed .data and .bss.
 */
__global_pointer$ = ORIGIN(RAM) + 0x800;

SECTIONS
{
    /*
     * IRAM: trap handler + time-critical ISR code.
     * RISCV_TrapHandler must be in IRAM so it remains callable during a
     * cache fault when Flash XIP is temporarily unavailable.  CLINT direct
     * mode requires 4-byte alignment of mtvec, satisfied by the function's
     * own __attribute__((aligned(4))).
     */
    .iram.text :
    {
        . = ALIGN(4);
        _iram_text_start = ABSOLUTE(.);
        KEEP(*(.iram.text))
        *(.iram.text.*)
        . = ALIGN(4);
        _iram_text_end = ABSOLUTE(.);
    } > IRAM
}

/* Stack top: referenced by ResetEntry.c via `la sp, __StackTop`. */
__StackTop = ORIGIN(RAM) + LENGTH(RAM);

/*
 * Bring in the shared sections (.reset / .text / .rodata / .data / .bss /
 * .heap / .stack_dummy), ENTRY(ResetEntry), and all ResetEntry symbol
 * definitions (__bss_start__, __bss_end__, __data_start__, __data_loc__,
 * __heap_start__, __heap_end__, etc.).
 */
INCLUDE "gcc_riscv_flash.ld"
