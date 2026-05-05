/**-------------------------------------------------------------------------
@file	gcc_esp32c3.ld

@brief	ESP32-C3 chip-specific linker script for Direct Boot.

Self-contained: does NOT INCLUDE gcc_riscv_flash.ld, because that script
assumes VMA == LMA (suitable for chips like the Renesas R9A02 where
flash is directly addressable at its physical address).  The ESP32-C3
needs LMA != VMA: the .hex/.bin must record bytes at physical flash
offsets (0..flash_size), while symbols resolve to IROM/DROM/IRAM/RAM
virtual addresses where code/data actually run.

Memory windows (ESP32-C3 TRM Rev 0.4 §3):
  FLASH (LMA)  : 0x00000000  4 MB    Physical SPI flash, where bytes live
  IROM  (VMA)  : 0x42000000  4 MB    Cache-mapped instruction view of FLASH
  DROM  (VMA)  : 0x3C000000  4 MB    Cache-mapped read-only data view
  IRAM  (VMA)  : 0x4037C000  80 KB   Internal SRAM, instruction-side
  RAM   (VMA)  : 0x3FC80000  320 KB  Internal SRAM, data-side

Direct Boot (silicon rev 3+, EFUSE_DIS_LEGACY_SPI_BOOT == 0):
  ROM checks flash offsets 0..7 for the 8-byte magic
  1d 04 db ae 1d 04 db ae (= 0xaedb041d twice).  On match, ROM enables
  the flash MMU mapping the entire 4 MB to IROM and DROM, then jumps to
  flash + 8 (virtual 0x42000008).  No second-stage bootloader, no ESP
  image header, no segment table.

Section layout in flash (LMA, what the .hex contains):
  0x00000000  .dboot_hdr    8 bytes  magic header
  0x00000008  .reset        ResetEntry
  ...         .text         all code
  ...         .rodata       read-only data
  ...         .data_load    initial values for .data (copied to RAM)
  ...         .iram_load    initial values for .iram.text (TODO: copy by
                            ResetEntry; not yet wired -- traps go to
                            uninitialised IRAM until then)

Section layout at runtime (VMA, what the CPU sees):
  0x42000000  .dboot_hdr    via IROM cache
  0x42000008  .reset+.text  via IROM cache (executes from flash)
  0x3C00....  .rodata       via DROM cache
  0x3FC80000  .data, .bss, .heap, stack
  0x4037C000  .iram.text    via direct SRAM access

@author	Hoang Nguyen Hoan
@date	Mar. 2026

@license	MIT
----------------------------------------------------------------------------*/

OUTPUT_ARCH( "riscv" )
ENTRY( ResetEntry )

GROUP( -lstdc++ -lgcc -lc )

MEMORY
{
    /*
     * LMA region: physical flash, addresses recorded in the .hex/.bin.
     * IOcomposer / esptool flashes bytes at these offsets.
     */
    FLASH      (R)   : ORIGIN = 0x00000000, LENGTH = 0x400000

    /*
     * VMA regions: where symbols resolve and code executes from.
     */
    IROM       (RX)  : ORIGIN = 0x42000000, LENGTH = 0x400000
    DROM       (R)   : ORIGIN = 0x3C000000, LENGTH = 0x400000
    IRAM       (RWX) : ORIGIN = 0x4037C000, LENGTH = 0x13C00
    RAM        (RW)  : ORIGIN = 0x3FC80000, LENGTH = 0x50000
}

/*
 * RESET_VECTOR: VMA where ResetEntry must live.
 * ROM jumps to FLASH+8 after the magic check; that physical offset
 * maps to virtual 0x42000008 via the IROM cache.
 */
RESET_VECTOR        = 0x42000008;

/* RISC-V ABI: small-data anchor.  ResetEntry loads this into gp. */
__global_pointer$   = ORIGIN(RAM) + 0x800;

/* Stack starts at top of RAM, grows down.  ResetEntry sets sp to here. */
__StackTop          = ORIGIN(RAM) + LENGTH(RAM);


SECTIONS
{
    /*-----------------------------------------------------------------------
     * Direct Boot magic header.
     * VMA: 0x42000000 (IROM cache base).  LMA: 0x00000000 (flash offset 0).
     * 8 bytes: 1d 04 db ae 1d 04 db ae  (0xaedb041d little-endian, twice).
     *---------------------------------------------------------------------*/
    .dboot_hdr ORIGIN(IROM) : AT(ORIGIN(FLASH))
    {
        LONG(0xaedb041d);
        LONG(0xaedb041d);
    } > IROM

    /*-----------------------------------------------------------------------
     * Reset entry.
     * VMA: 0x42000008.  LMA: 0x00000008 (immediately after the magic).
     * ResetEntry's __attribute__((section(".reset"))) drops it here.
     *---------------------------------------------------------------------*/
    .reset RESET_VECTOR : AT(ORIGIN(FLASH) + 8)
    {
        KEEP(*(.reset))
        . = ALIGN(4);
    } > IROM

    /*-----------------------------------------------------------------------
     * Code: .text.  XIP from IROM, stored in flash directly after .reset.
     *---------------------------------------------------------------------*/
    .text : ALIGN(4)
    {
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7t)
        *(.glue_7)
        *(.eh_frame*)
        . = ALIGN(4);
    } > IROM AT > FLASH

    /*-----------------------------------------------------------------------
     * Read-only data: .rodata.  XIP from DROM cache window.
     *---------------------------------------------------------------------*/
    .rodata : ALIGN(4)
    {
        *(.rodata)
        *(.rodata.*)
        *(.srodata)
        *(.srodata.*)
        *(.gnu.linkonce.r.*)
        . = ALIGN(4);
    } > DROM AT > FLASH

    /*-----------------------------------------------------------------------
     * Initialised data: .data.
     * VMA in RAM (where it executes), LMA in FLASH (where ResetEntry copies
     * from at boot).  __data_loc__ is the LMA, used by ResetEntry as src.
     *---------------------------------------------------------------------*/
    .data : ALIGN(4)
    {
        __data_start__ = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        *(.sdata)
        *(.sdata.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM AT > FLASH

    __data_loc__   = LOADADDR(.data);
    __data_size__  = __data_end__ - __data_start__;
    __etext        = LOADADDR(.data);   /* ResetEntry alternate alias */

    /*-----------------------------------------------------------------------
     * IRAM code: trap handler + time-critical ISRs.
     * VMA in IRAM (fast SRAM, no cache fault during ISR).
     * LMA in FLASH.  TODO: ResetEntry must be extended to copy from
     *   __iram_loc__ to __iram_start__ (size __iram_size__) at boot.
     *   Until that lands, traps jump to uninitialised IRAM.  Blinky does
     *   not generate any traps, so this is safe for first-boot validation.
     *---------------------------------------------------------------------*/
    .iram.text : ALIGN(4)
    {
        __iram_start__ = .;
        KEEP(*(.iram.text))
        *(.iram.text.*)
        . = ALIGN(4);
        __iram_end__ = .;
    } > IRAM AT > FLASH

    __iram_loc__   = LOADADDR(.iram.text);
    __iram_size__  = __iram_end__ - __iram_start__;

    /*-----------------------------------------------------------------------
     * Zero-init data: .bss + .sbss + COMMON.  Cleared by ResetEntry.
     * NOLOAD => no flash bytes, no LMA.
     *---------------------------------------------------------------------*/
    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_start__ = .;
        *(.bss)
        *(.bss.*)
        *(.sbss)
        *(.sbss.*)
        *(.gnu.linkonce.b.*)
        *(.gnu.linkonce.sb.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    __bss_size__ = __bss_end__ - __bss_start__;

    /*-----------------------------------------------------------------------
     * Heap: 2/3 of remaining RAM.  Used by _sbrk() for malloc / sbrk path.
     *---------------------------------------------------------------------*/
    .heap (NOLOAD) : ALIGN(8)
    {
        __heap_start__ = .;
        __end__        = .;     /* libc/_sbrk alias */
        end            = .;
        . = . + ((LENGTH(RAM) - (. - ORIGIN(RAM))) * 2 / 3);
        . = ALIGN(8);
        __heap_end__   = .;
        __HeapLimit    = .;
    } > RAM

    __heap_size__ = __heap_end__ - __heap_start__;
    __HeapSize    = __heap_size__;

    /*-----------------------------------------------------------------------
     * Stack guard: ensures the rest of RAM is reserved for stack growth
     * down from __StackTop.  No bytes emitted.
     *---------------------------------------------------------------------*/
    .stack_dummy (NOLOAD) :
    {
        . = ALIGN(8);
        __StackLimit = .;
    } > RAM

    PROVIDE(__stack = __StackTop);

    ASSERT(__StackLimit < __StackTop,
           "RAM exhausted: heap collides with stack")
    ASSERT(__heap_start__ < __StackTop,
           "RAM exhausted: heap region collides with stack")

    /*-----------------------------------------------------------------------
     * Discard: build artefacts that don't belong in the image.
     *---------------------------------------------------------------------*/
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
    }
}
