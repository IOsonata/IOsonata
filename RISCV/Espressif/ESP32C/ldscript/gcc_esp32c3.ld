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
  /*
   * IRAM: time-critical code (trap handler, INTMTX dispatch, ISRs).
   * ESP32-C3 TRM Rev 0.4 §3 — Internal SRAM 0 mapped as IRAM.
   * Origin: 0x4037C000  Size: 0x13C00 (~80 KB)
   */
  IRAM  (RWX) : ORIGIN = 0x4037C000, LENGTH = 0x13C00

  /*
   * DRAM: data, BSS, stack, heap.
   * Internal SRAM 1 mapped as DRAM (remainder after IRAM split).
   * Origin: 0x3FC80000  Size: 0x50000 (320 KB)
   */
  RAM   (RW)  : ORIGIN = 0x3FC80000, LENGTH = 0x50000

  /*
   * Flash XIP — code mapped via instruction cache.
   * ROM bootloader loads and verifies the image; execution jumps here.
   */
  FLASH (RX)  : ORIGIN = 0x42000000, LENGTH = 0x400000

  /*
   * Flash DROM — read-only data mapped via data cache.
   */
  FLASH_DROM (R) : ORIGIN = 0x3C000000, LENGTH = 0x400000
}

/*
 * RESET_VECTOR: entry point in Flash XIP space.
 * The ROM bootloader jumps here after loading the secondary bootloader
 * (or directly to the app when loaded via JTAG/OpenOCD).
 * Must match the address configured in esptool / OpenOCD flash map.
 */
RESET_VECTOR = 0x42000020;

/*
 * __global_pointer$: RISC-V GP-relative addressing anchor.
 * ResetEntry.c loads this into the gp register before SystemInit().
 * Placed 0x800 bytes into RAM so the +/-2 KB GP window covers the
 * most-accessed .data and .bss.
 */
__global_pointer$ = ORIGIN(RAM) + 0x800;

SECTIONS
{
  /*
   * IRAM: trap handler + time-critical ISR code.
   *
   * RISCV_TrapHandler must be in IRAM — it can be entered during a
   * cache fault when Flash XIP is temporarily unavailable.
   */
  .iram.text :
  {
    . = ALIGN(4);
    KEEP(*(.iram.text))
    *(.iram.text.*)
    . = ALIGN(4);
  } > IRAM

  /*
   * Vector (reset entry point) in Flash at RESET_VECTOR address.
   * ResetEntry.c provides the _start symbol.
   */
  .vectors RESET_VECTOR :
  {
    KEEP(*(.vectors))
  } > FLASH

  /* Main executable code in Flash XIP */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  /* ARM-style exception tables (C++ unwind, etc.) */
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } > FLASH

  /* Read-only data — mapped via DROM cache window */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    . = ALIGN(4);
  } > FLASH_DROM

  /* Initialised data — load from Flash, run in RAM */
  .data :
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    . = ALIGN(4);
    __data_end__ = .;
  } > RAM AT > FLASH_DROM

  __data_load__ = LOADADDR(.data);

  /* Zero-initialised data */
  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > RAM

  /* Stack and heap bookkeeping */
  ._stack :
  {
    . = ALIGN(8);
    __stack_start__ = .;
  } > RAM

  end = .;
  PROVIDE(heap_end = ORIGIN(RAM) + LENGTH(RAM));
}
