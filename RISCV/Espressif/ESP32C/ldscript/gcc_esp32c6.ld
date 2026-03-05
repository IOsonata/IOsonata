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
  /* IRAM: time-critical code (interrupt vectors, cache-miss-safe handlers) */
  IRAM (RWX) : ORIGIN = 0x40800000, LENGTH = 0x10000

  /* Main SRAM (data, stack, heap) */
  RAM  (RW)  : ORIGIN = 0x40810000, LENGTH = 0x70000

  /* External Flash -- XIP-mapped via cache */
  FLASH (RX) : ORIGIN = 0x42000000, LENGTH = 0x400000

  /* Flash data (read-only data mapped via cache at DROM window) */
  FLASH_DROM (R) : ORIGIN = 0x3C000000, LENGTH = 0x400000
}

/*
 * RESET_VECTOR: entry point in Flash XIP space.
 * The ROM bootloader jumps here after loading the secondary bootloader
 * (or directly to the app if no bootloader is used with JTAG loading).
 * Must match the address set in esptool / OpenOCD flash map.
 */
RESET_VECTOR = 0x42000020;

/*
 * __global_pointer$: the ABI small-data anchor for RISC-V GP-relative
 * addressing.  ResetEntry.c loads this into the gp register before
 * calling SystemInit().  Placed 0x800 bytes into RAM so that the +/-2KB
 * GP-relative window covers the most-used data section.
 */
__global_pointer$ = ORIGIN(RAM) + 0x800;

/*
 * Section Definitions
 * Map program sections to the defined memory regions.
 */
SECTIONS
{
  /*
   * IRAM: CLIC interrupt vector table + time-critical ISR code.
   * g_Esp32C6Vectors is placed here via __attribute__((section(".iram.text"))).
   * The 512-byte alignment satisfies the CLIC mtvec alignment requirement
   * (96 entries x 4 bytes = 384 bytes -> next power-of-2 = 512).
   */
  .iram.text :
  {
    . = ALIGN(512);
    _iram_text_start = ABSOLUTE(.);
    *(.iram.text)
    *(.iram.text.*)
    . = ALIGN(4);
    _iram_text_end = ABSOLUTE(.);
  } > IRAM

  /* The .dram.data section contains initialized data variables. */
  .dram.data :
  {
    . = ALIGN(4);
    _dram_data_start = ABSOLUTE(.);
    *(.dram.data)
    *(.dram.data.*)
    . = ALIGN(4);
    _dram_data_end = ABSOLUTE(.);
  } > RAM AT > FLASH_DROM

  /* The .rodata section contains read-only data (constants). */
  .rodata :
  {
    . = ALIGN(4);
    _rodata_start = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
    _rodata_end = ABSOLUTE(.);
  } > FLASH_DROM

  /* The .irom.text section contains the main program code executed from Flash. */
  .irom.text :
  {
    . = ALIGN(4);
    _irom_text_start = ABSOLUTE(.);
    *(.irom.text)
    *(.irom.text.*)
    . = ALIGN(4);
    _irom_text_end = ABSOLUTE(.);
  } > FLASH

  /* The .bss section contains uninitialized data. */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _bss_start = ABSOLUTE(.);
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > RAM
}

/* Stack top: referenced by ResetEntry.c */
__StackTop = ORIGIN(RAM) + LENGTH(RAM);

INCLUDE "gcc_riscv_flash.ld"
