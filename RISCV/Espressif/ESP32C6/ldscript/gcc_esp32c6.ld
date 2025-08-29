OUTPUT_ARCH( "riscv" )

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/RISCV/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
  /* ROM code and data */
  IRAM (RX) : ORIGIN = 0x40300000, LENGTH = 0x10000
  DRAM (RW) : ORIGIN = 0x3FCA0000, LENGTH = 0x30000

  /* External Flash Memory (Mapped into memory space) */
  FLASH_IROM (RX) : ORIGIN = 0x42000020, LENGTH = 0x200000
  FLASH_DROM (R)  : ORIGIN = 0x3C000020, LENGTH = 0x40000
}

/*
 * Section Definitions
 * Map program sections to the defined memory regions.
 */
SECTIONS
{
  /* The .iram.text section contains code that is executed from IRAM. */
  .iram.text :
  {
    . = ALIGN(4);
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
  } > DRAM AT > FLASH_DROM

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
  } > FLASH_IROM

  /* The .bss section contains uninitialized data. */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _bss_start = ABSOLUTE(.);
    *(.bss)
    *(.bss.*)
    . = ALIGN(4);
    _bss_end = ABSOLUTE(.);
  } > DRAM
}

INCLUDE "gcc_flash.ld"
