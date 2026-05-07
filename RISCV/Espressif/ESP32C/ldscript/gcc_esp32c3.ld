/**-------------------------------------------------------------------------
@file	gcc_esp32c3.ld

@brief	ESP32-C3 chip-specific linker script for Direct Boot.

	Companion to the shared gcc_riscv_flash.ld.  Defines what's
	C3-specific (memory map, Direct Boot magic, ABI anchors) and
	delegates the standard section layout (.reset, .text, .rodata,
	.iram.text, .data, .bss, .heap) to the shared script.

	Memory layout (ESP32-C3 TRM Rev 0.4 §3):
	  FLASH  0x42000000  4 MB    IROM cache window — code XIP
	  IRAM   0x4037C000  80 KB   Internal SRAM, instruction-side
	  RAM    0x3FC80000  320 KB  Internal SRAM, data-side

	Note on FLASH region: we use the IROM cache window (0x42000000)
	rather than the physical flash address space (0x00000000) as
	the code region.  All `> FLASH` sections in the shared script
	end up at IROM virtual addresses, and `objcopy -O binary`
	produces a flat image starting from the lowest LMA = ORIGIN(FLASH)
	= 0x42000000.  The bytes of that image, when flashed at physical
	flash offset 0, are exactly what the C3 ROM's IROM cache will
	read back — because ROM Direct Boot maps physical flash 0..size
	1:1 to IROM.  This avoids needing a separate physical-FLASH
	memory region and keeps __data_loc__ / __iram_loc__ pointing at
	cache-readable addresses.

	Direct Boot (silicon rev 3+, EFUSE_DIS_LEGACY_SPI_BOOT == 0):
	  ROM checks flash[0..7] for 1d 04 db ae 1d 04 db ae.  On match,
	  ROM enables IROM/DROM cache mapping covering the whole flash
	  and jumps to flash + 8 = IROM 0x42000008.
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
    /* Code XIP region — IROM cache mapped to flash physical 0..0x3FFFFF.
     * objcopy -O binary outputs from the lowest LMA in this region. */
    FLASH  (RX)  : ORIGIN = 0x42000000, LENGTH = 0x400000

    /* IRAM — fast SRAM accessed from the instruction bus.  Reserved for
     * trap handlers and other I-side code.  Not used directly here:
     * shared script's .iram.text uses RAM, which aliases the same
     * physical SRAM via the data bus. */
    IRAM   (RWX) : ORIGIN = 0x4037C000, LENGTH = 0x13C00

    /* Main SRAM — .data, .bss, heap, stack. */
    RAM    (RW)  : ORIGIN = 0x3FC80000, LENGTH = 0x50000
}

/* RESET_VECTOR: where ResetEntry must live.  Direct Boot magic occupies
 * flash 0..7, so ResetEntry starts at flash 8 = IROM 0x42000008. */
RESET_VECTOR        = 0x42000008;

/* RISC-V ABI small-data anchor.  ResetEntry loads this into gp. */
__global_pointer$   = ORIGIN(RAM) + 0x800;

SECTIONS
{
    /*-----------------------------------------------------------------------
     * Direct Boot magic header.  8 bytes at FLASH origin.  Must occupy
     * flash 0..7.  ROM checks these on reset to enable Direct Boot.
     *---------------------------------------------------------------------*/
    .dboot_hdr ORIGIN(FLASH) :
    {
        LONG(0xaedb041d);
        LONG(0xaedb041d);
    } > FLASH
}

INCLUDE "gcc_riscv_flash.ld"
