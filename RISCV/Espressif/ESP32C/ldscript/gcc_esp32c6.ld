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
  /* Reserve 0x40800000..0x40807FFF for cache before executable SRAM. */
  IRAM (RWX) : ORIGIN = 0x40808000, LENGTH = 0x8000

  /* Main SRAM (data, stack, heap) */
  RAM  (RW)  : ORIGIN = 0x40810000, LENGTH = 0x70000

  /* External Flash -- XIP-mapped via cache */
  FLASH (RX) : ORIGIN = 0x42000000, LENGTH = 0x400000
}

/* Direct Boot: flash offset 0 contains the ROM magic; entry follows it. */
RESET_VECTOR = 0x42000008;
__global_pointer$ = ORIGIN(RAM) + 0x800;

SECTIONS
{
	.dboot_hdr ORIGIN(FLASH) :
	{
		LONG(0xaedb041d);
		LONG(0xaedb041d);
	} > FLASH
}

/* Common sections keep .iram.text.vectors first and aligned to 256 bytes,
 * copy initialized RAM from the unified C6 flash mapping, and place .reset. */
INCLUDE "gcc_riscv_flash.ld"
