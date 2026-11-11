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

/* Direct Boot, flash offset zero. Reserve upper SRAM used by ROM.
 * Memory limits checked against ESP-IDF v5.5.1 esp32c61 memory.ld.in.
 * C2 reserves cache SRAM and uses separate instruction/data aliases.
 */
MEMORY
{
	IRAM (RWX) : ORIGIN = 0x40800000, LENGTH = 0x4000
	RAM (RW) : ORIGIN = 0x40804000, LENGTH = 0x38000
	FLASH (RX) : ORIGIN = 0x42000000, LENGTH = 0x200000
}
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
INCLUDE "gcc_riscv_flash.ld"
