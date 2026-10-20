SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP( -lstdc++ -lgcc -lc)

/* Data and stack in DTCM; AXI SRAM is a separate region.
   No internal-flash storage driver or NVM partitions in this initial port. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM2 (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}
INCLUDE "gcc_arm_flash.ld"
