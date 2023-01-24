/* Linker script to configure memory regions. */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
	EXTFLASH (rx) : ORIGIN = 0x10000000, LENGTH = 0x8000000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	RAM1 (rwx) : ORIGIN = 0x20040000, LENGTH = 0x40000
}


INCLUDE "gcc_arm_flash.ld"
