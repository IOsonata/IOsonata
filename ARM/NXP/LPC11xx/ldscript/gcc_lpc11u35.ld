/*
 * LPC11xx
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
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
GROUP(-lstdc++ -lgcc -lc -lnosys)


MEMORY
{
  
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
  	RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
  	RAM2 (rwx) : ORIGIN = 0x20000000, LENGTH = 2K
  	RAM3 (rwx) : ORIGIN = 0x20004000, LENGTH = 2K
}

INCLUDE "gcc_arm_flash.ld"



