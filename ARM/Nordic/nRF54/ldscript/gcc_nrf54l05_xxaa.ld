/* Linker script to configure memory regions. */

SEARCH_DIR(".")
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x7D000 /* Inside global RRAM0 */
  NVM0 (rw) : ORIGIN = 0x07D000, LENGTH = 0x3000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x18000
}


SECTIONS
{
	nvm0 (NOLOAD) :
	{
		__start_nvm0 = .;
		KEEP(*(nvm0))
		. = ORIGIN(NVM0) + LENGTH(NVM0);
		__stop_nvm0 = .;
	} > NVM0
}

INCLUDE "gcc_arm_flash.ld"
