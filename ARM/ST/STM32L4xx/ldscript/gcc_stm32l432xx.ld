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

/* STM32L432KC : 256 KB flash in one bank of 2 KB pages, 48 KB SRAM1 and
   16 KB SRAM2. The two storage partitions take the last 16 KB of flash,
   four pages each. FLASH stops short of them, so an image that grows into
   a store fails to link. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0x3C000
	NVM0 (rw) : ORIGIN = 0x803C000, LENGTH = 0x2000		/* bond store */
	NVM1 (rw) : ORIGIN = 0x803E000, LENGTH = 0x2000		/* application */
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
	RAM2 (rwx) : ORIGIN = 0x10000000, LENGTH = 16K
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

	nvm1 (NOLOAD) :
	{
		__start_nvm1 = .;
		KEEP(*(nvm1))
		. = ORIGIN(NVM1) + LENGTH(NVM1);
		__stop_nvm1 = .;
	} > NVM1
}

INCLUDE "gcc_arm_flash.ld"
