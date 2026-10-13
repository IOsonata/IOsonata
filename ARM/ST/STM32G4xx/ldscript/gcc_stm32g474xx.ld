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

/* STM32G474xE : 512 KB flash, two banks of 2 KB pages (dual bank, the reset
   setting), 80 KB SRAM1 and 16 KB SRAM2 contiguous at 0x20000000, 32 KB CCM
   SRAM at 0x10000000. The two storage partitions take the last 16 KB of
   bank 2, four pages each: an erase there does not stall the image running
   from bank 1. FLASH stops short of them, so an image that grows into a
   store fails to link. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0x7C000
	NVM0 (rw) : ORIGIN = 0x807C000, LENGTH = 0x2000		/* bond store */
	NVM1 (rw) : ORIGIN = 0x807E000, LENGTH = 0x2000		/* application */
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
	RAM2 (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
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
