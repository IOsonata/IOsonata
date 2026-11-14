/* STM32WBA65I PowerOffDemo only. Keep test buffers outside startup RAM. */
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
GROUP(-lstdc++ -lgcc -lc)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x1F8000
	NVM0 (rw) : ORIGIN = 0x081F8000, LENGTH = 0x4000
	NVM1 (rw) : ORIGIN = 0x081FC000, LENGTH = 0x4000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x68000
	RAMDROP (rw) : ORIGIN = 0x20068000, LENGTH = 0x8000
	RAMRET (rw) : ORIGIN = 0x20070000, LENGTH = 0x10000
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
	ramdrop (NOLOAD) :
	{
		__start_ramdrop = .;
		KEEP(*(ramdrop))
		. = ORIGIN(RAMDROP) + LENGTH(RAMDROP);
		__stop_ramdrop = .;
	} > RAMDROP
	ramret (NOLOAD) :
	{
		__start_ramret = .;
		KEEP(*(ramret))
		. = ORIGIN(RAMRET) + LENGTH(RAMRET);
		__stop_ramret = .;
	} > RAMRET
}
INCLUDE "gcc_arm_flash.ld"
