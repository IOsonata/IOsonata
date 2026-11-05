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

/* Full code flash and SRAM. Data EEPROM is a separate mapped region. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
	NVM0 (rw) : ORIGIN = 0x08080000, LENGTH = 0x4000 /* dedicated data EEPROM */
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 80K
	RAM2 (rwx) : ORIGIN = 0x20014000, LENGTH = 0
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
