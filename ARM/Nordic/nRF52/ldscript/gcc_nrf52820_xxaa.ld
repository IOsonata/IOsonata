/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x3F000
  NVM0 (rw) : ORIGIN = 0x3F000, LENGTH = 0x1000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000
  CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x8000
}


INCLUDE "nrf_common.ld"


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
