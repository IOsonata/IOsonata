/* Nordic nrfx standalone PRBS benchmark linker; not an IOsonata linker layout. */
/* Linker script to configure memory regions. */

SEARCH_DIR(.)
SEARCH_DIR("../../../../../../../../external/nrfx/bsp/stable/mdk")
SEARCH_DIR("../../../../../../../../../external/nrfx/bsp/stable/mdk")
SEARCH_DIR("../../../../../../../../../../external/nrfx/bsp/stable/mdk")

GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x7D000
	NVM0 (rw) : ORIGIN = 0x0007D000, LENGTH = 0x00002000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000
  CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x10000
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

INCLUDE "nrf_common.ld"
