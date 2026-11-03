/* NVM0: 8 KB Bluetooth PDS; NVM1: 4 KB user storage.
 * No MBR/DFU: storage occupies the top 12 KB of flash.
 * Flash reservations are page-aligned (nRF52 4 KB erase pages).
 */
/* Linker script to configure memory regions. */
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
	/* nRF52840 has 1MB Flash (0x100000), 256KB RAM (0x40000) */

	/* App flash stops short of the store below, so an image that grows
	   into it fails to link rather than overlapping it. */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0xFD000
	EXTFLASH (rx) : ORIGIN = 0x12000000, LENGTH = 0x8000000
	/* The bond store. Declared here so the linker owns the layout: FLASH
	   above stops short of it, so an image that grows into the store fails
	   to link. NvmRegionAddr(0) reads the section below, so nothing
	   states the address twice. */
	NVM0 (rw) : ORIGIN = 0x000FD000, LENGTH = 0x2000
  NVM1 (rw) : ORIGIN = 0x000FF000, LENGTH = 0x1000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x40000
}

SECTIONS
{
	/* The region as a section, so the reservation shows up in the map file
	   with its size and the bounds arrive as __start_ and __stop_ symbols,
	   the naming the SDK uses for its own section variables. KEEP lets a
	   variable be placed in here later if anything ever wants to. */
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
