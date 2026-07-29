/* Linker script for STM32WBA52/55: 1 MB flash, 128 KB contiguous SRAM,
 * with the IOsonata NVM storage regions.
 */

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


MEMORY
{

	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0xF8000	/* stops at the stores */
	/* The two storage partitions, named so the linker owns the layout:
	   FLASH stops short of them, so an image that grows into a store
	   fails to link. NvmRegionAddr reads the sections below, so nothing
	   states the addresses twice. 8 KB pages on WBA5x, two per region,
	   the store's minimum. WBA52/55 are single bank: a program or erase
	   stalls a core fetching from flash for the operation, which the
	   storage code accepts and no placement can avoid on these parts. */
	NVM0 (rw) : ORIGIN = 0x80F8000, LENGTH = 0x4000		/* bond store */
	NVM1 (rw) : ORIGIN = 0x80FC000, LENGTH = 0x4000		/* application */
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* === The storage regions ===
 *
 * The bounds arrive as __start_ and __stop_ symbols, so the size is the
 * distance between them and the length is stated once, in the MEMORY
 * line. NOLOAD because nothing is programmed here at flash time; a store
 * writes it at run time. KEEP lets a variable be placed in one if
 * anything ever wants to.
 */
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
