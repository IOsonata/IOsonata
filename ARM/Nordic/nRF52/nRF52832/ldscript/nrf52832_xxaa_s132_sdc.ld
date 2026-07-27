OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
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
	/* nRF52832 has 512KB Flash (0x80000), 64KB RAM (0x10000) */

	/* App flash stops short of the store below, so an image that grows
	   into it fails to link rather than overlapping it. */
	FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0x57000
	/* The bond store. Declared here so the linker owns the layout: FLASH
	   above stops short of it, so an image that grows into the store fails
	   to link. NvmRegionAddr(0) reads the section below, so nothing
	   states the address twice. */
	NVM0 (rw) : ORIGIN = 0x0007D000, LENGTH = 0x00002000
	RAM (rwx) :  ORIGIN = 0x20000400, LENGTH = 0xFC00

	/** Location of non initialized RAM. Non initialized RAM is used for exchanging bond information
	*  from application to bootloader when using buttonless DFU OTA.
	*/
	NOINIT (rwx) :  ORIGIN = 0x20007F80, LENGTH = 0x80

	/** BLE bond store: two flash pages, written by the bt_pds NVMC backend
	    through MPSL timeslots. Reserved here so the application does not place
	    code in it. */
	BT_PDS (rw) : ORIGIN = 0x0007D000, LENGTH = 0x2000

	/** Location of bootloader setting in flash. */
	BOOTLOADER_SETTINGS (rw) : ORIGIN = 0x0007F000, LENGTH = 0x1000
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
}


SECTIONS
{
	/* Place the bootloader settings page in flash. */
	.bootloaderSettings(NOLOAD) :
	{

	} > BOOTLOADER_SETTINGS

	/* No init RAM section in bootloader. Used for bond information exchange. */
	.noinit(NOLOAD) :
	{

	} > NOINIT
}


INCLUDE "gcc_arm_flash.ld"