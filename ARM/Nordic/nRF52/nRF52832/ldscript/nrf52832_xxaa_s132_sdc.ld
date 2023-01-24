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

	FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0x5a000
	RAM (rwx) :  ORIGIN = 0x20000400, LENGTH = 0xFC00

	/** Location of non initialized RAM. Non initialized RAM is used for exchanging bond information
	*  from application to bootloader when using buttonless DFU OTA.
	*/
	NOINIT (rwx) :  ORIGIN = 0x20007F80, LENGTH = 0x80

	/** Location of bootloader setting in flash. */
	BOOTLOADER_SETTINGS (rw) : ORIGIN = 0x0007F000, LENGTH = 0x1000
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