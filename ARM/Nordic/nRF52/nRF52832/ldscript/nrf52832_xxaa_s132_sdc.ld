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

	/* App flash ends at 0x7D000, leaving two pages (0x7D000..0x7EFFF) for the
	   BLE bond store (bt_pds NVMC backend) and one page (0x7F000) for the
	   bootloader settings. Keep BT_PDS_NVMC_REGION_ADDR=0x7D000 in sync. */
	FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0x57000
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