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

	/* App flash ends at 0xFD000, leaving two pages (0xFD000..0xFEFFF) for the
	   BLE bond store (bt_pds NVMC backend) and one page (0xFF000) for the
	   bootloader settings. Keep the nRF52840 BT_PDS_NVMC_REGION_ADDR=0xFD000
	   default in bt_pds_sdc_nrf52.cpp in sync with this. */
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0xFD000
	EXTFLASH (rx) : ORIGIN = 0x12000000, LENGTH = 0x8000000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x40000
}


INCLUDE "gcc_arm_flash.ld"
