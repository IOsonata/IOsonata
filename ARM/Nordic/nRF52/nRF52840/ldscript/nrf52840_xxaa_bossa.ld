/* Linker script for nRF52840 boards with the Arduino (bossa) bootloader,
	such as the Arduino Nano 33 BLE. The bootloader occupies the first 64 KB.
	It sets VTOR to 0x10000 and starts the application from there. */
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	/* nRF52840 has 1MB Flash (0x100000), 256KB RAM (0x40000) */

	/* App flash starts after the bootloader and stops short of the store
	   below, so an image that grows into it fails to link rather than
	   overlapping it. */
	FLASH (rx) : ORIGIN = 0x00010000, LENGTH = 0xED000
	EXTFLASH (rx) : ORIGIN = 0x12000000, LENGTH = 0x8000000
	/* The bond store, same place as nrf52840_xxaa_sdc.ld. NvmRegionAddr(0)
	   reads the section below, so nothing states the address twice. */
	NVM0 (rw) : ORIGIN = 0x000FD000, LENGTH = 0x00003000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x40000
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
