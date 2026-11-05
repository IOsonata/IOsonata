/* Bare-metal nRF52840: 192 KiB application, 832 KiB USB flash disk.
 * No MBR, Nordic DFU, or NVM0; only use without upper-flash bootloader.
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
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x30000
	EXTFLASH (rx) : ORIGIN = 0x12000000, LENGTH = 0x8000000
	/* Dedicated USB flash disk; firmware remains below this boundary. */
	NVM1 (rw) : ORIGIN = 0x00030000, LENGTH = 0xD0000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x40000
}

SECTIONS
{
	/* The region as a section, so the reservation shows up in the map file
	   with its size and the bounds arrive as __start_ and __stop_ symbols,
	   the naming the SDK uses for its own section variables. KEEP lets a
	   variable be placed in here later if anything ever wants to. */
nvm1 (NOLOAD) :
	{
		__start_nvm1 = .;
		KEEP(*(nvm1))
		. = ORIGIN(NVM1) + LENGTH(NVM1);
		__stop_nvm1 = .;
	} > NVM1
}

SECTIONS
{
	. = ALIGN(4);
	.cli_sorted_cmd_ptrs :
	{
		PROVIDE(__start_cli_sorted_cmd_ptrs = .);
		KEEP(*(.cli_sorted_cmd_ptrs))
		PROVIDE(__stop_cli_sorted_cmd_ptrs = .);
	} > RAM AT > FLASH
} INSERT AFTER .data;

SECTIONS
{
	.cli_command :
	{
		PROVIDE(__start_cli_command = .);
		KEEP(*(.cli_command))
		PROVIDE(__stop_cli_command = .);
	} > FLASH
} INSERT AFTER .text;

INCLUDE "gcc_arm_flash.ld"
