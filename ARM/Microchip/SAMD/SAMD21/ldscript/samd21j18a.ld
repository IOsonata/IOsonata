/* SAMD21J18A Xplained Pro: 256 KB Flash, 32 KB SRAM.
 * Include the IOsonata ARM generic section layout via the linker search path.
 */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}
INCLUDE gcc_arm_flash.ld
