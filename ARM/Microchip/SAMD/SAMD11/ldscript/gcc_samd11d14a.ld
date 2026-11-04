/* SAMD11D14A Xplained Pro: 16 KB Flash, 4 KB SRAM.
 * Include the IOsonata ARM generic section layout via the linker search path.
 */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 16K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}
INCLUDE gcc_arm_flash.ld
