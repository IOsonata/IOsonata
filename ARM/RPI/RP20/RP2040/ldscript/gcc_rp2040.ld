/* Raspberry Pi Pico: boot ROM loads and verifies the first 256 bytes. */
MEMORY
{
	BOOT2 (rx) : ORIGIN = 0x10000000, LENGTH = 256
	FLASH (rx) : ORIGIN = 0x10000100, LENGTH = 2M - 256
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 264K
}
EXTERN(RpBoot2 __Vectors)
SECTIONS { .boot2 : { KEEP(*(.boot2)) } > BOOT2 }
ASSERT(SIZEOF(.boot2) == 256, "RP2040 boot2 must be 256 bytes")
INCLUDE gcc_arm_flash.ld
