/* Raspberry Pi Pico 2: secure Arm image, vectors at flash origin. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 4M
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 520K
}
EXTERN(__Vectors RpImageDef)
INCLUDE gcc_arm_flash.ld
ASSERT(RpImageDef < ORIGIN(FLASH) + 4096, "image definition outside boot ROM search window")
