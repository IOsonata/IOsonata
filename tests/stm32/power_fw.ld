/* Link layout of the STM32 power functions emulator test firmware */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x1F8000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x80000
}

ENTRY(ResetHandler)

SECTIONS
{
	.text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
	.ARM.exidx : { *(.ARM.exidx*) } > FLASH
	.data : { *(.data*) . = ALIGN(4); } > RAM AT > FLASH
	.bss : { *(.bss*) *(COMMON) . = ALIGN(4); } > RAM
}
