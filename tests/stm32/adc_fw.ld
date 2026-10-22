/* Link layout of the STM32 ADC emulator test firmware */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x1F8000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x80000
}

ENTRY(ResetHandler)

SECTIONS
{
	.isr_vector : { KEEP(*(.isr_vector)) } > FLASH
	.text : { *(.text*) *(.rodata*) *(.init*) *(.fini*) . = ALIGN(4); } > FLASH
	.ARM.exidx : { *(.ARM.exidx*) } > FLASH
	_sidata = LOADADDR(.data);
	.data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
	.bss : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
	_estack = ORIGIN(RAM) + LENGTH(RAM);
}
