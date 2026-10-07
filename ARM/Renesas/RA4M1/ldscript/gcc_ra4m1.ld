/* IOsonata RA4M1, normal boot: 256 KiB code flash, 32 KiB SRAM.
 * MIT License. Copyright (c) 2026 I-SYST inc.
 * Reuses ARM/src/ResetEntry.c and IOsonata section/symbol names.
 * The RA4M1 option area must never contain ordinary executable code.
 */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(ResetEntry)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

/* Override these with --defsym or a project linker fragment as required. */
__StackSize = DEFINED(__STACK_SIZE) ? __STACK_SIZE : 0x800;
__HeapSize  = DEFINED(__HEAP_SIZE)  ? __HEAP_SIZE  : 0x400;
__StackTop = ORIGIN(RAM) + LENGTH(RAM);
__StackLimit = __StackTop - __StackSize;
PROVIDE(__stack = __StackTop);
PROVIDE(__CheckSum = 0);

SECTIONS
{
    .ivector ORIGIN(FLASH) : ALIGN(256)
    {
        __text_start__ = .;
        __isr_vector = .;
        KEEP(*(.vectors))
    } > FLASH

    .option_setting 0x400 : ALIGN(4)
    {
        KEEP(*(.option_setting))
    } > FLASH

    .Version 0x440 : ALIGN(4)
    {
        KEEP(*(.Version))
        LONG(__CheckSum)
    } > FLASH

    .AppStart : ALIGN(4)
    {
        KEEP(*(.AppStart))
    } > FLASH

    .text : ALIGN(4)
    {
        KEEP(*(.init))
        KEEP(*(.fini))
        KEEP(*crtbegin.o(.ctors))
        KEEP(*crtbegin?.o(.ctors))
        KEEP(*(EXCLUDE_FILE(*crtend.o *crtend?.o) .ctors))
        KEEP(*(SORT(.ctors.*)))
        KEEP(*crtend.o(.ctors))
        KEEP(*crtend?.o(.ctors))
        KEEP(*crtbegin.o(.dtors))
        KEEP(*crtbegin?.o(.dtors))
        KEEP(*(EXCLUDE_FILE(*crtend.o *crtend?.o) .dtors))
        KEEP(*(SORT(.dtors.*)))
        KEEP(*crtend.o(.dtors))
        KEEP(*crtend?.o(.dtors))
        *(.text .text.* .gnu.linkonce.t.*)
    } > FLASH

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.* .rodata1 .gnu.linkonce.r.*)
        *(.eh_frame_hdr .eh_frame)
        *(.gcc_except_table .gcc_except_table.*)
    } > FLASH

    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH
    __text_end__ = .;
    __etext = .;

    .data : ALIGN(4)
    {
        __data_start__ = .;
        PROVIDE(__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE(__preinit_array_end = .);
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
        PROVIDE(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__fini_array_end = .);
        . = ALIGN(4);
        *(vtable)
        *(.data .data.* .gnu.linkonce.d.*)
        *(.fastrun .fastrun.*)
        *(.jcr)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM AT > FLASH
    __data_loc__ = LOADADDR(.data);
    __data_size__ = SIZEOF(.data);
    __flash_image_end__ = LOADADDR(.data) + SIZEOF(.data);

    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_start__ = .;
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM
    __bss_size__ = SIZEOF(.bss);

    .noinit (NOLOAD) : ALIGN(4)
    {
        *(.noinit .noinit.*)
        . = ALIGN(8);
    } > RAM

    .heap (NOLOAD) : ALIGN(8)
    {
        __HeapBase = .;
        __heap_start__ = .;
        __end__ = .;
        PROVIDE(end = .);
        PROVIDE(_end = .);
        _pvHeapStart = .;
        KEEP(*(.heap .heap.*))
        . = MAX(., __HeapBase + __HeapSize);
        . = ALIGN(8);
        __HeapLimit = .;
        __heap_end__ = .;
    } > RAM
    /* Absolute byte count, never an expression using the SRAM base address. */
    __heap_size__ = __HeapLimit - __HeapBase;

    .stack __StackLimit (NOLOAD) : ALIGN(8)
    {
        KEEP(*(.stack .stack.*))
        . = MAX(., __StackTop);
    } > RAM

    /DISCARD/ : { *(.note.GNU-stack) }
}
ASSERT(ADDR(.ivector) == 0, "RA4M1 normal-boot vector must start at zero")
ASSERT(SIZEOF(.ivector) == 192, "RA4M1 vector must contain exactly 48 words")
ASSERT(SIZEOF(.ivector) <= 0x400, "RA4M1 vector overlaps boot options")
ASSERT(SIZEOF(.option_setting) == 0x3C, "Missing or duplicate RA4M1 boot options")
ASSERT(ADDR(.Version) >= 0x43C, "RA4M1 code overlaps option-setting memory")
ASSERT(__flash_image_end__ <= ORIGIN(FLASH) + LENGTH(FLASH), "RA4M1 flash overflow")
ASSERT(__StackSize >= 0x100 && (__StackSize % 8) == 0, "Invalid RA4M1 stack size")
ASSERT(__HeapLimit <= __StackLimit, "RA4M1 data/heap overlaps stack")
ASSERT(SIZEOF(.stack) <= __StackSize, "RA4M1 explicit stack exceeds reservation")
