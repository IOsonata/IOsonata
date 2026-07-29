/*
******************************************************************************
**
**  File        : stm32wba5xxx_flash.ld
**
**  Abstract    : Linker script for STM32WBA52/55 (1024K flash, 128K RAM)
**                with the IOsonata NVM storage regions.
**
**                Adapted from the STM32CubeIDE template in cmsis-device-wba
**                (Source/Templates/gcc/linker/STM32WBA52xx_FLASH.ld).
**
**  Copyright (c) 2022 STMicroelectronics, (c) 2026 I-SYST inc.
**  All rights reserved.
**
**  This software is licensed under terms that can be found in the LICENSE
**  file in the root directory of this software component. If no LICENSE
**  file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);
_sstack = _estack - _Min_Stack_Size;

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

/* Memories definition.
 *
 * The two storage partitions are named so the linker owns the layout, the
 * same shape as the Nordic scripts: FLASH stops short of them, so an image
 * that grows into a store fails to link, and NvmRegionAddr reads the
 * sections below, so nothing states the addresses twice.
 *
 * 8 KB flash pages on this family, and the store needs at least two whole
 * pages, so each region is two. WBA52/55 are single bank; a program or
 * erase stalls a core fetching from flash for the operation, which the
 * storage code accepts and the placement cannot avoid on these parts.
 */
MEMORY
{
	RAM    (xrw)  : ORIGIN = 0x20000000, LENGTH = 64K
	RAM2   (xrw)  : ORIGIN = 0x20010000, LENGTH = 64K
	FLASH  (rx)   : ORIGIN = 0x08000000, LENGTH = 0xF8000	/* stops at the stores */
	NVM0   (rw)   : ORIGIN = 0x080F8000, LENGTH = 0x4000	/* bond store */
	NVM1   (rw)   : ORIGIN = 0x080FC000, LENGTH = 0x4000	/* application */
}

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* === The storage regions ===
   *
   * The bounds arrive as __start_ and __stop_ symbols, so the size is the
   * distance between them and the length is stated once, in the MEMORY
   * line. NOLOAD because nothing is programmed here at flash time; a store
   * writes it at run time. KEEP lets a variable be placed in one if
   * anything ever wants to.
   */
  nvm0 (NOLOAD) :
  {
    __start_nvm0 = .;
    KEEP(*(nvm0))
    . = ORIGIN(NVM0) + LENGTH(NVM0);
    __stop_nvm0 = .;
  } >NVM0

  nvm1 (NOLOAD) :
  {
    __start_nvm1 = .;
    KEEP(*(nvm1))
    . = ORIGIN(NVM1) + LENGTH(NVM1);
    __stop_nvm1 = .;
  } >NVM1

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
