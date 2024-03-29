/*--------------------------------------------------------------------------
File   : gcc_arm_flash.ld

Author : Hoang Nguyen Hoan          Jan. 19, 2012

Desc   : Generic common linker script sections.  General sections
         definitions for of GCC compiler.  This file is to be inlcuded
	 	in the device specific linker script.

	 	Require memory region name definition as follow
		 
	        FLASH : Read only code memory region
	        RAM : Ram memory region for data  

Copyright (c) 2012, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by        	Date        	Description
Hoan				July 3, 2015	Add Freescale Flash security config area	
----------------------------------------------------------------------------*/

/* 
Require custom section definitions
   .vectors - ARM interrupt vector
   .Version - Firmware version data 
*/

ENTRY(ResetEntry)


__CheckSum = 0;
__StackTop = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {

    /* Require ARM vectors located at begining of FLASH region at address 0*/
	/* Reseve 1K for the vector */
	.ivector : ALIGN(4)
	{
		__text_start__ = .;
		__isr_vector = .;
		/* At addr zero is the stack pointer */
		/*LONG(__StackTop);*/

		/* interrupt vector */
		KEEP(*(.vectors*))
	} > FLASH 
	
	/* This is where the firmware informaton is stored. Usefull for bootloader */

	.Version : ALIGN(4)
	{
		KEEP(*(.Version))
		LONG(__CheckSum);
		FILL(0xff)  /* fill unsused area to have consistant checksum */
/*		. = 0x400 - SIZEOF(.ivector);/* - SIZEOF(.Version);*/
	} > FLASH
		
	.AppStart : ALIGN(4)
	{
		/* Application Entry point starts here.  Normaly is the ResetEntry function */ 
		KEEP(*(.AppStart))
	} > FLASH
	
	/* Code region starts here */
	.text : ALIGN(4)
	{	
		KEEP(*(.init))
		KEEP(*(.fini))
		
		/* .ctors */
		KEEP(*crtbegin.o (.ctors))
		KEEP(*crtbegin?.o(.ctors))
		KEEP(*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
		KEEP(*(SORT(.ctors.*)))
		KEEP(*crtend.o (.ctors))

		/* .dtors */
 		KEEP(*crtbegin.o(.dtors))
 		KEEP(*crtbegin?.o(.dtors))
 		KEEP(*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
 		KEEP(*(SORT(.dtors.*)))
 		KEEP(*crtend.o (.dtors))
		*(.text*)
		*(.gnu.linkonce.r.*)
	} > FLASH
    
	.rodata : ALIGN(4) 
	{ 
		*(.rodata .rodata.* .rodata1)
	} > FLASH

	.ARM.extab : ALIGN(4)
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	PROVIDE (__exidx_start = .);
	.ARM.exidx : ALIGN(4)
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > FLASH
	PROVIDE (__exidx_end = .);

	.eh_frame_hdr : ALIGN(4)
	{ 
		*(.eh_frame_hdr) 
	} > FLASH
	
	.eh_frame : ONLY_IF_RO 
	{ 
		. = ALIGN(4);
		*(.eh_frame)
	} > FLASH
	
	.gcc_except_table : ONLY_IF_RO 
	{ 
		. = ALIGN(4);
		*(.gcc_except_table .gcc_except_table.*) 
	} > FLASH
	
	. = ALIGN(4);
	__text_end__ = .;
    __etext = .;
	
	.eh_frame : ONLY_IF_RW 
	{ 
		*(.eh_frame) 
	} > RAM
	
	.gcc_except_table : ONLY_IF_RW 
	{ 
		*(.gcc_except_table .gcc_except_table.*) 
	} > RAM
	
		
	.data : ALIGN(4)
	{
		/* preinit data */
		PROVIDE (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE (__init_array_start = .);
		*(SORT(.init_array.*))
		KEEP(*(.init_array))
		PROVIDE (__init_array_end = .);

		. = ALIGN(4);
		/* finit data */
		PROVIDE (__fini_array_start = .);
		*(SORT(.fini_array.*))
		KEEP(*(.fini_array))
		PROVIDE (__fini_array_end = .);

		. = ALIGN(4);        

		*(vtable)
		*(.data*)
		*(.fastrun*)

		*(.jcr)

		/* All data end */
		__data_end__ = .;

	} > RAM AT > FLASH
	
	__data_start__ = ADDR(.data);
   	__data_loc__ = LOADADDR(.data);

   	/* calculate size to take full range from .data to .bss in case other data sections
   	   were inserted in between from outside of this file */
   	__data_size__ = __bss_start__ - __data_start__ ; /* SIZEOF(.data);*/
	
	.bss : ALIGN(4)
	{
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
      	*(.gnu.linkonce.b.*)
		__bss_end__ = .;
		PROVIDE(end = .);		              
	} > RAM
        __bss_size__ = SIZEOF(.bss);
	
	.heap : ALIGN(4)
	{
      __HeapBase = .;
		__end__ = .;
		end = __end__;
		__heap_start__ = __end__;
		_pvHeapStart = .;
		KEEP(*(.heap*))

		__heap_size__ = 	(ORIGIN(RAM) + LENGTH(RAM)) * 2 / 3;
		__HeapSize = __heap_size__;
		__heap_end__ = __heap_start__ + __heap_size__;
		__HeapLimit = __HeapBase + __HeapSize;
	} > RAM

	/* .stack_dummy section doesn't contains any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy : ALIGN(4)
	{
		KEEP(*(.stack*))
	} > RAM

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackLimit = __HeapLimit + 4;/*__StackTop - SIZEOF(.stack_dummy);*/
	PROVIDE(__stack = __StackTop);
	
	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit < __StackTop, "region RAM overflowed with stack")
	ASSERT(__heap_start__ < __StackTop, "region RAM overflowed with heap")

}

