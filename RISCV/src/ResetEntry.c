/**-------------------------------------------------------------------------
@file	ResetEntry.c

@brief	Generic ResetEntry code for RISC-V with GCC compiler

@author	Hoang Nguyen Hoan
@date	Aug. 20, 2025

@license

MIT

Copyright (c) 2025, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#if defined ( __ARMCC_VERSION )
extern int Image$$ER_ZI$$Length;
extern char Image$$ER_ZI$$Base[];
#else
extern unsigned long __etext;	// Begin of data in FLASH location
extern unsigned long __data_loc__;
extern unsigned long __data_start__;	// RAM data start
extern unsigned long __data_size__;
extern unsigned long __data_end__;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;
extern unsigned long __bss_size__;
extern unsigned long __heap_start__;
extern unsigned long __heap_size__;
extern unsigned long __heap_end__;
#endif

#ifdef __ICCARM__
extern void __iar_init_core( void );
extern void __iar_init_vfp( void );
extern void __iar_data_init3(void);
extern void _call_main(void);
#endif

extern void _start(void);
extern void _rtos_start(void);
extern int main (void);
extern void __libc_init_array(void);
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern uint32_t SystemCoreClock;

extern	void (* const __Vectors[])(void);

// Just to make the linker to keep the __Vectors
__attribute__ ((used)) static uint32_t s_StackPtr = (uint32_t)__Vectors;

// Nop count for usDelay base on ARM NOP instruction timing on 16MHz clock
uint32_t SystemMicroSecLoopCnt = 1;

/**
 *	This is entry point after reset
 */
#if defined ( __ARMCC_VERSION )
__attribute__ ((section (".AppStart")))
void Reset_Handler (void)
{
	/*
	 * Clear the ".bss" segment.
	 */
	memset(Image$$ER_ZI$$Base, 0, (size_t)&Image$$ER_ZI$$Length);
#else
__attribute__ ((section (".AppStart")))
void ResetEntry (void)
{
#ifdef __ICCARM__
	__iar_init_core();
	__iar_init_vfp();
	__iar_data_init3();
#else

	asm volatile("la gp, global_pointer");
	asm volatile("csrw satp, zero");
	asm volatile("la sp, stack_top");

  	/*
	 * Copy the initialized data of the ".data" segment
	 * from the flash to ram.
	 */

	if (&__data_start__ != &__data_loc__)
	{
		size_t dsize = (size_t)&__data_size__;	// this is a workaround for m0+ compiler in debug build
		memcpy((void *)&__data_start__, (void *)&__data_loc__, dsize);
	}

	/*
	 * Clear the ".bss" segment.
	 */
	size_t sz = (uint32_t)&__bss_size__; // this is a workaround for m0+ compiler in debug build
	memset((void *)&__bss_start__, 0, sz);
#endif
#endif

	/*
	 * Core clock initialization using CMSIS
	 */
	SystemInit();

	/*
	 * Call C++ library initialization
	 */
//	__libc_init_array();

	/*
	 * Now memory has been initialized
	 * Update core clock data
	 */
	SystemCoreClockUpdate();

	// Update count for usDelay
	SystemMicroSecLoopCnt = (SystemCoreClock + 8000000) / 16000000;

	/*
	 * We are ready to enter main application
	 */
#ifndef __CMSIS_RTOS
	// Bare bone app
#if defined ( __ARMCC_VERSION )
	main();
#else
#ifdef __ICCARM__
	   //__iar_program_start();
	   _call_main();
#else
	_start();
#endif
#endif
#else
	// RTX based app
	_rtos_start();
#endif

	/*
	 * Embedded system don't return to OS.  main() should not normally
	 * returns.  In case it does, just loop here.
	 */
	while(1);

}

#if 1

__attribute__((weak)) void _exit(int Status)
{
	while(1);
}

__attribute__((weak)) caddr_t _sbrk(int incr)
{
	static uint32_t top = (uint32_t)&__heap_start__;
	uint32_t heap = top;

	top += incr;

	if (top > (uint32_t)&__heap_end__)
	{
		top = heap;
		errno = ENOMEM;

		return (caddr_t)-1; // error - no more memory
	}

	return (caddr_t) heap;
}

__attribute__ ((weak)) int _open(const char * const pPathName, int Flags, int Mode)
{
	return -1;
}

__attribute__ ((weak)) int _close(int Fd)
{
	return -1;
}

__attribute__ ((weak)) int _lseek(int Fd, int Offset)
{
	return -1;
}


__attribute__ ((weak)) int _read (int Fd, char *pBuff, size_t Len)
{
	return -1;
}

__attribute__ ((weak)) int _write (int Fd, char *pBuff, size_t Len)
{
	return -1;
}

__attribute__ ((weak)) int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;

	return 0;
}

__attribute__ ((weak)) int _isatty(int file)
{
	return 1;
}

__attribute__ ((weak)) void _kill(int pid, int sig)
{
	return;
}

__attribute__ ((weak)) int _getpid(void)
{
	return -1;
}

#endif
