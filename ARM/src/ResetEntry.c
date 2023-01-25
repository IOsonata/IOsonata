/**-------------------------------------------------------------------------
@file	ResetEntry.c

@brief	Generic ResetEntry code for ARM CMSIS with GCC compiler

@author	Hoang Nguyen Hoan
@date	Mar. 14, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

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
extern unsigned long __end__;
extern unsigned long __HeapSize;
extern unsigned long __HeapBase;
extern unsigned long __HeapLimit;
#endif

#ifdef __ICCARM__
extern void __iar_init_core( void );
extern void __iar_init_vfp( void );
extern void __iar_data_init3(void);
extern void _call_main(void);
#endif

extern void _start();
extern void _rtos_start();
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
	__asm("BKPT #0");
	while(1);

}

__attribute__((weak)) void _exit(int Status)
{
	__asm("BKPT #0");
	while(1);
}

__attribute__((weak)) caddr_t _sbrk(int incr)
{
	static unsigned long *heap_end = 0;
	unsigned long *prev_heap_end;

	if (heap_end == 0)
	{
		heap_end = &__HeapBase;
	}

	prev_heap_end = heap_end;
	if ((heap_end + incr) > (unsigned long*) &__HeapLimit)
	{
		return ((void*)-1); // error - no more memory
	}
	heap_end += incr;

	return (caddr_t) prev_heap_end;
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

