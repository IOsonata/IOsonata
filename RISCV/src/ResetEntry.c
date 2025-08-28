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
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

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
extern uint32_t __StackTop;

// Small data pointer for ABI compliance
extern unsigned long __GlobalPointer;

/* Forward declarations */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern void _start(void);

__attribute__((section(".reset"), used, noreturn))
void ResetEntry(void)
{
    uint32_t *src, *dst;

    /* Initialize stack pointer */
//    asm volatile("mv sp, %0" : : "r"(&__StackTop));
    // Initialize stack pointer ---
    asm volatile("la sp, __StackTop");   // load address of stack top

    // Initialize global pointer (ABI requirement) ---
    asm volatile("la gp, __GlobalPointer");

    // Disable virtual memory / MMU ---
    asm volatile("csrw satp, zero");

  	/*
	 * Copy the initialized data of the ".data" segment
	 * from the flash to ram.
	 */
    src = &__data_loc__;
    dst = &__data_start__;
    while (dst < &__data_end__)
    {
        *dst++ = *src++;
    }

	/*
	 * Clear the ".bss" segment.
	 */
    dst = &__bss_start__;
    while (dst < &__bss_end__)
    {
        *dst++ = 0;
    }

	/*
	 * Now memory has been initialized
	 * Update core clock data
	 */
    SystemInit();
    SystemCoreClockUpdate();

    _start();

    /* Loop forever if main returns */
    while(1) { }
}

/* --- Minimal heap (_sbrk) implementation --- */
__attribute__((weak))
caddr_t _sbrk(ptrdiff_t incr)
{
    static uint8_t *heap_ptr = NULL;
    uint8_t *prev_heap;

    if (heap_ptr == NULL)
        heap_ptr = (uint8_t *)&__heap_start__;

    prev_heap = heap_ptr;

    if ((heap_ptr + incr) > (uint8_t *)&__heap_end__) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_ptr += incr;
    return (caddr_t)prev_heap;
}

__attribute__((weak)) int _close(int fd)      { return -1; }
__attribute__((weak)) int _fstat(int fd, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
__attribute__((weak)) int _isatty(int fd)    { return 1; }
__attribute__((weak)) int _lseek(int fd, int offset, int whence) { return -1; }
__attribute__((weak)) int _read(int fd, char *buf, size_t len)    { return -1; }
__attribute__((weak)) int _write(int fd, char *buf, size_t len)   { return -1; }
__attribute__((weak)) void _exit(int status) { while(1); }
__attribute__((weak)) void _kill(int pid, int sig) {}
__attribute__((weak)) int _getpid(void) { return -1; }
