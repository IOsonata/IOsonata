/**-------------------------------------------------------------------------
@file	interrupt.h

@brief	Interrupt functions.

@author	Hoang Nguyen Hoan
@date	Sep. 12, 1996

@license

MIT

Copyright (c) 1996-2025, I-SYST, all rights reserved

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
#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#if 0
#ifndef __unix__
#ifdef __GNUC__
#ifndef __PROGRAM_START
#define __PROGRAM_START		// Define to fix compile bug in CMSIS 5.6
#endif
#endif
//#include "cmsis_compiler.h"
#endif
#endif	// if 1

#ifdef __unix__
#elif defined(__arm__) || defined(__ICCARM__)
// ARM
#ifdef __GNUC__
#ifndef __PROGRAM_START
#define __PROGRAM_START		// Define to fix compile bug in CMSIS 5.6
#endif
#endif

#include "cmsis_compiler.h"

static inline uint32_t DisableInterrupt() {
	uint32_t __primmask = __get_PRIMASK();
	__disable_irq();
	return __primmask;
}

static inline void EnableInterrupt(uint32_t __primmask) {
	__set_PRIMASK(__primmask);
}
#elif defined(__riscv)
// RISC-V
static inline uint32_t DisableInterrupt() {
	uint32_t mstatus_val;
	asm volatile("csrr %0, mstatus" : "=r"(mstatus_val)); // Read mstatus
    __asm__ volatile("csrrc x0, mstatus, %0" :: "r"(1 << 3)); // MIE is bit 3

    return mstatus_val;
}

static inline void EnableInterrupt(uint32_t mstatus_val) {
//	mstatus_val |= (1UL << 3); // Set MIE bit (bit 3)
	asm volatile("csrw mstatus, %0" :: "r"(mstatus_val)); // Write mstatus//    __asm__ volatile("csrrsi x0, mstatus, %0" :: "r"(1 << 3)); // MIE is bit 3
}
#endif

#endif // __INTERRUPT_H__




