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

#include <stdint.h>

#ifdef __unix__
// UNIX

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
// ----------------------------- RISC-V (bare-metal M-mode) --------------------

#include <stdint.h>

// Require CSR instructions. Without Zicsr you cannot touch mstatus.MIE.
#ifndef __riscv_zicsr
# error "Bare-metal interrupt control needs CSR ops. Build with -march=..._zicsr."
#endif

// If <riscv_encoding.h> is present, use its helpers; else use minimal asm.
#if __has_include(<riscv_encoding.h>)
  #include <riscv_encoding.h>
  static inline uintptr_t DisableInterrupt(void) {
    uintptr_t prev = read_csr(mstatus);
    clear_csr(mstatus, MSTATUS_MIE);           // atomically clear global IE
    return prev;
  }
  static inline void EnableInterrupt(uintptr_t prev_status) {
    write_csr(mstatus, prev_status);           // restore full mstatus
  }
#else
  // mstatus CSR = 0x300; MIE bit = 3 -> immediate 8 for csrci/csrsi
  static inline uintptr_t DisableInterrupt(void) {
    uintptr_t prev;
    __asm__ volatile(
      "csrr  %0, 0x300\n"   // read mstatus
      "csrci 0x300, 8\n"    // clear MIE (bit 3) atomically
      : "=r"(prev) :: "memory");
    return prev;
  }
  static inline void EnableInterrupt(uintptr_t prev_status) {
    __asm__ volatile(
      "csrw 0x300, %0\n"    // restore whole mstatus (nestable)
      :: "r"(prev_status) : "memory");
  }
#endif

#endif

#endif // __INTERRUPT_H__




