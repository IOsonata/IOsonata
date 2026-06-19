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

// Application interrupt priority. NVIC: lower number is more urgent, 0 is the
// most urgent level the part implements. Same tier names as the scheduler
// priorities but a separate set; pass these to IntPrio fields, never a task
// priority. MCU-generic defaults across vendors. A vendor or RTOS layer that
// must reserve levels (for example a radio stack that reserves the top
// priorities, or a kernel that bounds the maskable range) redefines any of
// these before this header is reached.
#ifndef IRQ_PRIO_CRITICAL
#define IRQ_PRIO_CRITICAL   0
#endif
#ifndef IRQ_PRIO_HIGHEST
#define IRQ_PRIO_HIGHEST    1
#endif
#ifndef IRQ_PRIO_HIGH
#define IRQ_PRIO_HIGH       2
#endif
#ifndef IRQ_PRIO_NORMAL
#define IRQ_PRIO_NORMAL     3
#endif
#ifndef IRQ_PRIO_LOW
#define IRQ_PRIO_LOW        5
#endif
#ifndef IRQ_PRIO_LOWEST
#define IRQ_PRIO_LOWEST     7
#endif

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

// Application interrupt priority. RISC-V orders the opposite way from Cortex-M:
// higher number is more urgent. Generic defaults; the ESP32-C3/C5/C6 interrupt
// matrix and the Renesas N22 CLIC have different usable ranges, so each vendor
// header defines these before including this file and the values below stand in
// only when it does not. Same tier names as the scheduler priorities; pass
// these to IntPrio fields, never a task priority.
#ifndef IRQ_PRIO_LOWEST
#define IRQ_PRIO_LOWEST     1
#endif
#ifndef IRQ_PRIO_LOW
#define IRQ_PRIO_LOW        2
#endif
#ifndef IRQ_PRIO_NORMAL
#define IRQ_PRIO_NORMAL     3
#endif
#ifndef IRQ_PRIO_HIGH
#define IRQ_PRIO_HIGH       5
#endif
#ifndef IRQ_PRIO_HIGHEST
#define IRQ_PRIO_HIGHEST    6
#endif
#ifndef IRQ_PRIO_CRITICAL
#define IRQ_PRIO_CRITICAL   7
#endif

#endif

#endif // __INTERRUPT_H__




