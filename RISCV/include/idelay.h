/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Implementation on RISC-V.

If the target supports the Zicntr extension (rdcycle available), delays are
implemented using the CPU cycle counter for maximum accuracy.

If rdcycle is not available (e.g. Nordic VPR / RV32E cores that omit
performance counters), a calibrated 2-instruction NOP loop is used instead.
The loop path is selected automatically via __riscv_zicntr / __riscv_32e
compiler-defined macros — no manual configuration needed.

Variable meaning per path:
  rdcycle path : SystemCoreClockPeriodus = CPU MHz  (cycles per µs)
                 SystemCoreClockPeriodns = ns per cycle (integer, e.g. 6 @ 160MHz)
  NOP loop path: SystemCoreClockPeriodus = CPU_MHz / 2  (loop iters per µs)
                 SystemCoreClockPeriodns = CPU_MHz / 2000  (loop iters per ns, min 1)

@author Hoang Nguyen Hoan
@date	Aug. 16, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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
#ifndef __IDELAY_H__
#define __IDELAY_H__

#include <stdint.h>

#include "coredev/system_core_clock.h"

// These globals are pre-calculated at system startup to reduce overhead
extern uint32_t SystemCoreClock;
extern uint64_t SystemCoreClockPeriodns;	// Nanosecond period
extern uint64_t SystemCoreClockPeriodus;	// Microsecond period

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt) {

#if defined(__riscv_zicntr) || \
    (defined(__riscv) && !defined(__riscv_32e) && defined(__riscv_zicsr))
	// rdcycle available.
	// SystemCoreClockPeriodus = CPU MHz (cycles per µs)
	__asm volatile (
		// Read the current cycle count
		"rdcycle t0\n\t"

		// Load 'SystemCoreClockPeriodus' from the global variable.
		"lui t1, %%hi(SystemCoreClockPeriodus)\n\t"
		"lw t1, %%lo(SystemCoreClockPeriodus)(t1)\n\t"

		// t2 = cnt * SystemCoreClockPeriodus
		"mul t2, %0, t1\n\t"

		// t1 = ending cycle count
		"add t1, t0, t2\n\t"

		// Loop
		"1:\n\t"
		"rdcycle t0\n\t"
		"bltu t0, t1, 1b"

		: // No output operands
		: "r"(cnt) // Input: cnt
		: "t0", "t1", "t2" // Clobbered registers: t0, t1, t2
	);
#else
	// No cycle counter (e.g. Nordic VPR / RV32E).
	// Pure 2-instruction NOP loop — compiler independent.
	// SystemCoreClockPeriodus = CPU_MHz / 2  (loop iterations per µs)
	uint32_t n = cnt * (uint32_t)SystemCoreClockPeriodus;
	if (n == 0) n = 1;
	__asm volatile (
		"1: addi %0, %0, -1\n\t"
		"   bne  %0, zero, 1b"
		: "+r"(n) : : "memory"
	);
#endif
}

/**
 * @brief	Nanosecond delay.
 *
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt) {

#if defined(__riscv_zicntr) || \
    (defined(__riscv) && !defined(__riscv_32e) && defined(__riscv_zicsr))
	// rdcycle available.
	// SystemCoreClockPeriodns = nanoseconds per cycle (rounded integer)
	__asm volatile (
		// Read the current cycle count
		"rdcycle t0\n\t"

		// Load 'SystemCoreClockPeriodns' from the global variable.
		"lui t1, %%hi(SystemCoreClockPeriodns)\n\t"
		"lw t1, %%lo(SystemCoreClockPeriodns)(t1)\n\t"

		// t2 = cnt * SystemCoreClockPeriodns
		"mul t2, %0, t1\n\t"

		// t1 = ending cycle count
		"add t1, t0, t2\n\t"

		// Loop
		"1:\n\t"
		"rdcycle t0\n\t"
		"bltu t0, t1, 1b"

		: // No output operands
		: "r"(cnt) // Input: cnt
		: "t0", "t1", "t2" // Clobbered registers: t0, t1, t2
	);
#else
	// No cycle counter — NOP loop.
	// SystemCoreClockPeriodns = CPU_MHz / 2000  (loop iterations per ns)
	// Minimum 1 iteration to guarantee a non-zero delay.
	uint32_t n = cnt * (uint32_t)SystemCoreClockPeriodns;
	if (n == 0) n = 1;
	__asm volatile (
		"1: addi %0, %0, -1\n\t"
		"   bne  %0, zero, 1b"
		: "+r"(n) : : "memory"
	);
#endif
}

static inline __attribute__((always_inline)) void msDelay(uint32_t ms) {
	usDelay(ms * 1000UL);
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__

