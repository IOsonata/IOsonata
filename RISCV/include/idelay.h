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
static inline __attribute__((always_inline)) uint32_t IOsonataRiscvCycle32(void)
{
	uint32_t v;
	__asm volatile ("rdcycle %0" : "=r"(v));
	return v;
}

static inline __attribute__((always_inline)) void usDelay(uint32_t cnt)
{
#if defined(__riscv_zicntr) || \
    (defined(__riscv) && !defined(__riscv_32e) && defined(__riscv_zicsr))
	uint32_t start = IOsonataRiscvCycle32();
	uint32_t wait = cnt * (uint32_t)SystemCoreClockPeriodus;
	if (wait == 0U)
	{
		wait = 1U;
	}

	while ((uint32_t)(IOsonataRiscvCycle32() - start) < wait)
	{
		__asm volatile("nop");
	}
#else
	uint32_t n = cnt * (uint32_t)SystemCoreClockPeriodus;
	if (n == 0U)
	{
		n = 1U;
	}
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
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)
{
#if defined(__riscv_zicntr) || \
    (defined(__riscv) && !defined(__riscv_32e) && defined(__riscv_zicsr))
	/* cycles = ns * Hz / 1e9.  Use MHz to keep this RV32-friendly. */
	uint32_t cycles_per_us = (uint32_t)SystemCoreClockPeriodus;
	uint32_t wait = (uint32_t)(((uint64_t)cnt * cycles_per_us + 999ULL) / 1000ULL);
	if (wait == 0U)
	{
		wait = 1U;
	}

	uint32_t start = IOsonataRiscvCycle32();
	while ((uint32_t)(IOsonataRiscvCycle32() - start) < wait)
	{
		__asm volatile("nop");
	}
#else
	uint32_t n = cnt * (uint32_t)SystemCoreClockPeriodns;
	if (n == 0U)
	{
		n = 1U;
	}
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

