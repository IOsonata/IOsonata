/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Implementation on RISC-V using the core's cycle counter

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

	asm volatile (
		// Read the current cycle count
		"rdcycle t0\n\t"

		// Load 'SystemCoreClockPeriodus' from the global variable.
		"lui t1, %%hi(SystemCoreClockPeriodus)\n\t"		// Loads the upper 20 bits of the address into t0
		"lw t1, %%lo(SystemCoreClockPeriodus)(t1)\n\t"	// Adds the lower 12 bits of the address to t0 and loads the value at that address into t0

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
}

/**
 * @brief	Nanosecond delay.
 *
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt) {
	asm volatile (
		// Read the current cycle count
		"rdcycle t0\n\t"

		// Load 'SystemCoreClockPeriodns' from the global variable.
		"lui t1, %%hi(SystemCoreClockPeriodns)\n\t"		// Loads the upper 20 bits of the address into t0
		"lw t1, %%lo(SystemCoreClockPeriodns)(t1)\n\t"	// Adds the lower 12 bits of the address to t0 and loads the value at that address into t0

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
}

static inline __attribute__((always_inline)) void msDelay(uint32_t ms) {
	usDelay(ms * 1000UL);
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__

