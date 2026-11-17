/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Implementation on RISC-V.

CSR-capable builds read the CPU cycle counter through IOsonataRiscvCycle32,
implemented by the MCU port. The port selects and enables its counter before
using delays. The shared delay code does not select a CSR address.

Builds without CSR support use a two-instruction loop. Its nominal two-cycle
calibration is approximate; pipeline and branch timing depend on the core.

SystemCoreClockUpdate refreshes SystemCoreClock after CPU clock changes.
Both delay calculations retain fractional MHz from that frequency.

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

#ifdef __cplusplus
extern "C" {
#endif

// Updated by the MCU clock implementation.
extern uint32_t SystemCoreClock;
extern uint64_t SystemCoreClockPeriodns;	// Nanosecond period
extern uint64_t SystemCoreClockPeriodus;	// Cycles per microsecond

// Low 32 bits of the running CPU cycle counter; wraps modulo 2^32.
uint32_t IOsonataRiscvCycle32(void);

#ifdef __cplusplus
}
#endif

/** @addtogroup Utilities
  * @{
  */

// The MCU port supplies the counter for CSR-capable builds. Limit each wait
// to half the counter range so normal wraparound is handled by subtraction.
static inline __attribute__((always_inline)) void IOsonataRiscvDelayCycles(uint64_t Cycles)
{
#if defined(__riscv_zicntr) || \
    (defined(__riscv) && !defined(__riscv_32e) && defined(__riscv_zicsr))
	while (Cycles != 0U)
	{
		uint32_t wait = Cycles > 0x7FFFFFFFULL ? 0x7FFFFFFFU : (uint32_t)Cycles;
		uint32_t start = IOsonataRiscvCycle32();
		while ((uint32_t)(IOsonataRiscvCycle32() - start) < wait)
		{
			__asm volatile("nop");
		}
		Cycles -= wait;
	}
#else
	// Nominal two-cycle loop; branch and instruction-fetch timing vary by core.
	uint64_t remaining = (Cycles >> 1) + (Cycles & 1U);
	while (remaining != 0U)
	{
		uint32_t loops = remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
		remaining -= loops;
		__asm volatile (
			"1: addi %0, %0, -1\n\t"
			"   bne  %0, zero, 1b"
			: "+r"(loops) : : "memory"
		);
	}
#endif
}

/**
 * @brief	Microsecond delay.
 * @param	cnt : Microseconds to wait.
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt)
{
	uint64_t cycles = ((uint64_t)cnt * SystemCoreClock + 999999ULL) / 1000000ULL;
	IOsonataRiscvDelayCycles(cycles);
}

/**
 * @brief	Nanosecond delay.
 *
 * Calculation and counter-read overhead extend short delays. Use a hardware
 * timer when precise timing is required.
 *
 * @param	cnt : Nanoseconds to wait.
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)
{
	uint64_t cycles = ((uint64_t)cnt * SystemCoreClock + 999999999ULL) / 1000000000ULL;
	IOsonataRiscvDelayCycles(cycles);
}

/**
 * @brief	Millisecond delay.
 * @param	ms : Milliseconds to wait.
 */
static inline __attribute__((always_inline)) void msDelay(uint32_t ms)
{
	uint64_t cycles = ((uint64_t)ms * SystemCoreClock + 999ULL) / 1000ULL;
	IOsonataRiscvDelayCycles(cycles);
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__

