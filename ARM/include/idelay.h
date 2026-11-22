/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Only the calibrated assembly busy-waits live here. System Core owns all
calibration calculations and optional DWT measurement.

@author Hoang Nguyen Hoan
@date	May 22, 2015

@license

MIT License

Copyright (c) 2011 I-SYST inc. All rights reserved.

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
#include <stdbool.h>
#include <stddef.h>
#include "coredev/system_core_clock.h"
#ifdef __cplusplus
extern "C" {
#endif
// Core frequency is maintained by each MCU System Core implementation.
extern uint32_t SystemCoreClock;
// Calibrated loop iterations per second for each existing assembly loop.
extern uint32_t SystemMicroSecLoopCnt;
extern uint32_t SystemnsDelayFactor;

#ifdef __cplusplus
}
#endif

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 * SystemCoreClockUpdate refreshes the calibrated iterations per second
 * after a CPU clock change.
 * Long intervals retain the full calculated loop count.
 * Interrupt service and instruction fetch stalls can extend this delay.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt)
{
	uint64_t loops;
	uint32_t chunk;
	uint32_t remaining;

	loops = ((uint64_t)cnt * SystemMicroSecLoopCnt + 999999ULL) /
		1000000ULL;

	// Keep the original ten-NOP signed countdown inside usDelay.
	// Each chunk is bounded because BGT compares a signed counter.
	while (loops != 0U)
	{
		chunk = loops > INT32_MAX ? INT32_MAX : (uint32_t)loops;
		__asm__ volatile (
#ifdef __GNUC__
			".syntax unified\n"
#endif
			" MOVS %[remaining], %[count]\n"
			"1:\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" SUBS %[remaining], #1\n"
			" BGT 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
			".syntax divided\n"
#endif
			: [remaining] "=&l" (remaining)
			: [count] "l" (chunk)
			: "cc"
		);
		loops -= chunk;
	}
}

/**
 * @brief	Nanosecond delay.
 *
 * SystemnsDelayFactor is the calibrated loop rate in iterations per second.
 * Keep the original signed countdown in this public function. Long delays
 * are split into signed-counter-sized chunks before entering assembly.
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)
{
	uint64_t loops;
	uint32_t chunk;
	uint32_t remaining;

	loops = ((uint64_t)cnt * SystemnsDelayFactor + 999999999ULL) /
		1000000000ULL;

	while (loops != 0U)
	{
		chunk = loops > INT32_MAX ? INT32_MAX : (uint32_t)loops;
		__asm__ volatile (
#ifdef __GNUC__
			".syntax unified\n"
#endif
			" MOVS %[remaining], %[count]\n"
			// Align the inlined countdown to a flash fetch boundary.
			" .p2align 5\n"
			"1:\n"
			" SUBS %[remaining], #1\n"
			" BGT 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
			".syntax divided\n"
#endif
			: [remaining] "=&l" (remaining)
			: [count] "l" (chunk)
			: "cc"
		);
		loops -= chunk;
	}
}

/**
 * @brief	Millisecond delay.
 * @param	ms : Milliseconds to wait.
 */
static inline void msDelay(uint32_t ms)
{
	// Split before converting milliseconds to the 32-bit microsecond count.
	while (ms > UINT32_MAX / 1000U)
	{
		usDelay((UINT32_MAX / 1000U) * 1000U);
		ms -= UINT32_MAX / 1000U;
	}
	usDelay(ms * 1000U);
}

/** @} End of group Utilities */

// Compatibility with existing MCU systems and managed-project include paths.
// The implementation lives in the System Core calibration header.
#include "system_core_clock_calibration.h"

#endif

