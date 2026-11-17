/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Delay-loop calibration is maintained by the MCU clock implementation.
The microsecond loop count uses 16 fractional bits.

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
#include "coredev/system_core_clock.h"

#ifdef __cplusplus
extern "C" {
#endif
// Iterations per microsecond for each loop, with 16 fractional bits.
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
 * SystemCoreClockUpdate refreshes SystemMicroSecLoopCnt after a CPU clock
 * change. Scale the full interval before removing the fractional bits.
 * Interrupt service and instruction fetch stalls can extend this delay.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt) {
	uint32_t loops = ((uint64_t)cnt * SystemMicroSecLoopCnt + 65535ULL) >> 16;
	__asm__ volatile (
#ifdef __GNUC__
		".syntax unified\n"
#endif
			"ORRS %[ucnt], %[ucnt]\n"
			"BEQ 2f\n"
			"MOVS r0, %[ucnt]\n"
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
			" SUBS r0, #1\n"
			" BNE 1b\n"
		"2:\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		:
		: [ucnt] "l" (loops)
		:"r0", "cc"
		 );
}

/**
 * @brief	Nanosecond delay.
 *
 * SystemnsDelayFactor uses the same fractional format as the microsecond
 * loop count and is refreshed by the MCU clock implementation.
 * Calculation overhead, instruction fetch and interrupts extend the delay.
 * Use a hardware timer when precise timing is required.
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt) {
	uint32_t loops = ((uint64_t)cnt * SystemnsDelayFactor + 65535999ULL) / 65536000ULL;
	if (loops == 0U)
	{
		return;
	}
	__asm__ volatile (
#ifdef __GNUC__
		".syntax unified\n"
#endif
			"MOVS r1, %[ucnt]\n"
		"1:\n"
			" SUBS r1, #1\n"
			" BNE 1b\n"
		"2:\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		:
		: [ucnt] "l" (loops)
		:"r1", "cc"
		 );
}

static inline __attribute__((always_inline)) void msDelay(uint32_t ms) {
	usDelay(ms * 1000UL);
}

/** @} End of group Utilities */

#endif

