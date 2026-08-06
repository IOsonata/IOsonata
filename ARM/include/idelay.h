/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

The delay loop is calculated based on a 16MHz ARM loop is invariant to compiler
optimization. nsec delay cannot be achieved is at 295ns per loop.

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

extern uint32_t SystemMicroSecLoopCnt;
extern uint32_t SystemnsDelayFactor;

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 * This function is based on a 16MHz clock. For higher clock
 * rate SystemMicroSecNopCnt needs to be adjusted.  Adjustment of this variable
 * should be done in the CMSIS SystemCoreCLockUpdate function.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt) {
	asm volatile (
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
			" BGT 1b\n"
		"2:\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		:
		: [ucnt] "l" (cnt * SystemMicroSecLoopCnt)
		:"r0"
		 );
}

/**
 * @brief	Nanosecond delay.
 *
 * This is highly inaccurate use at you own risk
 *
 * nsec delay cannot be achieved for low cpu clock.
 * this loop is 295ns on a 16MHz cpu
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt) {
	asm volatile (
#ifdef __GNUC__
		".syntax unified\n"
#endif
			"MOVS r1, %[ucnt]\n"
		"1:\n"
			" SUBS r1, #1\n"
			" BGT 1b\n"
		"2:\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		:
		: [ucnt] "l" ((cnt + (SystemnsDelayFactor >> 1)) / SystemnsDelayFactor)//(SYSTEM_NSDELAY_CORE_FACTOR >> 1)) / SYSTEM_NSDELAY_CORE_FACTOR)
//		: [ucnt] "l" ((cnt + (SYSTEM_NSDELAY_CORE_FACTOR >> 1)) / SYSTEM_NSDELAY_CORE_FACTOR)
		:"r1"
		 );
}

static inline __attribute__((always_inline)) void msDelay(uint32_t ms) {
	usDelay(ms * 1000UL);
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__

