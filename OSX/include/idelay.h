/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay functions for macOS user-space.

POSIX implementation. macOS does not implement clock_nanosleep (it has the
POSIX timer types but only as of macOS 10.12 with limitations); plain
nanosleep against the realtime clock is the portable choice. It is signal-
interruptible, so we loop on EINTR with the kernel-returned remainder so
signal delivery does not shorten the delay.

Sub-µs accuracy is not achievable via OS sleep on macOS -- the kernel will
typically wake the thread no sooner than the next scheduler tick. Callers
that need cycle-accurate delays should busy-wait against
mach_absolute_time() at a higher level.

API matches IOsonata's ARM/RISCV idelay.h so cross-platform code can
include this header unchanged.

@author Hoang Nguyen Hoan
@date	May 20, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <time.h>
#include <errno.h>

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Nanosecond delay.
 *
 * Sleeps for at least @p cnt nanoseconds via nanosleep(). Loops on EINTR
 * with the kernel-returned remainder.
 *
 * @param	cnt : nanosecond count
 */
static inline void nsDelay(uint32_t cnt)
{
	struct timespec req, rem;
	req.tv_sec  = (time_t)(cnt / 1000000000UL);
	req.tv_nsec = (long)  (cnt % 1000000000UL);

	while (nanosleep(&req, &rem) == -1 && errno == EINTR)
	{
		req = rem;
	}
}

/**
 * @brief	Microsecond delay.
 *
 * @param	cnt : microsecond delay count
 */
static inline void usDelay(uint32_t cnt)
{
	struct timespec req, rem;
	req.tv_sec  = (time_t)(cnt / 1000000UL);
	req.tv_nsec = (long) ((cnt % 1000000UL) * 1000UL);

	while (nanosleep(&req, &rem) == -1 && errno == EINTR)
	{
		req = rem;
	}
}

/**
 * @brief	Millisecond delay.
 *
 * @param	ms : millisecond delay count
 */
static inline void msDelay(uint32_t ms)
{
	struct timespec req, rem;
	req.tv_sec  = (time_t)(ms / 1000UL);
	req.tv_nsec = (long) ((ms % 1000UL) * 1000000UL);

	while (nanosleep(&req, &rem) == -1 && errno == EINTR)
	{
		req = rem;
	}
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__
