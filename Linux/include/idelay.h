/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay functions for Linux user-space.

POSIX implementation. Uses clock_nanosleep(CLOCK_MONOTONIC) -- monotonic so
it is not affected by wall-clock adjustments (NTP, DST, settimeofday), and
nanosleep-grade so calling code can pass sub-millisecond delays without
losing them to coarse OS scheduling. The kernel's hr-timer subsystem will
still impose a floor of a few microseconds for the actual wakeup (longer
under load); callers needing tight sub-µs accuracy must busy-wait at a
higher level -- there is no portable way to do that from user-space.

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
 * Sleeps for at least @p cnt nanoseconds against CLOCK_MONOTONIC. Actual
 * wakeup granularity depends on the kernel's hr-timer resolution (a few
 * µs on a modern kernel). Re-enters the wait on EINTR so signal delivery
 * does not shorten the delay.
 *
 * @param	cnt : nanosecond count
 */
static inline void nsDelay(uint32_t cnt)
{
	struct timespec req;
	req.tv_sec  = (time_t)(cnt / 1000000000UL);
	req.tv_nsec = (long)  (cnt % 1000000000UL);

	while (clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &req) == EINTR)
	{
		// resume from where the signal interrupted -- req is overwritten
		// with the remaining time by clock_nanosleep on EINTR.
	}
}

/**
 * @brief	Microsecond delay.
 *
 * @param	cnt : microsecond delay count
 */
static inline void usDelay(uint32_t cnt)
{
	struct timespec req;
	req.tv_sec  = (time_t)(cnt / 1000000UL);
	req.tv_nsec = (long) ((cnt % 1000000UL) * 1000UL);

	while (clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &req) == EINTR) { }
}

/**
 * @brief	Millisecond delay.
 *
 * @param	ms : millisecond delay count
 */
static inline void msDelay(uint32_t ms)
{
	struct timespec req;
	req.tv_sec  = (time_t)(ms / 1000UL);
	req.tv_nsec = (long) ((ms % 1000UL) * 1000000UL);

	while (clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &req) == EINTR) { }
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__
