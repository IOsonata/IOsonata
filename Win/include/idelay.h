/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay functions for Windows user-space.

Win32 has two relevant primitives:
  - Sleep(ms) -- ms-granular kernel sleep, accuracy bounded by the
    system timer resolution (default 15.6 ms on desktop Windows; can be
    requested down to 0.5 ms via timeBeginPeriod, but that's a global
    side effect we don't want a delay routine to impose).
  - QueryPerformanceCounter -- sub-µs monotonic counter. Reliable for
    busy-wait.

Strategy:
  - msDelay(ms): Sleep(ms) for ms >= 2, busy-wait QPC otherwise (Sleep
    can over-shoot small values by a full timer tick).
  - usDelay / nsDelay: always QPC busy-wait. There is no Win32 API that
    will reliably sleep for less than a ms.

Busy-waiting at µs/ns granularity burns a CPU; in long-running code
prefer scheduling primitives where possible. This file is intended for
the same use-case as IOsonata's MCU idelay.h: short hardware-protocol
timings where blocking briefly is the right thing.

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

// Avoid pulling in the full Windows.h here -- consumers of idelay.h shouldn't
// inherit MIN/MAX macros, near/far, etc.  Forward-declare just the bits we
// need; the linker resolves them against kernel32.
#ifndef _WINDOWS_
#ifdef __cplusplus
extern "C" {
#endif

__declspec(dllimport) void __stdcall Sleep(unsigned long dwMilliseconds);
__declspec(dllimport) int  __stdcall QueryPerformanceCounter(  long long *lpPerformanceCount);
__declspec(dllimport) int  __stdcall QueryPerformanceFrequency(long long *lpFrequency);

#ifdef __cplusplus
}
#endif
#endif // _WINDOWS_

/** @addtogroup Utilities
  * @{
  */

static inline long long __idelay_qpc_freq(void)
{
	// QPF is constant for the life of the process (Win XP+ guarantee), so
	// cache it after the first call. Not strictly thread-safe at first
	// init (benign race -- both threads compute the same value), so we
	// don't bother with a mutex.
	static long long s_freq = 0;
	if (s_freq == 0)
	{
		QueryPerformanceFrequency(&s_freq);
	}
	return s_freq;
}

static inline void __idelay_qpc_spin(long long ticks)
{
	long long start, now;
	QueryPerformanceCounter(&start);
	do
	{
		QueryPerformanceCounter(&now);
	} while ((now - start) < ticks);
}

/**
 * @brief	Nanosecond delay.
 *
 * Busy-waits via QueryPerformanceCounter. Practical floor is the QPC
 * period (~100 ns on most hardware) plus the cost of the QPC syscall
 * itself; sub-100ns requests are rounded up to one QPC tick.
 *
 * @param	cnt : nanosecond count
 */
static inline void nsDelay(uint32_t cnt)
{
	long long freq  = __idelay_qpc_freq();
	long long ticks = ((long long)cnt * freq + 999999999LL) / 1000000000LL;
	if (ticks < 1) ticks = 1;
	__idelay_qpc_spin(ticks);
}

/**
 * @brief	Microsecond delay.
 *
 * Busy-waits via QueryPerformanceCounter. Sleep() is unsuitable for
 * sub-millisecond delays on Windows -- the kernel timer tick is 15.6 ms
 * by default, so Sleep(1) commonly takes ~16 ms.
 *
 * @param	cnt : microsecond delay count
 */
static inline void usDelay(uint32_t cnt)
{
	long long freq  = __idelay_qpc_freq();
	long long ticks = ((long long)cnt * freq + 999999LL) / 1000000LL;
	if (ticks < 1) ticks = 1;
	__idelay_qpc_spin(ticks);
}

/**
 * @brief	Millisecond delay.
 *
 * Hands off to the kernel via Sleep() for ms >= 2; busy-waits via QPC for
 * sub-2ms to avoid the timer-tick over-shoot.
 *
 * @param	ms : millisecond delay count
 */
static inline void msDelay(uint32_t ms)
{
	if (ms >= 2)
	{
		Sleep((unsigned long)ms);
	}
	else if (ms != 0)
	{
		long long freq  = __idelay_qpc_freq();
		long long ticks = ((long long)ms * freq + 999LL) / 1000LL;
		__idelay_qpc_spin(ticks);
	}
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__
