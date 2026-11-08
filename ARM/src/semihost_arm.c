/**-------------------------------------------------------------------------
@file	semihost_arm.c

@brief	Semihosting call for Arm Cortex-M

The M profile semihosting call is BKPT 0xAB with the operation number in r0
and the parameter in r1. The debugger performs the operation, writes the
result in r0 and resumes the CPU after the BKPT.

Used by the semihosting console interface, see semihost_intrf.h.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#include <stdint.h>
#include <stdbool.h>

#include "semihost_intrf.h"

// Debug Halting Control and Status Register, C_DEBUGEN is set while a
// debugger has halting debug enabled.
#define SEMIHOST_DHCSR				(*(volatile uint32_t *)0xE000EDF0UL)
#define SEMIHOST_DHCSR_C_DEBUGEN	(1UL << 0)

bool SemihostDebuggerAttached(void)
{
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || \
	defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8_1M_MAIN__)
	return (SEMIHOST_DHCSR & SEMIHOST_DHCSR_C_DEBUGEN) != 0;
#else
	// ARMv6-M does not give software access to DHCSR, ARMv8-M Baseline is
	// handled the same way. The call is always made on these cores.
	return true;
#endif
}

int SemihostCall(int Op, void *pArg)
{
	register int r0 __asm__("r0") = Op;
	register void *r1 __asm__("r1") = pArg;

	// The debugger reads the parameter block and may write to memory it
	// points to, hence the memory clobber.
	__asm__ volatile ("bkpt 0xAB" : "+r" (r0) : "r" (r1) : "memory");

	return r0;
}
