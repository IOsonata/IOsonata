/**-------------------------------------------------------------------------
@file	agt_tick_r9a02.h

@brief	AGT0-based 1 ms system tick for R9A02G021.

Configures AGT0 (Asynchronous General-Purpose Timer 0) as a periodic
1 kHz underflow source, then increments a 64-bit ms counter from the
AGT0 underflow ISR.  Provides a portable system-tick base for
`MsDelay`-style polling, timeouts, and software timers.

Usage:

    extern void R9A02_AgtTickIsr(void);

    // In your application or in a Vectors_R9A02.c override, route AGT0
    // event ID to whichever IELSR slot is free and call the ISR shim:
    R9A02_ICU->IELSR[5] = R9A02_EVENT_AGT0_INT;
    void IELSR5_IRQHandler(void) { R9A02_AgtTickIsr(); }

    // From main() after SystemInit():
    R9A02_AgtTickInit();

    // Anywhere afterwards:
    uint64_t now = R9A02_GetMsTickCount();

Implementation chooses PCLKB / 8 as the AGT0 source, giving 6 MHz with
PCLKB = 48 MHz.  Reload value = 6000 - 1 = 5999 for a 1 ms period --
well within the 16-bit AGT counter range and giving a comfortable margin
against clock variation.

@author	Nguyen Hoan Hoang
@date	May 12, 2026

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
#ifndef __AGT_TICK_R9A02_H__
#define __AGT_TICK_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialise AGT0 as a 1 kHz underflow source.
 *
 * Brings AGT0 out of module-stop, configures PCLKB/8 source, sets
 * reload for 1 ms period, then starts the counter.  Caller is
 * responsible for routing R9A02_EVENT_AGT0_INT into a CLIC slot and
 * installing R9A02_AgtTickIsr as that slot's handler.
 *
 * @return	true on success, false if the configuration is invalid for
 *          the current PCLKB (e.g. PCLKB outside the supported range).
 */
bool R9A02_AgtTickInit(void);

/**
 * @brief	ISR shim -- call from the IELSR slot wired to AGT0_INT.
 *
 * Increments the internal ms counter and clears the AGT0 status flag.
 * Safe to call from interrupt context; takes no other action.
 */
void R9A02_AgtTickIsr(void);

/**
 * @brief	Read the running ms-tick counter.
 *
 * 64-bit monotonic, wraps after ~584 million years -- effectively never.
 * Reads are atomic on RV32 only when interrupts are disabled.  For most
 * uses the small probability of a torn read across the 32-bit boundary
 * is acceptable since the high word advances once every ~50 days; if
 * exact correctness is required, the caller can mask interrupts around
 * the read.
 */
uint64_t R9A02_GetMsTickCount(void);

#ifdef __cplusplus
}
#endif

#endif // __AGT_TICK_R9A02_H__
