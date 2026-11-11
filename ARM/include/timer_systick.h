/**-------------------------------------------------------------------------
@file	timer_systick.h

@brief	ARM Cortex-M SysTick timer implementation

SysTick is the 24-bit system timer present on all ARM Cortex-M devices
(M0/M0+/M3/M4/M7/M23/M33/M55). This MCU port wraps it in the IOsonata
TimerDev_t interface.

The counter runs on the processor clock: the timer frequency is the core
clock (SystemCoreClockGet) and the tick count is in core clock cycles. It has
one trigger, the reload interrupt, with a period from 2 to 2^24 cycles.

This port defines TimerInit and SysTick_Handler. It cannot be linked with an
MCU timer port or with code that has its own SysTick_Handler, such as an RTOS
tick.

@author	Hoang Nguyen Hoan
@date	Mar. 2026

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

#ifndef __TIMER_SYSTICK_H__
#define __TIMER_SYSTICK_H__

#include <stdint.h>
#include "coredev/timer.h"

// SysTick has exactly one trigger slot (the reload interrupt).
#define SYSTICK_TRIG_MAXCNT		1

#pragma pack(push, 4)

typedef struct {
	uint32_t	CurPeriod;		//!< Cycles of the period in progress
	uint32_t	NextPeriod;		//!< Cycles of the next periods, LOAD + 1
	uint32_t	Wraps;			//!< Wraps counted, not seen by the interrupt handler yet
	uint64_t	nsMax;			//!< Shortest nsec period taken as 2^24 cycles
	uint64_t	CycQ32;			//!< Cycles per nsec in 1/2^32, rounded down
	uint32_t	NsHi;			//!< nsec per cycle, integer part
	uint32_t	NsLo;			//!< nsec per cycle, fraction in 1/2^32
	TimerTrig_t	Trigger[SYSTICK_TRIG_MAXCNT];
	TimerDev_t	*pTimer;		//!< Back-pointer to the TimerDev_t
} SysTick_TimerData_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Cortex-M SysTick timer initialisation.
 *
 * Wires all TimerDev_t function pointers and starts SysTick on the processor
 * clock with the longest reload, 2^24 cycles, and its interrupt on. The
 * interrupt extends the tick count to 64 bits.
 *
 * pCfg->DevNo, ClkSrc and Freq are not used: the timer frequency is always
 * the core clock, the closest value SysTick has to any requested one.
 * pCfg->IntPrio is the interrupt priority level, 0 the highest, as for the
 * NVIC interrupts. A level past the lowest one gives the lowest one.
 * bTickInt is refused, the count changes every cycle.
 *
 * A 1 kHz tick:
 *   TimerCfg_t cfg = {
 *       .DevNo      = 0,
 *       .ClkSrc     = TIMER_CLKSRC_DEFAULT,
 *       .Freq       = 0,
 *       .IntPrio    = 7,
 *       .EvtHandler = NULL,
 *   };
 *   TimerInit(&g_SysTickDev, &cfg);
 *   msTimerEnableTrigger(&g_SysTickDev, 0, 1, TIMER_TRIG_TYPE_CONTINUOUS, TickHandler, NULL);
 *
 * @param	pTimer	: Pointer to TimerDev_t (caller provides storage)
 * @param	pCfg	: Timer configuration.
 *
 * @return	true on success, false if the core clock is unknown (0) or
 * 			bTickInt is set
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_SYSTICK_H__
