/**-------------------------------------------------------------------------
@file	timer_systick.h

@brief	ARM Cortex-M SysTick timer implementation

SysTick is the 24-bit system timer present on all ARM Cortex-M devices
(M0/M0+/M3/M4/M7/M23/M33/M55). This MCU port wraps it in the IOsonata
TimerDev_t interface so any kernel or driver that accepts a TimerDev_t*
works identically regardless of the underlying tick source.

When SysTick is not the preferred tick source (e.g. nRF52 uses RTC1 for
lower power, RE01 uses AGT), include the appropriate MCU port instead.
This port and those are mutually exclusive per project.

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
	int			DevNo;			//!< Must be 0 — only one SysTick per core
	uint32_t	CoreClkHz;		//!< Core clock in Hz at Init() time
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
 * Wires all TimerDev_t function pointers and configures SysTick to fire
 * at pCfg->Freq Hz. The core clock frequency must be passed in via
 * pCfg->DevNo which is repurposed as CoreClkHz (DevNo is always 0 for
 * SysTick — there is only one per core).
 *
 * Recommended usage:
 *   static SysTick_TimerData_t s_SysTickData;
 *   static TimerDev_t          g_SysTickDev;
 *   static const TimerCfg_t s_Cfg = {
 *       .DevNo      = (int)SystemCoreClock,   // pass core clock here
 *       .ClkSrc     = TIMER_CLKSRC_DEFAULT,
 *       .Freq       = 1000,                   // 1 kHz tick
 *       .IntPrio    = 0xFF,                   // lowest priority
 *       .EvtHandler = NULL,                   // filled by TaktOS
 *   };
 *   TimerInit(&g_SysTickDev, &s_Cfg);
 *
 * @param	pTimer	: Pointer to TimerDev_t (caller provides storage)
 * @param	pCfg	: Timer configuration.
 *
 * @return	true on success
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_SYSTICK_H__
