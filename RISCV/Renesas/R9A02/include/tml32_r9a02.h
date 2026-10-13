/**-------------------------------------------------------------------------
@file	tml32_r9a02.h

@brief	R9A02G021 32-bit interval timer (TML32)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 19. One interval: the
counter counts the selected clock divided by 2^n (n = 0 to 7) up to the
compare value and starts again, TML32_OUTI rises at each interval and stays
high until the interval flag is cleared (19.3.3).

The TML32 has no readable counter (19.2), it cannot be a coredev/timer.h
timer (the tick count). It serves intervals: an interrupt, a wake-up from
Software Standby, the trigger of an A/D conversion (ADM1.ADTRS = TML32 or an
ELC link) without the CPU, and the Snooze request of an A/D conversion in
Software Standby (19.3.7).

The period is (compare + 1) x 2^n clocks. 16-bit mode is used when the
compare value fits, 32-bit mode otherwise, with both halves of the compare
value not 0 (19.2.1). Init takes the period closest to nsPeriod.

The TML32 runs in Software Standby and Snooze only from R9A02_TML32_CLK_LF
(table 10.2). Its registers are reset by the power-on and voltage monitor
resets only (table 5.3): Init stops whatever a previous program left
running.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __TML32_R9A02_H__
#define __TML32_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

/// Count clock, ITLCSEL0.ISEL (19.2.4)
typedef enum __R9a02_Tml32_Clk {
	R9A02_TML32_CLK_HOCO = 1,		//!< HOCO, 24, 32 or 48 MHz
	R9A02_TML32_CLK_MOCO = 2,		//!< MOCO, 8 MHz
	R9A02_TML32_CLK_EXTAL = 3,		//!< EXTAL input set by SystemCoreClockSelect
	R9A02_TML32_CLK_LF = 4,			//!< LOCO or SOSC as OSMCR selects, 32768 Hz
} R9A02_TML32_CLK;

/// What clears the interval flag
typedef enum __R9a02_Tml32_Mode {
	R9A02_TML32_MODE_INT,			//!< CPU interrupt at each interval, Handler called
	R9A02_TML32_MODE_DTC,			//!< The DTC clears it, no CPU interrupt: TML32_OUTI rises at each
									//!< interval for the ADC and the event links
	R9A02_TML32_MODE_SNOOZE,		//!< As MODE_DTC, and also at each Snooze entry (SYSTEM_SNZREQ): the
									//!< TML32 as Snooze request (R9a02SnoozeSet), CLK_LF only
} R9A02_TML32_MODE;

/// @brief	Interval handler, in interrupt context.
///
/// @param	pCtx	: Context given in the configuration
typedef void (*R9a02Tml32Handler_t)(void *pCtx);

#pragma pack(push, 4)

/// TML32 configuration
typedef struct __R9a02_Tml32_Cfg {
	R9A02_TML32_CLK ClkSrc;			//!< Count clock
	R9A02_TML32_MODE Mode;			//!< Interval flag clearing
	uint64_t nsPeriod;				//!< Interval in ns
	R9a02Tml32Handler_t Handler;	//!< MODE_INT, NULL to only wake up the CPU
	void *pCtx;						//!< Handler context
	int IntPrio;					//!< MODE_INT, interrupt level 0 to 15
	bool bWakeup;					//!< MODE_INT with CLK_LF: the interval ends Software Standby
} R9a02Tml32Cfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	Set up the TML32 and start it.
///
/// @param	pCfg	: Configuration
///
/// @return	false if the configuration is not valid, the clock is not
/// 			running, or the event route or the DTC is not available
bool R9a02Tml32Init(const R9a02Tml32Cfg_t *pCfg);

/// @brief	Interval set by R9a02Tml32Init.
///
/// @return	Period in ns, 0 when not initialized
uint64_t R9a02Tml32Period(void);

/// @brief	Start counting again from 0, stopped first when running.
///
/// @return	false when not initialized
bool R9a02Tml32Start(void);

/// @brief	Stop counting, the counter is cleared. The setup is kept.
void R9a02Tml32Stop(void);

/// @brief	Stop, remove the routes and the DTC records, module stop.
void R9a02Tml32PowerOff(void);

#ifdef __cplusplus
}
#endif

#endif
