/**-------------------------------------------------------------------------
@file	timer_lpcxx.h

@brief	Shared LPC CTIMER state and family hooks; register types come from vendor CMSIS.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef IOSONATA_TIMER_LPCXX_H
#define IOSONATA_TIMER_LPCXX_H

#include "timer_lpc_target.h"
#include "coredev/timer.h"

#define LPC_TIMER_TRIG_MAX		3			// Match 0 to 2
#define LPC_TIMER_WRAP_MR		3			// Match 3, half cycle wrap check
#define LPC_TIMER_TICKS_MIN	4ULL		// Shortest trigger period in ticks
#define LPC_TIMER_MATCH_FLAGS	0x0FU		// MR0INT to MR3INT; capture is disabled
#define LPC_TIMER_MCR_BITS		3			// MRnI, MRnR, MRnS per match register


#define LPC_TIMER_ENABLE		1U
#define LPC_TIMER_RESET		2U
#define LPC_TIMER_MATCH_INT	1U
#define LPC_TIMER_MATCH_FLAG	1U

typedef struct {
	TimerTrig_t Info;				//!< Type, actual period, handler and context
	uint64_t Deadline;				//!< Next trigger count
	uint64_t Ticks;					//!< Period in ticks
	bool bActive;					//!< Trigger enabled
} LpcTimerTrig_t;

typedef struct {
	LpcTimerReg_t *pReg;				//!< CTIMER registers
	IRQn_Type IrqNo;				//!< Interrupt number
	bool bAsync;					//!< On the async APB bridge, else on the AHB clock
	uint32_t ClkMask;				//!< Clock enable bit
	uint32_t RstMask;				//!< Reset bit
	uint32_t CounterMask;			//!< Implemented counter bits
	uint32_t PrescalerMax;		//!< Maximum prescale register value
	TimerDev_t *pTimer;				//!< Timer using this CTIMER, NULL when free
	uint32_t Epoch;					//!< Incremented on counter reset
	LpcTimerTrig_t Trig[LPC_TIMER_TRIG_MAX];
} LpcTimer_t;


extern LpcTimer_t g_LpcTimer[];
extern const int g_LpcTimerCount;
uint32_t LpcTimerBaseFreq(const LpcTimer_t *pDev);
void LpcTimerClockEnable(const LpcTimer_t *pDev);
void LpcTimerIrqHandler(LpcTimer_t *pDev);

#endif
