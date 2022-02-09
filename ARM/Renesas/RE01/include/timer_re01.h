/**-------------------------------------------------------------------------
@file	timer_re01.h

@brief	timer implementation on Renesas RE01 series

@author	Hoang Nguyen Hoan
@date	Feb. 3, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __TIMER_RE01_H__
#define __TIMER_RE01_H__

#include <stdint.h>

#include "coredev/timer.h"

#include "re01xxx.h"

#define RE01_TIMER_AGT_CNT				2	// AGT0, AGT1
#define RE01_TIMER_TMR_CNT				1	// TMR0 + TMR1 linked as one 16bits high freq timer
#define RE01_TIMER_MAXCNT				(RE01_TIMER_AGT_CNT + RE01_TIMER_TMR_CNT)

#define RE01_TIMER_CC_MAXCNT			2
#define RE01_TIMER_TRIG_MAXCNT			2

#pragma pack(push, 4)

typedef struct {
	int DevNo;		//!< Device number (index)
	union {
		AGT0_Type *pAgtReg;
		AGT1_Type *pAgtReg1;
		TMR01_Type *pTmrReg;
	};
	uint32_t BaseFreq;
    uint32_t CC[RE01_TIMER_CC_MAXCNT];
    TimerTrig_t Trigger[RE01_TIMER_TRIG_MAXCNT];
    TimerDev_t *pTimer;
    int IntPrio;
    IRQn_Type IrqOvr;
    IRQn_Type IrqMatch[RE01_TIMER_CC_MAXCNT];
} RE01_TimerData_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool Re01AgtInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg);
bool Re01TmrInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_RE01_H__
