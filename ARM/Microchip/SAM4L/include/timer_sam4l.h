/**-------------------------------------------------------------------------
@file	timer_sam4l.h

@brief	timer implementation on SAM4Lxx series

@author	Hoang Nguyen Hoan
@date	Aug. 24, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#ifndef __TIMER_SAM4L_H__
#define __TIMER_SAM4L_H__

#include "sam4lxxx.h"

#include "coredev/timer.h"

#define SAM4L_AST_TIMER_MAXCNT			1
#define SAM4L_AST_TIMER_TRIG_MAXCNT		1

#define SAM4L_TC_TIMER_MAXCNT			2
#if defined(SAME4LCXC) || defined(SAM4LSXC)
#define SAM4L_TC_TIMER_TIG_MAXCNT		6
#else
#define SAM4L_TC_TIMER_TIG_MAXCNT		3
#endif

#define SAM4L_TIMER_MAXCNT		(SAM4L_AST_TIMER_MAXCNT + SAM4L_TC_TIMER_MAXCNT)


typedef struct {
	uint32_t DevNo;
	union {
		Ast *pAstReg;
		Tc *pTcReg;
	};
	uint32_t BaseFreq;
    uint32_t CC[SAM4L_AST_TIMER_TRIG_MAXCNT];
    TimerTrig_t Trigger[SAM4L_AST_TIMER_TRIG_MAXCNT];
	TimerDev_t *pTimer;
} Sam4l_TimerData_t;

#ifdef __cplusplus
extern "C" {
#endif

bool Sam4lASTTimerInit(Sam4l_TimerData_t * const pHandle, const TimerCfg_t * const pCfg);
bool Sam4lTCTimerInit(Sam4l_TimerData_t * const pHandle, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_SAM4L_H__

