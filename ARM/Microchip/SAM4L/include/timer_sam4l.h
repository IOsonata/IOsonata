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
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

// Virtual devices: AST, TC0 channels 0..2, TC1 channels 0..2.
#define SAM4L_AST_TIMER_MAXCNT 1
#define SAM4L_AST_TIMER_TRIG_MAXCNT 1
#define SAM4L_TC_TIMER_MAXCNT 6
#define SAM4L_TC_TIMER_TRIG_MAXCNT 3
#define SAM4L_TIMER_MAXCNT 7

struct Sam4lTimerTrigger {
	TimerTrig_t Info;
	uint64_t Deadline;
	uint32_t Ticks;
	bool Active;
};
struct Sam4l_TimerData_t {
	TimerDev_t *Timer;
	TcChannel *TcReg; // NULL selects AST.
	IRQn_Type Irq;
	uint32_t ClockMask, BaseFreq, Select, Epoch;
	bool Running, Healthy, Overflow;
	Sam4lTimerTrigger Trigger[3];
};
extern Sam4l_TimerData_t g_Sam4lTimerData[SAM4L_TIMER_MAXCNT];
void Sam4lTimerIRQ(int devno);
void Sam4lTimerPmWrite(volatile uint32_t *reg, uint32_t value);
bool Sam4lAstWait(uint32_t mask);
bool Sam4lAstSetup(Sam4l_TimerData_t &d);
bool Sam4lAstRun(Sam4l_TimerData_t &d, bool run);
bool Sam4lAstReset(Sam4l_TimerData_t &d);
uint64_t Sam4lAstCount(Sam4l_TimerData_t &d);
bool Sam4lAstArm(Sam4l_TimerData_t &d, bool enable);
void Sam4lTcSetup(Sam4l_TimerData_t &d);
void Sam4lTcRun(Sam4l_TimerData_t &d, bool run);
void Sam4lTcReset(Sam4l_TimerData_t &d);
uint64_t Sam4lTcCount(Sam4l_TimerData_t &d);
void Sam4lTcArm(Sam4l_TimerData_t &d, int n, bool enable);
#endif
