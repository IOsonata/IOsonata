/**-------------------------------------------------------------------------
@file	timer_sam4l.cpp

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "timer_sam4l.h"

Sam4l_TimerData_t g_Sam4lTimerData[SAM4L_TIMER_MAXCNT] = {
	{ 0, {.pAstReg = SAM4L_AST}, 32768,},
	{ 1, {.pTcReg = SAM4L_TC0}, 48000000,},
	{ 2, {.pTcReg = SAM4L_TC1}, 48000000},
};

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= SAM4L_TIMER_MAXCNT)
	{
		return false;
	}

	g_Sam4lTimerData[pCfg->DevNo].pTimer = pTimer;

	if (pCfg->DevNo < 1)
	{
		return Sam4lASTTimerInit(&g_Sam4lTimerData[pCfg->DevNo], pCfg);
	}

	return Sam4lTCTimerInit(&g_Sam4lTimerData[pCfg->DevNo], pCfg);
}

int TimerGetLowFreqDevCount()
{
	return SAM4L_AST_TIMER_MAXCNT;
}

int TimerGetHighFreqDevCount()
{
	return SAM4L_TIMER_MAXCNT - SAM4L_AST_TIMER_MAXCNT;
}

int TimerGetHighFreqDevNo()
{
	return SAM4L_TC_TIMER_MAXCNT;
}

