/**-------------------------------------------------------------------------
@file	timer_re01.cpp

@brief	Renesas RE01 timer class implementation

This generic implementation combines all available core timer in to one
device list. Each timer is indexed by DevNo starting from 0..max. Indexing
low frequency timer to high frequency.

For example MCU having 2 low frequency timers and 4 high frequency timers
are indexed as follow:

DevNo : 0..1 are low frequency timers
DevNo : 2..5 are high frequency timers

There are 2 ways to determine the start DevNo for high frequency timer

int HFreqDevNoStart = TimerGetLowFreqDevCount();
int HFreqDevNoStart = TimerGetHighFreqDevNo();

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
#include "re01xxx.h"

#include "timer_re01.h"

RE01_TimerData_t g_Re01TimerData[RE01_TIMER_MAXCNT] = {
	{0, {.pAgtReg = AGT0}, .IrqOvr = (IRQn_Type)-1, .IrqMatch = {(IRQn_Type)-1, (IRQn_Type)-1},},
	{1, {.pAgtReg1 = AGT1}, .IrqOvr = (IRQn_Type)-1, .IrqMatch = {(IRQn_Type)-1, (IRQn_Type)-1},},
	{2, {.pTmrReg = TMR01}, .IrqOvr = (IRQn_Type)-1, .IrqMatch = {(IRQn_Type)-1, (IRQn_Type)-1},},
};

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= RE01_TIMER_MAXCNT)
	{
		return false;
	}

	g_Re01TimerData[pCfg->DevNo].pTimer = pTimer;

	if (pCfg->DevNo < RE01_TIMER_AGT_CNT)
	{
		return Re01AgtInit(&g_Re01TimerData[pCfg->DevNo], pCfg);
	}

	return Re01TmrInit(&g_Re01TimerData[pCfg->DevNo], pCfg);
}

int TimerGetLowFreqDevCount()
{
	return RE01_TIMER_AGT_CNT;
}

int TimerGetHighFreqDevCount()
{
	return RE01_TIMER_TMR_CNT;
}

int TimerGetHighFreqDevNo()
{
	return RE01_TIMER_AGT_CNT;
}

