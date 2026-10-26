/**-------------------------------------------------------------------------
@file	timer_fixture.cpp

@brief	Host test support: timer fixture.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

// Deterministic timer for crypto host/register-model tests only.
// Seven ticks per read, including a 32-bit rollover, at a non-integer MHz.
// Reported benchmark times are simulated, not performance measurements.
#include <cassert>
#include "coredev/timer.h"

bool g_CryptoTimerFail = false;
bool g_CryptoTimerStopped = true;
static uint64_t s_Count;

static uint64_t Read(TimerDev_t *)
{
	assert(!g_CryptoTimerStopped);
	s_Count += 7;
	return s_Count;
}

static void Stop(TimerDev_t *)
{
	g_CryptoTimerStopped = true;
}

extern "C" int TimerGetHighFreqDevNo(void)
{
	return 7;
}

extern "C" bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	assert(pCfg->DevNo == 7);
	assert(pCfg->Freq == 1000000U);
	assert(pCfg->ClkSrc == TIMER_CLKSRC_DEFAULT);
	assert(!pCfg->bTickInt);
	if (g_CryptoTimerFail)
	{
		return false;
	}
	pTimer->Freq = 32768U;
	pTimer->GetTickCount = Read;
	pTimer->Disable = Stop;
	s_Count = 0xfffffffcULL;
	g_CryptoTimerStopped = false;
	return true;
}
