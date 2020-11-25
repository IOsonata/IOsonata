/**-------------------------------------------------------------------------
@example	timer_demo.cpp


@brief	Timer class usage demo.


@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#include <stdbool.h>
#include <stdint.h>

#include "idelay.h"
#include "coredev/timer.h"
#include "iopinctrl.h"
//#include "bsdlib_os_bare.h"
#include "board.h"

void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

uint64_t g_TickCount = 0;
uint32_t g_Diff = 0;

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

Timer g_Timer;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
#if 1
    	uint64_t c = TimerGetNanosecond(pTimer);
    	g_Diff = c - g_TickCount;
    	g_TickCount = c;
#endif
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
	NRF_REGULATORS_S->DCDCEN = REGULATORS_DCDCEN_DCDCEN_Enabled;
	NRF_CLOCK_S->LFCLKSRC = CLOCK_LFCLKSRCCOPY_SRC_LFXO;

	msDelay(2000);
	int res = bsdlid_init(NULL, true);

	printf("res = %d %x\n", res, res);
	msDelay(1000);

	IOPinCfg(s_Leds, s_NbLeds);

    g_Timer.Init(s_TimerCfg);
	//uint64_t period = g_Timer.EnableTimerTrigger(0, 100000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("Period = %u\r\n", (uint32_t)period);
    while (1)
    {
        __WFE();
        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_Timer.mSecond(), g_Diff);
    }
}

/** @} */
