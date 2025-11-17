/**-------------------------------------------------------------------------
@example	timer_demo.cpp


@brief	Timer class usage demo.

This example demonstrate how to use the generic timer

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

#include "board.h"

//#define DEMO_C
#define DEMO_C_OBJ

void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

#ifdef MCUOSC
// Set custom board oscillator
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

uint64_t g_TickCount = 0;
uint32_t g_Period[3] = {0,};

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

#ifdef DEMO_C
TimerDev_t g_TimerDev;
#else
Timer g_Timer;
#endif

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
	static uint64_t precnt[3] = {0, };
	uint64_t c = TimerGetNanosecond(pTimer);

    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
    	g_Period[0] = c - precnt[0];
    	precnt[0] = c;
    }
    if (Evt & TIMER_EVT_TRIGGER(1))
    {
    	// Flip GPIO for oscilloscope measurement
    	if (s_NbLeds > 0)
    	{
    		IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
    	}
    	g_Period[1] = c - precnt[1];
    	precnt[1] = c;
    }
    if (Evt & TIMER_EVT_COUNTER_OVR)
    {
    	if (s_NbLeds > 1)
    	{
    		IOPinToggle(s_Leds[2].PortNo, s_Leds[2].PinNo);
    	}
    	g_Period[2] = c - precnt[2];
    	precnt[2] = c;
    }
	g_TickCount = c;
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
	IOPinCfg(s_Leds, s_NbLeds);

#ifdef DEMO_C
	TimerInit(&g_TimerDev, &s_TimerCfg);

	uint64_t period = msTimerEnableTrigger(&g_TimerDev, 0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS, NULL, NULL);
	if (period == 0)
	{
		printf("Trigger 0 failed\r\n");
	}
	period = msTimerEnableTrigger(&g_TimerDev, 1, 100UL, TIMER_TRIG_TYPE_CONTINUOUS, NULL, NULL);
	if (period == 0)
	{
		printf("Trigger 1 failed\r\n");
	}

#else
    g_Timer.Init(s_TimerCfg);

    // Configure 100ms timer interrupt trigger
	uint64_t period = g_Timer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS);
	if (period == 0)
	{
		printf("Trigger 0 failed\r\n");
	}
	period = g_Timer.EnableTimerTrigger(1, 100UL, TIMER_TRIG_TYPE_CONTINUOUS);
	if (period == 0)
	{
		printf("Trigger 1 failed\r\n");
	}
#endif

	printf("Period = %u\r\n", (uint32_t)period);
    while (1)
    {
        __WFE();
#ifdef DEMO_C
        printf("Count = %u ms, TrigPeriod = %u us, %u us\r\n", (uint32_t)TimerGetMilisecond(&g_TimerDev), g_Period[0] / 1000, g_Period[1] / 1000);
#elif defined(DEMO_C_OBJ)
        printf("Count = %u ms, TrigPeriod = %u us, %u us\r\n", (uint32_t)TimerGetMilisecond(g_Timer), g_Period[0] / 1000, g_Period[1] / 1000);
#else
        printf("Count = %u ms, TrigPeriod = %u us, %u us\r\n", (uint32_t)g_Timer.mSecond(), g_Period[0] / 1000, g_Period[1] / 1000);
#endif
    }
}

/** @} */
