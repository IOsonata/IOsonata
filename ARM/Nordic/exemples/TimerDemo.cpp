/**-------------------------------------------------------------------------
@example	TimerDemo.cpp


@brief	Timer class usage demo.


@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

#include "timer_nrf5x.h"
#include "timer_nrf_app_timer.h"

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "stddev.h"

uint64_t g_TickCount = 0;
uint32_t g_Diff = 0;

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

#if 1
// Using RTC
TimerLFnRF5x g_Timer;
//TimerAppTimer g_Timer;
#else
// Using Timer
TimerHFnRF5x g_Timer;
#endif

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(0, 22);
#if 0
    	uint64_t c = pTimer->nSecond();
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
	IOPinConfig(0, 22, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    g_Timer.Init(s_TimerCfg);
	uint64_t period = g_Timer.EnableTimerTrigger(0, 500000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("Period = %u\r\n", (uint32_t)period);
    while (1)
    {
        __WFE();
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
    }
}
/** @} */
