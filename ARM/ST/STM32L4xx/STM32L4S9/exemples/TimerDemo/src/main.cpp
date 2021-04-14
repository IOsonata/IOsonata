/**-------------------------------------------------------------------------
@example	timer_demo.cpp

@brief	Timer example on STM32L4xx

This example shows how to use timer

@author	Hoang Nguyen Hoan
@date	July 21, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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
#include <stdio.h>

#include "timer_lptim_stm32l4xx.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_LFXTAL,
	.Freq = 5000,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler
};

LPTimerSTM32L4xx g_Timer;
uint32_t msec = 0;

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
	if (Evt & TIMER_EVT_TRIGGER(0))
	{
		//printf("Timer irq");
		IOPinToggle(3, 0);
		msec = pTimer->mSecond();
	}
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	IOPinConfig(3, 0, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    g_Timer.Init(s_TimerCfg);

    // change to lower freq to use low long trigger period
    uint32_t freq = g_Timer.Frequency();
    printf("Freq set : %u\r\n", freq);

	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_SINGLE);//CONTINUOUS);
	if (period == 0)
	{
		printf("period too low\r\n");
	}

    while (1)
    {
    	uint32_t cnt = g_Timer.TickCount();
    	printf("Tick : %d %x\r\n", cnt, LPTIM1->ISR);
    	//LPTIM1->ICR = LPTIM1->ISR;
     //   __WFE();
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
    }

	return 0;
}
