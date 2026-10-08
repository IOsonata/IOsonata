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
#include <stdio.h>

#include "idelay.h"
#include "coredev/timer.h"
#include "iopinctrl.h"

#include "board.h"

#ifdef TIMER_DEMO_UART
#include "coredev/uart.h"
#include "stddev.h"
#endif

//#define DEMO_C
#define DEMO_C_OBJ

#ifndef TIMER_DEMO_DEVNO
#ifdef TIMER_DEVNO
#define TIMER_DEMO_DEVNO TIMER_DEVNO
#else
#define TIMER_DEMO_DEVNO 0
#endif
#endif
#ifndef TIMER_DEMO_FREQ
#ifdef TIMER_FREQ
#define TIMER_DEMO_FREQ TIMER_FREQ
#else
#define TIMER_DEMO_FREQ 0
#endif
#endif

void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

#ifdef MCUOSC
// Set custom board oscillator
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

volatile uint64_t g_TickCount = 0;
volatile uint64_t g_Period[5] = {0,};
volatile uint32_t g_TriggerCount[4] = {0,};
// Actual accepted trigger periods, in nanoseconds; zero means not enabled.
volatile uint64_t g_TriggerPeriod[4] = {0,};
volatile bool g_TimerInitOk = false;

const static TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEMO_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = TIMER_DEMO_FREQ,
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

#ifdef DEMO_C
TimerDev_t g_TimerDev;
#else
Timer g_Timer;
#endif

static uint64_t s_PreCnt[5] = {0, };

#ifdef TIMER_DEMO_UART
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};
static UART s_Uart;
#endif

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
	uint64_t c = TimerGetNanosecond(pTimer);

	for (int i = 0; i < 4; i++)
	{
		if (Evt & TIMER_EVT_TRIGGER(i))
		{
			if (i < s_NbLeds)
				IOPinToggle(s_Leds[i].PortNo, s_Leds[i].PinNo);
			g_Period[i] = c - s_PreCnt[i];
			s_PreCnt[i] = c;
			g_TriggerCount[i] = g_TriggerCount[i] + 1;
		}
	}
	if (Evt & TIMER_EVT_COUNTER_OVR)
	{
		g_Period[4] = c - s_PreCnt[4];
		s_PreCnt[4] = c;
	}
	g_TickCount = c;
}

// Main application entry
int main(void)
{
	IOPinCfg(s_Leds, s_NbLeds);

#ifdef TIMER_DEMO_UART
	if (!s_Uart.Init(s_UartCfg)) while (1) __WFE();
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, NULL, _IONBF, 0);
#endif

#ifdef DEMO_C
	g_TimerInitOk = TimerInit(&g_TimerDev, &s_TimerCfg);
	TimerDev_t *dev = &g_TimerDev;
#else
	g_TimerInitOk = g_Timer.Init(s_TimerCfg);
	TimerDev_t *dev = g_Timer;
#endif
	if (!g_TimerInitOk)
	{
		printf("Timer %d initialization failed\r\n", s_TimerCfg.DevNo);
		return 1;
	}

	// Keep trigger 1 slower than trigger 0 for timers with ordered compares.
	static const uint32_t periodsMs[] = {100, 1000, 250, 500};
	int triggers = TimerGetMaxTrigger(dev);
	if (triggers < 1)
	{
		printf("No timer triggers available\r\n");
		TimerDisable(dev);
		return 1;
	}
	if (triggers > 4) triggers = 4;
	for (int i = 0; i < triggers; i++)
	{
		uint64_t ns = (uint64_t)periodsMs[i] * 1000000ULL;
#ifdef DEMO_C
		uint64_t period = nsTimerEnableTrigger(dev, i,
			ns, TIMER_TRIG_TYPE_CONTINUOUS, NULL, NULL);
#else
		uint64_t period = g_Timer.EnableTimerTrigger(i,
			ns, TIMER_TRIG_TYPE_CONTINUOUS);
#endif
		g_TriggerPeriod[i] = period;
		printf("Timer %d trigger %d: %lu ms\r\n", s_TimerCfg.DevNo, i, (unsigned long)(period / 1000000ULL));
		if (period == 0)
		{
			for (int n = 0; n < i; n++) TimerDisableTrigger(dev, n);
			TimerDisable(dev);
			return 1;
		}
	}

	uint32_t previousCounts[4] = {0,};
	while (1)
	{
		__WFE();
		uint64_t periods[4];
		bool changed = false;
		uint32_t state = DisableInterrupt();
		for (int i = 0; i < triggers; i++)
		{
			periods[i] = g_Period[i];
			if (previousCounts[i] != g_TriggerCount[i]) changed = true;
			previousCounts[i] = g_TriggerCount[i];
		}
		EnableInterrupt(state);
		if (!changed) continue;
		printf("Count = %lu ms", (unsigned long)TimerGetMilisecond(dev));
		for (int i = 0; i < triggers; i++)
			printf(", T%d = %lu us", i, (unsigned long)(periods[i] / 1000));
		printf("\r\n");
	}
}
