/**-------------------------------------------------------------------------
@file	pei_blinky.cpp

@brief	PEI blinky: timer compare toggles a GPIO task without any ISR.

All hardware preparation is handled by the IOsonata Timer and GPIO drivers.
PEI manages only the hardware event-to-task relationship. This source
contains no MCU-specific register access.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;

extern "C" {
volatile uint32_t g_PeiStatus;
}

int main(void)
{
	Pei pei;
	Timer timer;
	TimerCfg_t timerCfg = {};
	PeiEvent_t evt = 0U;
	int trig;
	int task;

	timerCfg.DevNo = PEI_BLINKY_TIMER_DEVNO;
	timerCfg.Freq = 1000000U;
	timerCfg.IntPrio = IRQ_PRIO_NORMAL;
	if (!timer.Init(timerCfg))
	{
		g_PeiStatus = 2U;
		return 1;
	}
	if (!timer.Enable())
	{
		g_PeiStatus = 2U;
		return 1;
	}

	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo, s_Leds[0].PinOp,
		s_Leds[0].PinDir, s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);

	trig = timer.EnableTimerTrigger(pei, evt, 500U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		g_PeiStatus = 3U;
		return 1;
	}
	task = IOPinConnectTask(pei, evt, s_Leds[0].PortNo, s_Leds[0].PinNo,
		IOPIN_TASK_TOGGLE);
	if (task < 0)
	{
		g_PeiStatus = 4U;
		timer.DisableTimerTrigger(trig);
		return 1;
	}
	if (!pei.Enable(evt))
	{
		g_PeiStatus = 5U;
		timer.DisableTimerTrigger(trig);
		// Release the logical links before releasing the GPIO task.
		if (pei.ReleaseEvent(evt))
		{
			IOPinCloseTask(task);
		}
		return 1;
	}

	g_PeiStatus = 1U;
	for (;;)
	{
		__WFE();
	}
}
