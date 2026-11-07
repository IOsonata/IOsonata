/**-------------------------------------------------------------------------
@file	pei_blinky.cpp

@brief	PEI blinky: one timer event alternates two LEDs without an ISR.

Timer and GPIO drivers prepare the event and task hardware. PEI owns only
the two event-to-task relationships. The shared application contains no
MCU-specific register access or software event forwarding.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiBlinky requires two LEDs");

// Release the PEI routes before returning GPIO tasks to the pin driver.
static void PeiBlinkyCleanup(Pei &pei, Timer &timer, PeiEvent_t &evt,
	int trig, const int tasks[2])
{
	int i;

	timer.DisableTimerTrigger(trig);
	if (!pei.ReleaseEvent(evt))
	{
		return;
	}
	for (i = 0; i < 2; i++)
	{
		if (tasks[i] >= 0)
		{
			IOPinCloseTask(tasks[i]);
		}
	}
}

int main(void)
{
	Pei pei;
	Timer timer;
	TimerCfg_t timerCfg = {};
	PeiEvent_t evt = 0U;
	int trig;
	int tasks[2] = {-1, -1};
	int i;

	timerCfg.DevNo = PEI_BLINKY_TIMER_DEVNO;
	timerCfg.Freq = 1000000U;
	timerCfg.IntPrio = IRQ_PRIO_NORMAL;
	if (!timer.Init(timerCfg))
	{
		return 1;
	}
	if (!timer.Enable())
	{
		return 1;
	}

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
	}

	// Opposite initial levels: on active-low BLYST-NANO-DK,
	// LED 1 starts off and LED 2 starts on.
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);

	trig = timer.EnableTimerTrigger(pei, evt, 500U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		return 1;
	}

	// One event, two hardware tasks: PEI fan-out.
	for (i = 0; i < 2; i++)
	{
		tasks[i] = IOPinConnectTask(pei, evt, s_Leds[i].PortNo,
			s_Leds[i].PinNo, IOPIN_TASK_TOGGLE);
		if (tasks[i] < 0)
		{
			PeiBlinkyCleanup(pei, timer, evt, trig, tasks);
			return 1;
		}
	}
	if (!pei.Enable(evt))
	{
		PeiBlinkyCleanup(pei, timer, evt, trig, tasks);
		return 1;
	}

	for (;;)
	{
		__WFE();
	}
}
