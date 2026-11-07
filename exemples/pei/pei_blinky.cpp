/**-------------------------------------------------------------------------
@file	pei_blinky.cpp

@brief	PEI blinky: two LEDs alternate; button 1 starts, button 2 stops.

Peripheral drivers prepare all hardware events and tasks. PEI manages
only the event-to-task connections. No interrupt forwards any event.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiBlinky requires two LEDs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiBlinky requires two buttons");

// When setup fails, detach each PEI event before its peripheral driver
// reuses the corresponding hardware task or event endpoint.
static void PeiBlinkyCleanup(Pei &pei, Timer &timer, PeiEvent_t &blinkEvt,
	PeiEvent_t buttonEvt[2], int trig, const int ledTasks[2],
	const int buttonInputs[2])
{
	int i;
	bool blinkReleased;

	for (i = 0; i < 2; i++)
	{
		if (buttonEvt[i] != 0U)
		{
			pei.Disable(buttonEvt[i]);
		}
	}
	timer.ExecuteTask(TIMER_TASK_STOP);
	timer.DisableTimerTrigger(trig);

	blinkReleased = false;
	if (blinkEvt != 0U)
	{
		blinkReleased = pei.ReleaseEvent(blinkEvt);
	}
	for (i = 0; i < 2; i++)
	{
		if (buttonEvt[i] == 0U)
		{
			continue;
		}
		if (!pei.ReleaseEvent(buttonEvt[i]))
		{
			continue;
		}
		if (buttonInputs[i] >= 0)
		{
			IOPinCloseEvent(buttonInputs[i]);
		}
	}
	if (blinkReleased)
	{
		for (i = 0; i < 2; i++)
		{
			if (ledTasks[i] >= 0)
			{
				IOPinCloseTask(ledTasks[i]);
			}
		}
	}
}

int main(void)
{
	Pei pei;
	Timer timer;
	TimerCfg_t timerCfg = {};
	PeiEvent_t blinkEvt = 0U;
	PeiEvent_t buttonEvt[2] = {0U, 0U};
	int ledTasks[2] = {-1, -1};
	int buttonInputs[2] = {-1, -1};
	int trig;
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
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	// Active-low LEDs start in opposite states.
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);

	// A compare event toggles both LEDs every 500 ms.
	trig = timer.EnableTimerTrigger(pei, blinkEvt, 500U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		return 1;
	}

	// Keep the timer clock enabled, but leave the counter stopped
	// until button 1 activates its hardware START task.
	if (!timer.ExecuteTask(TIMER_TASK_STOP))
	{
		PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
			trig, ledTasks, buttonInputs);
		return 1;
	}

	for (i = 0; i < 2; i++)
	{
		ledTasks[i] = IOPinConnectTask(pei, blinkEvt,
			s_Leds[i].PortNo, s_Leds[i].PinNo, IOPIN_TASK_TOGGLE);
		if (ledTasks[i] < 0)
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledTasks, buttonInputs);
			return 1;
		}
	}

	// Pressing button 1 starts TIMER2; pressing button 2 stops it.
	// GPIOTE detects the falling edges in hardware, not via a GPIO ISR.
	for (i = 0; i < 2; i++)
	{
		buttonInputs[i] = IOPinConnectEvent(pei, buttonEvt[i],
			s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			PEI_BLINKY_BUTTON_SENSE);
		if (buttonInputs[i] < 0)
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledTasks, buttonInputs);
			return 1;
		}
		if (!timer.ConnectTask(pei, buttonEvt[i],
			i == 0 ? TIMER_TASK_START : TIMER_TASK_STOP))
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledTasks, buttonInputs);
			return 1;
		}
	}

	if (!pei.Enable(blinkEvt))
	{
		PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
			trig, ledTasks, buttonInputs);
		return 1;
	}
	for (i = 0; i < 2; i++)
	{
		if (!pei.Enable(buttonEvt[i]))
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledTasks, buttonInputs);
			return 1;
		}
	}

	for (;;)
	{
		__WFE();
	}
}
