/**-------------------------------------------------------------------------
@file	pei_blinky.cpp

@brief	Two LEDs alternate; button 1 starts, button 2 stops the timer.

Timer and GPIO drivers prepare hardware event/task endpoints. PEI owns
only the connections. C and C++ applications use the same PeiDev_t
matrix; no interrupt or software callback forwards these events.

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

static void PeiBlinkyCleanup(Pei &pei, Timer &timer, PeiEvent_t &blinkEvt,
	PeiEvent_t buttonEvt[2], int trig, const bool ledReady[2],
	const bool buttonReady[2])
{
	int i;

	for (i = 0; i < 2; i++)
	{
		if (buttonEvt[i] != 0U)
		{
			pei.Disable(buttonEvt[i]);
		}
	}
	timer.ExecuteTask(TIMER_TASK_STOP);

	// Peripheral event channels are released only after PEI unbinds them.
	for (i = 0; i < 2; i++)
	{
		if (buttonReady[i])
		{
			if (IOPinDisableEvent(pei, buttonEvt[i]))
			{
				pei.ReleaseEvent(buttonEvt[i]);
			}
		}
	}
	timer.DisableTimerTrigger(trig);

	// Each GPIO task remains owned until its PEI relationship is removed.
	for (i = 0; i < 2; i++)
	{
		if (ledReady[i])
		{
			IOPinDisableTask(pei, blinkEvt,
				s_Leds[i].PortNo, s_Leds[i].PinNo);
		}
	}
	if (blinkEvt != 0U)
	{
		pei.ReleaseEvent(blinkEvt);
	}
}

int main(void)
{
	Pei pei;
	Timer timer;
	TimerCfg_t timerCfg = {};
	PeiEvent_t blinkEvt = 0U;
	PeiEvent_t buttonEvt[2] = {0U, 0U};
	bool ledReady[2] = {false, false};
	bool buttonReady[2] = {false, false};
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

	// Opposite initial levels on active-low BLYST-NANO-DK LEDs.
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);

	trig = timer.EnableTimerTrigger(pei, blinkEvt, 500U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		return 1;
	}
	if (!timer.ExecuteTask(TIMER_TASK_STOP))
	{
		PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
			trig, ledReady, buttonReady);
		return 1;
	}

	for (i = 0; i < 2; i++)
	{
		ledReady[i] = IOPinEnableTask(pei, blinkEvt,
			s_Leds[i].PortNo, s_Leds[i].PinNo, IOPIN_TASK_TOGGLE);
		if (!ledReady[i])
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledReady, buttonReady);
			return 1;
		}
	}

	for (i = 0; i < 2; i++)
	{
		// GPIOTE captures button presses in hardware; no GPIO ISR.
		buttonReady[i] = IOPinEnableEvent(pei, buttonEvt[i],
			s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			PEI_BLINKY_BUTTON_SENSE);
		if (!buttonReady[i])
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledReady, buttonReady);
			return 1;
		}
		if (!timer.ConnectTask(pei, buttonEvt[i],
			i == 0 ? TIMER_TASK_START : TIMER_TASK_STOP))
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledReady, buttonReady);
			return 1;
		}
	}

	if (!pei.Enable(blinkEvt))
	{
		PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
			trig, ledReady, buttonReady);
		return 1;
	}
	for (i = 0; i < 2; i++)
	{
		if (!pei.Enable(buttonEvt[i]))
		{
			PeiBlinkyCleanup(pei, timer, blinkEvt, buttonEvt,
				trig, ledReady, buttonReady);
			return 1;
		}
	}

	for (;;)
	{
		__WFE();
	}
}
