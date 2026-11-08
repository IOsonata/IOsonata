/**-------------------------------------------------------------------------
@file	pei_sw_event.cpp

@brief	PEI software event: one CPU write toggles two LEDs, a button too.

A software event (an EGU channel on nRF52) is connected to the toggle tasks
of two LEDs. The CPU fires it every 500 ms: both LEDs change at the same
time, from one register write. Button 1 is connected to the task that fires
the same software event, so a press toggles both LEDs in hardware, with no
GPIO interrupt.

The LEDs start in opposite states and alternate. A press adds one toggle of
both, they keep alternating.

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiSwEvent requires two LEDs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 1,
	"PeiSwEvent requires one button");

int main(void)
{
	Pei pei;
	PeiEvent_t swEvt = 0U;
	PeiEvent_t butEvt = 0U;
	int i;

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
	}
	IOPinConfig(s_Buttons[0].PortNo, s_Buttons[0].PinNo, s_Buttons[0].PinOp,
		s_Buttons[0].PinDir, s_Buttons[0].Res, s_Buttons[0].Type);

	// Opposite initial levels
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);

	if (!pei.BindSwEvent(swEvt))
	{
		return 1;
	}
	for (i = 0; i < 2; i++)
	{
		if (!IOPinEnableTask(pei, swEvt, s_Leds[i].PortNo, s_Leds[i].PinNo,
			IOPIN_TASK_TOGGLE))
		{
			return 1;
		}
	}

	// Button 1 fires the software event through its hardware task
	if (!IOPinEnableEvent(pei, butEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, PEI_SW_EVENT_BUTTON_SENSE))
	{
		return 1;
	}
	if (!pei.Connect(butEvt, pei.SwEventTask(swEvt)))
	{
		return 1;
	}

	if (!pei.Enable(swEvt) || !pei.Enable(butEvt))
	{
		return 1;
	}

	for (;;)
	{
		msDelay(500);
		pei.Trigger(swEvt);
	}
}
