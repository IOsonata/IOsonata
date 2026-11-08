/**-------------------------------------------------------------------------
@file	pei_port_demo.cpp
@brief	Shared GPIOTE PORT and dedicated IN[n] events routed via PEI.

BLYST-NANO-DK BUT1 uses SENSE -> GPIOTE.PORT -> PPI -> LED0.
BUT2 uses GPIOTE.IN[n] -> PPI -> LED1.
No CPU interrupt is used for event-to-task forwarding.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiPortDemo requires two LEDs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiPortDemo requires two buttons");

// 1: both sources enabled. Negative values identify failed steps.
volatile int g_PeiPortDemoStatus;

int main(void)
{
	Pei pei;
	PeiEvent_t portEvt = 0U;
	PeiEvent_t pinEvt = 0U;
	PeiEvent_t staleEvt = 0U;
	int i;

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	// One shared PORT source for SENSE-enabled pins. Only BUT1 has SENSE.
	IOPinSetSense(s_Buttons[0].PortNo, s_Buttons[0].PinNo,
		IOPINSENSE_LOW_TRANSITION);
	if (!IOPinEnablePortEvent(pei, portEvt))
	{
		g_PeiPortDemoStatus = -1;
		return 1;
	}
	if (!IOPinEnableTask(pei, portEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiPortDemoStatus = -2;
		return 1;
	}

	// Unbind and rebind the same logical event; the LED route is retained.
	if (!IOPinDisablePortEvent(pei, portEvt))
	{
		g_PeiPortDemoStatus = -3;
		return 1;
	}
	if (pei.EventEndpoint(portEvt) != 0U)
	{
		g_PeiPortDemoStatus = -4;
		return 1;
	}
	if (!IOPinEnablePortEvent(pei, portEvt))
	{
		g_PeiPortDemoStatus = -5;
		return 1;
	}

	// Full lifecycle: free the task channel and source identity, then
	// allocate both again. Stale logical event handles must stay invalid.
	staleEvt = portEvt;
	if (!IOPinDisableTask(pei, portEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo))
	{
		g_PeiPortDemoStatus = -10;
		return 1;
	}
	if (!IOPinDisablePortEvent(pei, portEvt))
	{
		g_PeiPortDemoStatus = -11;
		return 1;
	}
	if (!pei.ReleaseEvent(portEvt))
	{
		g_PeiPortDemoStatus = -12;
		return 1;
	}
	if (!IOPinEnablePortEvent(pei, portEvt))
	{
		g_PeiPortDemoStatus = -13;
		return 1;
	}
	if (pei.EventEndpoint(staleEvt) != 0U)
	{
		g_PeiPortDemoStatus = -14;
		return 1;
	}
	if (!IOPinEnableTask(pei, portEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiPortDemoStatus = -15;
		return 1;
	}

	// Dedicated GPIOTE IN[n] input does not use GPIO SENSE/PORT.
	if (!IOPinEnableEvent(pei, pinEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiPortDemoStatus = -6;
		return 1;
	}
	if (!IOPinEnableTask(pei, pinEvt, s_Leds[1].PortNo,
		s_Leds[1].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiPortDemoStatus = -7;
		return 1;
	}
	if (!pei.Enable(portEvt))
	{
		g_PeiPortDemoStatus = -8;
		return 1;
	}
	if (!pei.Enable(pinEvt))
	{
		g_PeiPortDemoStatus = -9;
		return 1;
	}
	g_PeiPortDemoStatus = 1;

	for (;;)
	{
		__WFE();
	}
}
