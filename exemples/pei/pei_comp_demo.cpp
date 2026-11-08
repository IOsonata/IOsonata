/**-------------------------------------------------------------------------
@file	pei_comp_demo.cpp

@brief	nRF52832 LPCOMP events and hardware START task routed by PEI.

On BLYST-NANO-DK, BUT2 starts LPCOMP through PPI. BUT1 uses AIN0:
the input toggles above/below the VDD/2 reference on press/release.
READY, CROSS, UP and DOWN events each route to a different LED task.
No comparator IRQ or CPU event forwarding is used.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "converters/analog_comp.h"
#include "coredev/iopincfg.h"
#include "coredev/pei.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static const AnalogCompCfg_t s_CompCfg = {
	.DevNo = 0,
	.Mode = ANALOG_COMP_MODE_CONTINUOUS,
	.RefSrc = 0,
	.RefVolt = 3000,
	.CompVolt = 1500,
	.AnalogIn = PEI_COMP_AIN,
	.bHystersys = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = nullptr
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiCompDemo needs four LEDs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiCompDemo needs two buttons");

// 1: PEI routes ready. Negative values identify setup failures.
volatile int g_PeiCompStatus;

int main(void)
{
	Pei pei;
	AnalogComp comp;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t readyEvt = 0U;
	PeiEvent_t crossEvt = 0U;
	PeiEvent_t upEvt = 0U;
	PeiEvent_t downEvt = 0U;
	int i;

	for (i = 0; i < 4; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}
	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	// BUT1 (P0.02) is LPCOMP AIN0, pulled high at rest and grounded
	// when pressed. Keep its SENSE pin interrupt disabled.
	if (!comp.Init(s_CompCfg))
	{
		g_PeiCompStatus = -1;
		return 1;
	}

	// BUT2 is a GPIOTE event; PEI connects it to comparator START.
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiCompStatus = -2;
		return 1;
	}
	if (!comp.ConnectTask(pei, startEvt, ANALOG_COMP_PEI_TASK_START))
	{
		g_PeiCompStatus = -3;
		return 1;
	}

	if (!comp.EnablePeiEvent(pei, readyEvt, ANALOG_COMP_PEI_EVENT_READY))
	{
		g_PeiCompStatus = -4;
		return 1;
	}
	if (!comp.EnablePeiEvent(pei, crossEvt, ANALOG_COMP_PEI_EVENT_CROSS))
	{
		g_PeiCompStatus = -5;
		return 1;
	}
	if (!comp.EnablePeiEvent(pei, upEvt, ANALOG_COMP_PEI_EVENT_UP))
	{
		g_PeiCompStatus = -6;
		return 1;
	}
	if (!comp.EnablePeiEvent(pei, downEvt, ANALOG_COMP_PEI_EVENT_DOWN))
	{
		g_PeiCompStatus = -7;
		return 1;
	}

	// Four independent, hardware-only comparator event indications.
	// LED0: READY once; LED1: CROSS both edges; LED2: UP;
	// LED3: DOWN. GPIO task channels are allocated by IOPin.
	if (!IOPinEnableTask(pei, readyEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiCompStatus = -8;
		return 1;
	}
	if (!IOPinEnableTask(pei, crossEvt, s_Leds[1].PortNo,
		s_Leds[1].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiCompStatus = -9;
		return 1;
	}
	if (!IOPinEnableTask(pei, upEvt, s_Leds[2].PortNo,
		s_Leds[2].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiCompStatus = -10;
		return 1;
	}
	if (!IOPinEnableTask(pei, downEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiCompStatus = -11;
		return 1;
	}

	if (!pei.Enable(readyEvt))
	{
		g_PeiCompStatus = -12;
		return 1;
	}
	if (!pei.Enable(crossEvt))
	{
		g_PeiCompStatus = -13;
		return 1;
	}
	if (!pei.Enable(upEvt))
	{
		g_PeiCompStatus = -14;
		return 1;
	}
	if (!pei.Enable(downEvt))
	{
		g_PeiCompStatus = -15;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiCompStatus = -16;
		return 1;
	}

	// Only BUT2 STARTs the comparator. No CPU START call here.
	g_PeiCompStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
