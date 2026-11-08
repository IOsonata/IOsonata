/**-------------------------------------------------------------------------
@file	pei_qdec_demo.cpp
@brief	nRF52832 QDEC sampling START/STOP and hardware event routing.

BUT1 starts QDEC; BUT2 stops it. SAMPLERDY toggles discrete LED1,
STOPPED toggles RGB blue. The decoder's normal IRQ still owns READCLRACC
and accumulator folding; the buttons are not a quadrature encoder.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/qdec.h"
#include "coredev/iopincfg.h"
#include "coredev/pei.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static const QdecCfg_t s_QdecCfg = {
	.DevNo = 0,
	.pPins = s_Buttons,
	.NbPins = sizeof(s_Buttons) / sizeof(s_Buttons[0]),
	.usSamplePeriod = 131072U,
	.usReportPeriod = 131072U,
	.bDebounce = true,
	.bLedActHigh = false,
	.usLedLead = 0U,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = nullptr
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiQdecDemo needs discrete LED1 and RGB blue");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiQdecDemo needs two buttons");

volatile int g_PeiQdecStatus;

int main(void)
{
	Pei pei;
	Qdec qdec;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t stopEvt = 0U;
	PeiEvent_t sampleEvt = 0U;
	PeiEvent_t stoppedEvt = 0U;
	int i;

	for (i = 0; i < 4; i += 3)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo,
			s_Leds[i].PinOp, s_Leds[i].PinDir,
			s_Leds[i].Res, s_Leds[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}
	if (!qdec.Init(s_QdecCfg))
	{
		g_PeiQdecStatus = -1;
		return 1;
	}
	if (!qdec.PreparePeiStart())
	{
		g_PeiQdecStatus = -2;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiQdecStatus = -3;
		return 1;
	}
	if (!IOPinEnableEvent(pei, stopEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiQdecStatus = -4;
		return 1;
	}
	if (!qdec.ConnectTask(pei, startEvt, QDEC_PEI_TASK_START))
	{
		g_PeiQdecStatus = -5;
		return 1;
	}
	if (!qdec.ConnectTask(pei, stopEvt, QDEC_PEI_TASK_STOP))
	{
		g_PeiQdecStatus = -6;
		return 1;
	}
	if (!qdec.EnablePeiEvent(pei, sampleEvt, QDEC_PEI_EVENT_SAMPLERDY))
	{
		g_PeiQdecStatus = -7;
		return 1;
	}
	if (!qdec.EnablePeiEvent(pei, stoppedEvt, QDEC_PEI_EVENT_STOPPED))
	{
		g_PeiQdecStatus = -8;
		return 1;
	}
	if (!IOPinEnableTask(pei, sampleEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiQdecStatus = -9;
		return 1;
	}
	if (!IOPinEnableTask(pei, stoppedEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiQdecStatus = -10;
		return 1;
	}
	if (!pei.Enable(sampleEvt))
	{
		g_PeiQdecStatus = -11;
		return 1;
	}
	if (!pei.Enable(stoppedEvt))
	{
		g_PeiQdecStatus = -12;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiQdecStatus = -13;
		return 1;
	}
	if (!pei.Enable(stopEvt))
	{
		g_PeiQdecStatus = -14;
		return 1;
	}
	g_PeiQdecStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
