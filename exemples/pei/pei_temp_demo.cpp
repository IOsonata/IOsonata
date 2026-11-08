/**-------------------------------------------------------------------------
@file	pei_temp_demo.cpp
@brief	nRF52832 on-chip TEMP start and DATARDY routed by PPI.

BUT1 drives TEMP START. DATARDY drives STOP and the standalone LED1.
The normal TEMP IRQ still reads the temperature and clears DATARDY.
No CPU interrupt forwards events to PPI.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "temp_nrf5x.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

volatile int g_PeiTempStatus;

int main(void)
{
	Pei pei;
	TempnRF5x temp;
	TempSensorCfg_t cfg = {};
	PeiEvent_t startEvt = 0U;
	PeiEvent_t dataEvt = 0U;

	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo,
		s_Leds[0].PinOp, s_Leds[0].PinDir,
		s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinConfig(s_Buttons[0].PortNo, s_Buttons[0].PinNo,
		s_Buttons[0].PinOp, s_Buttons[0].PinDir,
		s_Buttons[0].Res, s_Buttons[0].Type);

	cfg.IntPrio = IRQ_PRIO_NORMAL;
	if (!temp.Init(cfg))
	{
		g_PeiTempStatus = -1;
		return 1;
	}
	if (!temp.Enable())
	{
		g_PeiTempStatus = -2;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiTempStatus = -3;
		return 1;
	}
	if (!temp.ConnectTask(pei, startEvt, TEMP_NRF_PEI_TASK_START))
	{
		g_PeiTempStatus = -4;
		return 1;
	}
	if (!temp.EnablePeiEvent(pei, dataEvt, TEMP_NRF_PEI_EVENT_DATARDY))
	{
		g_PeiTempStatus = -5;
		return 1;
	}
	if (!temp.ConnectTask(pei, dataEvt, TEMP_NRF_PEI_TASK_STOP))
	{
		g_PeiTempStatus = -6;
		return 1;
	}
	if (!IOPinEnableTask(pei, dataEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiTempStatus = -7;
		return 1;
	}
	if (!pei.Enable(dataEvt))
	{
		g_PeiTempStatus = -8;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiTempStatus = -9;
		return 1;
	}
	g_PeiTempStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
