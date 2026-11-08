/**-------------------------------------------------------------------------
@file	pei_rng_demo.cpp
@brief	nRF52832 RNG START/VALRDY/STOP hardware PPI chain.

BUT1 starts RNG. VALRDY stops RNG and toggles the standalone LED1.
No random value is exposed through PEI. RNG remains exclusively
owned by the RNG entropy driver; active SoftDevice/SDC is refused.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "rng_nrf_pei.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

volatile int g_PeiRngStatus;

int main(void)
{
	Pei pei;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t readyEvt = 0U;

	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo,
		s_Leds[0].PinOp, s_Leds[0].PinDir,
		s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinConfig(s_Buttons[0].PortNo, s_Buttons[0].PinNo,
		s_Buttons[0].PinOp, s_Buttons[0].PinDir,
		s_Buttons[0].Res, s_Buttons[0].Type);

	if (!RngNrfPeiPrepare())
	{
		g_PeiRngStatus = -1;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiRngStatus = -2;
		return 1;
	}
	if (!RngNrfPeiConnectTask(pei, startEvt, RNG_NRF_PEI_TASK_START))
	{
		g_PeiRngStatus = -3;
		return 1;
	}
	if (!RngNrfPeiEnableEvent(pei, &readyEvt, RNG_NRF_PEI_EVENT_VALRDY))
	{
		g_PeiRngStatus = -4;
		return 1;
	}
	if (!RngNrfPeiConnectTask(pei, readyEvt, RNG_NRF_PEI_TASK_STOP))
	{
		g_PeiRngStatus = -5;
		return 1;
	}
	if (!IOPinEnableTask(pei, readyEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiRngStatus = -6;
		return 1;
	}
	if (!pei.Enable(readyEvt))
	{
		g_PeiRngStatus = -7;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiRngStatus = -8;
		return 1;
	}
	g_PeiRngStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
