/**-------------------------------------------------------------------------
@file	pei_group_demo.cpp
@brief	nRF52832 PPI channel-group EN/DIS hardware task demonstration.

TIMER3 generates one tick every 500 ms. Its compare->clear route
remains outside the group, so the timer runs continuously.
The separate tick->LED1 route is in an initially disabled PPI
group. BUT1 enables the group; BUT2 disables it. No IRQ forwards
the timer or button events, and the CPU does not toggle LEDs.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 1,
	"PeiGroupDemo needs the standalone LED1");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiGroupDemo needs two buttons");

volatile int g_PeiGroupStatus;

int main(void)
{
	Pei pei;
	PeiGroup_t group = {};
	Timer timer;
	TimerCfg_t cfg = {};
	PeiEvent_t timerEvt = 0U;
	PeiEvent_t ledEvt = 0U;
	PeiEvent_t enableEvt = 0U;
	PeiEvent_t disableEvt = 0U;
	int trig;
	int i;

	cfg.DevNo = PEI_GROUP_TIMER_DEVNO;
	cfg.Freq = PEI_GROUP_TIMER_FREQ;
	cfg.IntPrio = IRQ_PRIO_NORMAL;
	if (!timer.Init(cfg))
	{
		g_PeiGroupStatus = -1;
		return 1;
	}
	if (!timer.Enable())
	{
		g_PeiGroupStatus = -2;
		return 1;
	}
	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo,
		s_Leds[0].PinOp, s_Leds[0].PinDir,
		s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	trig = timer.EnableTimerTrigger(pei, timerEvt, 500U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		g_PeiGroupStatus = -3;
		return 1;
	}
	// A separate logical event for the same compare endpoint
	// keeps timer's own compare->clear route outside the group.
	if (!pei.BindEvent(ledEvt, pei.EventEndpoint(timerEvt)))
	{
		g_PeiGroupStatus = -4;
		return 1;
	}
	if (!IOPinEnableTask(pei, ledEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiGroupStatus = -5;
		return 1;
	}
	if (!pei.GroupInit(group))
	{
		g_PeiGroupStatus = -6;
		return 1;
	}
	if (!pei.GroupAddEvent(group, ledEvt))
	{
		g_PeiGroupStatus = -7;
		return 1;
	}

	if (!IOPinEnableEvent(pei, enableEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiGroupStatus = -8;
		return 1;
	}
	if (!IOPinEnableEvent(pei, disableEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiGroupStatus = -9;
		return 1;
	}
	if (!pei.Connect(enableEvt, pei.GroupEnableTask(group)))
	{
		g_PeiGroupStatus = -10;
		return 1;
	}
	if (!pei.Connect(disableEvt, pei.GroupDisableTask(group)))
	{
		g_PeiGroupStatus = -11;
		return 1;
	}

	// Only enable the timer's own housekeeping event and both
	// button routes. The LED route stays controlled by the group.
	if (!pei.Enable(timerEvt))
	{
		g_PeiGroupStatus = -12;
		return 1;
	}
	if (!pei.Enable(enableEvt))
	{
		g_PeiGroupStatus = -13;
		return 1;
	}
	if (!pei.Enable(disableEvt))
	{
		g_PeiGroupStatus = -14;
		return 1;
	}
	if (!timer.ExecuteTask(TIMER_TASK_START))
	{
		g_PeiGroupStatus = -15;
		return 1;
	}
	g_PeiGroupStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
