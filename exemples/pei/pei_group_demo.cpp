/**-------------------------------------------------------------------------
@file	pei_group_demo.cpp
@brief	nRF52832 hardware PPI group one-shot enable/disable.

BUT1 enables a PPI group through its EN task. BUT2 is a second
hardware event with two group-owned routes: LED1.TOGGLE and its
own group DIS task. One BUT2 press toggles LED1 and disarms both
group channels. Another BUT2 press does nothing until BUT1 re-arms.
No CPU interrupt forwards either event.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 1,
	"PeiGroupDemo needs discrete LED1");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiGroupDemo needs two buttons");

volatile int g_PeiGroupStatus;

int main(void)
{
	Pei pei;
	PeiGroup_t group = {};
	PeiEvent_t armEvt = 0U;
	PeiEvent_t fireEvt = 0U;
	int i;

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
	if (!IOPinEnableEvent(pei, armEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiGroupStatus = -1;
		return 1;
	}
	if (!IOPinEnableEvent(pei, fireEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiGroupStatus = -2;
		return 1;
	}
	if (!pei.GroupInit(group))
	{
		g_PeiGroupStatus = -3;
		return 1;
	}
	// The arming channel is outside the group and always enabled.
	if (!pei.Connect(armEvt, pei.GroupEnableTask(group)))
	{
		g_PeiGroupStatus = -4;
		return 1;
	}
	if (!IOPinEnableTask(pei, fireEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiGroupStatus = -5;
		return 1;
	}
	// Hardware one-shot: the group disables itself after a BUT2 event.
	if (!pei.Connect(fireEvt, pei.GroupDisableTask(group)))
	{
		g_PeiGroupStatus = -6;
		return 1;
	}
	if (!pei.GroupAddEvent(group, fireEvt))
	{
		g_PeiGroupStatus = -7;
		return 1;
	}
	if (!pei.Enable(armEvt))
	{
		g_PeiGroupStatus = -8;
		return 1;
	}
	// Deliberately do not call pei.Enable(fireEvt):
	// BUT2's two PPI routes are enabled only by the group's EN task.
	g_PeiGroupStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
