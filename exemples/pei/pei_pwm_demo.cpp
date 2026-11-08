/**-------------------------------------------------------------------------
@file	pei_pwm_demo.cpp
@brief	nRF52832 hardware GPIO -> PWM tasks -> GPIOTE indication.

BUT1 starts PWM0 sequence 0 through PPI.
BUT2 stops PWM0 through PPI. No CPU interrupt forwards events.
PWM0 channel 0 drives the RGB red LED with 30 percent duty at 1 kHz.
SEQSTARTED0 toggles the separate board LED1; STOPPED toggles RGB blue.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/pwm.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static const PwmCfg_t s_PwmCfg = {
	.DevNo = 0,
	.Freq = 1000U,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.pEvtHandler = nullptr
};

static const PwmChanCfg_t s_PwmChannel = {
	.Chan = 0,
	.Pol = PWM_POL_LOW,
	.Port = BLYST_NANO_DK_LEDR_PORT,
	.Pin = BLYST_NANO_DK_LEDR_PIN,
	.PinOp = BLYST_NANO_DK_LEDR_PINOP
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiPwmDemo needs LED1 and RGB outputs");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 2,
	"PeiPwmDemo needs two buttons");

volatile int g_PeiPwmStatus; // 1 means hardware PEI routes armed.

int main(void)
{
	Pei pei;
	Pwm pwm;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t stopEvt = 0U;
	PeiEvent_t startedEvt = 0U;
	PeiEvent_t stoppedEvt = 0U;
	int i;

	// The discrete LED1 and RGB blue are GPIOTE task destinations.
	// RGB red is owned exclusively by PWM0.
	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo, s_Leds[0].PinOp,
		s_Leds[0].PinDir, s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinConfig(s_Leds[3].PortNo, s_Leds[3].PinNo, s_Leds[3].PinOp,
		s_Leds[3].PinDir, s_Leds[3].Res, s_Leds[3].Type);
	IOPinSet(s_Leds[3].PortNo, s_Leds[3].PinNo);
	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	// PWM owns the output pin, static RAM sequence and duty cycle.
	// Hardware SEQSTART0 is not triggered by initialization.
	if (!pwm.Init(s_PwmCfg))
	{
		g_PeiPwmStatus = -1;
		return 1;
	}
	if (!pwm.OpenChannel(&s_PwmChannel, 1))
	{
		g_PeiPwmStatus = -2;
		return 1;
	}
	if (!pwm.DutyCycle(0, 30))
	{
		g_PeiPwmStatus = -3;
		return 1;
	}

	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiPwmStatus = -4;
		return 1;
	}
	if (!IOPinEnableEvent(pei, stopEvt, s_Buttons[1].PortNo,
		s_Buttons[1].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiPwmStatus = -5;
		return 1;
	}
	if (!pwm.ConnectTask(pei, startEvt, PWM_PEI_TASK_SEQSTART0))
	{
		g_PeiPwmStatus = -6;
		return 1;
	}
	if (!pwm.ConnectTask(pei, stopEvt, PWM_PEI_TASK_STOP))
	{
		g_PeiPwmStatus = -7;
		return 1;
	}
	if (!pwm.EnablePeiEvent(pei, startedEvt, PWM_PEI_EVENT_SEQSTARTED0))
	{
		g_PeiPwmStatus = -8;
		return 1;
	}
	if (!pwm.EnablePeiEvent(pei, stoppedEvt, PWM_PEI_EVENT_STOPPED))
	{
		g_PeiPwmStatus = -9;
		return 1;
	}
	if (!IOPinEnableTask(pei, startedEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiPwmStatus = -10;
		return 1;
	}
	if (!IOPinEnableTask(pei, stoppedEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiPwmStatus = -11;
		return 1;
	}

	if (!pei.Enable(startedEvt))
	{
		g_PeiPwmStatus = -12;
		return 1;
	}
	if (!pei.Enable(stoppedEvt))
	{
		g_PeiPwmStatus = -13;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiPwmStatus = -14;
		return 1;
	}
	if (!pei.Enable(stopEvt))
	{
		g_PeiPwmStatus = -15;
		return 1;
	}
	g_PeiPwmStatus = 1;

	for (;;)
	{
		__WFE();
	}
}
