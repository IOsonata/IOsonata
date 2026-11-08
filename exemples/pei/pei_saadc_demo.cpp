/**-------------------------------------------------------------------------
@file	pei_saadc_demo.cpp
@brief	Hardware TIMER -> SAADC SAMPLE -> END/START and GPIOTE via PPI.

The ADC driver prepares the internal EasyDMA buffer. Its channels and
result buffer are never configured by PEI. No ISR forwards events/tasks.
Two LEDs distinguish TIMER compare from actual SAADC END events.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "adc_nrf52_saadc.h"
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;

static const AdcRefVolt_t s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6f, .Pin = -1},
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_RefVolt,
	.NbRefVolt = sizeof(s_RefVolt) / sizeof(s_RefVolt[0]),
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = nullptr
};

static const AdcChanCfg_t s_ChanCfg[] = {
	{
		.Chan = 0,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,
		.AcqTime = 10, // nRF52832 erratum 150: PPI START requires > 5 us
		.BurstMode = false,
		.PinP = {.PinNo = PEI_SAADC_INPUT, .Conn = ADC_PIN_CONN_NONE},
		.PinN = {.PinNo = 0, .Conn = ADC_PIN_CONN_NONE},
		.FifoMemSize = 0,
		.pFifoMem = nullptr
	}
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 2,
	"PeiSaadcDemo requires two LEDs");

volatile int g_PeiSaadcStatus;
volatile uint32_t g_PeiSaadcElapsedMs;

int main(void)
{
	Pei pei;
	Timer timer;
	AdcnRF52 adc;
	TimerCfg_t timerCfg = {};
	PeiEvent_t compareEvt = 0U;
	PeiEvent_t endEvt = 0U;
	int trig;
	int i;

	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Leds[i].PortNo, s_Leds[i].PinNo, s_Leds[i].PinOp,
			s_Leds[i].PinDir, s_Leds[i].Res, s_Leds[i].Type);
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	// ADC owns channels and a static, RAM-resident EasyDMA result buffer.
	if (!adc.Init(s_AdcCfg))
	{
		g_PeiSaadcStatus = -1;
		return 1;
	}
	if (!adc.OpenChannel(s_ChanCfg, 1))
	{
		g_PeiSaadcStatus = -2;
		return 1;
	}
	if (!adc.PreparePeiSampling())
	{
		g_PeiSaadcStatus = -3;
		return 1;
	}

	timerCfg.DevNo = PEI_SAADC_TIMER_DEVNO;
	timerCfg.Freq = PEI_SAADC_TIMER_FREQ;
	timerCfg.IntPrio = IRQ_PRIO_NORMAL;
	if (!timer.Init(timerCfg))
	{
		g_PeiSaadcStatus = -4;
		return 1;
	}
	if (!timer.Enable())
	{
		g_PeiSaadcStatus = -5;
		return 1;
	}
	if (!timer.ExecuteTask(TIMER_TASK_STOP))
	{
		g_PeiSaadcStatus = -6;
		return 1;
	}

	// One compare pulse fans out to SAADC SAMPLE and the timing LED.
	trig = timer.EnableTimerTrigger(pei, compareEvt,
		PEI_SAADC_PERIOD_MS, TIMER_TRIG_TYPE_CONTINUOUS);
	if (trig < 0)
	{
		g_PeiSaadcStatus = -7;
		return 1;
	}
	if (!adc.EnablePeiEvent(pei, endEvt, ADC_NRF52_PEI_EVENT_END))
	{
		g_PeiSaadcStatus = -8;
		return 1;
	}
	if (!adc.ConnectTask(pei, compareEvt, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		g_PeiSaadcStatus = -9;
		return 1;
	}
	if (!adc.ConnectTask(pei, endEvt, ADC_NRF52_PEI_TASK_START))
	{
		g_PeiSaadcStatus = -10;
		return 1;
	}
	if (!IOPinEnableTask(pei, compareEvt, s_Leds[1].PortNo,
		s_Leds[1].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiSaadcStatus = -11;
		return 1;
	}
	if (!IOPinEnableTask(pei, endEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiSaadcStatus = -12;
		return 1;
	}

	// Routes are enabled before the counter starts. END automatically
	// re-arms START in hardware for the next timer-triggered SAMPLE.
	if (!pei.Enable(endEvt))
	{
		g_PeiSaadcStatus = -13;
		return 1;
	}
	if (!pei.Enable(compareEvt))
	{
		g_PeiSaadcStatus = -14;
		return 1;
	}
	if (!timer.ExecuteTask(TIMER_TASK_START))
	{
		g_PeiSaadcStatus = -15;
		return 1;
	}

	g_PeiSaadcStatus = 1;

	for (;;)
	{
		__WFE();
		g_PeiSaadcElapsedMs = timer.mSecond();
	}
}
