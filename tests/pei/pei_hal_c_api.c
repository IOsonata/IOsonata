/**-------------------------------------------------------------------------
@file	pei_hal_c_api.c
@brief	Compile-only proof that PEI, Timer and GPIO APIs work from C

The test is intentionally not executed: hardware setup and routing require
a target MCU. tests/pei/run.py compiles this as C11 with -Werror.
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "adc_nrf52_pei.h"

bool PeiHalCApplication(TimerDev_t *pTimer)
{
	PeiDev_t pei;
	PeiEvent_t evt;
	int trigger;

	evt = 0U;
	if (!PeiInit(&pei, 0))
	{
		return false;
	}
	if (!IOPinEnableEvent(&pei, &evt, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		return false;
	}
	if (!IOPinEnableTask(&pei, evt, 0, 30, IOPIN_TASK_TOGGLE))
	{
		return false;
	}
	trigger = TimerEnablePeiTrigger(pTimer, &pei, &evt, 500,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trigger < 0)
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, &pei, evt, TIMER_TASK_START))
	{
		return false;
	}
	TimerExecuteTask(pTimer, TIMER_TASK_STOP);
	PeiEnableEvent(&pei, evt);
	PeiDisableEvent(&pei, evt);
	TimerDisableTrigger(pTimer, trigger);
	IOPinDisableTask(&pei, evt, 0, 30);
	IOPinDisableEvent(&pei, evt);
	PeiReleaseEvent(&pei, &evt);
	return PeiDeinit(&pei);
}

// Compile-only check of RTC event producer and hardware task endpoint.
bool PeiHalRtcCApplication(TimerDev_t *pTimer, PeiDev_t *pPei)
{
	PeiEvent_t event;

	event = 0U;
	if (!TimerEnablePeiEvent(pTimer, pPei, &event, TIMER_EVT_TICK))
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, pPei, event, TIMER_TASK_CLEAR))
	{
		TimerDisablePeiEvent(pTimer, pPei, event);
		return false;
	}
	if (!TimerDisablePeiEvent(pTimer, pPei, event))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &event);
}

/* Compile-only check of the shared PORT source and its lifetime. */
bool PeiHalPortCApplication(PeiDev_t *pPei)
{
	PeiEvent_t evt;

	evt = 0U;
	IOPinSetSense(0, 2, IOPINSENSE_LOW_TRANSITION);
	if (!IOPinEnablePortEvent(pPei, &evt))
	{
		return false;
	}
	if (!IOPinDisablePortEvent(pPei, evt))
	{
		return false;
	}
	IOPinSetSense(0, 2, IOPINSENSE_DISABLE);
	return PeiReleaseEvent(pPei, &evt);
}

// Compile-only check of a software event starting a Timer and a GPIO task,
// fired from code and from a GPIO event.
bool PeiHalSwEventCApplication(TimerDev_t *pTimer, PeiDev_t *pPei)
{
	PeiEvent_t sw;
	PeiEvent_t button;

	sw = 0U;
	button = 0U;
	if (!PeiBindSwEvent(pPei, &sw))
	{
		return false;
	}
	if (!TimerConnectTask(pTimer, pPei, sw, TIMER_TASK_START) ||
		!IOPinEnableTask(pPei, sw, 0, 30, IOPIN_TASK_SET))
	{
		PeiReleaseEvent(pPei, &sw);
		return false;
	}
	if (IOPinEnableEvent(pPei, &button, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		PeiConnectTask(pPei, button, PeiGetSwEventTask(pPei, sw));
		PeiEnableEvent(pPei, button);
	}
	PeiEnableEvent(pPei, sw);
	PeiTriggerSwEvent(pPei, sw);
	return PeiReleaseEvent(pPei, &sw);
}

// Compile-only: SAADC remains configured by the ADC HAL, not by PEI.
// These are real C-linkage routing functions, not C++ debugger globals.
bool PeiSaadcCApplication(PeiDev_t *pPei, PeiEvent_t trigger)
{
	PeiEvent_t endEvent;

	endEvent = 0U;
	if (!AdcNrf52PreparePeiSampling())
	{
		return false;
	}
	if (!AdcNrf52PeiBindEvent(pPei, &endEvent, ADC_NRF52_PEI_EVENT_END))
	{
		return false;
	}
	if (!AdcNrf52PeiConnectTask(pPei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiConnectTask(pPei, endEvent, ADC_NRF52_PEI_TASK_START))
	{
		return false;
	}
	if (!AdcNrf52PeiDisconnectTask(pPei, endEvent, ADC_NRF52_PEI_TASK_START))
	{
		return false;
	}
	if (!AdcNrf52PeiDisconnectTask(pPei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiUnbindEvent(pPei, endEvent))
	{
		return false;
	}
	return PeiReleaseEvent(pPei, &endEvent);
}
