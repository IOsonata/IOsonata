/**-------------------------------------------------------------------------
@file	pei_hal_c_api.c
@brief	Compile-only proof that PEI, Timer and GPIO APIs work from C

The test is intentionally not executed: hardware setup and routing require
a target MCU. tests/pei/run.py compiles this as C11 with -Werror.
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"

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
