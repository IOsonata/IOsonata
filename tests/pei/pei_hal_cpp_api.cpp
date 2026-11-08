/**-------------------------------------------------------------------------
@file	pei_hal_cpp_api.cpp
@brief	Compile-only check of C++ wrappers over the shared C PEI matrix
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "adc_nrf52_pei.h"

bool PeiHalCppApplication(Timer &timer)
{
	Pei pei;
	PeiEvent_t evt = 0U;
	int trigger;

	if (!IOPinEnableEvent(pei, evt, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		return false;
	}
	if (!IOPinEnableTask(pei, evt, 0, 30, IOPIN_TASK_TOGGLE))
	{
		return false;
	}
	trigger = timer.EnableTimerTrigger(pei, evt, 500,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trigger < 0)
	{
		return false;
	}
	if (!timer.ConnectTask(pei, evt, TIMER_TASK_START))
	{
		return false;
	}
	timer.ExecuteTask(TIMER_TASK_STOP);
	pei.Enable(evt);
	pei.Disable(evt);
	timer.DisableTimerTrigger(trigger);
	IOPinDisableTask(pei, evt, 0, 30);
	IOPinDisableEvent(pei, evt);
	return pei.ReleaseEvent(evt);
}

// Compile-only proof that RTC C++ event methods delegate to the C matrix.
bool PeiHalRtcCppApplication(Timer &timer, Pei &pei)
{
	PeiEvent_t event = 0U;

	if (!timer.EnableTimerEvent(pei, event, TIMER_EVT_COUNTER_OVR))
	{
		return false;
	}
	if (!timer.ConnectTask(pei, event, TIMER_TASK_TRIGOVRFLW))
	{
		timer.DisableTimerEvent(pei, event);
		return false;
	}
	if (!timer.DisableTimerEvent(pei, event))
	{
		return false;
	}
	return pei.ReleaseEvent(event);
}

/* Compile-only check: the shared source also accepts a Pei object. */
bool PeiHalPortCppApplication(Pei &pei)
{
	PeiEvent_t evt = 0U;

	if (!IOPinEnablePortEvent(pei, evt))
	{
		return false;
	}
	if (!IOPinDisablePortEvent(pei, evt))
	{
		return false;
	}
	return pei.ReleaseEvent(evt);
}

// Compile-only check of the software event wrappers.
bool PeiHalSwEventCppApplication(Timer &timer, Pei &pei)
{
	PeiEvent_t sw = 0U;

	if (!pei.BindSwEvent(sw))
	{
		return false;
	}
	if (!timer.ConnectTask(pei, sw, TIMER_TASK_START) ||
		pei.SwEventTask(sw) == 0U)
	{
		pei.ReleaseEvent(sw);
		return false;
	}
	pei.Enable(sw);
	pei.Trigger(sw);
	return pei.ReleaseEvent(sw);
}

// Compile-only C++ client: functions use the same C PEI matrix.
bool PeiSaadcCppApplication(Pei &pei, PeiEvent_t trigger)
{
	PeiEvent_t evt = 0U;

	if (!AdcNrf52PreparePeiSampling())
	{
		return false;
	}
	if (!AdcNrf52PeiBindEvent(pei, &evt, ADC_NRF52_PEI_EVENT_END))
	{
		return false;
	}
	if (!AdcNrf52PeiConnectTask(pei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiDisconnectTask(pei, trigger, ADC_NRF52_PEI_TASK_SAMPLE))
	{
		return false;
	}
	if (!AdcNrf52PeiUnbindEvent(pei, evt))
	{
		return false;
	}
	return pei.ReleaseEvent(evt);
}
