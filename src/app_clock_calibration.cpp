/**-------------------------------------------------------------------------
@file	app_clock_calibration.cpp

@brief	Queue a caller-owned timer's RC calibration request for AppRun.

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
-------------------------------------------------------------------------*/
#include "app_clock_calibration.h"
#include "app_evt_handler.h"
#include "coredev/system_core_clock.h"

#if defined(__GNUC__)
#pragma weak SystemHighFreqClockCalibrate
#pragma weak SystemLowFreqClockCalibrate
#endif

static void AppClockCalibrationWork(uint32_t Event, void *pContext)
{
	AppClockCalibration_t *pCal;
	bool success;

	(void)Event;
	pCal = (AppClockCalibration_t *)pContext;

	if (pCal == nullptr)
	{
		return;
	}
	if (!pCal->Active)
	{
		pCal->Queued = false;
		return;
	}
	success = false;

	if (pCal->HighFreq)
	{
		if (SystemHighFreqClockCalibrate != nullptr)
		{
			success = SystemHighFreqClockCalibrate();
		}
	}
	else if (SystemLowFreqClockCalibrate != nullptr)
	{
		success = SystemLowFreqClockCalibrate();
	}
	if (success)
	{
		pCal->Completed++;
	}
	else
	{
		pCal->Failed++;
	}
	pCal->Queued = false;
}

static void AppClockCalibrationTrigger(TimerDev_t *pTimer,
	int TriggerNo, void *pContext)
{
	AppClockCalibration_t *pCal;
	bool queued;

	pCal = (AppClockCalibration_t *)pContext;

	if (pCal == nullptr || !pCal->Active || pCal->Queued ||
		pCal->pTimer != pTimer || pCal->TriggerNo != TriggerNo)
	{
		return;
	}
	pCal->Queued = true;
	queued = AppEvtHandlerQue(0U, pCal, AppClockCalibrationWork);

	if (!queued)
	{
		pCal->Queued = false;
		pCal->QueueRefused++;
	}
}

bool AppClockCalibrationStart(AppClockCalibration_t *pCal,
	TimerDev_t *pTimer, int TriggerNo, uint32_t PeriodMs, bool HighFreq)
{
	uint64_t period;

	if (pCal == nullptr || pTimer == nullptr || PeriodMs == 0U ||
		pCal->Active || pCal->Queued ||
		pTimer->EnableTrigger == nullptr ||
		pTimer->DisableTrigger == nullptr)
	{
		return false;
	}
	pCal->pTimer = pTimer;
	pCal->TriggerNo = TriggerNo;
	pCal->HighFreq = HighFreq;
	pCal->Queued = false;
	pCal->Completed = 0U;
	pCal->Failed = 0U;
	pCal->QueueRefused = 0U;
	pCal->Active = true;

	period = nsTimerEnableTrigger(pTimer, TriggerNo,
		(uint64_t)PeriodMs * 1000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, AppClockCalibrationTrigger, pCal);

	if (period == 0U)
	{
		pCal->Active = false;
		return false;
	}
	return true;
}

void AppClockCalibrationStop(AppClockCalibration_t *pCal)
{
	if (pCal == nullptr || !pCal->Active)
	{
		return;
	}
	pCal->Active = false;

	TimerDisableTrigger(pCal->pTimer, pCal->TriggerNo);
}
