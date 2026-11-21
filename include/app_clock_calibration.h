/**-------------------------------------------------------------------------
@file	app_clock_calibration.h

@brief	Deferred periodic RC calibration using a caller-owned Timer trigger.

The application owns the Timer device and the chosen trigger channel.
The timer interrupt queues a request; AppRun executes the calibration in
foreground context. System Core never depends on Timer or AppRun.

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
#ifndef __APP_CLOCK_CALIBRATION_H__
#define __APP_CLOCK_CALIBRATION_H__

#include "coredev/timer.h"

typedef struct {
	TimerDev_t *pTimer;
	int TriggerNo;
	bool HighFreq;
	volatile bool Active;
	volatile bool Queued;
	volatile uint32_t Completed;
	volatile uint32_t Failed;
	volatile uint32_t QueueRefused;
} AppClockCalibration_t;

#ifdef __cplusplus
extern "C" {
#endif

// The caller initializes and enables pTimer before starting.
// TriggerNo must belong to the caller and be currently unused.
// A 4-second period is suitable for nRF5340 LFRC periodic calibration.
// The structure must remain allocated while an event may be queued.
bool AppClockCalibrationStart(AppClockCalibration_t *pCal,
	TimerDev_t *pTimer, int TriggerNo, uint32_t PeriodMs, bool HighFreq);

// Stop only the calibration trigger. The caller still owns the Timer.
// A queued event is ignored if it is dispatched after Stop.
void AppClockCalibrationStop(AppClockCalibration_t *pCal);

#ifdef __cplusplus
}
#endif

#endif
