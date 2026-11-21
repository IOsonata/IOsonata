/**-------------------------------------------------------------------------
@file	idelay_calibration.h

@brief	Measure the existing ARM delay loops using the core cycle counter.

The delay implementations stay in idelay.h. Calibration measures those
same assembly loops on the running CPU rather than assuming cycles per
iteration based on a compiler build or a past test.

@license
MIT License
Copyright (c) 2026, I-SYST inc. All rights reserved.
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
#ifndef __IDELAY_CALIBRATION_H__
#define __IDELAY_CALIBRATION_H__

#include <stdbool.h>
#include <stdint.h>
#include "idelay.h"

#define ARM_DELAY_CAL_LOOPS	8192U
#define ARM_DELAY_CAL_NS	(ARM_DELAY_CAL_LOOPS * 1000U)

// CPU frequency is obtained by the caller from the core clock register.
// No Timer, RTC, SysTick, or application event queue is used here.
// Called during initialization or after CPU frequency changes, not in ISR.
// Returns false if the DWT cycle counter is unavailable or inaccessible.
static inline bool ArmDelayCalibrate(uint32_t CpuHz)
{
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_NOCYCCNT_Msk) && \
	defined(DWT_CTRL_CYCCNTENA_Msk) && defined(CoreDebug_DEMCR_TRCENA_Msk)
	uint32_t prevMask;
	uint32_t prevTrace;
	uint32_t prevCtrl;
	uint32_t start;
	uint32_t usCycles;
	uint32_t nsCycles;
	uint32_t prevUsFactor;
	uint32_t prevNsFactor;
	uint64_t scale;
	bool valid;

	if (CpuHz == 0U || (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
	{
		return false;
	}

	prevMask = __get_PRIMASK();
	__disable_irq();

	prevTrace = CoreDebug->DEMCR;
	prevCtrl = DWT->CTRL;
	prevUsFactor = SystemMicroSecLoopCnt;
	prevNsFactor = SystemnsDelayFactor;
	CoreDebug->DEMCR = prevTrace | CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL = prevCtrl | DWT_CTRL_CYCCNTENA_Msk;

	// A non-running cycle counter cannot provide a real calibration.
	start = DWT->CYCCNT;
	__asm__ volatile ("nop\n nop\n nop\n nop\n");
	valid = DWT->CYCCNT != start;

	if (valid)
	{
		// One loop per microsecond in the provisional fixed-point format.
		// These calls execute the real assembly loops from idelay.h.
		SystemMicroSecLoopCnt = 65536U;
		SystemnsDelayFactor = 65536U;

		usDelay(ARM_DELAY_CAL_LOOPS);
		start = DWT->CYCCNT;
		usDelay(ARM_DELAY_CAL_LOOPS);
		usCycles = DWT->CYCCNT - start;

		nsDelay(ARM_DELAY_CAL_NS);
		start = DWT->CYCCNT;
		nsDelay(ARM_DELAY_CAL_NS);
		nsCycles = DWT->CYCCNT - start;

		valid = usCycles != 0U && nsCycles != 0U;
	}

	if (valid)
	{
		// Scale actual measured iterations per CPU cycle to iterations
		// per microsecond, using the existing 16-bit fractional format.
		scale = (uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS * 65536ULL;
		SystemMicroSecLoopCnt = (uint32_t)(scale /
			((uint64_t)usCycles * 1000000ULL));
		SystemnsDelayFactor = (uint32_t)(scale /
			((uint64_t)nsCycles * 1000000ULL));
		valid = SystemMicroSecLoopCnt != 0U &&
			SystemnsDelayFactor != 0U;
	}

	if (!valid)
	{
		SystemMicroSecLoopCnt = prevUsFactor;
		SystemnsDelayFactor = prevNsFactor;
	}

	DWT->CTRL = prevCtrl;
	CoreDebug->DEMCR = prevTrace;
	__set_PRIMASK(prevMask);

	return valid;
#else
	(void)CpuHz;

	return false;
#endif
}

#endif
