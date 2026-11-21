/**-------------------------------------------------------------------------
@file	idelay_clock.h

@brief	Rescale ARM delay factors after the CPU frequency changes.

This header has no DWT or ARM assembly dependencies. CPU clock owners use
arithmetic to refresh both delay factors when the effective CPU clock changes.

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
#ifndef __IDELAY_CLOCK_H__
#define __IDELAY_CLOCK_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemMicroSecLoopCnt;
extern uint32_t SystemnsDelayFactor;

#ifdef __cplusplus
}
#endif

#define ARM_DELAY_CAL_LOOPS	8192U

// Measured cycles for ARM_DELAY_CAL_LOOPS iterations. The fallback uses
// documented per-target assembly-loop cycles until measurement succeeds.
typedef struct {
	uint32_t UsCycles;
	uint32_t NsCycles;
	uint32_t CpuHz;
	bool Measured;
} ArmDelayCalibration_t;

// Recalculate BOTH delay factors after every actual CPU frequency change.
// Uses the cached loop cost; never starts a counter or disables interrupts.
// Returns false if no valid loop cost or CPU frequency is available.
static inline bool ArmDelayClockUpdate(ArmDelayCalibration_t *pCal, uint32_t CpuHz)
{
	uint64_t scale;
	uint64_t usFactor;
	uint64_t nsFactor;

	if (pCal == nullptr || CpuHz == 0U ||
		pCal->UsCycles == 0U || pCal->NsCycles == 0U)
	{
		return false;
	}

	scale = (uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS * 65536ULL;
	usFactor = scale / ((uint64_t)pCal->UsCycles * 1000000ULL);
	nsFactor = scale / ((uint64_t)pCal->NsCycles * 1000000ULL);

	if (usFactor == 0U || nsFactor == 0U ||
		usFactor > UINT32_MAX || nsFactor > UINT32_MAX)
	{
		return false;
	}

	SystemMicroSecLoopCnt = (uint32_t)usFactor;
	SystemnsDelayFactor = (uint32_t)nsFactor;
	pCal->CpuHz = CpuHz;

	return true;
}

// Per-target documented loop-cycle fallback. No DWT is required.
// Each clock owner invokes this once and calls ArmDelayClockUpdate whenever
// the CPU clock changes. Reinitialization clears an earlier measurement.
static inline bool ArmDelayClockInit(ArmDelayCalibration_t *pCal,
	uint32_t CpuHz, uint32_t UsLoopCycles, uint32_t NsLoopCycles)
{
	if (pCal == nullptr || UsLoopCycles == 0U || NsLoopCycles == 0U ||
		UsLoopCycles > UINT32_MAX / ARM_DELAY_CAL_LOOPS ||
		NsLoopCycles > UINT32_MAX / ARM_DELAY_CAL_LOOPS)
	{
		return false;
	}

	pCal->UsCycles = UsLoopCycles * ARM_DELAY_CAL_LOOPS;
	pCal->NsCycles = NsLoopCycles * ARM_DELAY_CAL_LOOPS;
	pCal->CpuHz = 0U;
	pCal->Measured = false;

	return ArmDelayClockUpdate(pCal, CpuHz);
}

// Generic non-DWT fallback using a target's existing loop divisors in
// cycles-per-iteration * 1000000. Preserve their previous calibration
// constants exactly, and call after every successful CPU frequency change.
// The function is pure clock-factor arithmetic: no calibration delay or
// higher-level timer dependency.
static inline bool ArmDelayClockSetDivisors(uint32_t CpuHz,
	uint64_t UsDivisor, uint64_t NsDivisor)
{
	uint64_t usFactor;
	uint64_t nsFactor;

	if (CpuHz == 0U || UsDivisor == 0U || NsDivisor == 0U)
	{
		return false;
	}

	usFactor = ((uint64_t)CpuHz << 16) / UsDivisor;
	nsFactor = ((uint64_t)CpuHz << 16) / NsDivisor;

	if (usFactor == 0U || nsFactor == 0U ||
		usFactor > UINT32_MAX || nsFactor > UINT32_MAX)
	{
		return false;
	}

	SystemMicroSecLoopCnt = (uint32_t)usFactor;
	SystemnsDelayFactor = (uint32_t)nsFactor;

	return true;
}


#endif // __IDELAY_CLOCK_H__
