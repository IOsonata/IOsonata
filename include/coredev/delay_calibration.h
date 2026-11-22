/**-------------------------------------------------------------------------
@file	delay_calibration.h

@brief	Architecture-neutral delay calibration data and interpolation.

Calibration values describe the delay implementation's measured loop rates
in iterations per second. The meaning of those units belongs to the architecture's
idelay.h. The generic CoreClockTest never changes active delay factors.

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved.
-------------------------------------------------------------------------*/
#ifndef __DELAY_CALIBRATION_H__
#define __DELAY_CALIBRATION_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
	uint32_t CpuHz;
	uint32_t UsLoopsPerSecond;
	uint32_t NsLoopsPerSecond;
	bool Measured;
} DelayCalibrationPoint_t;

// Interpolation is arithmetic only; the MCU's clock owner applies factors.
// A fixed-frequency MCU can use one measured endpoint.
static inline bool DelayCalibrationInterpolate(uint32_t CpuHz,
	const DelayCalibrationPoint_t *pLow,
	const DelayCalibrationPoint_t *pHigh,
	DelayCalibrationPoint_t *pResult)
{
	uint32_t span;
	uint32_t lowWeight;
	uint32_t highWeight;
	uint64_t us;
	uint64_t ns;

	if (pLow == NULL || pHigh == NULL || pResult == NULL || CpuHz == 0U ||
		!pLow->Measured || !pHigh->Measured ||
		pLow->CpuHz == 0U || pHigh->CpuHz < pLow->CpuHz ||
		pLow->UsLoopsPerSecond == 0U || pLow->NsLoopsPerSecond == 0U ||
		pHigh->UsLoopsPerSecond == 0U || pHigh->NsLoopsPerSecond == 0U)
	{
		return false;
	}

	*pResult = *pLow;
	pResult->CpuHz = CpuHz;
	if (pLow->CpuHz == pHigh->CpuHz || CpuHz <= pLow->CpuHz)
	{
		return true;
	}
	if (CpuHz >= pHigh->CpuHz)
	{
		pResult->UsLoopsPerSecond = pHigh->UsLoopsPerSecond;
		pResult->NsLoopsPerSecond = pHigh->NsLoopsPerSecond;
		return true;
	}

	span = pHigh->CpuHz - pLow->CpuHz;
	lowWeight = pHigh->CpuHz - CpuHz;
	highWeight = CpuHz - pLow->CpuHz;
	us = ((uint64_t)pLow->UsLoopsPerSecond * lowWeight +
		(uint64_t)pHigh->UsLoopsPerSecond * highWeight + span / 2U) / span;
	ns = ((uint64_t)pLow->NsLoopsPerSecond * lowWeight +
		(uint64_t)pHigh->NsLoopsPerSecond * highWeight + span / 2U) / span;

	if (us == 0U || ns == 0U || us > UINT32_MAX || ns > UINT32_MAX)
	{
		return false;
	}
	pResult->UsLoopsPerSecond = (uint32_t)us;
	pResult->NsLoopsPerSecond = (uint32_t)ns;
	return true;
}

#endif
