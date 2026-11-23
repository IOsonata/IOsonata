/**-------------------------------------------------------------------------
@file	system_core_clock_calibration.h

@brief	ARM System Core delay-factor calculation and optional calibration.

Calibration uses CPU-local DWT when available, or an HF hardware Timer
measurement supplied by CoreClockTest. Runtime delays never read DWT.
This header contains no vendor or MCU-specific conditional behavior.

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __ARM_SYSTEM_CORE_CLOCK_CALIBRATION_H__
#define __ARM_SYSTEM_CORE_CLOCK_CALIBRATION_H__

#include "idelay.h"

typedef struct {
	uint32_t CpuHz;
	uint32_t UsLoopsPerSecond;
	uint32_t NsLoopsPerSecond;
	bool Measured;
} DelayCalibrationPoint_t;

// DWT is available only to the explicit calibration probe, never runtime delay.
#if defined(DWT) && defined(CoreDebug) && \
	defined(DWT_CTRL_CYCCNTENA_Msk) && \
	defined(CoreDebug_DEMCR_TRCENA_Msk) && \
	(!defined(__DWT_PRESENT) || (__DWT_PRESENT == 1U))
#define ARM_DELAY_CAL_DWT_AVAILABLE 1
#else
#define ARM_DELAY_CAL_DWT_AVAILABLE 0
#endif

#define ARM_DELAY_CAL_LOOPS	8192U

// Optional MCU-owned DWT gate. On SoCs such as nRF54L, the trace/debug
// peripheral must be enabled temporarily before CYCCNT can advance.
// These weak hooks are no-ops when an MCU does not implement them.
// Calibrating never enables trace pins or changes application clock sources.
#if defined(__GNUC__)
#ifdef __cplusplus
extern "C" {
#endif
bool SystemDelayDwtPrepare(void) __attribute__((weak));
void SystemDelayDwtRestore(void) __attribute__((weak));
#ifdef __cplusplus
}
#endif
#endif

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
	uint64_t usFactor;
	uint64_t nsFactor;

	if (pCal == NULL || CpuHz == 0U ||
		pCal->UsCycles == 0U || pCal->NsCycles == 0U)
	{
		return false;
	}

	usFactor = ((uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS) / pCal->UsCycles;
	nsFactor = ((uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS) / pCal->NsCycles;

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

// Per-target documented loop-cycle fallback.
// Each clock owner invokes this once and calls ArmDelayClockUpdate whenever
// the CPU clock changes. Reinitialization clears an earlier measurement.
static inline bool ArmDelayClockInit(ArmDelayCalibration_t *pCal,
	uint32_t CpuHz, uint32_t UsLoopCycles, uint32_t NsLoopCycles)
{
	if (pCal == NULL || UsLoopCycles == 0U || NsLoopCycles == 0U ||
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

// A port with two measured CPU speeds can interpolate both delay factors.
// Factors are loop iterations per second measured at actual CPU frequencies, not the
// oscillator's input frequency. Clamp to an endpoint outside the measured
// range: extrapolation is not supported. No runtime measurement is done.
typedef struct {
	uint32_t CpuHz;
	uint32_t UsLoopsPerSecond;
	uint32_t NsLoopsPerSecond;
} ArmDelayPoint_t;

static inline bool ArmDelayClockInterpolate(uint32_t CpuHz,
	const ArmDelayPoint_t *pLow, const ArmDelayPoint_t *pHigh)
{
	uint32_t span;
	uint32_t lowWeight;
	uint32_t highWeight;
	uint64_t us;
	uint64_t ns;

	if (pLow == NULL || pHigh == NULL || CpuHz == 0U ||
		pLow->CpuHz == 0U || pHigh->CpuHz <= pLow->CpuHz ||
		pLow->UsLoopsPerSecond == 0U || pLow->NsLoopsPerSecond == 0U ||
		pHigh->UsLoopsPerSecond == 0U || pHigh->NsLoopsPerSecond == 0U)
	{
		return false;
	}

	if (CpuHz <= pLow->CpuHz)
	{
		SystemMicroSecLoopCnt = pLow->UsLoopsPerSecond;
		SystemnsDelayFactor = pLow->NsLoopsPerSecond;

		return true;
	}

	if (CpuHz >= pHigh->CpuHz)
	{
		SystemMicroSecLoopCnt = pHigh->UsLoopsPerSecond;
		SystemnsDelayFactor = pHigh->NsLoopsPerSecond;

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

	SystemMicroSecLoopCnt = (uint32_t)us;
	SystemnsDelayFactor = (uint32_t)ns;

	return true;
}

// Generic factor calculation using a target's existing loop divisors in
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

	usFactor = ((uint64_t)CpuHz * 1000000ULL) / UsDivisor;
	nsFactor = ((uint64_t)CpuHz * 1000000ULL) / NsDivisor;

	if (usFactor == 0U || nsFactor == 0U ||
		usFactor > UINT32_MAX || nsFactor > UINT32_MAX)
	{
		return false;
	}

	SystemMicroSecLoopCnt = (uint32_t)usFactor;
	SystemnsDelayFactor = (uint32_t)nsFactor;

	return true;
}

#define ARM_DELAY_CAL_NS	(ARM_DELAY_CAL_LOOPS * 1000U)

// Optional CPU-cycle measurement of the assembly fallback, not DWT waits.
// This is an explicit foreground initialization operation, not a clock
// change callback. The measured costs are cached; later frequency changes
// only call ArmDelayClockUpdate without timing loops or interrupt masking.
// Returns false on MCUs without an accessible DWT cycle counter; in that
// case, existing per-target fallback factors remain in effect.
static inline bool ArmDelayCalibrate(ArmDelayCalibration_t *pCal, uint32_t CpuHz)
{
#if ARM_DELAY_CAL_DWT_AVAILABLE
	uint32_t prevMask;
	uint32_t prevTrace;
	uint32_t prevCtrl;
	uint32_t prevUsFactor;
	uint32_t prevNsFactor;
	uint32_t start;
	uint32_t usCycles;
	uint32_t nsCycles;
	uint32_t i;
	bool valid;

	if (pCal == NULL || CpuHz == 0U || __get_IPSR() != 0U)
	{
		return false;
	}
	prevMask = __get_PRIMASK();
	__disable_irq();
	prevTrace = CoreDebug->DEMCR;
	prevUsFactor = SystemMicroSecLoopCnt;
	prevNsFactor = SystemnsDelayFactor;
	usCycles = 0U;
	nsCycles = 0U;
	valid = false;

	CoreDebug->DEMCR = prevTrace | CoreDebug_DEMCR_TRCENA_Msk;
	__DSB();
	__ISB();
	prevCtrl = DWT->CTRL;
#if defined(DWT_CTRL_NOCYCCNT_Msk)
	if ((prevCtrl & DWT_CTRL_NOCYCCNT_Msk) == 0U)
#endif
	{
		DWT->CTRL = prevCtrl | DWT_CTRL_CYCCNTENA_Msk;
		__DSB();
		__ISB();
		start = DWT->CYCCNT;
		for (i = 0U; i < 64U; i++)
		{
			__NOP();
		}
		valid = DWT->CYCCNT != start;
		if (valid)
		{
			SystemMicroSecLoopCnt = 1000000U;
			SystemnsDelayFactor = 1000000U;

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
	}
	SystemMicroSecLoopCnt = prevUsFactor;
	SystemnsDelayFactor = prevNsFactor;
	DWT->CTRL = prevCtrl;
	CoreDebug->DEMCR = prevTrace;
	__DSB();
	__ISB();
	__set_PRIMASK(prevMask);

	if (valid)
	{
		pCal->UsCycles = usCycles;
		pCal->NsCycles = nsCycles;
		pCal->Measured = true;
		valid = ArmDelayClockUpdate(pCal, CpuHz);
	}
	return valid;
#else
	(void)pCal;
	(void)CpuHz;
	return false;
#endif
}

/**
 * @brief Measure delay factors with DWT, leaving runtime factors unchanged.
 *
 * The application calls this explicitly from foreground context. Targets
 * without DWT return false and may use the generic high-frequency timer test.
 */
static inline bool DelayCalibrationMeasure(DelayCalibrationPoint_t *pPoint)
{
	ArmDelayCalibration_t measured;
	uint32_t oldUs;
	uint32_t oldNs;
	uint32_t cpuHz;
	bool success;

	if (pPoint == NULL)
	{
		return false;
	}
	cpuHz = SystemCoreClockGet();
	pPoint->CpuHz = cpuHz;
	pPoint->UsLoopsPerSecond = 0U;
	pPoint->NsLoopsPerSecond = 0U;
	pPoint->Measured = false;

	oldUs = SystemMicroSecLoopCnt;
	oldNs = SystemnsDelayFactor;
	measured.UsCycles = 0U;
	measured.NsCycles = 0U;
	measured.CpuHz = 0U;
	measured.Measured = false;
#if defined(__GNUC__) && ARM_DELAY_CAL_DWT_AVAILABLE
	bool prepared;

	prepared = SystemDelayDwtPrepare == NULL ||
		SystemDelayDwtPrepare();
	success = prepared && ArmDelayCalibrate(&measured, cpuHz);
	if (prepared && SystemDelayDwtRestore != NULL)
	{
		SystemDelayDwtRestore();
	}
#else
	success = ArmDelayCalibrate(&measured, cpuHz);
#endif
	if (success)
	{
		pPoint->UsLoopsPerSecond = SystemMicroSecLoopCnt;
		pPoint->NsLoopsPerSecond = SystemnsDelayFactor;
		pPoint->Measured = true;
	}
	SystemMicroSecLoopCnt = oldUs;
	SystemnsDelayFactor = oldNs;
	return success;
}

// The high-frequency Timer reports recommended default factors.
// Measuring does not replace production factors.
static inline bool DelayCalibrationFromTimer(uint32_t CpuHz,
	uint32_t UsElapsed, uint32_t NsElapsed,
	DelayCalibrationPoint_t *pPoint)
{
	uint64_t us;
	uint64_t ns;

	if (pPoint == NULL || CpuHz == 0U ||
		UsElapsed == 0U || NsElapsed == 0U)
	{
		return false;
	}
	us = (uint64_t)SystemMicroSecLoopCnt * 100000U / UsElapsed;
	ns = (uint64_t)SystemnsDelayFactor * 100000U / NsElapsed;
	if (us == 0U || ns == 0U || us > UINT32_MAX || ns > UINT32_MAX)
	{
		return false;
	}
	pPoint->CpuHz = CpuHz;
	pPoint->UsLoopsPerSecond = (uint32_t)us;
	pPoint->NsLoopsPerSecond = (uint32_t)ns;
	pPoint->Measured = true;
	return true;
}


#endif // __ARM_SYSTEM_CORE_CLOCK_CALIBRATION_H__
