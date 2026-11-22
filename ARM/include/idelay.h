/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Runtime delays use calibrated assembly loops without invoking DWT, Timer,
or RTC. DWT is available only for explicit calibration-factor measurement.

@author Hoang Nguyen Hoan
@date	May 22, 2015

@license

MIT License

Copyright (c) 2011 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __IDELAY_H__
#define __IDELAY_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "coredev/system_core_clock.h"
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

#ifdef __cplusplus
extern "C" {
#endif
// Core frequency is maintained by each MCU System Core implementation.
extern uint32_t SystemCoreClock;
// Calibrated loop iterations per second for each existing assembly loop.
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

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 * SystemCoreClockUpdate refreshes the calibrated iterations per second
 * after a CPU clock change.
 * Long intervals retain the full calculated loop count.
 * Interrupt service and instruction fetch stalls can extend this delay.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt)
{
	uint64_t loops;
	uint32_t chunk;
	uint32_t remaining;

	loops = ((uint64_t)cnt * SystemMicroSecLoopCnt + 999999ULL) /
		1000000ULL;

	// Keep the original ten-NOP signed countdown inside usDelay.
	// Each chunk is bounded because BGT compares a signed counter.
	while (loops != 0U)
	{
		chunk = loops > INT32_MAX ? INT32_MAX : (uint32_t)loops;
		__asm__ volatile (
#ifdef __GNUC__
			".syntax unified\n"
#endif
			" MOVS %[remaining], %[count]\n"
			"1:\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" NOP\n"
			" SUBS %[remaining], #1\n"
			" BGT 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
			".syntax divided\n"
#endif
			: [remaining] "=&l" (remaining)
			: [count] "l" (chunk)
			: "cc"
		);
		loops -= chunk;
	}
}

/**
 * @brief	Nanosecond delay.
 *
 * SystemnsDelayFactor is the calibrated loop rate in iterations per second.
 * Keep the original signed countdown in this public function. Long delays
 * are split into signed-counter-sized chunks before entering assembly.
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)
{
	uint64_t loops;
	uint32_t chunk;
	uint32_t remaining;

	loops = ((uint64_t)cnt * SystemnsDelayFactor + 999999999ULL) /
		1000000000ULL;

	while (loops != 0U)
	{
		chunk = loops > INT32_MAX ? INT32_MAX : (uint32_t)loops;
		__asm__ volatile (
#ifdef __GNUC__
			".syntax unified\n"
#endif
			" MOVS %[remaining], %[count]\n"
			// Align the inlined countdown to a flash fetch boundary.
			" .p2align 5\n"
			"1:\n"
			" SUBS %[remaining], #1\n"
			" BGT 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
			".syntax divided\n"
#endif
			: [remaining] "=&l" (remaining)
			: [count] "l" (chunk)
			: "cc"
		);
		loops -= chunk;
	}
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
	success = ArmDelayCalibrate(&measured, cpuHz);
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

/**
 * @brief	Millisecond delay.
 * @param	ms : Milliseconds to wait.
 */
static inline void msDelay(uint32_t ms)
{
	// Split before converting milliseconds to the 32-bit microsecond count.
	while (ms > UINT32_MAX / 1000U)
	{
		usDelay((UINT32_MAX / 1000U) * 1000U);
		ms -= UINT32_MAX / 1000U;
	}
	usDelay(ms * 1000U);
}

/** @} End of group Utilities */

#endif

