/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

Delay-loop calibration is maintained by the MCU clock implementation.
Both delay factors use 16 fractional bits. Clock owners rescale the factors
from actual CPU frequency after every clock change. Optional DWT calibration
measures the existing assembly loops once; no Timer dependency is required.

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

#ifdef __cplusplus
extern "C" {
#endif
// Iterations per microsecond for each loop, with 16 fractional bits.
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

	if (pCal == NULL || CpuHz == 0U ||
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
// Factors are Q16 values measured at the actual CPU frequencies, not the
// oscillator's input frequency. Clamp to an endpoint outside the measured
// range: extrapolation is not supported. No runtime measurement is done.
typedef struct {
	uint32_t CpuHz;
	uint32_t UsQ16;
	uint32_t NsQ16;
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
		pLow->UsQ16 == 0U || pLow->NsQ16 == 0U ||
		pHigh->UsQ16 == 0U || pHigh->NsQ16 == 0U)
	{
		return false;
	}

	if (CpuHz <= pLow->CpuHz)
	{
		SystemMicroSecLoopCnt = pLow->UsQ16;
		SystemnsDelayFactor = pLow->NsQ16;

		return true;
	}

	if (CpuHz >= pHigh->CpuHz)
	{
		SystemMicroSecLoopCnt = pHigh->UsQ16;
		SystemnsDelayFactor = pHigh->NsQ16;

		return true;
	}

	span = pHigh->CpuHz - pLow->CpuHz;
	lowWeight = pHigh->CpuHz - CpuHz;
	highWeight = CpuHz - pLow->CpuHz;

	us = ((uint64_t)pLow->UsQ16 * lowWeight +
		(uint64_t)pHigh->UsQ16 * highWeight + span / 2U) / span;
	ns = ((uint64_t)pLow->NsQ16 * lowWeight +
		(uint64_t)pHigh->NsQ16 * highWeight + span / 2U) / span;

	if (us == 0U || ns == 0U || us > UINT32_MAX || ns > UINT32_MAX)
	{
		return false;
	}

	SystemMicroSecLoopCnt = (uint32_t)us;
	SystemnsDelayFactor = (uint32_t)ns;

	return true;
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

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 * SystemCoreClockUpdate refreshes SystemMicroSecLoopCnt after a CPU clock
 * change. Scale the full interval before removing the fractional bits.
 * Long intervals retain the full calculated loop count.
 * Interrupt service and instruction fetch stalls can extend this delay.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt)
{
	uint64_t loops = ((uint64_t)cnt * SystemMicroSecLoopCnt + 65535ULL) >> 16;
	uint32_t low = (uint32_t)loops;
	uint32_t high = (uint32_t)(loops >> 32);

	// Run the low word first. Each remaining high count adds 2^32 iterations.
	// Subtracting from zero clears carry and ends the outer loop.
	__asm__ volatile (
#ifdef __GNUC__
		".syntax unified\n"
#endif
		" CMP %[low], #0\n"
		" BEQ 2f\n"
		// Keep loop placement independent of surrounding code and optimization.
		".balign 8\n"
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
		" SUBS %[low], #1\n"
		" BNE 1b\n"
		"2:\n"
		" SUBS %[high], #1\n"
		" BCS 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		: [low] "+&l" (low), [high] "+&l" (high)
		:
		: "cc"
	);
}

/**
 * @brief	Nanosecond delay.
 *
 * SystemnsDelayFactor uses the same fractional format as the microsecond
 * loop count and is refreshed by the MCU clock implementation.
 * Calculation overhead, instruction fetch and interrupts extend the delay.
 * Use a hardware timer when precise timing is required.
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)
{
	uint64_t loops = ((uint64_t)cnt * SystemnsDelayFactor + 65535999ULL) / 65536000ULL;
	uint32_t low = (uint32_t)loops;
	uint32_t high = (uint32_t)(loops >> 32);

	// Run the low word first. Each remaining high count adds 2^32 iterations.
	// Subtracting from zero clears carry and ends the outer loop.
	__asm__ volatile (
#ifdef __GNUC__
		".syntax unified\n"
#endif
		" CMP %[low], #0\n"
		" BEQ 2f\n"
		// Keep loop placement independent of surrounding code and optimization.
		".balign 8\n"
		"1:\n"
		" SUBS %[low], #1\n"
		" BNE 1b\n"
		"2:\n"
		" SUBS %[high], #1\n"
		" BCS 1b\n"
#if defined ( __ARMCC_VERSION )
#elif defined ( __GNUC__ )
		".syntax divided\n"
#endif
		: [low] "+&l" (low), [high] "+&l" (high)
		:
		: "cc"
	);
}

#define ARM_DELAY_CAL_NS	(ARM_DELAY_CAL_LOOPS * 1000U)

// Optional CPU-cycle measurement of the EXACT assembly loops in idelay.h.
// This is an explicit foreground initialization operation, not a clock
// change callback. The measured costs are cached; later frequency changes
// only call ArmDelayClockUpdate without timing loops or interrupt masking.
// Returns false on MCUs without an accessible DWT cycle counter; in that
// case, existing per-target fallback factors remain in effect.
static inline bool ArmDelayCalibrate(ArmDelayCalibration_t *pCal, uint32_t CpuHz)
{
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_NOCYCCNT_Msk) && \
	defined(DWT_CTRL_CYCCNTENA_Msk) && defined(CoreDebug_DEMCR_TRCENA_Msk)
	uint32_t prevMask;
	uint32_t prevTrace;
	uint32_t prevCtrl;
	uint32_t prevUsFactor;
	uint32_t prevNsFactor;
	uint32_t start;
	uint32_t usCycles;
	uint32_t nsCycles;
	bool valid;

	if (pCal == NULL || CpuHz == 0U || __get_IPSR() != 0U ||
		(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
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

	start = DWT->CYCCNT;
	__asm__ volatile ("nop\n nop\n nop\n nop\n");
	valid = DWT->CYCCNT != start;
	usCycles = 0U;
	nsCycles = 0U;

	if (valid)
	{
		// A provisional factor of 65536 executes exactly N iterations.
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

	SystemMicroSecLoopCnt = prevUsFactor;
	SystemnsDelayFactor = prevNsFactor;
	DWT->CTRL = prevCtrl;
	CoreDebug->DEMCR = prevTrace;
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

