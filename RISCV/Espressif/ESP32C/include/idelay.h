/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay functions for Espressif ESP32-C series (bare-metal)

Implements usDelay / nsDelay / msDelay using the SYSTIMER Unit 0 hardware
counter directly.  No ESP-IDF dependency.

SYSTIMER hardware notes (ESP32-C3/C5/C6 TRM):
  - Base address: 0x60005000  (C5, C6, H2)
                  0x60008000  (C3 only — override by including this from
                               ESP32C3/include/ which redefines SYSTIMER_BASE)
  - Unit 0 counter: 52-bit, ticks at 16 MHz (62.5 ns per tick)
  - Reading sequence:
      1. Write 1 to SYSTIMER_UNIT0_OP_REG[30] (TIMER_UNIT0_UPDATE) to latch
      2. Poll SYSTIMER_UNIT0_OP_REG[29] (TIMER_UNIT0_VALUE_VALID) until set
      3. Read SYSTIMER_UNIT0_VALUE_LO then SYSTIMER_UNIT0_VALUE_HI

SystemInit() (system_esp32cX.c) must enable SYSTIMER before these
functions are called.  Reading before enable returns 0 — usDelay will
spin indefinitely, so always call SystemInit() first.

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

/** @addtogroup Utilities
  * @{
  */

/*---------------------------------------------------------------------------
 * SYSTIMER register map.
 * ESP32-C3 uses a different base (0x60008000); its idelay.h override
 * redefines SYSTIMER_BASE before including this header.
 *---------------------------------------------------------------------------*/
#ifndef SYSTIMER_BASE
#define SYSTIMER_BASE               0x60005000UL
#endif

#define SYSTIMER_CONF_REG_          (*(volatile uint32_t *)(SYSTIMER_BASE + 0x000U))
#define SYSTIMER_UNIT0_OP_REG_      (*(volatile uint32_t *)(SYSTIMER_BASE + 0x018U))
#define SYSTIMER_UNIT0_VALUE_LO_    (*(volatile uint32_t *)(SYSTIMER_BASE + 0x040U))
#define SYSTIMER_UNIT0_VALUE_HI_    (*(volatile uint32_t *)(SYSTIMER_BASE + 0x044U))

/* SYSTIMER_UNIT0_OP_REG bit fields */
#define SYSTIMER_UNIT0_UPDATE_      (1UL << 30)   /* write 1 to latch counter */
#define SYSTIMER_UNIT0_VALID_       (1UL << 29)   /* 1 when latched value is stable */

/*---------------------------------------------------------------------------
 * Internal: latch and return the 64-bit SYSTIMER Unit 0 count.
 * Counter ticks at 16 MHz → 1 tick = 62.5 ns.
 *---------------------------------------------------------------------------*/
static inline __attribute__((always_inline))
uint64_t _SysTimerRead(void)
{
	/* Trigger latch */
	SYSTIMER_UNIT0_OP_REG_ = SYSTIMER_UNIT0_UPDATE_;
	/* Wait for valid */
	while (!(SYSTIMER_UNIT0_OP_REG_ & SYSTIMER_UNIT0_VALID_))
	{
		__asm volatile("nop");
	}
	/* Read low word first (latched pair — order matters) */
	uint32_t lo = SYSTIMER_UNIT0_VALUE_LO_;
	uint32_t hi = SYSTIMER_UNIT0_VALUE_HI_;
	return ((uint64_t)hi << 32U) | lo;
}

/**
 * @brief	Microsecond delay.
 *
 * Spins until SYSTIMER Unit 0 has advanced by cnt × 16 ticks
 * (16 ticks = 1 µs at 16 MHz).  Handles 52-bit rollover correctly.
 *
 * @param	cnt : microsecond count
 */
static inline __attribute__((always_inline))
void usDelay(uint32_t cnt)
{
	if (cnt == 0U)
	{
		return;
	}
	/* 16 ticks per µs at 16 MHz */
	uint64_t start  = _SysTimerRead();
	uint64_t target = start + (uint64_t)cnt * 16UL;

	while (_SysTimerRead() < target)
	{
		__asm volatile("nop");
	}
}

/**
 * @brief	Millisecond delay.
 *
 * @param	ms : millisecond count
 */
static inline __attribute__((always_inline))
void msDelay(uint32_t ms)
{
	usDelay(ms * 1000UL);
}

/**
 * @brief	Nanosecond delay (approximate).
 *
 * Minimum granularity is 62.5 ns (one SYSTIMER tick at 16 MHz).
 * Counts of less than 63 ns will wait exactly one tick.
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline))
void nsDelay(uint32_t cnt)
{
	if (cnt == 0U)
	{
		return;
	}
	/* Convert ns → 16 MHz ticks: ticks = (cnt * 16) / 1000, minimum 1 */
	uint32_t ticks = (cnt * 16UL + 999UL) / 1000UL;
	if (ticks == 0U)
	{
		ticks = 1U;
	}
	uint64_t start  = _SysTimerRead();
	uint64_t target = start + ticks;

	while (_SysTimerRead() < target)
	{
		__asm volatile("nop");
	}
}

/** @} End of group Utilities */

#endif	/* __IDELAY_H__ */
