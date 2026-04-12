/**-------------------------------------------------------------------------
@file	systick.h

@brief	ARM SysTick utility functions

Land-layer primitives for the ARM Cortex-M SysTick timer.
Register layout is identical on all Cortex-M variants (ARM DDI 0403E §B3.3):

  0xE000E010  CTRL   control & status
  0xE000E014  LOAD   24-bit reload value  (counter wraps LOAD → 0 → LOAD)
  0xE000E018  VAL    current down-count   (write any value resets to 0)
  0xE000E01C  CALIB  calibration (optional, rarely used)

CTRL bit assignments:
  [16]  COUNTFLAG  set when counter reaches 0; cleared by reading CTRL or VAL
  [2]   CLKSOURCE  1 = processor clock (always use 1 for predictable timing)
  [1]   TICKINT    1 = assert SysTick exception on each wrap
  [0]   ENABLE     1 = counter running

Tick frequency:  f_tick = CoreClk / (LOAD + 1)
Maximum period:  LOAD ≤ 0x00FFFFFF  (24-bit)

These are pure static-inline primitives — no state, no function pointers.
The Roots-layer MCU port (timer_systick.cpp) builds TimerDev_t on top.

@author	Hoang Nguyen Hoan
@date	Feb. 27, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_compiler.h"

// ── SysTick Clock Source Select ────────────────────────────────────────────────────
#define SYSTICK_CLOCK_SRC_EXT	  0			  ///< External clock source
#define SYSTICK_CLOCK_SRC_MCU	  1			  ///< Processor clock source

// ── SysTick CTRL bit masks ────────────────────────────────────────────────────
#define SYSTICK_CTRL_ENABLE       (1UL << 0)  ///< Counter enable
#define SYSTICK_CTRL_TICKINT      (1UL << 1)  ///< Exception enable on wrap
#define SYSTICK_CTRL_CLKSOURCE    (1UL << 2)  ///< 1 = processor clock
#define SYSTICK_CTRL_COUNTFLAG    (1UL << 16) ///< Set on wrap; clears on CTRL read

// ── SysTick register base ─────────────────────────────────────────────────────
#define SYSTICK_BASE              0xE000E010UL

// ── SCB ICSR (for SysTick pending bit) ───────────────────────────────────────
#define SCB_ICSR_ADDR             0xE000ED04UL
#define SCB_ICSR_PENDSTSET        (1UL << 26)  ///< SysTick pending set
#define SCB_ICSR_PENDSTCLR        (1UL << 25)  ///< SysTick pending clear

// ── SCB SHPR3 (SysTick exception priority in bits [31:24]) ───────────────────
#define SCB_SHPR3_ADDR            0xE000ED20UL

#ifdef __cplusplus
extern "C" {
#endif

// ── Register accessors ────────────────────────────────────────────────────────

/**
 * @brief  Start the SysTick counter (processor clock source).
 */
__STATIC_FORCEINLINE void SysTickStart(void)
{
    *((volatile uint32_t*)SYSTICK_BASE) |=
        SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_ENABLE;
    __DSB();
}

/**
 * @brief  Stop the SysTick counter.
 */
__STATIC_FORCEINLINE void SysTickStop(void)
{
    *((volatile uint32_t*)SYSTICK_BASE) &= ~SYSTICK_CTRL_ENABLE;
    __DSB();
}

/**
 * @brief  Enable the SysTick wrap interrupt (TICKINT).
 */
__STATIC_FORCEINLINE void SysTickEnableInt(void)
{
    *((volatile uint32_t*)SYSTICK_BASE) |= SYSTICK_CTRL_TICKINT;
}

/**
 * @brief  Disable the SysTick wrap interrupt.
 */
__STATIC_FORCEINLINE void SysTickDisableInt(void)
{
    *((volatile uint32_t*)SYSTICK_BASE) &= ~SYSTICK_CTRL_TICKINT;
}

/**
 * @brief  Set SysTick reload value and reset the current counter.
 *
 * @param  Load  24-bit reload value (0 < Load ≤ 0x00FFFFFF).
 *               Counter fires every (Load + 1) clock cycles.
 */
__STATIC_FORCEINLINE void SysTickSetReload(uint32_t Load)
{
    *((volatile uint32_t*)(SYSTICK_BASE + 4UL)) = Load & 0x00FFFFFFul;
    *((volatile uint32_t*)(SYSTICK_BASE + 8UL)) = 0UL;  // reset VAL
}

/**
 * @brief  Configure SysTick for a given tick frequency.
 *
 * Computes LOAD = CoreClkHz / TickHz - 1 and applies it.
 * Stops and restarts the counter around the reload write.
 *
 * @param  ClkSrc	1 - MCU Core clock, 0 - external source
 * @param  ClkHz  	Clock in Hz (e.g. SystemCoreClock).
 * @param  TickHz   Desired interrupt rate in Hz (e.g. 1000 for 1 ms tick).
 *
 * @return Achieved tick frequency in Hz (may differ from TickHz due to
 *         integer division). Returns 0 if TickHz is 0 or LOAD overflows.
 */
__STATIC_FORCEINLINE uint32_t SysTickSetFrequency(uint32_t ClkSrc, uint32_t ClkHz, uint32_t TickHz)
{
    if (TickHz == 0UL) {
        return 0UL;
    }

    uint32_t load = (ClkHz / TickHz) - 1UL;

    if (load > 0x00FFFFFFul) {
        return 0UL;  // requested frequency too low for 24-bit counter
    }

    volatile uint32_t *ctrl = (volatile uint32_t*)SYSTICK_BASE;
    bool was_enabled = (*ctrl & SYSTICK_CTRL_ENABLE) != 0UL;

    // Stop, update, clear, restart
    *ctrl &= ~SYSTICK_CTRL_ENABLE;
    *((volatile uint32_t*)(SYSTICK_BASE + 4UL)) = load;
    *((volatile uint32_t*)(SYSTICK_BASE + 8UL)) = 0UL;

    if (was_enabled) {
    	if (ClkSrc == SYSTICK_CLOCK_SRC_MCU)
        {
    		*ctrl |= SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_ENABLE;
        }
    	else
    	{
    		*ctrl = (*ctrl & ~SYSTICK_CTRL_CLKSOURCE) | SYSTICK_CTRL_ENABLE;
    	}
    }

    return ClkHz / (load + 1UL);
}

/**
 * @brief  Read the current SysTick down-counter value.
 *
 * @return 24-bit current count (LOAD → 0).
 */
__STATIC_FORCEINLINE uint32_t SysTickGetVal(void)
{
    return *((volatile uint32_t*)(SYSTICK_BASE + 8UL)) & 0x00FFFFFFul;
}

/**
 * @brief  Read the SysTick reload (LOAD) register.
 */
__STATIC_FORCEINLINE uint32_t SysTickGetLoad(void)
{
    return *((volatile uint32_t*)(SYSTICK_BASE + 4UL)) & 0x00FFFFFFul;
}

/**
 * @brief  Read-and-clear COUNTFLAG.
 *
 * COUNTFLAG is set when the counter reaches 0. Reading CTRL clears it.
 * Returns true if the counter wrapped since the last call.
 */
__STATIC_FORCEINLINE bool SysTickGetCountFlag(void)
{
    return (*((volatile uint32_t*)SYSTICK_BASE) & SYSTICK_CTRL_COUNTFLAG) != 0UL;
}

/**
 * @brief  Read CTRL without clearing COUNTFLAG via a raw peek.
 *
 * Standard CTRL reads clear COUNTFLAG. This reads the pending bit in SCB
 * ICSR instead, which is equivalent but non-destructive.
 *
 * @return true if SysTick exception is pending (counter wrapped).
 */
__STATIC_FORCEINLINE bool SysTickIsPending(void)
{
    return (*((volatile uint32_t*)SCB_ICSR_ADDR) & SCB_ICSR_PENDSTSET) != 0UL;
}

/**
 * @brief  Set the SysTick exception priority.
 *
 * SysTick priority is in SHPR3 bits [31:24].
 * 0x00 = highest, 0xFF = lowest. Recommend 0xFF for an RTOS tick.
 *
 * @param  Prio  8-bit priority value.
 */
__STATIC_FORCEINLINE void SysTickSetPriority(uint8_t Prio)
{
    volatile uint32_t *shpr3 = (volatile uint32_t*)SCB_SHPR3_ADDR;
    *shpr3 = (*shpr3 & 0x00FFFFFFul) | ((uint32_t)Prio << 24);
    __DSB();
    __ISB();
}

/**
 * @brief  Return true if the SysTick counter is running.
 */
__STATIC_FORCEINLINE bool SysTickIsRunning(void)
{
    return (*((volatile uint32_t*)SYSTICK_BASE) & SYSTICK_CTRL_ENABLE) != 0UL;
}

#ifdef __cplusplus
}
#endif

#endif // __SYSTICK_H__
