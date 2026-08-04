/**-------------------------------------------------------------------------
@file	system_esp32_pcr.c

@brief	Shared system initialisation for Espressif RISC-V chips with PCR

Covers all Espressif RISC-V targets that use the PCR (Peripheral Clock /
Reset) controller:

    ESP32-C5, ESP32-C6, ESP32-H2, ESP32-H4

Provides:
  - Register definitions for PCR, SYSTIMER, TIMG WDTs, and LP_WDT
  - IOsonata global clock state (SystemCoreClock, g_McuOsc)
  - Internal helpers for watchdog disable and SYSTIMER enable
  - Esp32PcrSystemInit() — shared boot sequence called by chip-specific
    SystemInit() before installing the vector table in mtvec
  - SystemCoreClockUpdate() and all system_core_clock.h stubs

NOT included:
  - SystemInit() itself — chip-specific because mtvec installation
    requires a chip-specific vector table symbol
  - CPU frequency divider values — differ per chip (C6: max 160 MHz
    from 480 MHz SPLL; H2: max 96 MHz from 96 MHz PLL on 32 MHz XTAL)

Chip-specific system_espXXXX.c files define SystemInit() like this:

    #define ESP32_PLL_FREQ  480000000UL   // chip-specific
    void SystemInit(void)
    {
        Esp32PcrSystemInit(160000000UL);  // desired CPU Hz, or 0 to skip
        extern void (* const g_EspXXXXVectors[])(void);
        uintptr_t vt = (uintptr_t)g_EspXXXXVectors;
        __asm volatile("csrw mtvec, %0" : : "r"(vt | 0x3U) : "memory");
    }

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

@license

MIT

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"

/*---------------------------------------------------------------------------
 * PCR — Peripheral Clock / Reset controller
 * Base address identical across C5, C6, H2, H4.
 *---------------------------------------------------------------------------*/
#define PCR_BASE                    0x60096000UL
#define PCR_SYSCLK_CONF_REG         (*(volatile uint32_t *)(PCR_BASE + 0x000U))
#define PCR_CPU_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x004U))
#define PCR_SYSTIMER_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x064U))

#define PCR_SOC_CLK_SEL_Pos         (20U)
#define PCR_SOC_CLK_SEL_Msk         (0x3UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_XTAL            (0x0UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_PLL             (0x1UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_FOSC            (0x2UL << PCR_SOC_CLK_SEL_Pos)

/* CPU clock divider: field encodes (divisor - 1) */
#define PCR_CPU_HS_DIV_NUM_Pos      (8U)
#define PCR_CPU_HS_DIV_NUM_Msk      (0xFFUL << PCR_CPU_HS_DIV_NUM_Pos)

#define PCR_SYSTIMER_CLK_EN         (1UL << 1)
#define PCR_SYSTIMER_RST_EN         (1UL << 0)

/*---------------------------------------------------------------------------
 * SYSTIMER — fixed 16 MHz, same base on C5/C6/H2/H4
 *---------------------------------------------------------------------------*/
#define SYSTIMER_BASE               0x60005000UL
#define SYSTIMER_CONF_REG           (*(volatile uint32_t *)(SYSTIMER_BASE + 0x000U))
#define SYSTIMER_TIMER_UNIT0_WORK_EN  (1UL << 30)
#define SYSTIMER_CLK_EN               (1UL << 31)

/*---------------------------------------------------------------------------
 * Timer Group WDTs — same base addresses on C5/C6/H2/H4
 *---------------------------------------------------------------------------*/
#define TIMG0_BASE                  0x60008000UL
#define TIMG0_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG0_BASE + 0x0048U))
#define TIMG0_WDTFEED_REG           (*(volatile uint32_t *)(TIMG0_BASE + 0x0060U))
#define TIMG0_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG0_BASE + 0x0064U))

#define TIMG1_BASE                  0x60009000UL
#define TIMG1_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG1_BASE + 0x0048U))
#define TIMG1_WDTFEED_REG           (*(volatile uint32_t *)(TIMG1_BASE + 0x0060U))
#define TIMG1_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG1_BASE + 0x0064U))

#define TIMG_WDT_WPROTECT_MAGIC     0x50D83AA1UL
#define TIMG_WDT_EN                 (1UL << 31)
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 10)

/*---------------------------------------------------------------------------
 * LP Watchdog — RTC WDT and Super WDT
 *---------------------------------------------------------------------------*/
#define LP_WDT_BASE                 0x600B1C00UL
#define LP_WDT_RWDT_CONFIG0_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x0000U))
#define LP_WDT_RWDT_WPROTECT_REG    (*(volatile uint32_t *)(LP_WDT_BASE + 0x001CU))
#define LP_WDT_SWD_CONFIG_REG       (*(volatile uint32_t *)(LP_WDT_BASE + 0x0020U))
#define LP_WDT_SWD_WPROTECT_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x002CU))

#define LP_WDT_WPROTECT_MAGIC       0x50D83AA1UL
#define LP_WDT_RWDT_EN              (1UL << 31)
#define LP_WDT_RWDT_FLASHBOOT_EN    (1UL << 12)
#define LP_WDT_SWD_DISABLE          (1UL << 31)

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 *---------------------------------------------------------------------------*/

uint32_t SystemCoreClock __attribute__((used)) = 80000000UL;

/**
 * Default oscillator descriptor for 40 MHz XTAL targets (C5, C6).
 * Override g_McuOsc in the application for H2 (32 MHz XTAL) or any
 * board that does not use the standard 40 MHz crystal.
 */
__attribute__((weak))
McuOsc_t g_McuOsc = {
    .CoreOsc = {
        .Type     = OSC_TYPE_XTAL,
        .Freq     = 40000000UL,
        .Accuracy = 30,
        .LoadCap  = 0,
    },
    .LowPwrOsc = {
        .Type     = OSC_TYPE_RC,
        .Freq     = 32000UL,
        .Accuracy = 5000,
        .LoadCap  = 0,
    },
    .bUSBClk = false,
};

/*---------------------------------------------------------------------------
 * Internal helpers
 *---------------------------------------------------------------------------*/

static void DisableTimg0Wdt(void)
{
    TIMG0_WDTWPROTECT_REG = TIMG_WDT_WPROTECT_MAGIC;
    TIMG0_WDTFEED_REG     = 1U;
    TIMG0_WDTCONFIG0_REG &= ~(TIMG_WDT_EN | TIMG_WDT_FLASHBOOT_MOD_EN);
    TIMG0_WDTWPROTECT_REG = 0U;
}

static void DisableTimg1Wdt(void)
{
    TIMG1_WDTWPROTECT_REG = TIMG_WDT_WPROTECT_MAGIC;
    TIMG1_WDTFEED_REG     = 1U;
    TIMG1_WDTCONFIG0_REG &= ~(TIMG_WDT_EN | TIMG_WDT_FLASHBOOT_MOD_EN);
    TIMG1_WDTWPROTECT_REG = 0U;
}

static void DisableLpRtcWdt(void)
{
    LP_WDT_RWDT_WPROTECT_REG = LP_WDT_WPROTECT_MAGIC;
    LP_WDT_RWDT_CONFIG0_REG &= ~(LP_WDT_RWDT_EN | LP_WDT_RWDT_FLASHBOOT_EN);
    LP_WDT_RWDT_WPROTECT_REG = 0U;
}

static void DisableSuperWdt(void)
{
    LP_WDT_SWD_WPROTECT_REG = LP_WDT_WPROTECT_MAGIC;
    LP_WDT_SWD_CONFIG_REG  |= LP_WDT_SWD_DISABLE;
    LP_WDT_SWD_WPROTECT_REG = 0U;
}

static void EnableSysTimer(void)
{
    PCR_SYSTIMER_CONF_REG |=  PCR_SYSTIMER_CLK_EN;
    PCR_SYSTIMER_CONF_REG &= ~PCR_SYSTIMER_RST_EN;
    SYSTIMER_CONF_REG     |= (SYSTIMER_CLK_EN | SYSTIMER_TIMER_UNIT0_WORK_EN);
}

/*---------------------------------------------------------------------------
 * Esp32PcrSystemInit
 *
 * Shared boot sequence: disable WDTs, optionally switch CPU frequency,
 * enable SYSTIMER.  Does NOT touch mtvec.
 *
 * @param CpuHz  Target CPU frequency in Hz.  The caller must also define
 *               ESP32_PLL_FREQ (the chip's PLL output in Hz) before
 *               including this TU, or the frequency switch is skipped.
 *               Pass 0 to leave the clock at the ROM bootloader default.
 *---------------------------------------------------------------------------*/
void Esp32PcrSystemInit(uint32_t CpuHz)
{
    DisableTimg0Wdt();
    DisableTimg1Wdt();
    DisableLpRtcWdt();
    DisableSuperWdt();

#if defined(ESP32_PLL_FREQ)
    if (CpuHz != 0U)
    {
        uint32_t divisor = ESP32_PLL_FREQ / CpuHz;
        if (divisor < 1U) { divisor = 1U; }

        uint32_t r = PCR_CPU_FREQ_CONF_REG & ~PCR_CPU_HS_DIV_NUM_Msk;
        PCR_CPU_FREQ_CONF_REG = r | (((divisor - 1U) << PCR_CPU_HS_DIV_NUM_Pos)
                                     & PCR_CPU_HS_DIV_NUM_Msk);

        r = PCR_SYSCLK_CONF_REG & ~PCR_SOC_CLK_SEL_Msk;
        PCR_SYSCLK_CONF_REG = r | PCR_SOC_CLK_PLL;
    }
#else
    (void)CpuHz;
#endif

    EnableSysTimer();
}

/*---------------------------------------------------------------------------
 * SystemCoreClockUpdate — reads PCR to determine actual running frequency.
 *---------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    uint32_t sel = (PCR_SYSCLK_CONF_REG & PCR_SOC_CLK_SEL_Msk) >> PCR_SOC_CLK_SEL_Pos;

    switch (sel)
    {
        case 0U: /* XTAL */
            SystemCoreClock = g_McuOsc.CoreOsc.Freq;
            break;

        case 1U: /* PLL */
        {
            uint32_t div_n = (PCR_CPU_FREQ_CONF_REG & PCR_CPU_HS_DIV_NUM_Msk)
                              >> PCR_CPU_HS_DIV_NUM_Pos;
#if defined(ESP32_PLL_FREQ)
            SystemCoreClock = (uint32_t)(ESP32_PLL_FREQ / (div_n + 1U));
#else
            SystemCoreClock = 480000000UL / (div_n + 1U);   /* C5/C6 safe default */
#endif
            break;
        }

        case 2U: /* FOSC internal RC ~17.5 MHz */
            SystemCoreClock = 17500000UL;
            break;

        default:
            SystemCoreClock = 80000000UL;
            break;
    }
}

/*---------------------------------------------------------------------------
 * IOsonata system_core_clock.h stubs
 *---------------------------------------------------------------------------*/

uint32_t SystemCoreClockGet(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
    (void)Idx;
    return SystemCoreClock;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
    (void)Idx;
    (void)Freq;
    return 0U;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
    (void)ClkSrc;
    (void)OscFreq;
    return false;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
    (void)ClkSrc;
    (void)OscFreq;
    return false;
}

void SystemOscInit(void)
{
}

void * const SystemRamStart(void)
{
    extern unsigned long __data_start__;
    return (void *)&__data_start__;
}
