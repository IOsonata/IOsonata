/**-------------------------------------------------------------------------
@file	system_esp32_pcr.c

@brief	Shared system initialisation for Espressif RISC-V chips with PCR

C6 and C5/C61 use different PCR CPU clock fields. C5/C61 retain the ROM
clock setup and decode the selected root and divider. Their SYSTIMER and
watchdog registers follow ESP-IDF v5.5.1 per-chip register definitions.
The older H2/H4 branches have not been validated by this bring-up.

Chip startup calls Esp32PcrSystemInit() for watchdog and SYSTIMER setup,
then installs the target trap entry. Generic ResetEntry calls
SystemCoreClockUpdate() before entering the application.

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
#if defined(ESP32C5) || defined(ESP32C61)
#if defined(ESP32C5)
#define PCR_SYSCLK_OFFSET           0x110U
#define PCR_SYSTIMER_OFFSET         0x06CU
#else
#define PCR_SYSCLK_OFFSET           0x0E8U
#define PCR_SYSTIMER_OFFSET         0x058U
#endif
#define PCR_SYSCLK_CONF_REG         (*(volatile uint32_t *)(PCR_BASE + PCR_SYSCLK_OFFSET))
#define PCR_CPU_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + PCR_SYSCLK_OFFSET + 8U))
#define PCR_AHB_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + PCR_SYSCLK_OFFSET + 12U))
#define PCR_APB_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + PCR_SYSCLK_OFFSET + 16U))
#define PCR_SYSTIMER_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + PCR_SYSTIMER_OFFSET))
#define PCR_SOC_CLK_SEL_Pos         16U
#elif defined(ESP32C6)
#define PCR_SYSCLK_CONF_REG         (*(volatile uint32_t *)(PCR_BASE + 0x110U))
#define PCR_CPU_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x118U))
#define PCR_SYSTIMER_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x054U))
#define PCR_SOC_CLK_SEL_Pos         (16U)
#else
#define PCR_SYSCLK_CONF_REG         (*(volatile uint32_t *)(PCR_BASE + 0x000U))
#define PCR_CPU_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x004U))
#define PCR_SYSTIMER_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x064U))

#define PCR_SOC_CLK_SEL_Pos         (20U)
#endif
#define PCR_SOC_CLK_SEL_Msk         (0x3UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_XTAL            (0x0UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_PLL             (0x1UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_FOSC            (0x2UL << PCR_SOC_CLK_SEL_Pos)

/* CPU clock divider: field encodes (divisor - 1) */
#define PCR_CPU_HS_DIV_NUM_Pos      (8U)
#define PCR_CPU_HS_DIV_NUM_Msk      (0xFFUL << PCR_CPU_HS_DIV_NUM_Pos)

#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
#define PCR_SYSTIMER_CLK_EN         (1UL << 0)
#define PCR_SYSTIMER_RST_EN         (1UL << 1)
#else
#define PCR_SYSTIMER_CLK_EN         (1UL << 1)
#define PCR_SYSTIMER_RST_EN         (1UL << 0)
#endif

/*---------------------------------------------------------------------------
 * SYSTIMER — fixed 16 MHz, same base on C5/C6/H2/H4
 *---------------------------------------------------------------------------*/
#if defined(ESP32C5) || defined(ESP32C61)
#define SYSTIMER_BASE               0x6000A000UL
#else
#define SYSTIMER_BASE               0x60005000UL
#endif
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
#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 14)
#else
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 10)
#endif

/*---------------------------------------------------------------------------
 * LP Watchdog — RTC WDT and Super WDT
 *---------------------------------------------------------------------------*/
#define LP_WDT_BASE                 0x600B1C00UL
#define LP_WDT_RWDT_CONFIG0_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x0000U))
#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
#define LP_WDT_RWDT_WPROTECT_REG    (*(volatile uint32_t *)(LP_WDT_BASE + 0x0018U))
#define LP_WDT_SWD_CONFIG_REG       (*(volatile uint32_t *)(LP_WDT_BASE + 0x001CU))
#define LP_WDT_SWD_WPROTECT_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x0020U))
#define LP_WDT_SWD_DISABLE          (1UL << 30)
// LP watchdog key from the per-chip ESP-IDF v5.5.1 hal/lpwdt_ll.h.
#define LP_SWD_WPROTECT_MAGIC       0x50D83AA1UL
#else
#define LP_WDT_RWDT_WPROTECT_REG    (*(volatile uint32_t *)(LP_WDT_BASE + 0x001CU))
#define LP_WDT_SWD_CONFIG_REG       (*(volatile uint32_t *)(LP_WDT_BASE + 0x0020U))
#define LP_WDT_SWD_WPROTECT_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x002CU))

#define LP_WDT_SWD_DISABLE          (1UL << 31)
#define LP_SWD_WPROTECT_MAGIC       0x50D83AA1UL
#endif

#define LP_WDT_WPROTECT_MAGIC       0x50D83AA1UL
#define LP_WDT_RWDT_EN              (1UL << 31)
#define LP_WDT_RWDT_FLASHBOOT_EN    (1UL << 12)

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 *---------------------------------------------------------------------------*/

uint32_t SystemCoreClock __attribute__((used)) = 80000000UL;
uint64_t SystemCoreClockPeriodus __attribute__((used)) = 80ULL;
#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
uint64_t SystemCoreClockPeriodns __attribute__((used)) = 13ULL;
#endif

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
    LP_WDT_SWD_WPROTECT_REG = LP_SWD_WPROTECT_MAGIC;
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

#if defined(ESP32_PLL_FREQ) && !defined(ESP32C5) && !defined(ESP32C61)
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
#if defined(ESP32C5) || defined(ESP32C61)
static uint32_t Esp32PcrRootClock(void)
{
	uint32_t source;

	source = (PCR_SYSCLK_CONF_REG >> 16) & 3U;
	switch (source)
	{
		case 0:
			return g_McuOsc.CoreOsc.Freq;
		case 1:
			return 17500000UL;
		case 2:
			return 160000000UL;
#if defined(ESP32C5)
		case 3:
			return 240000000UL;
#endif
		default:
			return 0;
	}
}
#endif

void SystemCoreClockUpdate(void)
{
#if defined(ESP32C5) || defined(ESP32C61)
	SystemCoreClock = Esp32PcrRootClock() / ((PCR_CPU_FREQ_CONF_REG & 0xFFU) + 1U);
	SystemCoreClockPeriodus = (SystemCoreClock + 500000U) / 1000000U;
	SystemCoreClockPeriodns = SystemCoreClock != 0U ?
		(1000000000ULL + SystemCoreClock / 2U) / SystemCoreClock : 0U;
#elif defined(ESP32C6)
	uint32_t root = PCR_SYSCLK_CONF_REG;
	uint32_t cpu = PCR_CPU_FREQ_CONF_REG;
	uint32_t sel = (root & PCR_SOC_CLK_SEL_Msk) >> PCR_SOC_CLK_SEL_Pos;
	uint32_t divisor;

	if (sel == 1U)
	{
		divisor = (((root >> 8) & 0xFFU) + 1U) * (((cpu >> 8) & 0xFFU) + 1U);
		if ((cpu & PCR_CPU_HS_DIV_NUM_Msk) == 0U && (cpu & (1UL << 16)) != 0U)
		{
			divisor = 4U;
		}
		SystemCoreClock = 480000000UL / divisor;
	}
	else
	{
		divisor = ((root & 0xFFU) + 1U) * ((cpu & 0xFFU) + 1U);
		SystemCoreClock = (sel == 2U ? 17500000UL : g_McuOsc.CoreOsc.Freq) / divisor;
	}
	SystemCoreClockPeriodus = (SystemCoreClock + 500000U) / 1000000U;
	SystemCoreClockPeriodns = (1000000000ULL + SystemCoreClock / 2U) / SystemCoreClock;
#else
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

    SystemCoreClockPeriodus = (SystemCoreClock + 500000U) / 1000000U;
#endif
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
#if defined(ESP32C5) || defined(ESP32C61)
	if (Idx != 0)
	{
		return 0;
	}
	return Esp32PcrRootClock() / ((PCR_AHB_FREQ_CONF_REG & 0xFFU) + 1U) /
		(((PCR_APB_FREQ_CONF_REG >> 8) & 0xFFU) + 1U);
#else
    (void)Idx;
    return SystemCoreClock;
#endif
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

void *SystemRamStart(void)
{
    extern unsigned long __data_start__;
    return (void *)&__data_start__;
}
