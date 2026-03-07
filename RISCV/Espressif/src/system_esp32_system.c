/**-------------------------------------------------------------------------
@file	system_esp32_system.c

@brief	Shared system initialisation for Espressif RISC-V chips with SYSTEM
        peripheral clock controller

Covers Espressif RISC-V targets that use the legacy SYSTEM (not PCR)
peripheral for clock and reset control:

    ESP32-C3

Clock architecture (ESP32-C3 TRM Rev 0.4 §7):
  XTAL  :  40 MHz
  PLL   : 480 MHz  →  /6 = 80 MHz (PLL_F80M),  /3 = 160 MHz (PLL_F160M)
  FOSC  :  ~17.5 MHz internal RC

SYSTEM_SOC_CLK_SEL selects the CPU clock source directly:
  0 = XTAL (40 MHz)
  1 = PLL_F80M (80 MHz)    ← ROM bootloader default
  2 = PLL_F160M (160 MHz)
  3 = FOSC (~17.5 MHz)

Unlike PCR-based chips, there is no divisor register — speed is chosen
by selecting the appropriate PLL tap.

Define ESP32C3_CPU_160MHZ to run at 160 MHz.
Default (no define): stay at ROM bootloader default (80 MHz PLL_F80M).

Chip-specific system_espXXXX.c files call Esp32SystemInit() then install
the trap handler into mtvec.

NOT used by ESP32-C5, ESP32-C6, ESP32-H2, ESP32-H4 — those use PCR and
call Esp32PcrSystemInit() from system_esp32_pcr.c.

@author	Hoang Nguyen Hoan
@date	Mar. 6, 2026

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
 * SYSTEM — Clock and reset controller for CLINT-based ESP32 targets
 * ESP32-C3 TRM Rev 0.4, §7 (System Configuration)
 * Base address: 0x600C0000
 *---------------------------------------------------------------------------*/
#define SYSTEM_BASE                 0x600C0000UL
#define SYSTEM_SYSCLK_CONF_REG      (*(volatile uint32_t *)(SYSTEM_BASE + 0x058U))

#define SYSTEM_SOC_CLK_SEL_Pos      (10U)
#define SYSTEM_SOC_CLK_SEL_Msk      (0x3UL << SYSTEM_SOC_CLK_SEL_Pos)
#define SYSTEM_SOC_CLK_XTAL         (0x0UL << SYSTEM_SOC_CLK_SEL_Pos)  /* 40 MHz XTAL      */
#define SYSTEM_SOC_CLK_PLL_F80M     (0x1UL << SYSTEM_SOC_CLK_SEL_Pos)  /* 80 MHz PLL tap   */
#define SYSTEM_SOC_CLK_PLL_F160M    (0x2UL << SYSTEM_SOC_CLK_SEL_Pos)  /* 160 MHz PLL tap  */
#define SYSTEM_SOC_CLK_FOSC         (0x3UL << SYSTEM_SOC_CLK_SEL_Pos)  /* ~17.5 MHz RC     */

/*---------------------------------------------------------------------------
 * SYSTIMER — 16 MHz free-running counter
 * ESP32-C3 TRM, §22  Base: 0x60023000
 *---------------------------------------------------------------------------*/
#define SYSTIMER_BASE               0x60023000UL
#define SYSTIMER_CONF_REG           (*(volatile uint32_t *)(SYSTIMER_BASE + 0x000U))
#define SYSTIMER_TIMER_UNIT0_WORK_EN  (1UL << 30)
#define SYSTIMER_CLK_EN               (1UL << 31)

/*---------------------------------------------------------------------------
 * Timer Group WDTs
 * ESP32-C3 TRM, §24
 * TIMG0: 0x6001F000   TIMG1: 0x60020000
 *---------------------------------------------------------------------------*/
#define TIMG0_BASE                  0x6001F000UL
#define TIMG0_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG0_BASE + 0x0048U))
#define TIMG0_WDTFEED_REG           (*(volatile uint32_t *)(TIMG0_BASE + 0x0060U))
#define TIMG0_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG0_BASE + 0x0064U))

#define TIMG1_BASE                  0x60020000UL
#define TIMG1_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG1_BASE + 0x0048U))
#define TIMG1_WDTFEED_REG           (*(volatile uint32_t *)(TIMG1_BASE + 0x0060U))
#define TIMG1_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG1_BASE + 0x0064U))

#define TIMG_WDT_WPROTECT_MAGIC     0x50D83AA1UL
#define TIMG_WDT_EN                 (1UL << 31)
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 10)

/*---------------------------------------------------------------------------
 * RTC_CNTL — RTC watchdog and super watchdog
 * ESP32-C3 TRM, §8  Base: 0x60008000
 * NOTE: C3 uses RTC_CNTL instead of the LP_WDT found on PCR-based chips.
 *---------------------------------------------------------------------------*/
#define RTC_CNTL_BASE               0x60008000UL
#define RTC_CNTL_WDTCONFIG0_REG    (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x0094U))
#define RTC_CNTL_WDTFEED_REG       (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x009CU))
#define RTC_CNTL_WDTWPROTECT_REG   (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00A4U))
#define RTC_CNTL_SWD_CONF_REG      (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00B0U))
#define RTC_CNTL_SWD_WPROTECT_REG  (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00BCU))

#define RTC_WDT_WPROTECT_MAGIC      0x50D83AA1UL
#define RTC_WDT_EN                  (1UL << 31)
#define RTC_WDT_FLASHBOOT_EN        (1UL << 12)
#define RTC_SWD_DISABLE             (1UL << 31)

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 *---------------------------------------------------------------------------*/
uint32_t SystemCoreClock __attribute__((used)) = 80000000UL;

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

static void DisableRtcWdt(void)
{
	RTC_CNTL_WDTWPROTECT_REG = RTC_WDT_WPROTECT_MAGIC;
	RTC_CNTL_WDTFEED_REG     = 1U;
	RTC_CNTL_WDTCONFIG0_REG &= ~(RTC_WDT_EN | RTC_WDT_FLASHBOOT_EN);
	RTC_CNTL_WDTWPROTECT_REG = 0U;
}

static void DisableSuperWdt(void)
{
	RTC_CNTL_SWD_WPROTECT_REG = RTC_WDT_WPROTECT_MAGIC;
	RTC_CNTL_SWD_CONF_REG    |= RTC_SWD_DISABLE;
	RTC_CNTL_SWD_WPROTECT_REG = 0U;
}

static void EnableSysTimer(void)
{
	SYSTIMER_CONF_REG |= (SYSTIMER_CLK_EN | SYSTIMER_TIMER_UNIT0_WORK_EN);
}

/*---------------------------------------------------------------------------
 * Esp32SystemInit
 *
 * Shared boot sequence for SYSTEM-peripheral chips:
 *   1. Disable all hardware watchdogs.
 *   2. Optionally switch CPU to 160 MHz (define ESP32C3_CPU_160MHZ).
 *   3. Enable SYSTIMER.
 *
 * Does NOT touch mtvec — that is done in the chip-specific SystemInit().
 *---------------------------------------------------------------------------*/
void Esp32SystemInit(void)
{
	DisableTimg0Wdt();
	DisableTimg1Wdt();
	DisableRtcWdt();
	DisableSuperWdt();

#if defined(ESP32C3_CPU_160MHZ)
	uint32_t r = SYSTEM_SYSCLK_CONF_REG & ~SYSTEM_SOC_CLK_SEL_Msk;
	SYSTEM_SYSCLK_CONF_REG = r | SYSTEM_SOC_CLK_PLL_F160M;
	SystemCoreClock = 160000000UL;
#else
	SystemCoreClock = 80000000UL;   /* PLL_F80M — ROM bootloader default */
#endif

	EnableSysTimer();
}

/*---------------------------------------------------------------------------
 * SystemCoreClockUpdate — reads SYSTEM peripheral for actual running freq.
 *---------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
	uint32_t sel = (SYSTEM_SYSCLK_CONF_REG & SYSTEM_SOC_CLK_SEL_Msk)
	               >> SYSTEM_SOC_CLK_SEL_Pos;

	switch (sel)
	{
		case 0U: /* XTAL */
			SystemCoreClock = g_McuOsc.CoreOsc.Freq;
			break;
		case 1U: /* PLL_F80M */
			SystemCoreClock = 80000000UL;
			break;
		case 2U: /* PLL_F160M */
			SystemCoreClock = 160000000UL;
			break;
		case 3U: /* FOSC internal RC ~17.5 MHz */
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
