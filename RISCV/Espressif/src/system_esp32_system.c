/**-------------------------------------------------------------------------
@file	system_esp32_system.c

@brief	Shared system initialisation for Espressif RISC-V chips with SYSTEM
        peripheral clock controller

Covers Espressif RISC-V targets that use the legacy SYSTEM (not PCR)
peripheral for clock and reset control:

    ESP32-C3

Clock architecture (ESP32-C3 TRM Rev 0.4 §7, verified against ESP-IDF
v5.3 hal/esp32c3 clk_tree_ll.h):
  XTAL  :  40 MHz
  PLL   : 480 MHz raw, divided to 80 MHz or 160 MHz at the CPU mux
  FOSC  :  ~17.5 MHz internal RC ("RC_FAST")

CPU clock is selected by TWO independent register fields:

  SYSCLK_CONF.SOC_CLK_SEL    (offset 0x058, bits [11:10]) — source family
       0 = XTAL       1 = PLL       2 = RC_FAST       3 = (invalid)

  CPU_PER_CONF.CPUPERIOD_SEL (offset 0x008, bits  [1:0])  — PLL tap
       0 = 80 MHz     1 = 160 MHz   (only meaningful when SOC_CLK_SEL = 1)

Writing SOC_CLK_SEL = 1 alone does NOT pin the CPU to a specific MHz —
it leaves the divider at whatever value the previous owner of the field
left.  Since the C3 ROM bootloader can hand off with CPUPERIOD_SEL at
either 0 or 1 (depending on flash-speed / boot path), this code MUST
program both fields.

Define ESP32C3_CPU_160MHZ to run at 160 MHz.
Default (no define): 80 MHz from PLL.

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
#define SYSTEM_CPU_PER_CONF_REG     (*(volatile uint32_t *)(SYSTEM_BASE + 0x008U))
#define SYSTEM_SYSCLK_CONF_REG      (*(volatile uint32_t *)(SYSTEM_BASE + 0x058U))

/* SYSCLK_CONF.SOC_CLK_SEL [11:10] — clock SOURCE FAMILY only (verified
 * against ESP-IDF v5.3 components/hal/esp32c3/include/hal/clk_tree_ll.h
 * clk_ll_cpu_set_src).  Value 2 is RC_FAST (~17.5 MHz), NOT 160 MHz PLL.
 * Value 3 is reserved/invalid. */
#define SYSTEM_SOC_CLK_SEL_Pos      (10U)
#define SYSTEM_SOC_CLK_SEL_Msk      (0x3UL << SYSTEM_SOC_CLK_SEL_Pos)
#define SYSTEM_SOC_CLK_SEL_XTAL     (0x0UL << SYSTEM_SOC_CLK_SEL_Pos)  /* 40 MHz XTAL      */
#define SYSTEM_SOC_CLK_SEL_PLL      (0x1UL << SYSTEM_SOC_CLK_SEL_Pos)  /* PLL (80 or 160) */
#define SYSTEM_SOC_CLK_SEL_RC_FAST  (0x2UL << SYSTEM_SOC_CLK_SEL_Pos)  /* ~17.5 MHz RC     */

/* CPU_PER_CONF.CPUPERIOD_SEL [1:0] — selects the PLL tap when source is PLL.
 * 0 → 80 MHz, 1 → 160 MHz.  Independent of SOC_CLK_SEL.  Per IDF
 * clk_ll_cpu_set_freq_mhz_from_pll. */
#define SYSTEM_CPUPERIOD_SEL_Pos    (0U)
#define SYSTEM_CPUPERIOD_SEL_Msk    (0x3UL << SYSTEM_CPUPERIOD_SEL_Pos)
#define SYSTEM_CPUPERIOD_SEL_80M    (0x0UL << SYSTEM_CPUPERIOD_SEL_Pos)
#define SYSTEM_CPUPERIOD_SEL_160M   (0x1UL << SYSTEM_CPUPERIOD_SEL_Pos)

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
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 14)   /* C3 TRM: bitpos [14], default 1 at reset */

/*---------------------------------------------------------------------------
 * RTC_CNTL — RTC watchdog and super watchdog
 * ESP32-C3 TRM, §8  Base: 0x60008000
 * NOTE: C3 uses RTC_CNTL instead of the LP_WDT found on PCR-based chips.
 *
 * Register offsets verified against ESP-IDF C3 register file
 * (components/soc/esp32c3/register/soc/rtc_cntl_reg.h).  The previous
 * revision of this file had all five RTC CNTL offsets shifted, the
 * SWD using the wrong write-protect key, and SWD_DISABLE on the wrong
 * bit (31 vs 30).  Net effect was that the RTC WDT and the Super WDT
 * were never actually disabled — both fired at their default timeouts
 * (~9 s and ~3 s respectively) and reset the chip in a loop.  Symptom
 * was masked when a) the chip didn't reach user code anyway because of
 * an upstream IOcomposer wrap bug, or b) the WS2812 LED happened to
 * latch its last colour through each reset, looking like continuous
 * dim flicker rather than a 9-second reset cycle.
 *---------------------------------------------------------------------*/
#define RTC_CNTL_BASE               0x60008000UL
#define RTC_CNTL_WDTCONFIG0_REG    (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x0090U))
#define RTC_CNTL_WDTFEED_REG       (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00A4U))
#define RTC_CNTL_WDTWPROTECT_REG   (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00A8U))
#define RTC_CNTL_SWD_CONF_REG      (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00ACU))
#define RTC_CNTL_SWD_WPROTECT_REG  (*(volatile uint32_t *)(RTC_CNTL_BASE + 0x00B0U))

#define RTC_WDT_WPROTECT_MAGIC      0x50D83AA1UL    /* RTC_CNTL_WDT_WKEY */
#define RTC_SWD_WPROTECT_MAGIC      0x8F1D312AUL    /* RTC_CNTL_SWD_WKEY — different! */
#define RTC_WDT_EN                  (1UL << 31)
#define RTC_WDT_FLASHBOOT_EN        (1UL << 12)
#define RTC_SWD_DISABLE             (1UL << 30)     /* bit 30 — bit 31 is SWD_AUTO_FEED_EN */

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 *---------------------------------------------------------------------------*/
uint32_t SystemCoreClock __attribute__((used)) = 80000000UL;

/* idelay.h rdcycle path: Periodus = cycles/us, Periodns = ns/cycle */
uint64_t SystemCoreClockPeriodus __attribute__((used)) = 80ULL;        /* 80 MHz default */
uint64_t SystemCoreClockPeriodns __attribute__((used)) = 13ULL;        /* ~12.5 ns @ 80 MHz */

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
	RTC_CNTL_SWD_WPROTECT_REG = RTC_SWD_WPROTECT_MAGIC;
	RTC_CNTL_SWD_CONF_REG    |= RTC_SWD_DISABLE;
	RTC_CNTL_SWD_WPROTECT_REG = 0U;
}

static void EnableSysTimer(void)
{
	SYSTIMER_CONF_REG |= (SYSTIMER_CLK_EN | SYSTIMER_TIMER_UNIT0_WORK_EN);
}

/*---------------------------------------------------------------------------
 * Enable the Espressif RISC-V Machine Performance Counter so that the
 * MPCCR (CSR 0x7E2) increments once per CPU cycle.  IOsonataRiscvCycle32()
 * in idelay.h reads this CSR — without this setup, MPCCR stays at 0 and
 * usDelay/nsDelay's rdcycle path spins forever.
 *
 * The C3/C6/H2 do NOT implement the standard `cycle` / `mcycle` CSRs
 * (0xC00 / 0xB00).  Reading those raises an illegal-instruction trap.
 * Espressif provides this custom mpcer/mpcmr/mpccr trio at 0x7E0..0x7E2
 * in the user-reserved CSR range instead.
 *
 *   mpcer (0x7E0): event selector — bit 0 (CYCLE) selects per-cycle increment
 *   mpcmr (0x7E1): bit 0 (COUNT_EN) starts the counter
 *   mpccr (0x7E2): the running 32-bit count (R/W)
 *---------------------------------------------------------------------------*/
static void Esp32EnablePerfCounter(void)
{
	__asm volatile ("csrw 0x7E0, %0" :: "r"(1U));   /* mpcer = CYCLE event   */
	__asm volatile ("csrw 0x7E2, %0" :: "r"(0U));   /* mpccr = 0 (clear)     */
	__asm volatile ("csrw 0x7E1, %0" :: "r"(1U));   /* mpcmr = COUNT_EN      */
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
	/* 160 MHz from PLL: program CPUPERIOD_SEL = 1 (160) BEFORE switching
	 * SOC_CLK_SEL to PLL, so we never briefly see an unsupported tap. */
	uint32_t pc = SYSTEM_CPU_PER_CONF_REG & ~SYSTEM_CPUPERIOD_SEL_Msk;
	SYSTEM_CPU_PER_CONF_REG = pc | SYSTEM_CPUPERIOD_SEL_160M;
	uint32_t r = SYSTEM_SYSCLK_CONF_REG & ~SYSTEM_SOC_CLK_SEL_Msk;
	SYSTEM_SYSCLK_CONF_REG = r | SYSTEM_SOC_CLK_SEL_PLL;
	SystemCoreClock = 160000000UL;
#else
	/* 80 MHz from PLL.  The C3 ROM bootloader does NOT pin CPUPERIOD_SEL
	 * to any specific value — depending on flash speed config and ROM
	 * path it can land at 0 (80) or 1 (160).  Writing SOC_CLK_SEL=PLL
	 * alone (as the previous version of this file did) leaves the PLL
	 * tap at whatever ROM left it, so the CPU may end up at 160 MHz
	 * even though SystemCoreClock says 80 MHz.  WS2812 NOP timing,
	 * msDelay() via mpccr, and any other cycle-calibrated code then
	 * runs at 2x.  Force CPUPERIOD_SEL=0 here to actually pin 80 MHz. */
	uint32_t pc = SYSTEM_CPU_PER_CONF_REG & ~SYSTEM_CPUPERIOD_SEL_Msk;
	SYSTEM_CPU_PER_CONF_REG = pc | SYSTEM_CPUPERIOD_SEL_80M;
	uint32_t r = SYSTEM_SYSCLK_CONF_REG & ~SYSTEM_SOC_CLK_SEL_Msk;
	SYSTEM_SYSCLK_CONF_REG = r | SYSTEM_SOC_CLK_SEL_PLL;
	SystemCoreClock = 80000000UL;
#endif

	EnableSysTimer();
	Esp32EnablePerfCounter();
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
		case 1U: /* PLL — actual freq depends on CPUPERIOD_SEL */
		{
			uint32_t per = (SYSTEM_CPU_PER_CONF_REG & SYSTEM_CPUPERIOD_SEL_Msk)
			               >> SYSTEM_CPUPERIOD_SEL_Pos;
			SystemCoreClock = (per == 1U) ? 160000000UL : 80000000UL;
			break;
		}
		case 2U: /* RC_FAST ~17.5 MHz */
			SystemCoreClock = 17500000UL;
			break;
		default: /* 3 = invalid on C3, treat as PLL_F80M default */
			SystemCoreClock = 80000000UL;
			break;
	}

	/* Update idelay period variables for the rdcycle path. */
	SystemCoreClockPeriodus = (SystemCoreClock + 500000U) / 1000000U;
	SystemCoreClockPeriodns = (1000000000ULL + (uint64_t)(SystemCoreClock / 2))
	                          / SystemCoreClock;
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
