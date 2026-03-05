/**-------------------------------------------------------------------------
@file	system_esp32c6.c

@brief	ESP32-C6 CMSIS-style system initialisation

Implements SystemInit() and SystemCoreClockUpdate() called by ResetEntry.c
before _start() / main().

Responsibilities:
  - Disable all hardware watchdogs left armed by the ROM bootloader
  - Enable and configure SYSTIMER (Unit 0) so that idelay.h works
    immediately after SystemInit() returns
  - Optionally switch the CPU to 160 MHz via the on-chip SPLL
  - Populate SystemCoreClock and g_McuOsc

Clock architecture (ESP32-C6 TRM Rev 0.6 §7):
  XTAL:  40 MHz  (default XTAL_CLK)
  SPLL: 480 MHz  → divided down by PCR_CPU_FREQ_CONF dividers
  FOSC:  17.5 MHz internal RC fast oscillator
  SOSC:  32 768 Hz slow oscillator (LP domain)

After the ROM stage-2 bootloader the CPU typically runs at 80 MHz
(PLL/6, divider = 3 on a 40 MHz XTAL system).  We switch to 160 MHz
(PLL/3) for maximum performance.  Define ESP32C6_CPU_80MHZ to stay at
80 MHz.

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
 * Peripheral base addresses (ESP32-C6 TRM Rev 0.6 §Appendix A)
 *---------------------------------------------------------------------------*/

/* Peripheral Clock / Reset controller */
#define PCR_BASE                    0x60096000UL
#define PCR_SYSCLK_CONF_REG         (*(volatile uint32_t *)(PCR_BASE + 0x000U))
#define PCR_CPU_FREQ_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x004U))
#define PCR_SYSTIMER_CONF_REG       (*(volatile uint32_t *)(PCR_BASE + 0x064U))

/* PCR_SYSCLK_CONF_REG bit fields */
#define PCR_SOC_CLK_SEL_Pos         (20U)
#define PCR_SOC_CLK_SEL_Msk         (0x3UL << PCR_SOC_CLK_SEL_Pos)
#define PCR_SOC_CLK_XTAL            (0x0UL << PCR_SOC_CLK_SEL_Pos)  /* 40 MHz */
#define PCR_SOC_CLK_PLL_F80M        (0x1UL << PCR_SOC_CLK_SEL_Pos)  /* PLL/6  = 80 MHz  */
#define PCR_SOC_CLK_FOSC            (0x2UL << PCR_SOC_CLK_SEL_Pos)  /* ~17.5 MHz RC     */

/* PCR_CPU_FREQ_CONF_REG bit fields */
#define PCR_CPU_HS_DIV_NUM_Pos      (8U)
#define PCR_CPU_HS_DIV_NUM_Msk      (0xFFUL << PCR_CPU_HS_DIV_NUM_Pos)
/* Divider value n means divide-by-(n+1).
 * SPLL output = 480 MHz.
 * 480/(2+1)=160 MHz  → n=2
 * 480/(5+1)= 80 MHz  → n=5                                          */
#define PCR_CPU_HS_DIV_160M         (2UL << PCR_CPU_HS_DIV_NUM_Pos)
#define PCR_CPU_HS_DIV_80M          (5UL << PCR_CPU_HS_DIV_NUM_Pos)

#define PCR_CPU_CLK_SEL_Pos         (20U)
#define PCR_CPU_CLK_SEL_Msk         (0x3UL << PCR_CPU_CLK_SEL_Pos)
#define PCR_CPU_CLK_XTAL            (0x0UL << PCR_CPU_CLK_SEL_Pos)
#define PCR_CPU_CLK_PLL             (0x1UL << PCR_CPU_CLK_SEL_Pos)
#define PCR_CPU_CLK_FOSC            (0x2UL << PCR_CPU_CLK_SEL_Pos)

/* PCR_SYSTIMER_CONF_REG bit fields */
#define PCR_SYSTIMER_CLK_EN_Pos     (1U)
#define PCR_SYSTIMER_CLK_EN         (1UL << PCR_SYSTIMER_CLK_EN_Pos)
#define PCR_SYSTIMER_RST_EN_Pos     (0U)
#define PCR_SYSTIMER_RST_EN         (1UL << PCR_SYSTIMER_RST_EN_Pos)

/* ---- SYSTIMER -----------------------------------------------------------*/
#define SYSTIMER_BASE               0x60005000UL
#define SYSTIMER_CONF_REG           (*(volatile uint32_t *)(SYSTIMER_BASE + 0x000U))
#define SYSTIMER_UNIT0_OP_REG       (*(volatile uint32_t *)(SYSTIMER_BASE + 0x018U))
#define SYSTIMER_UNIT0_VALUE_LO     (*(volatile uint32_t *)(SYSTIMER_BASE + 0x040U))
#define SYSTIMER_UNIT0_VALUE_HI     (*(volatile uint32_t *)(SYSTIMER_BASE + 0x044U))

/* SYSTIMER_CONF_REG bit fields */
#define SYSTIMER_TIMER_UNIT0_WORK_EN  (1UL << 30)
#define SYSTIMER_CLK_EN               (1UL << 31)

/* ---- Timer Group WDT (TIMG0) --------------------------------------------*/
#define TIMG0_BASE                  0x60008000UL
#define TIMG0_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG0_BASE + 0x0048U))
#define TIMG0_WDTFEED_REG           (*(volatile uint32_t *)(TIMG0_BASE + 0x0060U))
#define TIMG0_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG0_BASE + 0x0064U))
#define TIMG0_WDT_WPROTECT_MAGIC    0x50D83AA1UL

/* TIMG0_WDTCONFIG0_REG bit fields */
#define TIMG_WDT_EN_Pos             (31U)
#define TIMG_WDT_EN                 (1UL << TIMG_WDT_EN_Pos)
#define TIMG_WDT_FLASHBOOT_MOD_EN   (1UL << 10)

/* ---- Timer Group 1 WDT (TIMG1) -----------------------------------------*/
#define TIMG1_BASE                  0x60009000UL
#define TIMG1_WDTCONFIG0_REG        (*(volatile uint32_t *)(TIMG1_BASE + 0x0048U))
#define TIMG1_WDTFEED_REG           (*(volatile uint32_t *)(TIMG1_BASE + 0x0060U))
#define TIMG1_WDTWPROTECT_REG       (*(volatile uint32_t *)(TIMG1_BASE + 0x0064U))

/* ---- LP Watchdog (RTC WDT + Super WDT) ---------------------------------*/
#define LP_WDT_BASE                 0x600B1C00UL
#define LP_WDT_RWDT_CONFIG0_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x0000U))
#define LP_WDT_RWDT_WPROTECT_REG    (*(volatile uint32_t *)(LP_WDT_BASE + 0x001CU))
#define LP_WDT_SWD_CONFIG_REG       (*(volatile uint32_t *)(LP_WDT_BASE + 0x0020U))
#define LP_WDT_SWD_WPROTECT_REG     (*(volatile uint32_t *)(LP_WDT_BASE + 0x002CU))

#define LP_WDT_RWDT_WPROTECT_MAGIC  0x50D83AA1UL
#define LP_WDT_SWD_WPROTECT_MAGIC   0x50D83AA1UL

/* LP_WDT_RWDT_CONFIG0_REG bit fields */
#define LP_WDT_RWDT_EN_Pos          (31U)
#define LP_WDT_RWDT_EN              (1UL << LP_WDT_RWDT_EN_Pos)
#define LP_WDT_RWDT_FLASHBOOT_EN    (1UL << 12)

/* LP_WDT_SWD_CONFIG_REG bit fields */
#define LP_WDT_SWD_DISABLE_Pos      (31U)
#define LP_WDT_SWD_DISABLE          (1UL << LP_WDT_SWD_DISABLE_Pos)

/*---------------------------------------------------------------------------
 * External declarations required by ResetEntry.c
 *---------------------------------------------------------------------------*/
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 *---------------------------------------------------------------------------*/

/** CPU core clock in Hz — updated by SystemCoreClockUpdate() */
uint32_t SystemCoreClock __attribute__((used)) = 80000000UL;

/**
 * Default oscillator descriptor.
 * Firmware can override g_McuOsc in the application to reflect actual
 * board oscillator choice (e.g. if not using 40 MHz XTAL).
 */
__attribute__((weak))
McuOsc_t g_McuOsc = {
    .CoreOsc = {
        .Type     = OSC_TYPE_XTAL,
        .Freq     = 40000000UL,   /* 40 MHz XTAL */
        .Accuracy = 30,           /* 30 PPM typical */
        .LoadCap  = 0,
    },
    .LowPwrOsc = {
        .Type     = OSC_TYPE_RC,
        .Freq     = 32000UL,      /* ~32 kHz internal RC */
        .Accuracy = 5000,
        .LoadCap  = 0,
    },
    .bUSBClk = false,
};

/*---------------------------------------------------------------------------
 * Internal helpers
 *---------------------------------------------------------------------------*/

/** Feed + disable TIMG0 watchdog timer. */
static void DisableTimg0Wdt(void)
{
    TIMG0_WDTWPROTECT_REG = TIMG0_WDT_WPROTECT_MAGIC;
    TIMG0_WDTFEED_REG     = 1U;
    TIMG0_WDTCONFIG0_REG &= ~(TIMG_WDT_EN | TIMG_WDT_FLASHBOOT_MOD_EN);
    TIMG0_WDTWPROTECT_REG = 0U;
}

/** Feed + disable TIMG1 watchdog timer. */
static void DisableTimg1Wdt(void)
{
    TIMG1_WDTWPROTECT_REG = TIMG0_WDT_WPROTECT_MAGIC;
    TIMG1_WDTFEED_REG     = 1U;
    TIMG1_WDTCONFIG0_REG &= ~(TIMG_WDT_EN | TIMG_WDT_FLASHBOOT_MOD_EN);
    TIMG1_WDTWPROTECT_REG = 0U;
}

/** Disable LP (RTC) watchdog. */
static void DisableLpRtcWdt(void)
{
    LP_WDT_RWDT_WPROTECT_REG = LP_WDT_RWDT_WPROTECT_MAGIC;
    LP_WDT_RWDT_CONFIG0_REG &= ~(LP_WDT_RWDT_EN | LP_WDT_RWDT_FLASHBOOT_EN);
    LP_WDT_RWDT_WPROTECT_REG = 0U;
}

/** Disable super-watchdog (runs independently in LP domain). */
static void DisableSuperWdt(void)
{
    LP_WDT_SWD_WPROTECT_REG = LP_WDT_SWD_WPROTECT_MAGIC;
    LP_WDT_SWD_CONFIG_REG  |= LP_WDT_SWD_DISABLE;
    LP_WDT_SWD_WPROTECT_REG = 0U;
}

/**
 * Enable SYSTIMER Unit 0.
 *
 * SYSTIMER runs at 16 MHz (fixed, independent of CPU clock).
 * It must be enabled before idelay.h functions are usable.
 */
static void EnableSysTimer(void)
{
    /* Gate on the SYSTIMER clock via PCR */
    PCR_SYSTIMER_CONF_REG |= PCR_SYSTIMER_CLK_EN;
    PCR_SYSTIMER_CONF_REG &= ~PCR_SYSTIMER_RST_EN;

    /* Enable Unit 0 counting */
    SYSTIMER_CONF_REG |= (SYSTIMER_CLK_EN | SYSTIMER_TIMER_UNIT0_WORK_EN);
}

/*---------------------------------------------------------------------------
 * Public API — called by ResetEntry.c
 *---------------------------------------------------------------------------*/

/**
 * @brief   CPU and peripheral initialisation before C runtime starts.
 *
 * Execution context: stack is valid, .data/.bss not yet initialised.
 * Do NOT call any function that touches zero-initialised globals.
 */
void SystemInit(void)
{
    /* 1 ── Disable all watchdogs ---------------------------------------- */
    DisableTimg0Wdt();
    DisableTimg1Wdt();
    DisableLpRtcWdt();
    DisableSuperWdt();

    /* 2 ── Configure CPU frequency --------------------------------------- */
#if defined(ESP32C6_CPU_160MHZ)
    /*
     * Switch CPU to 160 MHz: set SPLL divider to /3 (n=2) then select PLL.
     * The SPLL is already running at 480 MHz after ROM bootloader.
     */
    uint32_t freq_conf = PCR_CPU_FREQ_CONF_REG;
    freq_conf &= ~PCR_CPU_HS_DIV_NUM_Msk;
    freq_conf |= PCR_CPU_HS_DIV_160M;
    PCR_CPU_FREQ_CONF_REG = freq_conf;

    uint32_t sys_conf = PCR_SYSCLK_CONF_REG;
    sys_conf &= ~PCR_SOC_CLK_SEL_Msk;
    sys_conf |= PCR_SOC_CLK_PLL_F80M;   /* select PLL path */
    PCR_SYSCLK_CONF_REG = sys_conf;
#endif
    /* Default (no define): leave at whatever speed ROM bootloader set.
     * SystemCoreClockUpdate() will measure / read the actual frequency.  */

    /* 3 ── Enable SYSTIMER so idelay.h works from this point on ---------- */
    EnableSysTimer();

    /* 4 ── Set mtvec to CLIC vectored mode --------------------------------
     * Vectors_esp32c6.c places g_Esp32C6Vectors in .iram.text at a
     * 256-byte-aligned address.  CLIC vectored mode: mtvec[1:0] = 0b11.
     */
    extern void (* const g_Esp32C6Vectors[])(void);
    uintptr_t vt = (uintptr_t)g_Esp32C6Vectors;
    /* mtvec = table_address | 0x3  (CLIC vectored) */
    __asm volatile("csrw mtvec, %0" : : "r"(vt | 0x3U) : "memory");
}

/**
 * @brief   Update SystemCoreClock from hardware register state.
 *
 * Called by ResetEntry.c after SystemInit() and again whenever the
 * application changes the CPU clock.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t sel = (PCR_SYSCLK_CONF_REG & PCR_SOC_CLK_SEL_Msk) >> PCR_SOC_CLK_SEL_Pos;

    switch (sel)
    {
        case 0U: /* XTAL — typically 40 MHz */
            SystemCoreClock = g_McuOsc.CoreOsc.Freq;
            break;

        case 1U: /* PLL path — read actual divider */
        {
            uint32_t div_n = (PCR_CPU_FREQ_CONF_REG & PCR_CPU_HS_DIV_NUM_Msk)
                              >> PCR_CPU_HS_DIV_NUM_Pos;
            /* SPLL output = 480 MHz, divider = div_n + 1 */
            SystemCoreClock = 480000000UL / (div_n + 1U);
            break;
        }

        case 2U: /* FOSC — internal fast RC ~17.5 MHz */
            SystemCoreClock = 17500000UL;
            break;

        default:
            SystemCoreClock = 80000000UL;   /* safe default */
            break;
    }
}

/*---------------------------------------------------------------------------
 * IOsonata system_core_clock.h weak implementations
 *---------------------------------------------------------------------------*/

uint32_t SystemCoreClockGet(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
    (void)Idx;
    /* On ESP32-C6 most peripherals are clocked from APB = CPU_CLK.
     * Drivers that need a specific peripheral clock should read their
     * PCR_<PERIPH>_SCLK_CONF register directly.                       */
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
    return false;   /* not yet implemented */
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
    (void)ClkSrc;
    (void)OscFreq;
    return false;   /* not yet implemented */
}

void SystemOscInit(void)
{
    /* XTAL oscillator is already running — nothing to do for default config */
}

void * const SystemRamStart(void)
{
    extern unsigned long __data_start__;
    return (void *)&__data_start__;
}
