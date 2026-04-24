/**-------------------------------------------------------------------------
@file	system_r9a02.c

@brief	R9A02G021 system initialization

Implements SystemInit(), SystemCoreClockUpdate(), and all IOsonata
system_core_clock.h stubs for the Renesas R9A02G021 RISC-V MCU.

Clock architecture:
  HOCO  : High-speed On-Chip Oscillator  — 24 / 32 / 48 MHz (OFS1 selectable)
  MOCO  : Middle-speed On-Chip Oscillator — 8 MHz (fixed)
  LOCO  : Low-speed On-Chip Oscillator    — 32.768 kHz (fixed)
  MOSC  : External clock input
  SOSC  : Sub-clock (32.768 kHz crystal)

Default: HOCO at 24 MHz, ICLK divider = 1 (no division).

To select a different HOCO frequency, define R9A02_HOCO_HZ before including
this translation unit or in the project preprocessor settings:
  #define R9A02_HOCO_HZ  48000000UL   // 24, 32, or 48 MHz

To use an external clock input (MOSC), define R9A02_MOSC_HZ:
  #define R9A02_MOSC_HZ  12000000UL

ICLK divider: define R9A02_ICLK_DIV (power-of-2, 1-64, default 1):
  #define R9A02_ICLK_DIV  2           // ICLK = source / 2

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2025

@license

MIT

Copyright (c) 2025, I-SYST inc., all rights reserved

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
 * R_SYSC - System Control register block  (base 0x4001E000)
 *---------------------------------------------------------------------------*/
#define R_SYSC_BASE             0x4001E000UL

#define SYSC_SCKDIVCR           (*(volatile uint32_t *)(R_SYSC_BASE + 0x020U))
#define SYSC_SCKSCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x026U))
#define SYSC_MEMWAIT            (*(volatile uint8_t  *)(R_SYSC_BASE + 0x031U))
#define SYSC_MOSCCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x032U))
#define SYSC_HOCOCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x036U))
#define SYSC_HOCOCR2            (*(volatile uint8_t  *)(R_SYSC_BASE + 0x037U))
#define SYSC_MOCOCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x038U))
#define SYSC_OSCSF              (*(volatile uint8_t  *)(R_SYSC_BASE + 0x03CU))
#define SYSC_OPCCR              (*(volatile uint8_t  *)(R_SYSC_BASE + 0x0A0U))
#define SYSC_SOPCCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x0AAU))
#define SYSC_PRCR               (*(volatile uint16_t *)(R_SYSC_BASE + 0x3FEU))
#define SYSC_SOSCCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x480U))
#define SYSC_LOCOCR             (*(volatile uint8_t  *)(R_SYSC_BASE + 0x490U))

/* SCKSCR clock source select values */
#define CKSEL_HOCO              0x00U
#define CKSEL_MOCO              0x01U
#define CKSEL_LOCO              0x02U
#define CKSEL_MOSC              0x03U
#define CKSEL_SOSC              0x04U

/* SCKDIVCR: ICK[26:24] = ICLK divider = 2^ICK, PCKB[10:8] = PCLKB divider */
#define SCKDIVCR_ICK_Pos        24U
#define SCKDIVCR_ICK_Msk        (0x7UL << SCKDIVCR_ICK_Pos)
#define SCKDIVCR_PCKB_Pos       8U
#define SCKDIVCR_DEFAULT        0x4U    /* reserved bits must be written as 4 */

/* HOCOCR: bit 0 = HCSTP (1=stopped, 0=running) */
#define HOCOCR_HCSTP            0x01U
/* HOCOCR2: bits [5:3] = HCFRQ1 */
#define HOCOCR2_HCFRQ1_Pos      3U
#define HOCOCR2_24MHZ           (0x0U << HOCOCR2_HCFRQ1_Pos)
#define HOCOCR2_32MHZ           (0x2U << HOCOCR2_HCFRQ1_Pos)
#define HOCOCR2_48MHZ           (0x4U << HOCOCR2_HCFRQ1_Pos)

/* OSCSF: bit 0 = HOCOSF (1=HOCO stable) */
#define OSCSF_HOCOSF            0x01U

/* OPCCR operating modes */
#define OPCCR_HIGH_SPEED        0x00U

/* Flash wait state thresholds */
#define MEMWAIT_FREQ_THRESHOLD  32000000UL

/* PRCR unlock key and bit positions */
#define PRCR_KEY                0xA500U
#define PRCR_PRC0               (1U << 0)
#define PRCR_PRC1               (1U << 1)
#define PRCR_PRC3               (1U << 3)
#define PRCR_LOCK               0xA500U

/*---------------------------------------------------------------------------
 * R_FLCN - Flash Control  (base 0x407EC000)
 * FLDWAITR: 0=1 wait (<=32MHz), 1=2 waits (>32MHz)
 *---------------------------------------------------------------------------*/
#define R_FLCN_BASE             0x407EC000UL
#define FLCN_FLDWAITR           (*(volatile uint8_t *)(R_FLCN_BASE + 0x3FC4U))

/*---------------------------------------------------------------------------
 * Fixed internal oscillator frequencies
 *---------------------------------------------------------------------------*/
#define R9A02_MOCO_HZ           8000000UL
#define R9A02_LOCO_HZ           32768UL
#define R9A02_SOSC_HZ           32768UL

/*---------------------------------------------------------------------------
 * User-configurable defaults (override via project preprocessor defines)
 *---------------------------------------------------------------------------*/
#ifndef R9A02_HOCO_HZ
#define R9A02_HOCO_HZ           24000000UL
#endif

#ifndef R9A02_MOSC_HZ
#define R9A02_MOSC_HZ           0UL
#endif

#ifndef R9A02_ICLK_DIV
#define R9A02_ICLK_DIV          1U
#endif

/* Resolve HOCOCR2 frequency bits from R9A02_HOCO_HZ */
#if   R9A02_HOCO_HZ == 48000000UL
  #define HOCOCR2_FREQ_BITS     HOCOCR2_48MHZ
#elif R9A02_HOCO_HZ == 32000000UL
  #define HOCOCR2_FREQ_BITS     HOCOCR2_32MHZ
#else
  #define HOCOCR2_FREQ_BITS     HOCOCR2_24MHZ
  #undef  R9A02_HOCO_HZ
  #define R9A02_HOCO_HZ         24000000UL
#endif

/* Resolve ICLK log2 divider field from R9A02_ICLK_DIV */
#if   R9A02_ICLK_DIV >= 64
  #define ICLK_DIV_BITS         6U
#elif R9A02_ICLK_DIV >= 32
  #define ICLK_DIV_BITS         5U
#elif R9A02_ICLK_DIV >= 16
  #define ICLK_DIV_BITS         4U
#elif R9A02_ICLK_DIV >= 8
  #define ICLK_DIV_BITS         3U
#elif R9A02_ICLK_DIV >= 4
  #define ICLK_DIV_BITS         2U
#elif R9A02_ICLK_DIV >= 2
  #define ICLK_DIV_BITS         1U
#else
  #define ICLK_DIV_BITS         0U
#endif

/*---------------------------------------------------------------------------
 * IOsonata global clock state
 * Initialised to MOCO (8 MHz) safe defaults; overwritten by SystemInit().
 *---------------------------------------------------------------------------*/
uint32_t SystemCoreClock         __attribute__((used)) = R9A02_MOCO_HZ;

/* idelay.h rdcycle path : Periodus = cycles/us, Periodns = ns/cycle        */
/* idelay.h NOP loop path: Periodus = CPU_MHz/2, Periodns = CPU_MHz/2000    */
uint64_t SystemCoreClockPeriodns __attribute__((used)) = 125ULL;   /* 8 MHz */
uint64_t SystemCoreClockPeriodus __attribute__((used)) = 8ULL;     /* 8 MHz */

/*---------------------------------------------------------------------------
 * Default oscillator descriptor — override g_McuOsc in application.
 *---------------------------------------------------------------------------*/
__attribute__((weak))
McuOsc_t g_McuOsc = {
    .CoreOsc = {
        .Type     = OSC_TYPE_RC,
        .Freq     = R9A02_HOCO_HZ,
        .Accuracy = 1000,   /* HOCO: +-1% typical */
        .LoadCap  = 0,
    },
    .LowPwrOsc = {
        .Type     = OSC_TYPE_RC,
        .Freq     = R9A02_LOCO_HZ,
        .Accuracy = 15000,  /* LOCO: +-15% typical */
        .LoadCap  = 0,
    },
    .bUSBClk = false,
};

/*---------------------------------------------------------------------------
 * Internal helpers
 *---------------------------------------------------------------------------*/

static void SetFlashWait(uint32_t FreqHz)
{
    if (FreqHz > MEMWAIT_FREQ_THRESHOLD)
    {
        SYSC_MEMWAIT  = 0x01U;  /* 1 wait cycle */
        FLCN_FLDWAITR = 0x01U;  /* 2 wait cycles */
    }
    else
    {
        SYSC_MEMWAIT  = 0x00U;
        FLCN_FLDWAITR = 0x00U;
    }
}

static void SetOperatingMode(uint8_t Mode)
{
    /* Exit sub-oscillator speed mode first if active */
    if (SYSC_SOPCCR & 0x1U)
    {
        while (SYSC_SOPCCR & 0x10U) {}     /* wait SOPCMTSF = 0 */
        SYSC_SOPCCR = 0x00U;
        while (SYSC_SOPCCR) {}
    }

    if (SYSC_OPCCR != Mode)
    {
        /* HOCO must be stable before changing OPCCR */
        if (SYSC_HOCOCR & HOCOCR_HCSTP)
        {
            SYSC_HOCOCR = 0x00U;
            while (!(SYSC_OSCSF & OSCSF_HOCOSF)) {}
        }
        while (SYSC_OPCCR & 0x10U) {}      /* wait OPCMTSF = 0 */
        SYSC_OPCCR = Mode;
        while (SYSC_OPCCR & 0x10U) {}
    }
}

/*---------------------------------------------------------------------------
 * SystemCoreClockUpdate - read hardware and recompute globals.
 *---------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    uint32_t src_freq;
    uint8_t  cksel = SYSC_SCKSCR & 0x07U;

    switch (cksel)
    {
        case CKSEL_HOCO:
        {
            uint8_t hcfrq = (SYSC_HOCOCR2 >> HOCOCR2_HCFRQ1_Pos) & 0x7U;
            switch (hcfrq)
            {
                case 4U:  src_freq = 48000000UL; break;
                case 2U:  src_freq = 32000000UL; break;
                default:  src_freq = 24000000UL; break;
            }
            break;
        }
        case CKSEL_MOCO:
            src_freq = R9A02_MOCO_HZ;
            break;

        case CKSEL_LOCO:
            src_freq = R9A02_LOCO_HZ;
            break;

        case CKSEL_MOSC:
#if R9A02_MOSC_HZ != 0UL
            src_freq = R9A02_MOSC_HZ;
#else
            src_freq = g_McuOsc.CoreOsc.Freq;
#endif
            break;

        case CKSEL_SOSC:
            src_freq = R9A02_SOSC_HZ;
            break;

        default:
            src_freq = R9A02_MOCO_HZ;
            break;
    }

    /* Apply ICLK divider: ICK bits [26:24] = log2(divisor) */
    uint32_t ick = (SYSC_SCKDIVCR & SCKDIVCR_ICK_Msk) >> SCKDIVCR_ICK_Pos;
    SystemCoreClock = src_freq >> ick;

    /* Update idelay period variables */
    SystemCoreClockPeriodus = (SystemCoreClock + 500000U) / 1000000U;
    SystemCoreClockPeriodns = (1000000000ULL + (uint64_t)(SystemCoreClock / 2))
                              / SystemCoreClock;
}

/*---------------------------------------------------------------------------
 * SystemInit
 *
 * Clock switching sequence (per R9A02G021 HW manual):
 *   1. Unlock PRCR
 *   2. Enter high-speed operating mode
 *   3. Set flash wait states for target frequency
 *   4. Configure HOCO frequency and start oscillator
 *   5. Wait for HOCO stabilization
 *   6. Switch SCKSCR to HOCO
 *   7. Apply ICLK / PCLKB divider
 *   8. Stop unused MOCO
 *   9. Lock PRCR
 *  10. Update SystemCoreClock and idelay period variables
 *---------------------------------------------------------------------------*/
void SystemInit(void)
{
    /* 1. Unlock clock generation and low-power mode registers */
    SYSC_PRCR = PRCR_KEY | PRCR_PRC0 | PRCR_PRC1 | PRCR_PRC3;

    /* 2. Enter high-speed operating mode before raising frequency */
    SetOperatingMode(OPCCR_HIGH_SPEED);

    /* 3. Configure flash wait states for the target ICLK frequency */
    SetFlashWait(R9A02_HOCO_HZ / R9A02_ICLK_DIV);

    /* 4. Configure and start HOCO */
    if (SYSC_HOCOCR & HOCOCR_HCSTP)
    {
        /* Set frequency before starting */
        SYSC_HOCOCR2 = HOCOCR2_FREQ_BITS;
        SYSC_HOCOCR  = 0x00U;              /* start HOCO */
    }

    /* 5. Wait for HOCO stabilization flag */
    while (!(SYSC_OSCSF & OSCSF_HOCOSF)) {}

    /* 6. Switch system clock source to HOCO */
    SYSC_SCKSCR = CKSEL_HOCO;

    /* 7. Set ICLK and PCLKB dividers */
    SYSC_SCKDIVCR = ((uint32_t)(ICLK_DIV_BITS) << SCKDIVCR_ICK_Pos)
                  | ((uint32_t)(ICLK_DIV_BITS) << SCKDIVCR_PCKB_Pos)
                  | SCKDIVCR_DEFAULT;

    /* 8. Stop MOCO (not needed when running from HOCO) */
    SYSC_MOCOCR = 0x01U;

    /* 9. Re-lock protection register */
    SYSC_PRCR = PRCR_LOCK;

    /* 10. Update global clock state */
    SystemCoreClockUpdate();
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
    return SystemCoreClock;     /* PCLKB = ICLK in default config */
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
