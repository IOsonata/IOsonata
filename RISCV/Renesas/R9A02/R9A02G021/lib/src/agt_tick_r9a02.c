/**-------------------------------------------------------------------------
@file	agt_tick_r9a02.c

@brief	AGT0-based 1 ms system tick implementation.

See agt_tick_r9a02.h for the API description.

AGT0 register layout (16-bit at 0x40084000):
    AGT     +0x00  16-bit  Counter / reload  (writeable)
    AGTCMA  +0x02  16-bit  Compare match A   (unused here)
    AGTCMB  +0x04  16-bit  Compare match B   (unused here)
    AGTCR   +0x08   8-bit  Control     -- TSTART=bit0, TCSTF=bit1
    AGTMR1  +0x09   8-bit  Mode 1      -- TCK[2:0] clock source select
    AGTMR2  +0x0A   8-bit  Mode 2      -- LPM, dividers
    AGTIOC  +0x0B   8-bit  IO control
    AGTISR  +0x0C   8-bit  Event pin select
    AGTCMSR +0x0D   8-bit  Compare match function select
    AGTIOSEL+0x0E   8-bit  Pin output select

@author	Nguyen Hoan Hoang
@date	May 12, 2026

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
#include "agt_tick_r9a02.h"
#include "r9a02g021.h"
#include "mstpcr_r9a02.h"
#include "coredev/system_core_clock.h"

/*===========================================================================
 * AGT0 register block (R9A02G021 uses RA2-family layout)
 *==========================================================================*/
#define AGT0_BASE                   R9A02_AGT0_BASE

#define AGT0_AGT                    (*(volatile uint16_t *)(AGT0_BASE + 0x00U))
#define AGT0_AGTCMA                 (*(volatile uint16_t *)(AGT0_BASE + 0x02U))
#define AGT0_AGTCMB                 (*(volatile uint16_t *)(AGT0_BASE + 0x04U))
#define AGT0_AGTCR                  (*(volatile uint8_t  *)(AGT0_BASE + 0x08U))
#define AGT0_AGTMR1                 (*(volatile uint8_t  *)(AGT0_BASE + 0x09U))
#define AGT0_AGTMR2                 (*(volatile uint8_t  *)(AGT0_BASE + 0x0AU))
#define AGT0_AGTIOC                 (*(volatile uint8_t  *)(AGT0_BASE + 0x0BU))
#define AGT0_AGTISR                 (*(volatile uint8_t  *)(AGT0_BASE + 0x0CU))
#define AGT0_AGTCMSR                (*(volatile uint8_t  *)(AGT0_BASE + 0x0DU))
#define AGT0_AGTIOSEL               (*(volatile uint8_t  *)(AGT0_BASE + 0x0EU))

/* AGTCR bit positions */
#define AGTCR_TSTART                (1U << 0)   //!< 1 = start counter
#define AGTCR_TCSTF                 (1U << 1)   //!< 1 = counter running (RO)
#define AGTCR_TSTOP                 (1U << 2)   //!< W1 = stop counter
#define AGTCR_TEDGF                 (1U << 4)   //!< Edge / underflow flag
#define AGTCR_TUNDF                 (1U << 5)   //!< Underflow flag (W1C)

/* AGTMR1 -- mode 1.  TCK[2:0] selects clock source:
 *   000 = PCLKB         (default)
 *   001 = PCLKB / 8
 *   011 = PCLKB / 2
 *   100 = AGTLCLK       (sub-clock)
 *   101 = AGT0LCLK      (LOCO clocked)
 *   110 = AGTX1 underflow */
#define AGTMR1_TMOD_TIMER           0U
#define AGTMR1_TCK_PCLKB            (0U << 4)
#define AGTMR1_TCK_PCLKB_DIV8       (1U << 4)
#define AGTMR1_TCK_PCLKB_DIV2       (3U << 4)

/*===========================================================================
 * State
 *==========================================================================*/
static volatile uint64_t s_MsTicks = 0U;

/*===========================================================================
 * Public API
 *==========================================================================*/

bool R9A02_AgtTickInit(void)
{
    uint32_t pclkb = SystemCoreClockGet();      /* PCLKB == ICLK by default */
    if (pclkb == 0U)
    {
        return false;
    }

    /* Reload for 1 ms period using PCLKB/8.  Verify it fits in 16 bits. */
    uint32_t reload = (pclkb / 8U) / 1000U;
    if (reload == 0U || reload > 0x10000U)
    {
        return false;
    }
    reload -= 1U;                               /* AGT counts N+1 cycles */

    /* 1. Bring AGT0 out of module-stop. */
    R9A02_ModuleStart(R9A02_MOD_AGT0);

    /* 2. Stop AGT0 if running, wait for TCSTF clear. */
    AGT0_AGTCR = AGTCR_TSTOP;
    while ((AGT0_AGTCR & AGTCR_TCSTF) != 0U)
    {
        /* spin */
    }

    /* 3. Configure mode 1: timer mode, PCLKB/8 source. */
    AGT0_AGTMR1 = AGTMR1_TMOD_TIMER | AGTMR1_TCK_PCLKB_DIV8;
    AGT0_AGTMR2 = 0U;
    AGT0_AGTIOC = 0U;
    AGT0_AGTISR = 0U;
    AGT0_AGTCMSR = 0U;
    AGT0_AGTIOSEL = 0U;

    /* 4. Set reload value. */
    AGT0_AGT = (uint16_t)reload;

    /* 5. Clear any stale flags, then start. */
    AGT0_AGTCR = AGTCR_TUNDF;                   /* W1C */
    AGT0_AGTCR = AGTCR_TSTART;
    while ((AGT0_AGTCR & AGTCR_TCSTF) == 0U)
    {
        /* wait for running confirmation */
    }
    return true;
}

void R9A02_AgtTickIsr(void)
{
    /* Acknowledge underflow flag in AGTCR. */
    AGT0_AGTCR = AGTCR_TUNDF;

    /* Clear the IELSR.IR bit for the slot that fired -- the application
     * is responsible for knowing which slot was wired and clearing it
     * there if the CLIC convention requires.  On Andes N22 the IR flag
     * needs explicit clear by writing 0 to IELSR[n].IR; this is done by
     * the per-slot handler shell, not here. */

    s_MsTicks++;
}

uint64_t R9A02_GetMsTickCount(void)
{
    return s_MsTicks;
}
