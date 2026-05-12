/**-------------------------------------------------------------------------
@file	iopincfg_r9a02.c

@brief	I/O pin configuration for Renesas R9A02G021 (RISC-V Andes N22).

Implements the IOsonata coredev/iopincfg.h interface against the
RA-family IOPORT/PFS hardware.  See r9a02g021.h for the PFS field
positions and the PWPR unlock sequence.

PFS write protect sequence used by every function that touches PmnPFS:

   PMISC->PWPR = 0x00;     // clear B0WI (PFSWE-write disable)
   PMISC->PWPR = 0x40;     // set PFSWE
   ... write any PmnPFS register ...
   PMISC->PWPR = 0x00;     // clear PFSWE again
   PMISC->PWPR = 0x80;     // set B0WI to relock against accidental PFS writes

The two-step relock at the end is intentional: writing 0x80 with PFSWE
already cleared is what RA-family HW manuals describe as the recommended
"return to protected" state.

Scope of this initial bring-up:
   - IOPinConfig   : full PFS configuration (GPIO / peripheral / analog,
                     pull-up, open-drain, direction).
   - IOPinDisable  : clears the PFS, returning the pin to its reset state.
   - IOPinSetStrength / IOPinSetSpeed : drive-strength control via PFS.DSCR.
   - IOPinSetSense : edge select via PFS.EOFR / PFS.ISEL (ICU routing is
                     left to the user until the R9A02 vector / ICU layer
                     is brought up).

External-interrupt allocation/dispatch (IOPinEnableInterrupt and
IOPinAllocateInterrupt) requires the R9A02 ICU IELSR/IRQCR programming
which is not part of IOPORT.  Those functions currently return false /
-1 with a TODO marker; once the R9A02 vectors / interrupt_r9a02.c
layer lands the ICU side can be wired in here without touching the
PFS code below.

@author	Hoang Nguyen Hoan
@date	May. 11, 2026

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
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/iopincfg.h"

/*---------------------------------------------------------------------------
 * Internal helpers
 *--------------------------------------------------------------------------*/

static inline bool R9A02IsValidPortPin(int PortNo, int PinNo)
{
    return (PortNo >= 0)
        && ((uint32_t)PortNo < R9A02_MAX_PORT)
        && (PinNo >= 0)
        && ((uint32_t)PinNo < R9A02_PINS_PER_PORT);
}

static inline void R9A02PfsUnlock(void)
{
    /* B0WI must be cleared first; otherwise the write that sets PFSWE
     * is silently dropped.  These two writes do *not* fold into one
     * because PWPR latches B0WI/PFSWE independently per write. */
    PMISC->PWPR = 0U;
    PMISC->PWPR = R9A02_PMISC_PWPR_PFSWE_Msk;       /* PFSWE = 1, B0WI = 0 */
}

static inline void R9A02PfsLock(void)
{
    PMISC->PWPR = 0U;                                /* PFSWE = 0          */
    PMISC->PWPR = R9A02_PMISC_PWPR_B0WI_Msk;         /* B0WI  = 1 (relock) */
}

static inline void R9A02PfsWrite(int PortNo, int PinNo, uint32_t Value)
{
    *R9A02_PFS_REG_ADDR(PortNo, PinNo) = Value;
}

static inline uint32_t R9A02PfsRead(int PortNo, int PinNo)
{
    return *R9A02_PFS_REG_ADDR(PortNo, PinNo);
}

/*---------------------------------------------------------------------------
 * Drive-strength helpers (PFS.DSCR field is 2 bits, 0..3 = low..high).
 *
 * The RA-family hardware manual lists three classes of pad: standard,
 * 5 V tolerant, and high-current.  Not every pin honours every DSCR
 * value -- pins without a high-drive option silently latch the lower
 * value.  Mapping IOPINSTRENGTH_STRONG to the maximum field value
 * is the simplest portable choice and is what FSP also does.
 *--------------------------------------------------------------------------*/
static inline uint32_t R9A02DscrFromStrength(IOPINSTRENGTH Strength)
{
    return (Strength == IOPINSTRENGTH_STRONG) ? 3UL : 0UL;
}

static inline uint32_t R9A02DscrFromSpeed(IOPINSPEED Speed)
{
    switch (Speed)
    {
        case IOPINSPEED_LOW:        return 0UL;
        case IOPINSPEED_MEDIUM:     return 1UL;
        case IOPINSPEED_HIGH:       return 2UL;
        case IOPINSPEED_TURBO:      return 3UL;
        default:                    return 0UL;
    }
}

static uint32_t R9A02EofrFromSense(IOPINSENSE Sense)
{
    switch (Sense)
    {
        case IOPINSENSE_LOW_TRANSITION:     return R9A02_PFS_EOFR_FALLING;
        case IOPINSENSE_HIGH_TRANSITION:    return R9A02_PFS_EOFR_RISING;
        case IOPINSENSE_TOGGLE:             return R9A02_PFS_EOFR_BOTH;
        case IOPINSENSE_DISABLE:
        default:                            return 0UL;
    }
}

/*---------------------------------------------------------------------------
 * IOPinConfig
 *
 * PinOp interpretation on R9A02G021:
 *
 *   IOPINOP_GPIO  (0)         plain GPIO, PMR = 0, PSEL not used.
 *   IOPINOP_FUNCn (1..31)     peripheral function: PMR = 1,
 *                             PSEL[28:24] = (n - 1).
 *                             E.g. UART0 RXD on most parts is PSEL = 4 ->
 *                             use IOPINOP_FUNC5 (since IOPINOP_FUNCn maps
 *                             to PSEL = n - 1).
 *   IOPINOP_FUNC31            interpreted as analog input (ASEL = 1).
 *                             Note: the RA-family also offers ASEL for
 *                             analog comparator pads regardless of PSEL.
 *
 * Order of register writes follows the RA-family bring-up convention:
 *   1. PWPR unlock.
 *   2. Single 32-bit PFS write with everything composed in software.
 *      Writing the entire PFS in one transaction (vs. RMW) avoids
 *      intermediate states where the pin briefly drives garbage onto
 *      a peripheral pad.
 *   3. PWPR relock.
 *--------------------------------------------------------------------------*/
void IOPinConfig(int PortNo, int PinNo, int PinOp,
                 IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
    if (!R9A02IsValidPortPin(PortNo, PinNo))
    {
        return;
    }

    uint32_t pfs = 0U;

    /* Function selection. */
    if (PinOp == IOPINOP_GPIO)
    {
        /* Plain GPIO: PMR = 0.  Direction set below. */
    }
    else if (PinOp == IOPINOP_FUNC31)
    {
        /* Analog: ASEL = 1, no PMR.  Direction/pull bits left at 0,
         * matching how FSP configures analog pads. */
        pfs |= R9A02_PFS_ASEL_Msk;
        Resistor = IOPINRES_NONE;       /* pull-up nonsensical on analog  */
        Dir      = IOPINDIR_INPUT;
    }
    else if ((PinOp >= IOPINOP_FUNC0) && (PinOp < IOPINOP_FUNC31))
    {
        /* Peripheral function index n maps to PSEL = (n - IOPINOP_FUNC0). */
        uint32_t psel = (uint32_t)(PinOp - IOPINOP_FUNC0);
        pfs |= R9A02_PFS_PMR_Msk
             | ((psel << R9A02_PFS_PSEL_Pos) & R9A02_PFS_PSEL_Msk);
    }
    else
    {
        return;     /* unknown PinOp -- leave pin untouched */
    }

    /* Direction.  Renesas treats "direction = output" as PDR = 1.  For
     * peripheral mode (PMR = 1) PDR is overridden by the peripheral,
     * so setting it here is harmless. */
    if ((Dir == IOPINDIR_OUTPUT) || (Dir == IOPINDIR_BI))
    {
        pfs |= R9A02_PFS_PDR_Msk;
    }

    /* Pull resistor.  R9A02G021 only has a pull-up control bit (PCR);
     * pull-down is not supported.  Per the RA family rules the pull-up
     * is only effective when the pin is configured as input. */
    if ((Resistor == IOPINRES_PULLUP) || (Resistor == IOPINRES_FOLLOW))
    {
        pfs |= R9A02_PFS_PCR_Msk;
    }
    else if (Resistor == IOPINRES_PULLDOWN)
    {
        /* No hardware pull-down on R9A02G021.  Caller intent is honoured
         * by leaving PCR clear; we do not error out so portable code that
         * specifies PULLDOWN still gets a functional (resistor-less) pin
         * instead of being silently disabled. */
    }

    /* Output type.  Renesas calls N-channel open drain "NCODR".  Used
     * only when the pin is actively driving (output / bidirectional);
     * harmless on inputs.  Pure P-channel open drain is not supported
     * on R9A02G021. */
    if (Type == IOPINTYPE_OPENDRAIN)
    {
        pfs |= R9A02_PFS_NCODR_Msk;
    }

    /* Default drive strength: medium (1).  IOPinSetStrength / IOPinSetSpeed
     * lets the application override later without touching the rest of the
     * PFS. */
    pfs |= (1UL << R9A02_PFS_DSCR_Pos);

    R9A02PfsUnlock();
    R9A02PfsWrite(PortNo, PinNo, pfs);
    R9A02PfsLock();
}

/*---------------------------------------------------------------------------
 * IOPinDisable - return the pin to its (mostly) reset state.
 *
 * Writing 0 to PmnPFS clears PMR/ASEL/ISEL/PSEL/PDR/PCR/NCODR/DSCR, which
 * puts the pin back into input-mode GPIO with no pulls and minimum drive.
 * That matches the IOsonata "disable" contract (low-power, disconnected).
 *--------------------------------------------------------------------------*/
void IOPinDisable(int PortNo, int PinNo)
{
    if (!R9A02IsValidPortPin(PortNo, PinNo))
    {
        return;
    }

    R9A02PfsUnlock();
    R9A02PfsWrite(PortNo, PinNo, 0U);
    R9A02PfsLock();
}

/*---------------------------------------------------------------------------
 * IOPinSetSense - program PFS.EOFR + ISEL.
 *
 * This sets up the edge-detect circuit on the pad but does NOT touch the
 * R9A02 ICU.  Routing the resulting event into an NVIC/CLIC slot is the
 * responsibility of the (not-yet-implemented) R9A02 interrupt layer.
 *--------------------------------------------------------------------------*/
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
    if (!R9A02IsValidPortPin(PortNo, PinNo))
    {
        return;
    }

    uint32_t pfs = R9A02PfsRead(PortNo, PinNo);
    pfs &= ~(R9A02_PFS_EOFR_Msk | R9A02_PFS_ISEL_Msk);

    if (Sense != IOPINSENSE_DISABLE)
    {
        pfs |= R9A02EofrFromSense(Sense) | R9A02_PFS_ISEL_Msk;
    }

    R9A02PfsUnlock();
    R9A02PfsWrite(PortNo, PinNo, pfs);
    R9A02PfsLock();
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
    if (!R9A02IsValidPortPin(PortNo, PinNo))
    {
        return;
    }

    uint32_t pfs = R9A02PfsRead(PortNo, PinNo);
    pfs &= ~R9A02_PFS_DSCR_Msk;
    pfs |= (R9A02DscrFromStrength(Strength) << R9A02_PFS_DSCR_Pos);

    R9A02PfsUnlock();
    R9A02PfsWrite(PortNo, PinNo, pfs);
    R9A02PfsLock();
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
    if (!R9A02IsValidPortPin(PortNo, PinNo))
    {
        return;
    }

    uint32_t pfs = R9A02PfsRead(PortNo, PinNo);
    pfs &= ~R9A02_PFS_DSCR_Msk;
    pfs |= (R9A02DscrFromSpeed(Speed) << R9A02_PFS_DSCR_Pos);

    R9A02PfsUnlock();
    R9A02PfsWrite(PortNo, PinNo, pfs);
    R9A02PfsLock();
}

/*---------------------------------------------------------------------------
 * External pin interrupt allocation / dispatch.
 *
 * R9A02G021 routes pad edge events through the ICU IELSR table to a
 * CLIC interrupt.  Allocating an ICU slot and an interrupt handler
 * requires the R9A02 vector / interrupt management layer, which is
 * out of scope for the IOPORT driver and not yet present in this port.
 *
 * The functions below are wired to PFS.EOFR / ISEL (via IOPinSetSense)
 * so the pad-level edge-detect path is functional once the ICU side
 * is in place.  Until then, they return false / -1 to make sure
 * callers cannot assume an interrupt has been hooked.
 *
 * TODO(R9A02 ICU): implement Re9a02RegisterIntHandler equivalent and
 *                  wire IOPinEnableInterrupt to allocate an IELSR slot.
 *--------------------------------------------------------------------------*/
void IOPinDisableInterrupt(int IntNo)
{
    (void)IntNo;
    /* No-op: nothing has been enabled yet because IOPinEnableInterrupt
     * currently returns false.  Once the ICU layer is in place this
     * will free the corresponding IELSR slot. */
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio,
                          uint32_t PortNo, uint32_t PinNo,
                          IOPINSENSE Sense,
                          IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    (void)IntNo;
    (void)IntPrio;
    (void)pEvtCB;
    (void)pCtx;

    /* Program the pad-level edge detect so wake-from-sleep via DEEPCUT
     * still works even before the ICU side is finalized. */
    if (R9A02IsValidPortPin((int)PortNo, (int)PinNo))
    {
        IOPinSetSense((int)PortNo, (int)PinNo, Sense);
    }

    return false;       /* no full IRQ wiring yet */
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo,
                           IOPINSENSE Sense,
                           IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    (void)IntPrio;
    (void)pEvtCB;
    (void)pCtx;

    if (R9A02IsValidPortPin(PortNo, PinNo))
    {
        IOPinSetSense(PortNo, PinNo, Sense);
    }

    return -1;          /* no ICU slot allocated */
}
