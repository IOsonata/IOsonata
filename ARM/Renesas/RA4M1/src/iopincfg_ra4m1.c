/**-------------------------------------------------------------------------
@file	iopincfg_ra4m1.c

@brief	RA4M1 pin configuration and fixed-line GPIO interrupts.

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
#include <stddef.h>
#include "iopinctrl.h"
#include "interrupt_ra4m1.h"
#include "ra4m1_startup_regs.h"

#define RA4M1_VBTCR1    0x4001E41FUL
#define RA4M1_VBTSR     0x4001E4B1UL
#define RA4M1_VBTICTLR  0x4001E4BBUL
#define RA4M1_VBTOCTLR  0x4001E4BCUL
#ifndef RA4M1_IO_READY_POLLS
#define RA4M1_IO_READY_POLLS 1000000UL
#endif

/* Fixed hardware pin-to-IRQ map: manual section 1.7 / tables 19.6..19.13.
 * Packed (IRQ << 8) | (port << 4) | pin. No arbitrary board wiring here.
 */
static const uint16_t s_PinIrqMap[] = {
    0x000 | 0x15, 0x000 | 0x26, 0x000 | 0x40,
    0x100 | 0x11, 0x100 | 0x14, 0x100 | 0x25,
    0x200 | 0x02, 0x200 | 0x10, 0x200 | 0x2D,
    0x300 | 0x04, 0x300 | 0x1A, 0x300 | 0x2C,
    0x400 | 0x1B, 0x400 | 0x42, 0x400 | 0x4B,
    0x500 | 0x32, 0x500 | 0x41, 0x500 | 0x4A,
    0x600 | 0x00, 0x600 | 0x31, 0x600 | 0x49,
    0x700 | 0x01, 0x700 | 0x0F, 0x700 | 0x48,
    0x800 | 0x35, 0x800 | 0x4F,
    0x900 | 0x34, 0x900 | 0x4E,
    0xA00 | 0x05, 0xB00 | 0x51, 0xC00 | 0x52,
    0xE00 | 0x55, 0xF00 | 0x0B
};

typedef struct {
    IOPinEvtHandler_t Handler;
    void *pCtx;
    IRQn_Type Irq;
    uint8_t Pin; // port * 16 + pin; Handler is the ownership sentinel
} Ra4m1PinHook;
static Ra4m1PinHook s_PinHook[16];

static int Ra4m1PinIrq(int PortNo, int PinNo)
{
    if (!Ra4m1PinValid(PortNo, PinNo)) return -1;
    unsigned pin = (unsigned)(PortNo * 16 + PinNo);
    for (unsigned i = 0; i < sizeof(s_PinIrqMap)/sizeof(s_PinIrqMap[0]); ++i)
        if ((s_PinIrqMap[i] & 0xFFU) == pin) return s_PinIrqMap[i] >> 8;
    return -1;
}
static bool Ra4m1PinCanConfigure(int PortNo, int PinNo)
{
    if (!Ra4m1PinValid(PortNo, PinNo) || PortNo == 9) return false;
    if (PortNo == 2) {
        if ((PinNo == 12 || PinNo == 13) && !(RA4M1_RD8(RA4M1_MOSCCR) & 1U)) return false;
        if ((PinNo == 14 || PinNo == 15) && !(RA4M1_RD8(RA4M1_SOSCCR) & 1U)) return false;
    }
    if (PortNo >= 1 && PortNo <= 4) {
        uint32_t elc = RA4M1_RD32(RA4M1_PCNTR4(PortNo));
        if ((elc | (elc >> 16)) & (1UL << PinNo)) return false;
    }
    return true;
}
static bool Ra4m1AnalogPin(int PortNo, int PinNo)
{
    return PortNo == 0 || (PortNo == 1 && PinNo <= 3) || PortNo == 5;
}

/* Caller masks interrupts. Preserve PWPR, including an existing unlocked
 * nesting state, instead of unconditionally relocking another owner's scope.
 */
static bool Ra4m1PfsWrite(int PortNo, int PinNo, uint32_t Value)
{
    uint8_t pwpr = RA4M1_RD8(RA4M1_PWPR);
    RA4M1_WR8(RA4M1_PWPR, pwpr & (uint8_t)~RA4M1_PWPR_B0WI);
    RA4M1_WR8(RA4M1_PWPR, RA4M1_PWPR_PFSWE);
    uint32_t addr = RA4M1_PFS(PortNo, PinNo);
    uint32_t old = RA4M1_RD32(addr) & ~RA4M1_PFS_PIDR;
    /* Changing PSEL/ASEL must occur while PMR is zero. Preserve the current
     * latch while disconnecting the peripheral; configure the new value next.
     */
    bool muxChange = ((old ^ Value) & (RA4M1_PFS_PSEL | RA4M1_PFS_ASEL | RA4M1_PFS_PMR)) != 0;
    if (muxChange) {
        if (old & RA4M1_PFS_PMR) RA4M1_WR32(addr, old & ~RA4M1_PFS_PMR);
        RA4M1_WR32(addr, Value & ~(RA4M1_PFS_PIDR | RA4M1_PFS_PMR));
        if (Value & RA4M1_PFS_PMR) RA4M1_WR32(addr, Value & ~RA4M1_PFS_PIDR);
    } else RA4M1_WR32(addr, Value & ~RA4M1_PFS_PIDR);
    RA4M1_WR8(RA4M1_PWPR, pwpr & (uint8_t)~RA4M1_PWPR_B0WI);
    RA4M1_WR8(RA4M1_PWPR, pwpr);
    return (RA4M1_RD32(addr) & ~RA4M1_PFS_PIDR) == (Value & ~RA4M1_PFS_PIDR);
}

/* Do not reset the backup domain or commandeer retained RTC/VBATWIO pins.
 * Temporarily hold its VCC supply, qualify VBTRVLD before *any* retained
 * register read, then restore the prior power-switch/protection policy.
 */
static bool Ra4m1PinBufferReady(int PortNo, int PinNo)
{
    if (PortNo != 4 || PinNo < 2 || PinNo > 4) return true;
    uint16_t prcr = RA4M1_RD16(RA4M1_PRCR) & 0xFU;
    uint8_t vbt = RA4M1_RD8(RA4M1_VBTCR1) & 1U;
    RA4M1_WR16(RA4M1_PRCR, RA4M1_PRCR_KEY | prcr | RA4M1_PRCR_PRC1);
    RA4M1_WR8(RA4M1_VBTCR1, 1U);
    bool ready = false;
    for (uint32_t i = 0; i < RA4M1_IO_READY_POLLS; ++i) {
        if ((RA4M1_RD8(RA4M1_VBTCR1) & 1U) && (RA4M1_RD8(RA4M1_VBTSR) & 0x10U)) { ready = true; break; }
    }
    if (ready) {
        uint32_t mask = 1UL << (PinNo - 2);
        ready = ((RA4M1_RD8(RA4M1_VBTICTLR) | RA4M1_RD8(RA4M1_VBTOCTLR)) & mask) == 0U;
    }
    RA4M1_WR8(RA4M1_VBTCR1, vbt);
    RA4M1_WR16(RA4M1_PRCR, RA4M1_PRCR_KEY | prcr);
    return ready;
}

/* Reject unsupported electrical requests rather than replacing them with a
 * different pull or output topology. PSEL values are supplied by the caller
 * from the per-pin table, as in RE01; this function does not allocate muxes.
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir,
    IOPINRES Resistor, IOPINTYPE Type)
{
    if (!Ra4m1PinValid(PortNo, PinNo) || (unsigned)PinOp > IOPINOP_FUNC31 ||
        (Dir != IOPINDIR_INPUT && Dir != IOPINDIR_OUTPUT && Dir != IOPINDIR_BI) ||
        (Resistor != IOPINRES_NONE && Resistor != IOPINRES_PULLUP) ||
        (Type != IOPINTYPE_NORMAL && Type != IOPINTYPE_OPENDRAIN)) return;
    bool inputOnly = PortNo == 2 && (PinNo == 0 || PinNo >= 14);
    bool analog = PinOp == IOPINOP_FUNC31;
    if (inputOnly && (Dir != IOPINDIR_INPUT || PinOp != IOPINOP_GPIO ||
        Resistor != IOPINRES_NONE || Type != IOPINTYPE_NORMAL)) return;
    if (analog && (!Ra4m1AnalogPin(PortNo, PinNo) || Dir != IOPINDIR_INPUT ||
        Resistor != IOPINRES_NONE || Type != IOPINTYPE_NORMAL)) return;
    if (Type == IOPINTYPE_OPENDRAIN && PortNo == 0) return;
    if (PinOp != IOPINOP_GPIO && !analog) {
        /* Only PSEL function codes implemented somewhere on RA4M1. The
         * driver's pin map remains responsible for the per-pin combination.
         */
        const uint32_t functions = 0x000D37FEUL; // PSEL codes 1..10,12,13,16,18,19
        if (!(functions & (1UL << PinOp))) return;
    }
    uint32_t state = DisableInterrupt();
    if (!Ra4m1PinCanConfigure(PortNo, PinNo) || !Ra4m1PinBufferReady(PortNo, PinNo)) {
        EnableInterrupt(state); return;
    }
    uint32_t old = RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
    /* Configuration is not an implicit interrupt release. Leave an active
     * pin alone; explicit IOPinDisableInterrupt owns that teardown.
     */
    if (old & RA4M1_PFS_ISEL) { EnableInterrupt(state); return; }
    uint32_t value = inputOnly ? 0U : old & RA4M1_PFS_PODR;
    if (analog) value |= RA4M1_PFS_ASEL;
    else {
        if (Dir == IOPINDIR_OUTPUT) value |= RA4M1_PFS_PDR;
        if (PinOp != IOPINOP_GPIO) value |= ((uint32_t)PinOp << 24) | RA4M1_PFS_PMR;
        if (Resistor == IOPINRES_PULLUP) value |= RA4M1_PFS_PCR;
        if (Type == IOPINTYPE_OPENDRAIN) value |= RA4M1_PFS_NCODR;
    }
    Ra4m1PfsWrite(PortNo, PinNo, value);
    EnableInterrupt(state);
}

void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
    if (!Ra4m1PinValid(PortNo, PinNo) ||
        (Dir != IOPINDIR_INPUT && Dir != IOPINDIR_OUTPUT)) return;
    uint32_t state = DisableInterrupt();
    uint32_t value = RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
    if (!Ra4m1PinCanConfigure(PortNo, PinNo) ||
        (value & (RA4M1_PFS_PMR | RA4M1_PFS_ASEL | RA4M1_PFS_ISEL)) ||
        (Dir == IOPINDIR_OUTPUT && !(Ra4m1OutputMask(PortNo) & (1UL << PinNo))) ||
        !Ra4m1PinBufferReady(PortNo, PinNo)) { EnableInterrupt(state); return; }
    value &= ~(RA4M1_PFS_PDR | RA4M1_PFS_PIDR);
    if (Dir == IOPINDIR_OUTPUT) value |= RA4M1_PFS_PDR;
    Ra4m1PfsWrite(PortNo, PinNo, value);
    EnableInterrupt(state);
}

void IOPinDisable(int PortNo, int PinNo)
{
    if (!Ra4m1PinValid(PortNo, PinNo)) return;
    uint32_t state = DisableInterrupt();
    if (!Ra4m1PinCanConfigure(PortNo, PinNo)) { EnableInterrupt(state); return; }
    int line = Ra4m1PinIrq(PortNo, PinNo);
    if (line >= 0 && s_PinHook[line].Handler && s_PinHook[line].Pin == PortNo*16+PinNo)
        IOPinDisableInterrupt(line);
    uint32_t value = RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
    if (!(value & RA4M1_PFS_ISEL) && Ra4m1PinBufferReady(PortNo, PinNo))
        Ra4m1PfsWrite(PortNo, PinNo, value & RA4M1_PFS_PODR);
    EnableInterrupt(state);
}

static void Ra4m1PinCallback(int IrqNo, void *pCtx)
{
    uint32_t state = DisableInterrupt();
    Ra4m1PinHook *hook = (Ra4m1PinHook *)pCtx;
    /* A higher-priority ISR may have released this pin and reassigned its
     * hook before this wrapper was entered. Active CPU slots are never reused.
     */
    IOPinEvtHandler_t handler = hook->Irq == IrqNo ? hook->Handler : NULL;
    void *ctx = hook->pCtx;
    int line = (int)(hook - s_PinHook);
    EnableInterrupt(state);
    if (handler) handler(line, ctx);
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo,
    IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    if ((unsigned)IntNo >= 16U || IntNo == 13 || PortNo >= 10U || PinNo >= 16U ||
        Ra4m1PinIrq((int)PortNo, (int)PinNo) != IntNo || !pEvtCB ||
        (unsigned)IntPrio >= (1UL << __NVIC_PRIO_BITS) ||
        Sense < IOPINSENSE_LOW_TRANSITION || Sense > IOPINSENSE_TOGGLE) return false;
    uint32_t state = DisableInterrupt();
    uint32_t addr = RA4M1_PFS(PortNo, PinNo);
    uint32_t old = RA4M1_RD32(addr);
    if (s_PinHook[IntNo].Handler || !Ra4m1PinCanConfigure(PortNo, PinNo) ||
        (old & (RA4M1_PFS_ASEL | RA4M1_PFS_PDR | RA4M1_PFS_PMR | RA4M1_PFS_ISEL)) ||
        (RA4M1_RD32(RA4M1_WUPEN) & (1UL << IntNo))) {
        EnableInterrupt(state); return false;
    }
    uint32_t event = (uint32_t)IntNo + 1U;
    /* IRQCR may not change while any CPU/DTC/DMAC/wake route is present. */
    for (int i = 0; i < RA4M1_IELS_CNT; ++i)
        if ((RA4M1_RD32(RA4M1_IELSR(i)) & RA4M1_IELS_MASK) == event) {
            EnableInterrupt(state); return false;
        }
    for (int i = 0; i < 4; ++i)
        if ((RA4M1_RD32(RA4M1_DELSR(i)) & RA4M1_IELS_MASK) == event) {
            EnableInterrupt(state); return false;
        }
    for (unsigned i = 0; i < sizeof(s_PinIrqMap)/sizeof(s_PinIrqMap[0]); ++i) {
        unsigned pin = s_PinIrqMap[i] & 0xFFU;
        if ((s_PinIrqMap[i] >> 8) == IntNo && Ra4m1PinValid(pin >> 4, pin & 15U) &&
            (RA4M1_RD32(RA4M1_PFS(pin >> 4, pin & 15U)) & RA4M1_PFS_ISEL)) {
            EnableInterrupt(state); return false;
        }
    }
    if (!Ra4m1PinBufferReady(PortNo, PinNo)) { EnableInterrupt(state); return false; }
    uint8_t oldcr = RA4M1_RD8(RA4M1_IRQCR(IntNo));
    RA4M1_WR8(RA4M1_IRQCR(IntNo), oldcr & 0x33U); // disable filter before changing mode
    RA4M1_WR8(RA4M1_IRQCR(IntNo), (uint8_t)(Sense - IOPINSENSE_LOW_TRANSITION));
    Ra4m1PinHook *hook = &s_PinHook[IntNo];
    hook->pCtx = pCtx;
    hook->Pin = (uint8_t)(PortNo * 16U + PinNo);
    hook->Handler = pEvtCB;
    bool configured = RA4M1_RD8(RA4M1_IRQCR(IntNo)) ==
        (uint8_t)(Sense - IOPINSENSE_LOW_TRANSITION);
    if (configured) configured = Ra4m1PfsWrite(PortNo, PinNo, old | RA4M1_PFS_ISEL);
    (void)RA4M1_RD32(addr);
    __DSB();
    IRQn_Type irq = configured ? Ra4m1RegisterIntHandler((uint8_t)event,
        IntPrio, Ra4m1PinCallback, hook) : (IRQn_Type)-1;
    if (irq == (IRQn_Type)-1) {
        Ra4m1PfsWrite(PortNo, PinNo, old);
        RA4M1_WR8(RA4M1_IRQCR(IntNo), oldcr);
        hook->Handler = NULL;
        hook->pCtx = NULL;
        EnableInterrupt(state);
        return false;
    }
    hook->Irq = irq;
    EnableInterrupt(state);
    return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense,
    IOPinEvtHandler_t pEvtCB, void *pCtx)
{
    int line = Ra4m1PinIrq(PortNo, PinNo);
    return line >= 0 && IOPinEnableInterrupt(line, IntPrio, (uint32_t)PortNo,
        (uint32_t)PinNo, Sense, pEvtCB, pCtx) ? line : -1;
}

void IOPinDisableInterrupt(int IntNo)
{
    if ((unsigned)IntNo >= 16U || IntNo == 13) return;
    uint32_t state = DisableInterrupt();
    Ra4m1PinHook *hook = &s_PinHook[IntNo];
    if (hook->Handler) {
        /* This API owns ISEL but not the port's EOF/EOR event sense or the
         * system's low-power WUPEN policy. Neither is modified here.
         */
        unsigned port = hook->Pin >> 4, pin = hook->Pin & 15U;
        Ra4m1PfsWrite(port, pin, RA4M1_RD32(RA4M1_PFS(port, pin)) & ~RA4M1_PFS_ISEL);
        __DSB();
        Ra4m1UnregisterIntHandler(hook->Irq);
        hook->Handler = NULL;
        hook->pCtx = NULL;
    }
    EnableInterrupt(state);
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
    if (!Ra4m1PinValid(PortNo, PinNo) || PortNo < 1 || PortNo > 4 ||
        (unsigned)Sense > IOPINSENSE_TOGGLE) return;
    /* Port-group events, not IRQCR: as in the RE01 implementation. The
     * consumer must disconnect its ELC route during reconfiguration, per
     * section 19.5.2. This function does not reassign another ELC consumer.
     */
    uint32_t state = DisableInterrupt();
    if (Ra4m1PinCanConfigure(PortNo, PinNo) && Ra4m1PinBufferReady(PortNo, PinNo)) {
        uint32_t value = RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
        if (!(value & RA4M1_PFS_ASEL)) {
            value &= ~(RA4M1_PFS_EOF | RA4M1_PFS_EOR);
            if (Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE) value |= RA4M1_PFS_EOF;
            if (Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE) value |= RA4M1_PFS_EOR;
            Ra4m1PfsWrite(PortNo, PinNo, value);
            (void)RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
            __DSB();
        }
    }
    EnableInterrupt(state);
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
    if (!Ra4m1PinValid(PortNo, PinNo) ||
        (Strength != IOPINSTRENGTH_REGULAR && Strength != IOPINSTRENGTH_STRONG)) return;
    uint32_t state = DisableInterrupt();
    if (Ra4m1PinCanConfigure(PortNo, PinNo) &&
        (Ra4m1OutputMask(PortNo) & (1UL << PinNo)) && Ra4m1PinBufferReady(PortNo, PinNo)) {
        uint32_t value = RA4M1_RD32(RA4M1_PFS(PortNo, PinNo));
        if (!(value & RA4M1_PFS_ASEL)) {
            value &= ~(RA4M1_PFS_DSCR | RA4M1_PFS_DSCR1);
            if (Strength == IOPINSTRENGTH_STRONG) value |= RA4M1_PFS_DSCR;
            Ra4m1PfsWrite(PortNo, PinNo, value);
        }
    }
    EnableInterrupt(state);
}

void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
    /* No independently programmable slew-rate field on RA4M1. */
    (void)PortNo; (void)PinNo; (void)Speed;
}
