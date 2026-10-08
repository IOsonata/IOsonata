/**-------------------------------------------------------------------------
@file	interrupt_ra4m1.cpp

@brief	RA4M1 ICU event-to-IRQ registration following the RE01 interface.

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
#include "interrupt_ra4m1.h"
#include "ra4m1_ioregs.h"
#include "coredev/interrupt.h"

extern "C" void (* const __Vectors[])(void);
static void Ra4m1Dispatch(int IrqNo);

/* The vector object has unresolved references to these weak entries. This
 * forces archive extraction of the dispatcher; an application can still
 * override an individual entry. Its slot is excluded from dynamic allocation.
 */
#define RA4M1_IEL(n) \
    extern "C" void Ra4m1DefaultIEL##n(void) { Ra4m1Dispatch(n); } \
    extern "C" __attribute__((weak, alias("Ra4m1DefaultIEL" #n))) \
        void IEL##n##_IRQHandler(void);
RA4M1_IEL(0)
RA4M1_IEL(1)
RA4M1_IEL(2)
RA4M1_IEL(3)
RA4M1_IEL(4)
RA4M1_IEL(5)
RA4M1_IEL(6)
RA4M1_IEL(7)
RA4M1_IEL(8)
RA4M1_IEL(9)
RA4M1_IEL(10)
RA4M1_IEL(11)
RA4M1_IEL(12)
RA4M1_IEL(13)
RA4M1_IEL(14)
RA4M1_IEL(15)
RA4M1_IEL(16)
RA4M1_IEL(17)
RA4M1_IEL(18)
RA4M1_IEL(19)
RA4M1_IEL(20)
RA4M1_IEL(21)
RA4M1_IEL(22)
RA4M1_IEL(23)
RA4M1_IEL(24)
RA4M1_IEL(25)
RA4M1_IEL(26)
RA4M1_IEL(27)
RA4M1_IEL(28)
RA4M1_IEL(29)
RA4M1_IEL(30)
RA4M1_IEL(31)
#undef RA4M1_IEL
static void (* const s_DefaultIEL[RA4M1_IELS_CNT])(void) = {
    Ra4m1DefaultIEL0,
    Ra4m1DefaultIEL1,
    Ra4m1DefaultIEL2,
    Ra4m1DefaultIEL3,
    Ra4m1DefaultIEL4,
    Ra4m1DefaultIEL5,
    Ra4m1DefaultIEL6,
    Ra4m1DefaultIEL7,
    Ra4m1DefaultIEL8,
    Ra4m1DefaultIEL9,
    Ra4m1DefaultIEL10,
    Ra4m1DefaultIEL11,
    Ra4m1DefaultIEL12,
    Ra4m1DefaultIEL13,
    Ra4m1DefaultIEL14,
    Ra4m1DefaultIEL15,
    Ra4m1DefaultIEL16,
    Ra4m1DefaultIEL17,
    Ra4m1DefaultIEL18,
    Ra4m1DefaultIEL19,
    Ra4m1DefaultIEL20,
    Ra4m1DefaultIEL21,
    Ra4m1DefaultIEL22,
    Ra4m1DefaultIEL23,
    Ra4m1DefaultIEL24,
    Ra4m1DefaultIEL25,
    Ra4m1DefaultIEL26,
    Ra4m1DefaultIEL27,
    Ra4m1DefaultIEL28,
    Ra4m1DefaultIEL29,
    Ra4m1DefaultIEL30,
    Ra4m1DefaultIEL31,
};

struct Ra4m1IntHook {
    Ra4m1IRQHandler_t Handler;
    void *pCtx;
    uint32_t Generation;
    uint8_t Event;
};
static Ra4m1IntHook s_IntHook[RA4M1_IELS_CNT];

static bool Ra4m1CpuEventValid(uint8_t Event)
{
    /* CPU-only subset of Table 13.4. Do not treat the whole 8-bit selector as
     * valid. FIFO/DTC-completion-only and software-ELC sources are deliberately
     * deferred until their transfer-engine integration is implemented.
     * 0xDB in newer FSP event headers is not in the Rev.1.10 hardware table.
     */
    if (Event == 0 || Event > RA4M1_EVTID_SPI1_SPTEND) return false;
    switch (Event) {
        case 0x0E: case 0x16: case 0x40: // holes in the hardware event table
        case RA4M1_EVTID_SYSTEM_SNZREQ:
        case RA4M1_EVTID_USBFS_D0FIFO: case RA4M1_EVTID_USBFS_D1FIFO:
        case RA4M1_EVTID_SSIE0_SSITXI: case RA4M1_EVTID_SSIE0_SSIRXI:
        case RA4M1_EVTID_ELC_SWEVT0: case RA4M1_EVTID_ELC_SWEVT1:
            return false;
        default: return true;
    }
}

IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio,
    Ra4m1IRQHandler_t pHandler, void *pCtx)
{
    if (!pHandler || !Ra4m1CpuEventValid(EvtId) ||
        (unsigned)Prio >= (1UL << __NVIC_PRIO_BITS)) return (IRQn_Type)-1;

    uint32_t state = DisableInterrupt();
    int available = -1;
    for (int i = 0; i < RA4M1_IELS_CNT; ++i) {
        uint32_t reg = RA4M1_RD32(RA4M1_IELSR(i));
        if (s_IntHook[i].Event == EvtId || (reg & RA4M1_IELS_MASK) == EvtId) {
            EnableInterrupt(state);
            return (IRQn_Type)-1;
        }
        IRQn_Type irq = (IRQn_Type)i;
        if (available < 0 && !s_IntHook[i].Handler && reg == 0 &&
            !NVIC_GetEnableIRQ(irq) && !NVIC_GetActive(irq) &&
            __Vectors[16+i] == s_DefaultIEL[i]) available = i;
    }
    for (int i = 0; i < 4; ++i) {
        if ((RA4M1_RD32(RA4M1_DELSR(i)) & RA4M1_IELS_MASK) == EvtId) {
            EnableInterrupt(state);
            return (IRQn_Type)-1;
        }
    }
    if (available < 0) {
        EnableInterrupt(state);
        return (IRQn_Type)-1;
    }
    IRQn_Type irq = (IRQn_Type)available;
    Ra4m1IntHook &hook = s_IntHook[available];
    NVIC_DisableIRQ(irq);
    NVIC_ClearPendingIRQ(irq);
    hook.pCtx = pCtx;
    hook.Handler = pHandler;
    hook.Event = EvtId;
    ++hook.Generation;
    NVIC_SetPriority(irq, (uint32_t)Prio);
    RA4M1_WR32(RA4M1_IELSR(available), EvtId); // DTCE=0, never write IR=1
    if ((RA4M1_RD32(RA4M1_IELSR(available)) &
        (RA4M1_IELS_MASK | RA4M1_IELS_DTCE)) != EvtId) {
        RA4M1_WR32(RA4M1_IELSR(available), 0);
        hook.Handler = nullptr;
        hook.pCtx = nullptr;
        hook.Event = 0;
        NVIC_ClearPendingIRQ(irq);
        EnableInterrupt(state);
        return (IRQn_Type)-1;
    }
    __DSB();
    NVIC_EnableIRQ(irq);
    EnableInterrupt(state);
    return irq;
}

void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo)
{
    if ((unsigned)IrqNo >= RA4M1_IELS_CNT) return;
    uint32_t state = DisableInterrupt();
    Ra4m1IntHook &hook = s_IntHook[(unsigned)IrqNo];
    /* Do not commandeer a manually installed route or a DTC transfer. */
    uint32_t reg = RA4M1_RD32(RA4M1_IELSR((unsigned)IrqNo));
    if (hook.Handler && (reg & (RA4M1_IELS_MASK | RA4M1_IELS_DTCE)) == hook.Event) {
        NVIC_DisableIRQ(IrqNo);
        RA4M1_WR32(RA4M1_IELSR((unsigned)IrqNo), 0);
        __DSB();
        NVIC_ClearPendingIRQ(IrqNo);
        ++hook.Generation;
        hook.Handler = nullptr;
        hook.pCtx = nullptr;
        hook.Event = 0;
    }
    EnableInterrupt(state);
}

bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo)
{
    if ((unsigned)IrqNo >= RA4M1_IELS_CNT) return false;
    uint32_t state = DisableInterrupt();
    Ra4m1IntHook &hook = s_IntHook[(unsigned)IrqNo];
    uint32_t reg = RA4M1_RD32(RA4M1_IELSR((unsigned)IrqNo));
    bool owned = hook.Handler &&
        (reg & (RA4M1_IELS_MASK | RA4M1_IELS_DTCE)) == hook.Event;
    if (owned) {
        RA4M1_WR32(RA4M1_IELSR((unsigned)IrqNo), hook.Event);
        __DSB();
        ++hook.Generation; // Suppress only this dispatcher's trailing clear.
    }
    EnableInterrupt(state);
    return owned;
}

static void Ra4m1Dispatch(int IrqNo)
{
    uint32_t state = DisableInterrupt();
    Ra4m1IntHook &hook = s_IntHook[IrqNo];
    uint32_t reg = RA4M1_RD32(RA4M1_IELSR(IrqNo));
    if (!hook.Handler || (reg & (RA4M1_IELS_MASK | RA4M1_IELS_DTCE)) != hook.Event) {
        /* No software owner, or unsupported raw change: stop the IRQ storm
         * without clearing a hardware/DTC-owned route or its pending flag.
         */
        NVIC_DisableIRQ((IRQn_Type)IrqNo);
        EnableInterrupt(state);
        return;
    }
    if (!(reg & RA4M1_IELS_IR)) {
        EnableInterrupt(state);
        return;
    }
    Ra4m1IRQHandler_t handler = hook.Handler;
    void *ctx = hook.pCtx;
    uint32_t generation = hook.Generation;
    uint8_t event = hook.Event;
    bool pinEdge = event <= RA4M1_EVTID_PORT_IRQ15 &&
        (RA4M1_RD8(RA4M1_IRQCR(event-1U)) & 3U) != 3U;
    if (pinEdge) {
        /* The edge detector is a pulse source. Reading IRQCR above plus
         * this barrier settles the peripheral read before acknowledging IR.
         * The startup-supported PCLKB is ICLK or ICLK/2.
         */
        __NOP(); __NOP(); __NOP(); __NOP();
        RA4M1_WR32(RA4M1_IELSR(IrqNo), event);
        __DSB();
    }
    EnableInterrupt(state);
    handler(IrqNo, ctx);
    if (!pinEdge) {
        state = DisableInterrupt();
        reg = RA4M1_RD32(RA4M1_IELSR(IrqNo));
        if (hook.Generation == generation && hook.Handler == handler &&
            (reg & (RA4M1_IELS_MASK | RA4M1_IELS_DTCE)) == event) {
            RA4M1_WR32(RA4M1_IELSR(IrqNo), event);
            __DSB();
        }
        EnableInterrupt(state);
    }
    /* Never clear NVIC pending here: it may be a fresh edge, or a request
     * raised by a replacement registration inside the callback.
     */
}
