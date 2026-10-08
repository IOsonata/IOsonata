/**-------------------------------------------------------------------------
@file uart_ra4m1.cpp
@brief Native RA4M1 asynchronous UART, using IOsonata UARTDEV/DevIntrf/CFifo.

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
#include "ra4m1xxx.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "interrupt_ra4m1.h"
#include "ra4m1_startup_regs.h"
#include "ra4m1_uart_baud.h"

#ifndef RA4M1_UART_FIFO_SIZE
#define RA4M1_UART_FIFO_SIZE 16
#endif
static_assert(RA4M1_UART_FIFO_SIZE > 0, "UART FIFO must not be empty");

struct Ra4m1UartDev {
    UARTDEV *pDev;
    uintptr_t Base;
    uint32_t StopMask;
    IRQn_Type Irq[4];             // RXI, TXI, TEI, ERI
    IOPINCFG Pins[2];
    uint32_t OldPfs[2];
    uint32_t SyncCycles;
    uint8_t Event;
    bool Enabled, Rx, Tx;
    alignas(4) uint8_t RxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
    alignas(4) uint8_t TxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
};
static Ra4m1UartDev s_Uart[4];
static const uint8_t s_Channel[] = {0, 1, 2, 9};
static const uint8_t s_Event[] = {RA4M1_EVTID_SCI0_RXI, RA4M1_EVTID_SCI1_RXI,
    RA4M1_EVTID_SCI2_RXI, RA4M1_EVTID_SCI9_RXI};

static uint8_t Read(Ra4m1UartDev *d, unsigned reg)
{
    return RA4M1_RD8(d->Base + reg);
}
static void Write(Ra4m1UartDev *d, unsigned reg, uint8_t value)
{
    RA4M1_WR8(d->Base + reg, value);
}
static Ra4m1UartDev *Owner(DevIntrf_t *p)
{
    if (!p || !p->pDevData) return nullptr;
    Ra4m1UartDev *d = static_cast<Ra4m1UartDev *>(p->pDevData);
    return d->pDev && &d->pDev->DevIntrf == p ? d : nullptr;
}
/* Call after negating the source, before clearing IELSR.IR (manual 13.3).
 * No peripheral clock is changed here. Clock changes require Disable/SetRate.
 */
static void Sync(Ra4m1UartDev *d)
{
    (void)Read(d, SCI_SCR);
    __DSB();
    for (unsigned i = 0; i < d->SyncCycles; ++i) __NOP();
}
/* PRIMASK is held by the caller. Preserve unrelated module/protection bits. */
static bool Clock(Ra4m1UartDev *d, bool enable)
{
    uint16_t prcr = RA4M1_RD16(RA4M1_PRCR) & 15U;
    uint32_t value = RA4M1_RD32(RA4M1_UART_MSTPCRB);
    value = enable ? value & ~d->StopMask : value | d->StopMask;
    RA4M1_WR16(RA4M1_PRCR, RA4M1_PRCR_KEY | prcr | RA4M1_PRCR_PRC1);
    RA4M1_WR32(RA4M1_UART_MSTPCRB, value);
    bool ok = false;
    for (unsigned i = 0; i < 64; ++i) {
        if ((RA4M1_RD32(RA4M1_UART_MSTPCRB) & d->StopMask) ==
            (enable ? 0U : d->StopMask)) { ok = true; break; }
    }
    RA4M1_WR16(RA4M1_PRCR, RA4M1_PRCR_KEY | prcr);
    __DSB();
    return ok;
}
static bool ApplyBaud(Ra4m1UartDev *d, const Ra4m1UartBaud &b)
{
    // Both TE and RE must already be zero.
    uint8_t smr = (Read(d, SCI_SMR) & ~3U) | b.Cks;
    Write(d, SCI_SMR, smr);
    Write(d, SCI_SEMR, b.Semr);
    Write(d, SCI_BRR, b.Brr);
    Write(d, SCI_MDDR, b.Mddr);
    return Read(d, SCI_SMR) == smr && Read(d, SCI_SEMR) == b.Semr &&
        Read(d, SCI_BRR) == b.Brr && Read(d, SCI_MDDR) == b.Mddr;
}
static void RestorePins(Ra4m1UartDev *d, unsigned count)
{
    uint8_t pwpr = RA4M1_RD8(RA4M1_PWPR);
    RA4M1_WR8(RA4M1_PWPR, pwpr & ~RA4M1_PWPR_B0WI);
    RA4M1_WR8(RA4M1_PWPR, RA4M1_PWPR_PFSWE);
    for (unsigned i = 0; i < count; ++i) {
        const IOPINCFG &pin = d->Pins[i];
        if (pin.PortNo < 0) continue;
        uintptr_t addr = RA4M1_PFS(pin.PortNo, pin.PinNo);
        RA4M1_WR32(addr, RA4M1_RD32(addr) & ~(RA4M1_PFS_PMR | RA4M1_PFS_PIDR));
        RA4M1_WR32(addr, d->OldPfs[i] & ~RA4M1_PFS_PMR);
        if (d->OldPfs[i] & RA4M1_PFS_PMR) RA4M1_WR32(addr, d->OldPfs[i]);
    }
    RA4M1_WR8(RA4M1_PWPR, pwpr & ~RA4M1_PWPR_B0WI);
    RA4M1_WR8(RA4M1_PWPR, pwpr);
}
static void ClearRoutes(Ra4m1UartDev *d)
{
    Sync(d);
    for (unsigned i = 0; i < 4; ++i) if ((int)d->Irq[i] >= 0) {
        Ra4m1AcknowledgeInt(d->Irq[i]);
        NVIC_ClearPendingIRQ(d->Irq[i]); // Only while the source is stopped.
    }
}
static void ReleaseRoutes(Ra4m1UartDev *d)
{
    Sync(d);
    for (unsigned i = 0; i < 4; ++i) if ((int)d->Irq[i] >= 0) {
        Ra4m1UnregisterIntHandler(d->Irq[i]);
        d->Irq[i] = (IRQn_Type)-1;
    }
}
/* Drain RDR even on overflow/full software FIFO. Error frames are discarded.
 * Reading RDR clears RDRF; do NOT clear it again after a new byte could arrive.
 */
static bool Receive(Ra4m1UartDev *d, uint8_t *data, uint8_t *errors)
{
    uint8_t ssr = Read(d, SCI_SSR);
    *errors = 0;
    if (!(ssr & (SCI_SSR_RDRF | SCI_SSR_ERRORS))) return false;
    uint8_t byte = Read(d, SCI_RDR);
    UARTDEV *u = d->pDev;
    if (ssr & SCI_SSR_ERRORS) {
        if (ssr & SCI_SSR_ORER) { ++u->RxOvrErrCnt; *errors |= UART_LINESTATE_OVR; }
        if (ssr & SCI_SSR_PER) { ++u->ParErrCnt; *errors |= UART_LINESTATE_PARERR; }
        if (ssr & SCI_SSR_FER) {
            ++u->FramErrCnt; *errors |= UART_LINESTATE_FRMERR;
            if (!(Read(d, SCI_SPTR) & 1U)) *errors |= UART_LINESTATE_BRK;
        }
        ++u->RxDropCnt;
        // Write ones to all other W0C flags: never clear TDRE or a new RDRF.
        Write(d, SCI_SSR, (uint8_t)(0xF8U & ~(ssr & SCI_SSR_ERRORS)));
        (void)Read(d, SCI_SSR);
        u->LineState = *errors;
        return false;
    }
    *data = u->DataBits == 7 ? byte & 0x7FU : byte;
    return true;
}
/* Caller holds PRIMASK. Directly prime TDR rather than toggling TE and adding
 * a frame-long preamble between batches. Subsequent bytes use TXI.
 */
static void TxReady(Ra4m1UartDev *d, bool ready)
{
    d->pDev->bTxReady = ready;
    d->pDev->DevIntrf.bTxReady = ready;
}
static bool Kick(Ra4m1UartDev *d)
{
    if (!d->Enabled || !d->Tx || !CFifoUsed(d->pDev->hTxFifo)) return false;
    uint8_t scr = Read(d, SCI_SCR) & ~(SCI_SCR_TIE | SCI_SCR_TEIE);
    // Enqueue can follow the previous final-byte submission while TDR is
    // still occupied. Arm TXI even then; otherwise those bytes are stranded.
    Write(d, SCI_SCR, scr | SCI_SCR_TIE);
    if (!(Read(d, SCI_SSR) & SCI_SSR_TDRE)) return false;
    uint8_t *p = CFifoGet(d->pDev->hTxFifo);
    if (!p) return false;
    Write(d, SCI_TDR, d->pDev->DataBits == 7 ? *p & 0x7FU : *p);
    if (!CFifoUsed(d->pDev->hTxFifo))
        Write(d, SCI_SCR, scr | SCI_SCR_TEIE);
    TxReady(d, false);
    return true;
}
static void UartIrq(int irq, void *ctx)
{
    Ra4m1UartDev *d = static_cast<Ra4m1UartDev *>(ctx);
    uint32_t state = DisableInterrupt();
    UARTDEV *u = d->pDev;
    if (!u || !d->Enabled) { EnableInterrupt(state); return; }
    UART_EVT event = UART_EVT_RXDATA;
    uint8_t errors = 0;
    int length = 0;
    bool notify = false;
    if (irq == (int)d->Irq[0] || irq == (int)d->Irq[3]) {
        uint8_t byte;
        if (Receive(d, &byte, &errors)) {
            uint32_t dropped = u->hRxFifo->DropCnt;
            uint8_t *p = CFifoPut(u->hRxFifo);
            if (p) { *p = byte; notify = true; }
            else ++u->RxDropCnt;
            u->RxDropCnt += u->hRxFifo->DropCnt - dropped;
            length = CFifoUsed(u->hRxFifo);
            u->bRxReady = length != 0;
        } else if (errors) {
            event = UART_EVT_LINESTATE; length = 1; notify = true;
        }
    } else if (irq == (int)d->Irq[1]) {
        bool full = CFifoAvail(u->hTxFifo) == 0;
        if ((Read(d, SCI_SCR) & SCI_SCR_TIE) && Kick(d) && full) {
            event = UART_EVT_TXREADY; length = CFifoAvail(u->hTxFifo); notify = true;
        }
    } else if (irq == (int)d->Irq[2]) {
        uint8_t scr = Read(d, SCI_SCR);
        if ((scr & SCI_SCR_TEIE) && (Read(d, SCI_SSR) & SCI_SSR_TEND)) {
            Write(d, SCI_SCR, scr & ~SCI_SCR_TEIE);
            TxReady(d, true);
            event = UART_EVT_TXREADY; length = CFifoAvail(u->hTxFifo); notify = true;
        }
    }
    Sync(d);
    Ra4m1AcknowledgeInt((IRQn_Type)irq);
    UARTEvtHandler_t callback = u->EvtCallback;
    EnableInterrupt(state);
    // No register/state accesses after application code: it may power off/reinit.
    if (notify && callback) callback(u, event, errors ? &errors : nullptr, length);
}
static bool StartRx(DevIntrf_t *p, uint32_t)
{
    Ra4m1UartDev *d = Owner(p); return d && d->Enabled && d->Rx;
}
static bool StartTx(DevIntrf_t *p, uint32_t)
{
    Ra4m1UartDev *d = Owner(p); return d && d->Enabled && d->Tx;
}
static void Stop(DevIntrf_t *p)
{
    // Generic Start/Stop wrappers own bBusy. Polling has no TEI callback;
    // sample wire completion on API return without waiting with IRQs masked.
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    if (d && d->Enabled && d->Tx && !p->bIntEn)
        TxReady(d, (Read(d, SCI_SSR) & SCI_SSR_TEND) != 0U);
    EnableInterrupt(state);
}
static int RxData(DevIntrf_t *p, uint8_t *buffer, int len)
{
    if (!buffer || len <= 0) return 0;
    int count = 0;
    while (count < len) {
        uint32_t state = DisableInterrupt();
        Ra4m1UartDev *d = Owner(p);
        if (!d || !d->Enabled || !d->Rx) { EnableInterrupt(state); break; }
        if (p->bIntEn) {
            uint8_t *data = CFifoGet(d->pDev->hRxFifo);
            if (!data) { EnableInterrupt(state); break; }
            buffer[count++] = *data;
            d->pDev->bRxReady = CFifoUsed(d->pDev->hRxFifo) != 0;
        } else {
            uint8_t errors, byte;
            bool received = Receive(d, &byte, &errors);
            if (received) buffer[count++] = byte;
            else { EnableInterrupt(state); break; }
        }
        EnableInterrupt(state);
    }
    return count;
}
static int TxData(DevIntrf_t *p, const uint8_t *data, int len)
{
    if (!data || len <= 0) return 0;
    int count = 0;
    int tries = p->MaxRetry > 0 ? p->MaxRetry : 1;
    while (count < len && tries > 0) {
        uint32_t state = DisableInterrupt();
        Ra4m1UartDev *d = Owner(p);
        if (!d || !d->Enabled || !d->Tx) { EnableInterrupt(state); break; }
        bool accepted = false;
        if (p->bIntEn) {
            uint32_t dropped = d->pDev->hTxFifo->DropCnt;
            uint8_t *dest = CFifoPut(d->pDev->hTxFifo);
            if (dest) { *dest = data[count]; accepted = true; TxReady(d, false); }
            d->pDev->TxDropCnt += d->pDev->hTxFifo->DropCnt - dropped;
            Kick(d);
        } else if (Read(d, SCI_SSR) & SCI_SSR_TDRE) {
            Write(d, SCI_TDR, d->pDev->DataBits == 7 ? data[count] & 0x7FU : data[count]);
            TxReady(d, false);
            accepted = true;
        }
        EnableInterrupt(state);
        if (accepted) { ++count; tries = p->MaxRetry > 0 ? p->MaxRetry : 1; }
        else --tries;
    }
    return count; // Polling never leaves accepted bytes stranded in a SW FIFO.
}
static uint32_t GetRate(DevIntrf_t *p)
{
    Ra4m1UartDev *d = Owner(p); return d ? (uint32_t)d->pDev->Rate : 0U;
}
static uint32_t SetRate(DevIntrf_t *p, uint32_t rate)
{
    Ra4m1UartBaud baud;
    uint32_t clock = SystemPeriphClockGet(0); // SCI uses PCLKA, NOT PCLKB.
    if (!Ra4m1UartPlanBaud(clock, rate, &baud)) return 0;
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    // Receiver has no idle flag. Require Disable and a quiesced peer first.
    if (!d || d->Enabled) { EnableInterrupt(state); return 0; }
    Ra4m1UartBaud old = {(uint32_t)d->pDev->Rate, (uint8_t)(Read(d, SCI_SMR)&3U),
        Read(d, SCI_BRR), Read(d, SCI_MDDR), Read(d, SCI_SEMR)};
    bool ok = ApplyBaud(d, baud);
    if (ok) {
        d->SyncCycles = (2U * ((SystemCoreClock + clock-1U)/clock) + 2U);
        d->pDev->Rate = (int)baud.Actual;
    } else ApplyBaud(d, old);
    EnableInterrupt(state);
    return ok ? baud.Actual : 0U;
}
static void Disable(DevIntrf_t *p)
{
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    if (d && d->Enabled) {
        for (unsigned i = 0; i < 4; ++i)
            if ((int)d->Irq[i] >= 0) NVIC_DisableIRQ(d->Irq[i]);
        Write(d, SCI_SCR, 0);
        ClearRoutes(d);
        d->Enabled = false;
        d->pDev->TxDropCnt += CFifoUsed(d->pDev->hTxFifo);
        CFifoFlush(d->pDev->hTxFifo);
        TxReady(d, false);
    }
    EnableInterrupt(state);
}
static void Enable(DevIntrf_t *p)
{
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    if (d && !d->Enabled) {
        // Retain configuration/CFifo RX data. Disable does not gate the module.
        if (Read(d, SCI_SSR) & (SCI_SSR_RDRF | SCI_SSR_ERRORS)) {
            uint8_t byte, errors;
            if (Receive(d, &byte, &errors)) ++d->pDev->RxDropCnt;
        }
        ClearRoutes(d);
        d->Enabled = true;
        for (unsigned i = 0; i < 4; ++i)
            if ((int)d->Irq[i] >= 0) NVIC_EnableIRQ(d->Irq[i]);
        TxReady(d, d->Tx);
        Write(d, SCI_SCR, (d->Tx ? SCI_SCR_TE : 0U) | (d->Rx ? SCI_SCR_RE : 0U) |
            (d->Rx && p->bIntEn ? SCI_SCR_RIE : 0U));
    }
    EnableInterrupt(state);
}
static void Reset(DevIntrf_t *p)
{
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    if (d) {
        bool enabled = d->Enabled;
        Disable(p);
        CFifoFlush(d->pDev->hRxFifo);
        d->pDev->bRxReady = false;
        if (enabled) Enable(p);
    }
    EnableInterrupt(state);
}
static void PowerOff(DevIntrf_t *p)
{
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = Owner(p);
    if (d) {
        Disable(p);
        ReleaseRoutes(d);
        RestorePins(d, 2);
        Clock(d, false);
        CFifoFlush(d->pDev->hRxFifo);
        d->pDev->bRxReady = false;
        d->pDev = nullptr;
        p->pDevData = nullptr;
        p->EnCnt = 0;
    }
    EnableInterrupt(state);
}
static void *GetHandle(DevIntrf_t *p)
{
    Ra4m1UartDev *d = Owner(p); return d ? d->pDev : nullptr;
}
UARTDEV const *UARTGetInstance(int devno)
{
    return (unsigned)devno < 4U ? s_Uart[devno].pDev : nullptr;
}
void UARTSetCtrlLineState(UARTDEV *, uint32_t)
{
    // No modem-control/flow-control pins in this asynchronous UART increment.
}
static bool MemoryValid(const uint8_t *mem, int size)
{
    return mem ? size > (int)sizeof(CFifo_t) && ((uintptr_t)mem & 3U) == 0U : size == 0;
}
bool UARTInit(UARTDEV * const u, const UARTCFG *cfg)
{
    if (!u || !cfg || (unsigned)cfg->DevNo >= 4U || !cfg->pIOPinMap || cfg->NbIOPins != 2 ||
        cfg->Rate <= 0 || (cfg->DataBits != 7 && cfg->DataBits != 8) ||
        (cfg->StopBits != 1 && cfg->StopBits != 2) ||
        (cfg->Parity != UART_PARITY_NONE && cfg->Parity != UART_PARITY_ODD && cfg->Parity != UART_PARITY_EVEN) ||
        cfg->Mode != UART_MODE_UART || cfg->Duplex != UART_DUPLEX_FULL ||
        cfg->FlowControl != UART_FLWCTRL_NONE || cfg->bDMAMode || cfg->bIrDAMode ||
        (cfg->bIntMode && (unsigned)cfg->IntPrio >= (1UL << __NVIC_PRIO_BITS)) ||
        !MemoryValid(cfg->pRxMem, cfg->RxMemSize) || !MemoryValid(cfg->pTxMem, cfg->TxMemSize)) return false;
    if (cfg->pRxMem && cfg->pTxMem) {
        uintptr_t rx = (uintptr_t)cfg->pRxMem, tx = (uintptr_t)cfg->pTxMem;
        if (rx < tx ? tx-rx < (unsigned)cfg->RxMemSize : rx-tx < (unsigned)cfg->TxMemSize) return false;
    }
    const IOPINCFG *pins = static_cast<const IOPINCFG *>(cfg->pIOPinMap);
    bool present[2];
    for (unsigned i = 0; i < 2; ++i) {
        const IOPINCFG &pin = pins[i];
        present[i] = pin.PortNo != -1 || pin.PinNo != -1;
        if (!present[i]) continue;
        if (!Ra4m1PinValid(pin.PortNo, pin.PinNo) || pin.PortNo == 9 ||
            pin.PinOp < IOPINOP_FUNC0 || pin.PinOp > IOPINOP_FUNC30 ||
            pin.PinDir != (i == 0 ? IOPINDIR_INPUT : IOPINDIR_OUTPUT) ||
            (pin.Res != IOPINRES_NONE && pin.Res != IOPINRES_PULLUP) || pin.Type != IOPINTYPE_NORMAL) return false;
    }
    if ((!present[0] && !present[1]) || (present[0] && present[1] &&
        pins[0].PortNo == pins[1].PortNo && pins[0].PinNo == pins[1].PinNo)) return false;
    Ra4m1UartBaud baud;
    uint32_t clock = SystemPeriphClockGet(0);
    if (!Ra4m1UartPlanBaud(clock, (uint32_t)cfg->Rate, &baud) ||
        !SystemCoreClock || SystemCoreClock > 48000000U) return false;
    uint32_t state = DisableInterrupt();
    Ra4m1UartDev *d = &s_Uart[cfg->DevNo];
    for (unsigned i = 0; i < 4; ++i) {
        if (s_Uart[i].pDev == u || (i == (unsigned)cfg->DevNo && s_Uart[i].pDev)) {
            EnableInterrupt(state); return false;
        }
        if (s_Uart[i].pDev) for (unsigned a = 0; a < 2; ++a) for (unsigned b = 0; b < 2; ++b)
            if (present[a] && s_Uart[i].Pins[b].PortNo == pins[a].PortNo &&
                s_Uart[i].Pins[b].PinNo == pins[a].PinNo) { EnableInterrupt(state); return false; }
    }
    d->Base = RA4M1_SCI_BASE(s_Channel[cfg->DevNo]);
    d->StopMask = 1UL << (31U - s_Channel[cfg->DevNo]);
    d->Event = s_Event[cfg->DevNo];
    d->SyncCycles = (2U*((SystemCoreClock + clock-1U)/clock) + 2U);
    d->Enabled = false; d->Rx = present[0]; d->Tx = present[1];
    for (unsigned i = 0; i < 4; ++i) d->Irq[i] = (IRQn_Type)-1;
    // Do not take a running SCI instance from another peripheral implementation.
    if (!(RA4M1_RD32(RA4M1_UART_MSTPCRB) & d->StopMask)) { EnableInterrupt(state); return false; }
    for (unsigned i = 0; i < 2; ++i) {
        d->Pins[i] = pins[i];
        d->OldPfs[i] = present[i] ? RA4M1_RD32(RA4M1_PFS(pins[i].PortNo, pins[i].PinNo)) & ~RA4M1_PFS_PIDR : 0U;
        if (d->OldPfs[i] & RA4M1_PFS_ISEL) { EnableInterrupt(state); return false; }
    }
    if (!Clock(d, true)) { EnableInterrupt(state); return false; }
    Write(d, SCI_SCR, 0);
    if (cfg->DevNo < 2) RA4M1_WR16(d->Base + SCI_FCR, 0); // Disable hardware FIFO.
    Write(d, SCI_SPTR, SCI_SPTR_MARK);
    Write(d, SCI_SIMR1, 0); Write(d, SCI_SPMR, 0);
    Write(d, SCI_DCCR, 0x40U); // DCME=0, IDSEL=1 (reset value); address matching disabled.
    Write(d, SCI_SCMR, 0xF2U); // CHR1=1, LSB first, no inversion, SMIF=0.
    Write(d, SCI_SNFR, 0);
    uint8_t smr = (cfg->DataBits == 7 ? 0x40U : 0U) | (cfg->StopBits == 2 ? 8U : 0U);
    if (cfg->Parity != UART_PARITY_NONE) smr |= 0x20U | (cfg->Parity == UART_PARITY_ODD ? 0x10U : 0U);
    Write(d, SCI_SMR, smr);
    unsigned configured = 0;
    bool ok = Read(d, SCI_SCR) == 0 && Read(d, SCI_SCMR) == 0xF2 &&
        Read(d, SCI_SMR) == smr && Read(d, SCI_SIMR1) == 0 &&
        Read(d, SCI_SPMR) == 0 && Read(d, SCI_DCCR) == 0x40 &&
        (Read(d, SCI_SPTR) & SCI_SPTR_MARK) == SCI_SPTR_MARK &&
        (cfg->DevNo >= 2 || !(RA4M1_RD16(d->Base + SCI_FCR) & 1U)) && ApplyBaud(d, baud);
    if (ok && cfg->bIntMode) {
        for (unsigned i = 0; i < 4 && ok; ++i) {
            if ((i == 0 || i == 3) ? !d->Rx : !d->Tx) continue;
            d->Irq[i] = Ra4m1RegisterIntHandler(d->Event+i, cfg->IntPrio, UartIrq, d);
            ok = (int)d->Irq[i] >= 0;
        }
    }
    for (unsigned i = 0; i < 2 && ok; ++i) {
        configured = i+1;
        if (!present[i]) continue;
        IOPinCfg(&pins[i], 1);
        uint32_t mask = RA4M1_PFS_PSEL | RA4M1_PFS_PMR | RA4M1_PFS_PDR |
            RA4M1_PFS_ASEL | RA4M1_PFS_PCR | RA4M1_PFS_NCODR;
        uint32_t value = ((uint32_t)pins[i].PinOp << 24) | RA4M1_PFS_PMR |
            (i == 1 ? RA4M1_PFS_PDR : 0U) | (pins[i].Res == IOPINRES_PULLUP ? RA4M1_PFS_PCR : 0U);
        ok = (RA4M1_RD32(RA4M1_PFS(pins[i].PortNo, pins[i].PinNo)) & mask) == value;
    }
    hCFifo_t rx = nullptr, tx = nullptr;
    if (ok) {
        rx = CFifoInit(cfg->pRxMem ? cfg->pRxMem : d->RxMem,
            cfg->pRxMem ? (unsigned)cfg->RxMemSize : sizeof(d->RxMem), 1, cfg->bFifoBlocking);
        tx = CFifoInit(cfg->pTxMem ? cfg->pTxMem : d->TxMem,
            cfg->pTxMem ? (unsigned)cfg->TxMemSize : sizeof(d->TxMem), 1, cfg->bFifoBlocking);
        ok = rx && tx;
    }
    if (!ok) {
        Write(d, SCI_SCR, 0); ReleaseRoutes(d); RestorePins(d, configured); Clock(d, false);
        EnableInterrupt(state); return false;
    }
    u->Mode = cfg->Mode; u->Duplex = cfg->Duplex; u->Rate = (int)baud.Actual;
    u->DataBits = cfg->DataBits; u->Parity = cfg->Parity; u->StopBits = cfg->StopBits;
    u->FlowControl = cfg->FlowControl;
    u->bIrDAMode = u->bIrDAInvert = u->bIrDAFixPulse = false; u->IrDAPulseDiv = 0;
    u->EvtCallback = cfg->EvtCallback; u->hRxFifo = rx; u->hTxFifo = tx;
    u->LineState = 0; u->hStdIn = u->hStdOut = -1;
    u->RxOvrErrCnt = u->ParErrCnt = u->FramErrCnt = u->RxDropCnt = u->TxDropCnt = 0;
    u->bRxReady = u->bTxReady = false;
    // Preserve pObj installed by the existing C++ UART constructor.
    DevIntrf_t *p = &u->DevIntrf;
    p->pDevData = d; p->IntPrio = cfg->IntPrio; p->EvtCB = nullptr;
    p->Type = DEVINTRF_TYPE_UART; p->bDma = false; p->bIntEn = cfg->bIntMode;
    p->MaxRetry = UART_RETRY_MAX; p->EnCnt = 1; p->bTxReady = true; p->bNoStop = false;
    atomic_flag_clear(&p->bBusy);
    p->Enable = Enable; p->Disable = Disable; p->PowerOff = PowerOff; p->Reset = Reset;
    p->GetRate = GetRate; p->SetRate = SetRate; p->GetHandle = GetHandle;
    p->StartRx = StartRx; p->RxData = RxData; p->StopRx = Stop;
    p->StartTx = StartTx; p->TxData = TxData; p->TxSrData = TxData; p->StopTx = Stop;
    d->pDev = u;
    Enable(p);
    EnableInterrupt(state);
    return true;
}
