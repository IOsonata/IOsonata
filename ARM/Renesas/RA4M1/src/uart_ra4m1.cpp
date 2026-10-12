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
#include "ra4m1_uart_baud.h"
#define SCI_SCR_TEIE 0x04U
#define SCI_SCR_RE   0x10U
#define SCI_SCR_TE   0x20U
#define SCI_SCR_RIE  0x40U
#define SCI_SCR_TIE  0x80U
#define SCI_SSR_TEND 0x04U
#define SCI_SSR_PER  0x08U
#define SCI_SSR_FER  0x10U
#define SCI_SSR_ORER 0x20U
#define SCI_SSR_RDRF 0x40U
#define SCI_SSR_TDRE 0x80U
#define SCI_SSR_ERRORS (SCI_SSR_PER | SCI_SSR_FER | SCI_SSR_ORER)
#define SCI_SPTR_MARK 0x06U
#define RA4M1_DMA_MSTP_MASK (1UL << 22U)

#ifndef RA4M1_UART_FIFO_SIZE
#define RA4M1_UART_FIFO_SIZE 16
#endif
static_assert(RA4M1_UART_FIFO_SIZE > 0, "UART FIFO must not be empty");

struct Ra4m1UartDev {
    UARTDEV *pDev;
    R_SCI0_Type *Regs;
    uint32_t StopMask;
    IRQn_Type Irq[4];             // RXI, TXI, TEI, ERI
    IOPINCFG Pins[2];
    uint32_t OldPfs[2];
    uint32_t SyncCycles;
    uint8_t Event;
    uint8_t DmaChannel;
    IRQn_Type DmaIrq;
    bool DmaMode;
    bool DmaBusy;
    bool Enabled, Rx, Tx;
    uint16_t DmaCount;
    alignas(4) uint8_t RxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
    alignas(4) uint8_t TxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
};
static R_SCI0_Type * const s_SciRegs[4] = {R_SCI0, R_SCI1, R_SCI2, R_SCI9};
static Ra4m1UartDev s_Uart[4];
static R_DMAC0_Type * const s_DmaRegs[4] = {
    R_DMAC0,
    reinterpret_cast<R_DMAC0_Type *>(R_DMAC1),
    reinterpret_cast<R_DMAC0_Type *>(R_DMAC2),
    reinterpret_cast<R_DMAC0_Type *>(R_DMAC3)
};
static const uint8_t s_Channel[] = {0, 1, 2, 9};
static const uint8_t s_Event[] = {RA4M1_EVTID_SCI0_RXI, RA4M1_EVTID_SCI1_RXI,
    RA4M1_EVTID_SCI2_RXI, RA4M1_EVTID_SCI9_RXI};

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
    (void)d->Regs->SCR;
    __DSB();
    for (unsigned i = 0; i < d->SyncCycles; ++i) __NOP();
}
/* PRIMASK is held by the caller. Preserve unrelated module/protection bits. */
static bool Clock(Ra4m1UartDev *d, bool enable)
{
    uint16_t prcr = R_SYSTEM->PRCR & 15U;
    uint32_t value = R_MSTP->MSTPCRB;
    value = enable ? value & ~d->StopMask : value | d->StopMask;
    R_SYSTEM->PRCR =  0xA500U | prcr | 0x02U;
    R_MSTP->MSTPCRB =  value;
    bool ok = false;
    for (unsigned i = 0; i < 64; ++i) {
        if ((R_MSTP->MSTPCRB & d->StopMask) ==
            (enable ? 0U : d->StopMask)) { ok = true; break; }
    }
    R_SYSTEM->PRCR =  0xA500U | prcr;
    __DSB();
    return ok;
}
static bool ApplyBaud(Ra4m1UartDev *d, const Ra4m1UartBaud &b)
{
    // Both TE and RE must already be zero.
    uint8_t smr = (d->Regs->SMR & ~3U) | b.Cks;
    d->Regs->SMR = smr;
    d->Regs->SEMR = b.Semr;
    d->Regs->BRR = b.Brr;
    d->Regs->MDDR = b.Mddr;
    return d->Regs->SMR == smr && d->Regs->SEMR == b.Semr &&
        d->Regs->BRR == b.Brr && d->Regs->MDDR == b.Mddr;
}
static void RestorePins(Ra4m1UartDev *d, unsigned count)
{
    uint8_t pwpr = R_PMISC->PWPR;
    R_PMISC->PWPR =  pwpr & ~RA4M1_PWPR_B0WI;
    R_PMISC->PWPR =  RA4M1_PWPR_PFSWE;
    for (unsigned i = 0; i < count; ++i) {
        const IOPINCFG &pin = d->Pins[i];
        if (pin.PortNo < 0) continue;
        volatile uint32_t *addr = &R_PFS->PORT[pin.PortNo].PIN[pin.PinNo].PmnPFS;
        *addr = (*addr) & ~(RA4M1_PFS_PMR | RA4M1_PFS_PIDR);
        *addr = d->OldPfs[i] & ~RA4M1_PFS_PMR;
        if (d->OldPfs[i] & RA4M1_PFS_PMR) *addr = d->OldPfs[i];
    }
    R_PMISC->PWPR =  pwpr & ~RA4M1_PWPR_B0WI;
    R_PMISC->PWPR =  pwpr;
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
	uint8_t ssr = d->Regs->SSR;
	*errors = 0U;

	// RXI and ERI can both remain pending after an overrun. The first
	// handler may already have consumed RDR and cleared the error flags.
	// Check the current status before reading RDR again. In non-FIFO mode,
	// RDRF remains set until RDR is read (manual section 28.2.13).
	if ((ssr & (SCI_SSR_RDRF | SCI_SSR_ERRORS)) == 0U)
	{
		return false;
	}

	uint8_t byte = d->Regs->RDR;
	UARTDEV *u = d->pDev;

	if ((ssr & SCI_SSR_ERRORS) != 0U)
	{
		if ((ssr & SCI_SSR_ORER) != 0U)
		{
			++u->RxOvrErrCnt;
			*errors |= UART_LINESTATE_OVR;
		}
		if ((ssr & SCI_SSR_PER) != 0U)
		{
			++u->ParErrCnt;
			*errors |= UART_LINESTATE_PARERR;
		}
		if ((ssr & SCI_SSR_FER) != 0U)
		{
			++u->FramErrCnt;
			*errors |= UART_LINESTATE_FRMERR;
			if ((d->Regs->SPTR & 1U) == 0U)
			{
				*errors |= UART_LINESTATE_BRK;
			}
		}

		++u->RxDropCnt;
		// SCI SSR error flags clear by writing zero. Preserve the other
		// status bits and avoid clearing a newly asserted RX flag.
		d->Regs->SSR = (uint8_t)~SCI_SSR_ERRORS;
		(void)d->Regs->SSR;
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
static bool DmaStart(Ra4m1UartDev *d)
{
    if (!d->DmaMode || d->DmaBusy || !d->Enabled || !d->Tx)
    {
        return false;
    }

    int count = 0xFFFF;
    uint8_t *source = CFifoPeekMultiple(d->pDev->hTxFifo, &count);
    if (source == nullptr || count <= 0)
    {
        return false;
    }

    // An idle TDR has no future empty transition to trigger DMA. Submit
    // the first byte directly, then arm DMA for the remaining FIFO data.
    // This also avoids a DMA transaction for each one-byte UARTTx call.
    if ((d->Regs->SSR & SCI_SSR_TDRE) != 0U)
    {
        d->Regs->TDR = d->pDev->DataBits == 7 ?
            source[0] & 0x7FU : source[0];
        Sync(d);
        CFifoRelease(d->pDev->hTxFifo, 1);
        ++source;
        --count;

        if (count == 0)
        {
            TxReady(d, true);
            return true;
        }
    }

    R_DMAC0_Type *dma = s_DmaRegs[d->DmaChannel];
    dma->DMSAR = (uint32_t)(uintptr_t)source;
    dma->DMCRA = (uint32_t)count;
    d->DmaCount = (uint16_t)count;
    d->DmaBusy = true;
    dma->DMCNT_b.DTE = 1U;
    d->Regs->SCR |= SCI_SCR_TIE;

    return true;
}

static void DmaIrqHandler(int irq, void *ctx)
{
    Ra4m1UartDev *d = static_cast<Ra4m1UartDev *>(ctx);
    uint32_t state = DisableInterrupt();
    R_DMAC0_Type *dma = s_DmaRegs[d->DmaChannel];

    dma->DMCNT_b.DTE = 0U;
    dma->DMSTS = 0U;
    d->Regs->SCR &= (uint8_t)~SCI_SCR_TIE;
    CFifoRelease(d->pDev->hTxFifo, d->DmaCount);
    d->DmaCount = 0U;
    d->DmaBusy = false;
    Ra4m1AcknowledgeInt((IRQn_Type)irq);

    if (d->Enabled && d->pDev)
    {
        if (!DmaStart(d))
        {
            TxReady(d, true);
        }
    }

    EnableInterrupt(state);
}

static bool Kick(Ra4m1UartDev *d)
{
    if (!d->Enabled || !d->Tx || d->DmaMode)
    {
        return false;
    }

    if ((d->Regs->SSR & SCI_SSR_TDRE) == 0U)
    {
        // The current TDR byte will produce TXI when it is consumed.
        d->Regs->SCR = (d->Regs->SCR & (uint8_t)~SCI_SCR_TEIE) |
            SCI_SCR_TIE;
        return false;
    }

    int byte = CFifoGetByte(d->pDev->hTxFifo);
    if (byte < 0)
    {
        d->Regs->SCR &= (uint8_t)~(SCI_SCR_TIE | SCI_SCR_TEIE);
        TxReady(d, true);
        return false;
    }

    d->Regs->TDR = d->pDev->DataBits == 7 ?
        (uint8_t)byte & 0x7FU : (uint8_t)byte;
    Sync(d);

    if (CFifoUsed(d->pDev->hTxFifo) != 0)
    {
        d->Regs->SCR = (d->Regs->SCR & (uint8_t)~SCI_SCR_TEIE) |
            SCI_SCR_TIE;
    }
    else
    {
        // FIFO ownership ends as soon as the byte is submitted to TDR.
        // A later TxData may write TDR directly when TDRE is asserted.
        d->Regs->SCR &= (uint8_t)~(SCI_SCR_TIE | SCI_SCR_TEIE);
        TxReady(d, true);
    }

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
            if (CFifoPutByte(u->hRxFifo, byte))
            {
                notify = true;
            }
            else
            {
                ++u->RxDropCnt;
            }
            u->RxDropCnt += u->hRxFifo->DropCnt - dropped;
            length = CFifoUsed(u->hRxFifo);
            u->bRxReady = length != 0;
        } else if (errors) {
            event = UART_EVT_LINESTATE; length = 1; notify = true;
        }
    } else if (irq == (int)d->Irq[1]) {
        bool full = CFifoAvail(u->hTxFifo) == 0;
        if ((d->Regs->SCR & SCI_SCR_TIE) && Kick(d) && full) {
            event = UART_EVT_TXREADY; length = CFifoAvail(u->hTxFifo); notify = true;
        }
    } else if (irq == (int)d->Irq[2]) {
        uint8_t scr = d->Regs->SCR;
        if ((scr & SCI_SCR_TEIE) && (d->Regs->SSR & SCI_SSR_TEND)) {
            d->Regs->SCR = scr & (uint8_t)~SCI_SCR_TEIE;
            if (!d->DmaMode && CFifoUsed(u->hTxFifo) != 0)
            {
                Kick(d);
            }
            else
            {
                TxReady(d, true);
                event = UART_EVT_TXREADY;
                length = CFifoAvail(u->hTxFifo);
                notify = true;
            }
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
        TxReady(d, (d->Regs->SSR & SCI_SSR_TEND) != 0U);
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
            int byte = CFifoGetByte(d->pDev->hRxFifo);
            if (byte < 0)
            {
                EnableInterrupt(state);
                break;
            }
            buffer[count++] = (uint8_t)byte;
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
        if (d->DmaMode)
        {
            if (d->pDev->bTxReady &&
                CFifoUsed(d->pDev->hTxFifo) == 0 &&
                (d->Regs->SSR & SCI_SSR_TDRE) != 0U)
            {
                // Submit the first byte directly; there is no need to
                // enqueue and immediately remove it from the FIFO.
                d->Regs->TDR = d->pDev->DataBits == 7 ?
                    data[count] & 0x7FU : data[count];
                // Complete the SCI peripheral write before a subsequent
                // UARTTx call can observe the previous TDRE state.
                Sync(d);
                accepted = true;
            }
            else
            {
                uint32_t dropped = d->pDev->hTxFifo->DropCnt;
                bool wasReady = d->pDev->bTxReady;
                if (CFifoPutByte(d->pDev->hTxFifo, data[count]))
                {
                    accepted = true;
                    if (wasReady)
                    {
                        TxReady(d, false);
                        DmaStart(d);
                    }
                }
                d->pDev->TxDropCnt += d->pDev->hTxFifo->DropCnt - dropped;
            }
        }
        else if (p->bIntEn)
        {
            if (d->pDev->bTxReady &&
                (d->Regs->SSR & SCI_SSR_TDRE) != 0U)
            {
                // No queued bytes: submit directly to available TDR.
                // Keep software TX ready; TDR readiness gates the next write.
                uint8_t value = d->pDev->DataBits == 7 ?
                    data[count] & 0x7FU : data[count];
                d->Regs->SCR &= (uint8_t)~(SCI_SCR_TIE | SCI_SCR_TEIE);
                d->Regs->TDR = value;
                Sync(d);
                accepted = true;
            }
            else
            {
                uint32_t dropped = d->pDev->hTxFifo->DropCnt;
                if (CFifoPutByte(d->pDev->hTxFifo, data[count]))
                {
                    accepted = true;
                    TxReady(d, false);
                    // Service an already-empty TDR immediately. Merely
                    // enabling TXI does not guarantee a fresh TXI event.
                    Kick(d);
                }
                d->pDev->TxDropCnt += d->pDev->hTxFifo->DropCnt - dropped;
            }
        }
        else
        {
            // Conservative polling mode: require complete wire transmission
            // before accepting another byte. This isolates TDR/TSR timing
            // from the DMA and interrupt transfer paths.
            uint8_t status = d->Regs->SSR;
            if ((status & (SCI_SSR_TDRE | SCI_SSR_TEND)) ==
                (SCI_SSR_TDRE | SCI_SSR_TEND))
            {
                d->Regs->TDR = d->pDev->DataBits == 7 ?
                    data[count] & 0x7FU : data[count];
                TxReady(d, false);
                accepted = true;
            }
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
    Ra4m1UartBaud old = {(uint32_t)d->pDev->Rate, (uint8_t)(d->Regs->SMR&3U),
        d->Regs->BRR, d->Regs->MDDR, d->Regs->SEMR};
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
        d->Regs->SCR = 0;
        if (d->DmaMode)
        {
            s_DmaRegs[d->DmaChannel]->DMCNT_b.DTE = 0U;
            R_DMA->DELSR[d->DmaChannel] = 0U;
            d->DmaBusy = false;
            d->DmaCount = 0U;
        }
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
        if (d->Regs->SSR & (SCI_SSR_RDRF | SCI_SSR_ERRORS)) {
            uint8_t byte, errors;
            if (Receive(d, &byte, &errors)) ++d->pDev->RxDropCnt;
        }
        ClearRoutes(d);
        d->Enabled = true;
        for (unsigned i = 0; i < 4; ++i)
        {
            if ((int)d->Irq[i] >= 0)
            {
                NVIC_EnableIRQ(d->Irq[i]);
            }
        }
        if (d->DmaMode)
        {
            NVIC_EnableIRQ(d->DmaIrq);
        }
        TxReady(d, d->Tx);
        d->Regs->SCR = (d->Tx ? SCI_SCR_TE : 0U) | (d->Rx ? SCI_SCR_RE : 0U) |
            (d->Rx && p->bIntEn ? SCI_SCR_RIE : 0U);
        if (d->DmaMode)
        {
            DmaStart(d);
        }
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
        if (d->DmaMode)
        {
            NVIC_DisableIRQ(d->DmaIrq);
            Ra4m1UnregisterIntHandler(d->DmaIrq);
            d->DmaIrq = (IRQn_Type)-1;
        }
        RestorePins(d, 2);
        Clock(d, false);
        if (d->DmaMode)
        {
            bool used = false;
            for (unsigned i = 0; i < 4; ++i)
            {
                if (&s_Uart[i] != d && s_Uart[i].pDev && s_Uart[i].DmaMode)
                {
                    used = true;
                }
            }
            if (!used)
            {
                uint16_t protect = R_SYSTEM->PRCR & 15U;
                R_SYSTEM->PRCR = 0xA500U | protect | 0x02U;
                R_MSTP->MSTPCRA |= RA4M1_DMA_MSTP_MASK;
                R_SYSTEM->PRCR = 0xA500U | protect;
            }
        }
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
        cfg->FlowControl != UART_FLWCTRL_NONE || cfg->bIrDAMode ||
        (cfg->bDMAMode && (!cfg->bIntMode || !cfg->bFifoBlocking)) ||
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
    d->Regs = s_SciRegs[cfg->DevNo];
    d->StopMask = 1UL << (31U - s_Channel[cfg->DevNo]);
    d->Event = s_Event[cfg->DevNo];
    d->SyncCycles = (2U*((SystemCoreClock + clock-1U)/clock) + 2U);
    d->Enabled = false; d->Rx = present[0]; d->Tx = present[1];
    d->DmaMode = cfg->bDMAMode;
    d->DmaBusy = false;
    d->DmaCount = 0U;
    d->DmaChannel = (uint8_t)cfg->DevNo;
    d->DmaIrq = (IRQn_Type)-1;
    if (d->DmaMode && (R_MSTP->MSTPCRA & RA4M1_DMA_MSTP_MASK) == 0U &&
        (R_DMA->DELSR[d->DmaChannel] & 0x1FFU) != 0U)
    {
        EnableInterrupt(state);
        return false;
    }
    for (unsigned i = 0; i < 4; ++i) d->Irq[i] = (IRQn_Type)-1;
    // Do not take a running SCI instance from another peripheral implementation.
    if (!(R_MSTP->MSTPCRB & d->StopMask)) { EnableInterrupt(state); return false; }
    for (unsigned i = 0; i < 2; ++i) {
        d->Pins[i] = pins[i];
        d->OldPfs[i] = present[i] ? R_PFS->PORT[pins[i].PortNo].PIN[pins[i].PinNo].PmnPFS & ~RA4M1_PFS_PIDR : 0U;
        if (d->OldPfs[i] & RA4M1_PFS_ISEL) { EnableInterrupt(state); return false; }
    }
    if (!Clock(d, true)) { EnableInterrupt(state); return false; }
    if (d->DmaMode)
    {
        uint16_t protect = R_SYSTEM->PRCR & 15U;
        R_SYSTEM->PRCR = 0xA500U | protect | 0x02U;
        R_MSTP->MSTPCRA &= ~RA4M1_DMA_MSTP_MASK;
        R_SYSTEM->PRCR = 0xA500U | protect;
        __DSB();

        R_DMAC0_Type *dma = s_DmaRegs[d->DmaChannel];
        dma->DMCNT_b.DTE = 0U;
        dma->DMDAR = (uint32_t)(uintptr_t)&d->Regs->TDR;
        dma->DMCRB = 0U;
        dma->DMTMD = 1U | (2U << 12U);
        dma->DMAMD = (2U << 14U);
        dma->DMINT = 0x10U;

        R_DMA->DMAST_b.DMST = 1U;
        R_DMA->DELSR[d->DmaChannel] = d->Event + 1U;
    }
    d->Regs->SCR = 0;
    if (cfg->DevNo < 2) d->Regs->FCR = 0; // Disable hardware FIFO.
    d->Regs->SPTR = SCI_SPTR_MARK;
    d->Regs->SIMR1 = 0; d->Regs->SPMR = 0;
    d->Regs->DCCR = 0x40U; // DCME=0, IDSEL=1 (reset value); address matching disabled.
    d->Regs->SCMR = 0xF2U; // CHR1=1, LSB first, no inversion, SMIF=0.
    d->Regs->SNFR = 0;
    uint8_t smr = (cfg->DataBits == 7 ? 0x40U : 0U) | (cfg->StopBits == 2 ? 8U : 0U);
    if (cfg->Parity != UART_PARITY_NONE) smr |= 0x20U | (cfg->Parity == UART_PARITY_ODD ? 0x10U : 0U);
    d->Regs->SMR = smr;
    unsigned configured = 0;
    bool ok = d->Regs->SCR == 0 && d->Regs->SCMR == 0xF2 &&
        d->Regs->SMR == smr && d->Regs->SIMR1 == 0 &&
        d->Regs->SPMR == 0 && d->Regs->DCCR == 0x40 &&
        (d->Regs->SPTR & SCI_SPTR_MARK) == SCI_SPTR_MARK &&
        (cfg->DevNo >= 2 || !(d->Regs->FCR & 1U)) && ApplyBaud(d, baud);
    if (ok && cfg->bIntMode) {
        for (unsigned i = 0; i < 4 && ok; ++i) {
            if ((i == 0 || i == 3) ? !d->Rx : !d->Tx || (d->DmaMode && i == 1U)) continue;
            d->Irq[i] = Ra4m1RegisterIntHandler(d->Event+i, cfg->IntPrio, UartIrq, d);
            ok = (int)d->Irq[i] >= 0;
        }
    }
    if (ok && d->DmaMode)
    {
        d->DmaIrq = Ra4m1RegisterIntHandler(RA4M1_EVTID_DMAC0_INT +
            d->DmaChannel, cfg->IntPrio, DmaIrqHandler, d);
        ok = (int)d->DmaIrq >= 0;
    }
    for (unsigned i = 0; i < 2 && ok; ++i) {
        configured = i+1;
        if (!present[i]) continue;
        IOPinCfg(&pins[i], 1);
        uint32_t mask = RA4M1_PFS_PSEL | RA4M1_PFS_PMR | RA4M1_PFS_PDR |
            RA4M1_PFS_ASEL | RA4M1_PFS_PCR | RA4M1_PFS_NCODR;
        uint32_t value = ((uint32_t)pins[i].PinOp << 24) | RA4M1_PFS_PMR |
            (i == 1 ? RA4M1_PFS_PDR : 0U) | (pins[i].Res == IOPINRES_PULLUP ? RA4M1_PFS_PCR : 0U);
        ok = (R_PFS->PORT[pins[i].PortNo].PIN[pins[i].PinNo].PmnPFS & mask) == value;
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
        d->Regs->SCR = 0;
        ReleaseRoutes(d);
        if ((int)d->DmaIrq >= 0)
        {
            Ra4m1UnregisterIntHandler(d->DmaIrq);
        }
        RestorePins(d, configured);
        Clock(d, false);
        EnableInterrupt(state); return false;
    }
    u->Mode = cfg->Mode; u->Duplex = cfg->Duplex; u->Rate = (int)baud.Actual;
    u->DataBits = cfg->DataBits; u->Parity = cfg->Parity; u->StopBits = cfg->StopBits;
    u->FlowControl = cfg->FlowControl;
    u->bIrDAMode = u->bIrDAInvert = u->bIrDAFixPulse = false; u->IrDAPulseDiv = 0;
    u->EvtCallback = cfg->EvtCallback; u->hRxFifo = rx; u->hTxFifo = tx;
    u->LineState = 0; u->hStdIn = u->hStdOut = -1;
    u->RxOvrErrCnt = u->ParErrCnt = u->FramErrCnt = u->RxDropCnt = u->TxDropCnt = 0;
    u->bRxReady = false;
    u->bTxReady = false;
    // Preserve pObj installed by the existing C++ UART constructor.
    DevIntrf_t *p = &u->DevIntrf;
    p->pDevData = d; p->IntPrio = cfg->IntPrio; p->EvtCB = nullptr;
    p->Type = DEVINTRF_TYPE_UART; p->bDma = cfg->bDMAMode; p->bIntEn = cfg->bIntMode;
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

