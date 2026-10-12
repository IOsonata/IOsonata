/**-------------------------------------------------------------------------
@file	uart_ra4m1.cpp

@brief	Native RA4M1 UART with SCI0/SCI1 hardware FIFO support.

Uses the IOsonata UARTDEV, DevIntrf and CFifo interfaces. SCI2/SCI9 retain
single-register operation. Register definitions come from Renesas CMSIS.

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
#include <string.h>

#include "ra4m1xxx.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "interrupt_ra4m1.h"
#include "ra4m1_uart_baud.h"

#define SCI_SCR_RE R_SCI0_SCR_RE_Msk
#define SCI_SCR_TE R_SCI0_SCR_TE_Msk
#define SCI_SCR_RIE R_SCI0_SCR_RIE_Msk
#define SCI_SCR_TIE R_SCI0_SCR_TIE_Msk
#define SCI_SSR_TEND R_SCI0_SSR_TEND_Msk
#define SCI_SSR_PER R_SCI0_SSR_PER_Msk
#define SCI_SSR_FER R_SCI0_SSR_FER_Msk
#define SCI_SSR_ORER R_SCI0_SSR_ORER_Msk
#define SCI_SSR_RDRF R_SCI0_SSR_RDRF_Msk
#define SCI_SSR_TDRE R_SCI0_SSR_TDRE_Msk
#define SCI_SSR_ERRORS (SCI_SSR_PER | SCI_SSR_FER | SCI_SSR_ORER)
#define SCI_SPTR_MARK (R_SCI0_SPTR_SPB2IO_Msk | R_SCI0_SPTR_SPB2DT_Msk)
#define SCI_HW_FIFO_SIZE 16
#define SCI_RX_TRIGGER 8
#define SCI_TX_TRIGGER 8
#define RA4M1_DMA_MSTP_MASK (1UL << 22U)

#ifndef RA4M1_UART_FIFO_SIZE
#define RA4M1_UART_FIFO_SIZE 16
#endif
static_assert(RA4M1_UART_FIFO_SIZE > 0, "UART FIFO must not be empty");

struct Ra4m1UartDev
{
	UARTDEV *pDev;
	R_SCI0_Type *Regs;
	uint32_t StopMask;
	IRQn_Type Irq[4]; // RXI, TXI, unused TEI slot, ERI (hardware event order)
	IOPINCFG Pins[2];
	uint32_t OldPfs[2];
	uint32_t SyncCycles;
	uint32_t Generation;
	uint8_t Event;
	uint8_t DmaChannel;
	IRQn_Type DmaIrq;
	bool FifoMode;
	bool DmaMode;
	bool DmaBusy;
	bool Enabled;
	bool Rx;
	bool Tx;
	uint16_t DmaCount;
	alignas(4) uint8_t RxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
	alignas(4) uint8_t TxMem[CFIFO_MEMSIZE(RA4M1_UART_FIFO_SIZE)];
};
static R_SCI0_Type *const s_SciRegs[4] = {R_SCI0, R_SCI1, R_SCI2, R_SCI9};
static Ra4m1UartDev s_Uart[4];
static R_DMAC0_Type *const s_DmaRegs[4] = {R_DMAC0, reinterpret_cast<R_DMAC0_Type *>(R_DMAC1),
                                           reinterpret_cast<R_DMAC0_Type *>(R_DMAC2),
                                           reinterpret_cast<R_DMAC0_Type *>(R_DMAC3)};
static const uint8_t s_Channel[] = {0, 1, 2, 9};
static const uint8_t s_Event[] = {RA4M1_EVTID_SCI0_RXI, RA4M1_EVTID_SCI1_RXI, RA4M1_EVTID_SCI2_RXI,
                                  RA4M1_EVTID_SCI9_RXI};

static Ra4m1UartDev *Owner(DevIntrf_t *p)
{
	if (!p || !p->pDevData)
	{
		return nullptr;
	}
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
	for (unsigned i = 0; i < d->SyncCycles; ++i)
	{
		__NOP();
	}
}
/* PRIMASK is held by the caller. Preserve unrelated module/protection bits. */
static bool Clock(Ra4m1UartDev *d, bool enable)
{
	uint16_t prcr = R_SYSTEM->PRCR & 15U;
	uint32_t value = R_MSTP->MSTPCRB;
	value = enable ? value & ~d->StopMask : value | d->StopMask;
	R_SYSTEM->PRCR = 0xA500U | prcr | 0x02U;
	R_MSTP->MSTPCRB = value;
	bool ok = false;
	for (unsigned i = 0; i < 64; ++i)
	{
		if ((R_MSTP->MSTPCRB & d->StopMask) == (enable ? 0U : d->StopMask))
		{
			ok = true;
			break;
		}
	}
	R_SYSTEM->PRCR = 0xA500U | prcr;
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
	return d->Regs->SMR == smr && d->Regs->SEMR == b.Semr && d->Regs->BRR == b.Brr &&
	       d->Regs->MDDR == b.Mddr;
}
static void RestorePins(Ra4m1UartDev *d, unsigned count)
{
	uint8_t pwpr = R_PMISC->PWPR;
	R_PMISC->PWPR = pwpr & ~RA4M1_PWPR_B0WI;
	R_PMISC->PWPR = RA4M1_PWPR_PFSWE;
	for (unsigned i = 0; i < count; ++i)
	{
		const IOPINCFG &pin = d->Pins[i];
		if (pin.PortNo < 0)
		{
			continue;
		}
		volatile uint32_t *addr = &R_PFS->PORT[pin.PortNo].PIN[pin.PinNo].PmnPFS;
		*addr = (*addr) & ~(RA4M1_PFS_PMR | RA4M1_PFS_PIDR);
		*addr = d->OldPfs[i] & ~RA4M1_PFS_PMR;
		if (d->OldPfs[i] & RA4M1_PFS_PMR)
		{
			*addr = d->OldPfs[i];
		}
	}
	R_PMISC->PWPR = pwpr & ~RA4M1_PWPR_B0WI;
	R_PMISC->PWPR = pwpr;
}
static void ClearRoutes(Ra4m1UartDev *d)
{
	Sync(d);
	for (unsigned i = 0; i < 4; ++i)
	{
		if ((int)d->Irq[i] >= 0)
		{
			Ra4m1AcknowledgeInt(d->Irq[i]);
			NVIC_ClearPendingIRQ(d->Irq[i]); // Only while the source is stopped.
		}
	}
}
static void ReleaseRoutes(Ra4m1UartDev *d)
{
	Sync(d);
	for (unsigned i = 0; i < 4; ++i)
	{
		if ((int)d->Irq[i] >= 0)
		{
			Ra4m1UnregisterIntHandler(d->Irq[i]);
			d->Irq[i] = (IRQn_Type)-1;
		}
	}
}
/* Read hardware first, before software FIFO accounting or callbacks.
 * FRDRHL includes the error status of each individual FIFO entry. An overrun
 * loses an arriving byte, not the valid bytes already held in RDR/FRDRHL.
 * Manual 28.2.4, 28.2.13, 28.2.14 and 28.3.9.
 */
static int Receive(Ra4m1UartDev *d, uint8_t *data, int limit, uint8_t *errors)
{
	UARTDEV *u = d->pDev;
	uint8_t status = d->Regs->SSR;
	uint8_t clear = status & SCI_SSR_ERRORS;
	int available =
	    d->FifoMode ? (int)d->Regs->FDR_b.R : ((status & (SCI_SSR_RDRF | SCI_SSR_ERRORS)) != 0U);
	int count = available < limit ? available : limit;
	int received = 0;
	*errors = 0U;

	if (status & SCI_SSR_ORER)
	{
		++u->RxOvrErrCnt;
		++u->RxDropCnt;
		*errors |= UART_LINESTATE_OVR;
	}
	for (int i = 0; i < count; ++i)
	{
		uint16_t frame = d->FifoMode ? d->Regs->FRDRHL : d->Regs->RDR;
		uint8_t error = d->FifoMode ? (uint8_t)(frame >> 8) & (SCI_SSR_FER | SCI_SSR_PER)
		                            : status & (SCI_SSR_FER | SCI_SSR_PER);

		if (error & SCI_SSR_PER)
		{
			++u->ParErrCnt;
			*errors |= UART_LINESTATE_PARERR;
		}
		if (error & SCI_SSR_FER)
		{
			++u->FramErrCnt;
			*errors |= UART_LINESTATE_FRMERR;
			if ((d->Regs->SPTR & R_SCI0_SPTR_RXDMON_Msk) == 0U)
			{
				*errors |= UART_LINESTATE_BRK;
			}
		}
		if (error)
		{
			++u->RxDropCnt;
		}
		else
		{
			data[received++] = (uint8_t)frame & (u->DataBits == 7 ? 0x7FU : 0xFFU);
		}
	}
	if (d->FifoMode)
	{
		// A partial polling read must leave the remaining error entries
		// and the data-ready condition for the next receive operation.
		if (count < available)
		{
			clear &= (uint8_t)~(SCI_SSR_FER | SCI_SSR_PER);
		}
		clear |= status & R_SCI0_SSR_FIFO_RDF_Msk;
		if (d->Regs->FDR_b.R == 0U)
		{
			clear |= status & R_SCI0_SSR_FIFO_DR_Msk;
		}
	}
	if (clear)
	{
		// Write ones to unrelated flags; a read/modify/write could lose
		// a newly asserted flag. Never clear non-FIFO RDRF in software.
		d->Regs->SSR = (uint8_t)~clear;
		(void)d->Regs->SSR;
	}
	if (*errors)
	{
		u->LineState = *errors;
	}
	return received;
}

/* The CPU owns TX while DMA is idle. FIFO access is bounded by the room
 * observed on entry, not by a flag that can stay set after a CPU write.
 */
static int WriteTx(Ra4m1UartDev *d, const uint8_t *data, int count)
{
	uint8_t status = d->Regs->SSR;
	int room = d->FifoMode ? SCI_HW_FIFO_SIZE - d->Regs->FDR_b.T : ((status & SCI_SSR_TDRE) != 0U);
	if (count > room)
	{
		count = room;
	}
	for (int i = 0; i < count; ++i)
	{
		uint8_t byte = data[i] & (d->pDev->DataBits == 7 ? 0x7FU : 0xFFU);
		if (d->FifoMode)
		{
			d->Regs->FTDRL = byte;
		}
		else
		{
			d->Regs->TDR = byte;
		}
	}
	if (count)
	{
		if (d->FifoMode && (status & R_SCI0_SSR_FIFO_TDFE_Msk))
		{
			d->Regs->SSR_FIFO = (uint8_t)~R_SCI0_SSR_FIFO_TDFE_Msk;
		}
		Sync(d);
	}
	return count;
}

// Software readiness means the queued TX data have been handed to hardware.

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

	// Prime the available hardware space before arming DMA. Non-FIFO
	// channels need this to create their next TXI transition. FIFO DMA
	// uses TTRG=15 and one byte per activation, so it cannot overfill.
	int sent = WriteTx(d, source, count);
	CFifoRelease(d->pDev->hTxFifo, sent);
	source += sent;
	count -= sent;
	if (count == 0)
	{
		count = 0xFFFF;
		source = CFifoPeekMultiple(d->pDev->hTxFifo, &count);
		if (source == nullptr)
		{
			TxReady(d, true);
			return sent != 0;
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
	R_DMAC0_Type *dma = s_DmaRegs[d->DmaChannel];

	dma->DMCNT_b.DTE = 0U;
	dma->DMSTS = 0U;
	(void)dma->DMSTS;
	if (!d->Enabled || d->pDev == nullptr || !d->DmaBusy)
	{
		Sync(d);
		return;
	}
	d->Regs->SCR &= (uint8_t)~SCI_SCR_TIE;
	CFifoRelease(d->pDev->hTxFifo, d->DmaCount);
	d->DmaCount = 0U;
	d->DmaBusy = false;

	if (!DmaStart(d))
	{
		TxReady(d, true);
	}
	Sync(d);
	UARTDEV *u = d->pDev;
	if (u->EvtCallback)
	{
		int space = CFifoAvail(u->hTxFifo);
		Ra4m1AcknowledgeInt((IRQn_Type)irq);
		u->EvtCallback(u, UART_EVT_TXREADY, nullptr, space);
	}
}

/* Called by the UART ISR, or with interrupts masked by the foreground
 * producer. All routes for one UART, including DMA, have equal priority.
 */
static int FillTx(Ra4m1UartDev *d)
{
	int sent = 0;
	while (sent < SCI_HW_FIFO_SIZE)
	{
		int count = SCI_HW_FIFO_SIZE - sent;
		uint8_t *data = CFifoPeekMultiple(d->pDev->hTxFifo, &count);
		if (data == nullptr)
		{
			break;
		}
		int written = WriteTx(d, data, count);
		CFifoRelease(d->pDev->hTxFifo, written);
		sent += written;
		if (written < count || !d->FifoMode)
		{
			break;
		}
	}
	bool pending = CFifoUsed(d->pDev->hTxFifo) != 0;
	if (pending)
	{
		d->Regs->SCR |= SCI_SCR_TIE;
	}
	else
	{
		d->Regs->SCR &= (uint8_t)~SCI_SCR_TIE;
	}
	TxReady(d, !pending);
	return sent;
}

static void UartIrq(int irq, void *ctx)
{
	Ra4m1UartDev *d = static_cast<Ra4m1UartDev *>(ctx);
	UARTDEV *u = d->pDev;
	if (u == nullptr || !d->Enabled)
	{
		return;
	}
	uint8_t data[SCI_HW_FIFO_SIZE];
	uint8_t errors = 0;
	int received = 0;
	bool txReady = false;

	// RX is serviced first even when TXI caused this entry. Equal-priority
	// SCI interrupts cannot preempt one another; no handler-wide PRIMASK.
	if (d->Rx)
	{
		received = Receive(d, data, sizeof(data), &errors);
		uint32_t dropped = u->hRxFifo->DropCnt;
		for (int i = 0; i < received; ++i)
		{
			if (!CFifoPutByte(u->hRxFifo, data[i]))
			{
				++u->RxDropCnt;
			}
		}
		u->RxDropCnt += u->hRxFifo->DropCnt - dropped;
		if (received)
		{
			u->bRxReady = true;
		}
	}
	if (d->Tx && !d->DmaMode && (d->Regs->SCR & SCI_SCR_TIE))
	{
		txReady = FillTx(d) != 0;
	}
	Sync(d);
	UARTEvtHandler_t callback = u->EvtCallback;
	if (callback)
	{
		// Callback code may disable, power off, or reinitialize this device.
		// Retire this event first and do not deliver stale notifications.
		uint32_t generation = d->Generation;
		Ra4m1AcknowledgeInt((IRQn_Type)irq);
		if (errors)
		{
			callback(u, UART_EVT_LINESTATE, &errors, 1);
			if (d->Generation != generation || d->pDev != u || !d->Enabled)
			{
				return;
			}
		}
		if (received)
		{
			int length = CFifoUsed(u->hRxFifo);
			if (length)
			{
				callback(u, UART_EVT_RXDATA, nullptr, length);
			}
			if (d->Generation != generation || d->pDev != u || !d->Enabled)
			{
				return;
			}
		}
		if (txReady)
		{
			int space = CFifoAvail(u->hTxFifo);
			if (space)
			{
				callback(u, UART_EVT_TXREADY, nullptr, space);
			}
		}
	}
	// Without callbacks Ra4m1Dispatch acknowledges after source negation.
}

static bool StartRx(DevIntrf_t *p, uint32_t)
{
	Ra4m1UartDev *d = Owner(p);
	return d && d->Enabled && d->Rx;
}
static bool StartTx(DevIntrf_t *p, uint32_t)
{
	Ra4m1UartDev *d = Owner(p);
	return d && d->Enabled && d->Tx;
}
static void Stop(DevIntrf_t *p)
{
	// Generic Start/Stop wrappers own bBusy. Polling has no TEI callback;
	// sample wire completion on API return without waiting with IRQs masked.
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = Owner(p);
	if (d && d->Enabled && d->Tx && !p->bIntEn)
	{
		TxReady(d, (d->Regs->SSR & SCI_SSR_TEND) != 0U);
	}
	EnableInterrupt(state);
}
static int RxData(DevIntrf_t *p, uint8_t *buffer, int len)
{
	if (!buffer || len <= 0)
	{
		return 0;
	}

	int count = 0;

	while (count < len)
	{
		uint32_t state = DisableInterrupt();
		Ra4m1UartDev *d = Owner(p);

		if (!d || !d->Enabled || !d->Rx)
		{
			EnableInterrupt(state);
			break;
		}
		if (p->bIntEn)
		{
			// Bound each masked copy, even for a large application buffer.
			int length = len - count;

			if (length > 8)
			{
				length = 8;
			}
			uint8_t *data = CFifoGetMultiple(d->pDev->hRxFifo, &length);

			if (data == nullptr)
			{
				d->pDev->bRxReady = false;
				EnableInterrupt(state);
				break;
			}
			// GetMultiple releases the slots. Keep the producer excluded
			// until their contents have been copied to the caller.
			memcpy(buffer + count, data, length);
			count += length;
			d->pDev->bRxReady = CFifoUsed(d->pDev->hRxFifo) != 0;
		}
		else
		{
			uint8_t errors;
			int length = len - count;
			if (length > SCI_HW_FIFO_SIZE)
			{
				length = SCI_HW_FIFO_SIZE;
			}
			int received = Receive(d, buffer + count, length, &errors);
			if (received == 0)
			{
				EnableInterrupt(state);
				break;
			}
			count += received;
		}
		EnableInterrupt(state);
	}

	return count;
}
static int TxData(DevIntrf_t *p, const uint8_t *data, int len)
{
	if (data == nullptr || len <= 0)
	{
		return 0;
	}
	int total = 0;
	int retries = p->MaxRetry > 0 ? p->MaxRetry : 1;
	while (total < len && retries > 0)
	{
		uint32_t state = DisableInterrupt();
		Ra4m1UartDev *d = Owner(p);
		if (d == nullptr || !d->Enabled || !d->Tx)
		{
			EnableInterrupt(state);
			break;
		}
		int count = len - total;
		if (count > SCI_HW_FIFO_SIZE)
		{
			count = SCI_HW_FIFO_SIZE;
		}
		if (p->bIntEn)
		{
			// One bounded copy and one hardware kick per contiguous block.
			// DMA retains its queued source until transfer completion.
			uint32_t dropped = d->pDev->hTxFifo->DropCnt;
			uint8_t *dest = CFifoPutMultiple(d->pDev->hTxFifo, &count);
			if (dest)
			{
				memcpy(dest, data + total, count);
				TxReady(d, false);
			}
			else
			{
				count = 0;
			}
			d->pDev->TxDropCnt += d->pDev->hTxFifo->DropCnt - dropped;
			if (d->DmaMode)
			{
				DmaStart(d);
			}
			else
			{
				FillTx(d);
			}
		}
		else
		{
			// Polling never accepts bytes into a software queue.
			count = WriteTx(d, data + total, count);
		}
		EnableInterrupt(state);
		total += count;
		if (count)
		{
			retries = p->MaxRetry > 0 ? p->MaxRetry : 1;
		}
		else
		{
			--retries;
		}
	}
	return total;
}

static uint32_t GetRate(DevIntrf_t *p)
{
	Ra4m1UartDev *d = Owner(p);
	return d ? (uint32_t)d->pDev->Rate : 0U;
}
static uint32_t SetRate(DevIntrf_t *p, uint32_t rate)
{
	Ra4m1UartBaud baud;
	uint32_t clock = SystemPeriphClockGet(0); // SCI uses PCLKA, NOT PCLKB.
	if (!Ra4m1UartPlanBaud(clock, rate, &baud))
	{
		return 0;
	}
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = Owner(p);
	// Receiver has no idle flag. Require Disable and a quiesced peer first.
	if (!d || d->Enabled)
	{
		EnableInterrupt(state);
		return 0;
	}
	Ra4m1UartBaud old = {(uint32_t)d->pDev->Rate, (uint8_t)(d->Regs->SMR & 3U), d->Regs->BRR,
	                     d->Regs->MDDR, d->Regs->SEMR};
	bool ok = ApplyBaud(d, baud);
	if (ok)
	{
		d->SyncCycles = (2U * ((SystemCoreClock + clock - 1U) / clock) + 2U);
		d->pDev->Rate = (int)baud.Actual;
	}
	else
	{
		ApplyBaud(d, old);
	}
	EnableInterrupt(state);
	return ok ? baud.Actual : 0U;
}
static void Disable(DevIntrf_t *p)
{
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = Owner(p);
	if (d && d->Enabled)
	{
		++d->Generation;
		for (unsigned i = 0; i < 4; ++i)
		{
			if ((int)d->Irq[i] >= 0)
			{
				NVIC_DisableIRQ(d->Irq[i]);
			}
		}
		d->Regs->SCR = 0;
		if (d->DmaMode)
		{
			NVIC_DisableIRQ(d->DmaIrq);
			s_DmaRegs[d->DmaChannel]->DMCNT_b.DTE = 0U;
			s_DmaRegs[d->DmaChannel]->DMSTS = 0U;
			(void)s_DmaRegs[d->DmaChannel]->DMSTS;
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

static void ResetHardware(Ra4m1UartDev *d)
{
	if (d->FifoMode)
	{
		d->pDev->RxDropCnt += d->Regs->FDR_b.R;
		d->Regs->FCR |= R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk;
		Sync(d);
	}
	else if (d->Regs->SSR & (SCI_SSR_RDRF | SCI_SSR_ERRORS))
	{
		(void)d->Regs->RDR;
		++d->pDev->RxDropCnt;
	}
	uint8_t status = d->Regs->SSR;
	d->Regs->SSR = (uint8_t)~(status & SCI_SSR_ERRORS);
	(void)d->Regs->SSR;
}

static void Enable(DevIntrf_t *p)
{
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = Owner(p);
	if (d && !d->Enabled)
	{
		// Retain configuration/CFifo RX data. Disable does not gate the module.
		ResetHardware(d);
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
			R_DMA->DELSR[d->DmaChannel] = d->Event + 1U;
			Ra4m1AcknowledgeInt(d->DmaIrq);
			NVIC_ClearPendingIRQ(d->DmaIrq);
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
	if (d)
	{
		bool enabled = d->Enabled;
		Disable(p);
		CFifoFlush(d->pDev->hRxFifo);
		d->pDev->bRxReady = false;
		if (enabled)
		{
			Enable(p);
		}
	}
	EnableInterrupt(state);
}
static void PowerOff(DevIntrf_t *p)
{
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = Owner(p);
	if (d)
	{
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
	Ra4m1UartDev *d = Owner(p);
	return d ? d->pDev : nullptr;
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
bool UARTInit(UARTDEV *const u, const UARTCFG *cfg)
{
	if (!u || !cfg || (unsigned)cfg->DevNo >= 4U || !cfg->pIOPinMap || cfg->NbIOPins != 2 ||
	    cfg->Rate <= 0 || (cfg->DataBits != 7 && cfg->DataBits != 8) ||
	    (cfg->StopBits != 1 && cfg->StopBits != 2) ||
	    (cfg->Parity != UART_PARITY_NONE && cfg->Parity != UART_PARITY_ODD &&
	     cfg->Parity != UART_PARITY_EVEN) ||
	    cfg->Mode != UART_MODE_UART || cfg->Duplex != UART_DUPLEX_FULL ||
	    cfg->FlowControl != UART_FLWCTRL_NONE || cfg->bIrDAMode ||
	    (cfg->bDMAMode && (!cfg->bIntMode || !cfg->bFifoBlocking)) ||
	    (cfg->bIntMode && (unsigned)cfg->IntPrio >= (1UL << __NVIC_PRIO_BITS)) ||
	    !MemoryValid(cfg->pRxMem, cfg->RxMemSize) || !MemoryValid(cfg->pTxMem, cfg->TxMemSize))
	{
		return false;
	}
	if (cfg->pRxMem && cfg->pTxMem)
	{
		uintptr_t rx = (uintptr_t)cfg->pRxMem, tx = (uintptr_t)cfg->pTxMem;
		if (rx < tx ? tx - rx < (unsigned)cfg->RxMemSize : rx - tx < (unsigned)cfg->TxMemSize)
		{
			return false;
		}
	}
	const IOPINCFG *pins = static_cast<const IOPINCFG *>(cfg->pIOPinMap);
	bool present[2];
	for (unsigned i = 0; i < 2; ++i)
	{
		const IOPINCFG &pin = pins[i];
		present[i] = pin.PortNo != -1 || pin.PinNo != -1;
		if (!present[i])
		{
			continue;
		}
		if (!Ra4m1PinValid(pin.PortNo, pin.PinNo) || pin.PortNo == 9 || pin.PinOp < IOPINOP_FUNC0 ||
		    pin.PinOp > IOPINOP_FUNC30 ||
		    pin.PinDir != (i == 0 ? IOPINDIR_INPUT : IOPINDIR_OUTPUT) ||
		    (pin.Res != IOPINRES_NONE && pin.Res != IOPINRES_PULLUP) ||
		    pin.Type != IOPINTYPE_NORMAL)
		{
			return false;
		}
	}
	if ((!present[0] && !present[1]) ||
	    (present[0] && present[1] && pins[0].PortNo == pins[1].PortNo &&
	     pins[0].PinNo == pins[1].PinNo))
	{
		return false;
	}
	Ra4m1UartBaud baud;
	uint32_t clock = SystemPeriphClockGet(0);
	if (!Ra4m1UartPlanBaud(clock, (uint32_t)cfg->Rate, &baud) || !SystemCoreClock ||
	    SystemCoreClock > 48000000U)
	{
		return false;
	}
	uint32_t state = DisableInterrupt();
	Ra4m1UartDev *d = &s_Uart[cfg->DevNo];
	for (unsigned i = 0; i < 4; ++i)
	{
		if (s_Uart[i].pDev == u || (i == (unsigned)cfg->DevNo && s_Uart[i].pDev))
		{
			EnableInterrupt(state);
			return false;
		}
		if (s_Uart[i].pDev)
		{
			for (unsigned a = 0; a < 2; ++a)
			{
				for (unsigned b = 0; b < 2; ++b)
				{
					if (present[a] && s_Uart[i].Pins[b].PortNo == pins[a].PortNo &&
					    s_Uart[i].Pins[b].PinNo == pins[a].PinNo)
					{
						EnableInterrupt(state);
						return false;
					}
				}
			}
		}
	}
	d->Regs = s_SciRegs[cfg->DevNo];
	d->StopMask = 1UL << (31U - s_Channel[cfg->DevNo]);
	d->Event = s_Event[cfg->DevNo];
	d->SyncCycles = (2U * ((SystemCoreClock + clock - 1U) / clock) + 2U);
	d->Enabled = false;
	d->Rx = present[0];
	d->Tx = present[1];
	d->FifoMode = cfg->DevNo < 2;
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
	for (unsigned i = 0; i < 4; ++i)
	{
		d->Irq[i] = (IRQn_Type)-1;
	}
	// Do not take a running SCI instance from another peripheral implementation.
	if (!(R_MSTP->MSTPCRB & d->StopMask))
	{
		EnableInterrupt(state);
		return false;
	}
	for (unsigned i = 0; i < 2; ++i)
	{
		d->Pins[i] = pins[i];
		d->OldPfs[i] = present[i]
		                   ? R_PFS->PORT[pins[i].PortNo].PIN[pins[i].PinNo].PmnPFS & ~RA4M1_PFS_PIDR
		                   : 0U;
		if (d->OldPfs[i] & RA4M1_PFS_ISEL)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	if (!Clock(d, true))
	{
		EnableInterrupt(state);
		return false;
	}
	bool dmaClockStopped = (R_MSTP->MSTPCRA & RA4M1_DMA_MSTP_MASK) != 0U;
	if (d->DmaMode)
	{
		uint16_t protect = R_SYSTEM->PRCR & 15U;
		R_SYSTEM->PRCR = 0xA500U | protect | 0x02U;
		R_MSTP->MSTPCRA &= ~RA4M1_DMA_MSTP_MASK;
		R_SYSTEM->PRCR = 0xA500U | protect;
		__DSB();

		R_DMAC0_Type *dma = s_DmaRegs[d->DmaChannel];
		dma->DMCNT_b.DTE = 0U;
		dma->DMDAR = (uint32_t)(uintptr_t)(d->FifoMode ? &d->Regs->FTDRL : &d->Regs->TDR);
		dma->DMCRB = 0U;
		dma->DMTMD = 1U | (2U << 12U);
		dma->DMAMD = (2U << 14U);
		dma->DMINT = 0x10U;

		R_DMA->DMAST_b.DMST = 1U;
		R_DMA->DELSR[d->DmaChannel] = d->Event + 1U;
	}
	d->Regs->SCR = 0;
	if (d->FifoMode)
	{
		// RXI at eight bytes, with DR timeout delivering shorter bursts.
		// Interrupt TX refills half-empty FIFO; normal-mode DMA transfers
		// one byte whenever at least one of the sixteen slots is free.
		uint16_t txTrigger = d->DmaMode ? SCI_HW_FIFO_SIZE - 1 : SCI_TX_TRIGGER;
		d->Regs->FCR = R_SCI0_FCR_FM_Msk | R_SCI0_FCR_RFRST_Msk | R_SCI0_FCR_TFRST_Msk |
		               (SCI_RX_TRIGGER << R_SCI0_FCR_RTRG_Pos) |
		               (txTrigger << R_SCI0_FCR_TTRG_Pos) | R_SCI0_FCR_RSTRG_Msk;
		Sync(d);
	}
	d->Regs->SPTR = SCI_SPTR_MARK;
	d->Regs->SIMR1 = 0;
	d->Regs->SPMR = 0;
	d->Regs->DCCR = 0x40U; // DCME=0, IDSEL=1 (reset value); address matching disabled.
	d->Regs->SCMR = 0xF2U; // CHR1=1, LSB first, no inversion, SMIF=0.
	d->Regs->SNFR = 0;
	uint8_t smr = (cfg->DataBits == 7 ? 0x40U : 0U) | (cfg->StopBits == 2 ? 8U : 0U);
	if (cfg->Parity != UART_PARITY_NONE)
	{
		smr |= 0x20U | (cfg->Parity == UART_PARITY_ODD ? 0x10U : 0U);
	}
	d->Regs->SMR = smr;
	unsigned configured = 0;
	bool ok = d->Regs->SCR == 0 && d->Regs->SCMR == 0xF2 && d->Regs->SMR == smr &&
	          d->Regs->SIMR1 == 0 && d->Regs->SPMR == 0 && d->Regs->DCCR == 0x40 &&
	          (d->Regs->SPTR & SCI_SPTR_MARK) == SCI_SPTR_MARK &&
	          (!d->FifoMode || (d->Regs->FCR & R_SCI0_FCR_FM_Msk)) && ApplyBaud(d, baud);
	if (ok && cfg->bIntMode)
	{
		for (unsigned i = 0; i < 4 && ok; ++i)
		{
			// Readiness is software-FIFO availability; no TEI is needed.
			if (i == 2U || ((i == 0 || i == 3) ? !d->Rx : !d->Tx || d->DmaMode))
			{
				continue;
			}
			d->Irq[i] = Ra4m1RegisterIntHandler(d->Event + i, cfg->IntPrio, UartIrq, d);
			ok = (int)d->Irq[i] >= 0;
		}
	}
	if (ok && d->DmaMode)
	{
		d->DmaIrq = Ra4m1RegisterIntHandler(RA4M1_EVTID_DMAC0_INT + d->DmaChannel, cfg->IntPrio,
		                                    DmaIrqHandler, d);
		ok = (int)d->DmaIrq >= 0;
	}
	for (unsigned i = 0; i < 2 && ok; ++i)
	{
		configured = i + 1;
		if (!present[i])
		{
			continue;
		}
		IOPinCfg(&pins[i], 1);
		uint32_t mask = RA4M1_PFS_PSEL | RA4M1_PFS_PMR | RA4M1_PFS_PDR | RA4M1_PFS_ASEL |
		                RA4M1_PFS_PCR | RA4M1_PFS_NCODR;
		uint32_t value = ((uint32_t)pins[i].PinOp << 24) | RA4M1_PFS_PMR |
		                 (i == 1 ? RA4M1_PFS_PDR : 0U) |
		                 (pins[i].Res == IOPINRES_PULLUP ? RA4M1_PFS_PCR : 0U);
		ok = (R_PFS->PORT[pins[i].PortNo].PIN[pins[i].PinNo].PmnPFS & mask) == value;
	}
	hCFifo_t rx = nullptr, tx = nullptr;
	if (ok)
	{
		rx = CFifoInit(cfg->pRxMem ? cfg->pRxMem : d->RxMem,
		               cfg->pRxMem ? (unsigned)cfg->RxMemSize : sizeof(d->RxMem), 1,
		               cfg->bFifoBlocking);
		tx = CFifoInit(cfg->pTxMem ? cfg->pTxMem : d->TxMem,
		               cfg->pTxMem ? (unsigned)cfg->TxMemSize : sizeof(d->TxMem), 1,
		               cfg->bFifoBlocking);
		ok = rx && tx;
	}
	if (!ok)
	{
		d->Regs->SCR = 0;
		ReleaseRoutes(d);
		if (d->DmaMode)
		{
			s_DmaRegs[d->DmaChannel]->DMCNT_b.DTE = 0U;
			s_DmaRegs[d->DmaChannel]->DMSTS = 0U;
			R_DMA->DELSR[d->DmaChannel] = 0U;
			if (dmaClockStopped)
			{
				uint16_t protect = R_SYSTEM->PRCR & 15U;
				R_SYSTEM->PRCR = 0xA500U | protect | 0x02U;
				R_MSTP->MSTPCRA |= RA4M1_DMA_MSTP_MASK;
				R_SYSTEM->PRCR = 0xA500U | protect;
			}
		}
		if ((int)d->DmaIrq >= 0)
		{
			Ra4m1UnregisterIntHandler(d->DmaIrq);
		}
		RestorePins(d, configured);
		Clock(d, false);
		EnableInterrupt(state);
		return false;
	}
	u->Mode = cfg->Mode;
	u->Duplex = cfg->Duplex;
	u->Rate = (int)baud.Actual;
	u->DataBits = cfg->DataBits;
	u->Parity = cfg->Parity;
	u->StopBits = cfg->StopBits;
	u->FlowControl = cfg->FlowControl;
	u->bIrDAMode = u->bIrDAInvert = u->bIrDAFixPulse = false;
	u->IrDAPulseDiv = 0;
	u->EvtCallback = cfg->EvtCallback;
	u->hRxFifo = rx;
	u->hTxFifo = tx;
	u->LineState = 0;
	u->hStdIn = u->hStdOut = -1;
	u->RxOvrErrCnt = u->ParErrCnt = u->FramErrCnt = u->RxDropCnt = u->TxDropCnt = 0;
	u->bRxReady = false;
	u->bTxReady = false;
	// Preserve pObj installed by the existing C++ UART constructor.
	DevIntrf_t *p = &u->DevIntrf;
	p->pDevData = d;
	p->IntPrio = cfg->IntPrio;
	p->EvtCB = nullptr;
	p->Type = DEVINTRF_TYPE_UART;
	p->bDma = cfg->bDMAMode;
	p->bIntEn = cfg->bIntMode;
	p->MaxRetry = UART_RETRY_MAX;
	p->EnCnt = 1;
	p->bTxReady = true;
	p->bNoStop = false;
	atomic_flag_clear(&p->bBusy);
	p->Enable = Enable;
	p->Disable = Disable;
	p->PowerOff = PowerOff;
	p->Reset = Reset;
	p->GetRate = GetRate;
	p->SetRate = SetRate;
	p->GetHandle = GetHandle;
	p->StartRx = StartRx;
	p->RxData = RxData;
	p->StopRx = Stop;
	p->StartTx = StartTx;
	p->TxData = TxData;
	p->TxSrData = TxData;
	p->StopTx = Stop;
	++d->Generation;
	d->pDev = u;
	Enable(p);
	EnableInterrupt(state);
	return true;
}
