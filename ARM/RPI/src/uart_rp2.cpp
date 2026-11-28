// RP2 PL011 UART. Interrupt-driven FIFOs or direct polling, core 0.
#include "rp2.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/uart.h"
#include "coredev/interrupt.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"

struct RpUart {
	uintptr_t Base;
	UARTDev_t *Owner;
	const IOPinCfg_t *Pins;
	uint32_t RequestedRate;
	bool Enabled;
	alignas(4) uint8_t Rx[CFIFO_MEMSIZE(32)];
	alignas(4) uint8_t Tx[CFIFO_MEMSIZE(32)];
};
static RpUart s_Port[2] = {{UART0_BASE}, {UART1_BASE}};

static RpUart *Port(DevIntrf_t *i) { return (RpUart *)i->pDevData; }
static bool Owns(DevIntrf_t *i)
{
	RpUart *p = Port(i);
	return p && p->Owner && &p->Owner->DevIntrf == i;
}
static void *Handle(DevIntrf_t *i) { return Owns(i) ? Port(i)->Owner : nullptr; }
static bool Start(DevIntrf_t *i, uint32_t) { return Owns(i) && i->EnCnt > 0; }
static void Stop(DevIntrf_t *) {}
static uint32_t Rate(DevIntrf_t *i) { return Owns(i) ? Port(i)->Owner->Rate : 0U; }

static uint32_t SetRate(DevIntrf_t *i, uint32_t rate)
{
	RpUart *p = Port(i);
	uint32_t clk = SystemPeriphClockGet(0);
	uint64_t div;
	uint32_t control;
	if (!Owns(i) || !rate || !clk) return Rate(i);
	div = (4ULL * clk + rate / 2U) / rate;
	if (div < 64U) div = 64U;
	if (div > 0x3fffffU) div = 0x3fffffU;
	while (RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_BUSY_BITS) {}
	control = RP_REG(p->Base, UART_UARTCR_OFFSET);
	RP_REG(p->Base, UART_UARTCR_OFFSET) = 0;
	RP_REG(p->Base, UART_UARTIBRD_OFFSET) = div >> 6;
	RP_REG(p->Base, UART_UARTFBRD_OFFSET) = div & 63U;
	// The LCR write latches both divisors.
	RP_REG(p->Base, UART_UARTLCR_H_OFFSET) = RP_REG(p->Base, UART_UARTLCR_H_OFFSET);
	RP_REG(p->Base, UART_UARTCR_OFFSET) = control;
	p->RequestedRate = rate;
	p->Owner->Rate = (4ULL * clk + div / 2U) / div;
	return p->Owner->Rate;
}

static void TxFill(RpUart *p)
{
	int byte;
	while (!(RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_TXFF_BITS))
	{
		byte = CFifoGetByte(p->Owner->hTxFifo);
		if (byte < 0) break;
		RP_REG(p->Base, UART_UARTDR_OFFSET) = byte;
	}
}

static int Tx(DevIntrf_t *i, const uint8_t *buf, int len)
{
	RpUart *p = Port(i);
	uint32_t state;
	int n = 0;
	if (!i->bIntEn)
	{
		while (n < len && !(RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_TXFF_BITS))
			RP_REG(p->Base, UART_UARTDR_OFFSET) = buf[n++];
		return n;
	}
	state = DisableInterrupt();
	while (n < len && CFifoPutByte(p->Owner->hTxFifo, buf[n])) n++;
	TxFill(p);
	if (n) RP_SET(p->Base, UART_UARTIMSC_OFFSET) = UART_UARTIMSC_TXIM_BITS;
	EnableInterrupt(state);
	return n;
}

static int Rx(DevIntrf_t *i, uint8_t *buf, int len)
{
	RpUart *p = Port(i);
	int byte, n = 0;
	while (n < len)
	{
		if (i->bIntEn)
		{
			byte = CFifoGetByte(p->Owner->hRxFifo);
			if (byte < 0) break;
		}
		else
		{
			if (RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_RXFE_BITS) break;
			byte = RP_REG(p->Base, UART_UARTDR_OFFSET);
		}
		buf[n++] = (uint8_t)byte;
	}
	return n;
}

extern "C" void RpUartIrq(void)
{
	RpUart *p = &s_Port[__get_IPSR() - 16U - UART0_IRQ];
	UARTDev_t *u = p->Owner;
	uint32_t mis = RP_REG(p->Base, UART_UARTMIS_OFFSET);
	int n = 0;
	if (!u) return;
	RP_REG(p->Base, UART_UARTICR_OFFSET) = mis;
	while (!(RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_RXFE_BITS))
	{
		uint32_t data = RP_REG(p->Base, UART_UARTDR_OFFSET);
		if (!CFifoPutByte(u->hRxFifo, (uint8_t)data)) u->RxDropCnt++;
		else n++;
	}
	if (n && u->DevIntrf.EvtCB)
		u->DevIntrf.EvtCB(&u->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, n);
	if (mis & UART_UARTMIS_TXMIS_BITS)
	{
		TxFill(p);
		if (!CFifoUsed(u->hTxFifo))
		{
			RP_CLR(p->Base, UART_UARTIMSC_OFFSET) = UART_UARTIMSC_TXIM_BITS;
			if (u->DevIntrf.EvtCB)
				u->DevIntrf.EvtCB(&u->DevIntrf, DEVINTRF_EVT_TX_READY, nullptr, CFifoAvail(u->hTxFifo));
		}
	}
}

static void Enable(DevIntrf_t *i)
{
	RpUart *p = Port(i);
	if (!Owns(i) || p->Enabled || !SystemHighFreqClockEnable()) return;
	p->Enabled = true;
	SetRate(i, p->RequestedRate);
	RP_REG(p->Base, UART_UARTCR_OFFSET) = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
	RP_REG(p->Base, UART_UARTIMSC_OFFSET) = i->bIntEn ? UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS : 0U;
}
static void Disable(DevIntrf_t *i)
{
	RpUart *p = Port(i);
	if (!Owns(i) || !p->Enabled) return;
	RP_REG(p->Base, UART_UARTIMSC_OFFSET) = 0;
	while (CFifoUsed(p->Owner->hTxFifo)) TxFill(p);
	while (RP_REG(p->Base, UART_UARTFR_OFFSET) & UART_UARTFR_BUSY_BITS) {}
	RP_REG(p->Base, UART_UARTCR_OFFSET) = 0;
	p->Enabled = false;
	SystemHighFreqClockDisable();
}
static void Off(DevIntrf_t *i)
{
	RpUart *p = Port(i);
	if (!Owns(i)) return;
	Disable(i);
	IOPinDis(p->Pins, 2);
	RP_SET(RESETS_BASE, RESETS_RESET_OFFSET) = p == s_Port ? RESETS_RESET_UART0_BITS : RESETS_RESET_UART1_BITS;
	p->Owner = nullptr;
	i->pDevData = nullptr;
	i->EnCnt = 0;
}
UARTDev_t const *UARTGetInstance(int dev) { return (unsigned)dev < 2 ? s_Port[dev].Owner : nullptr; }
void UARTSetCtrlLineState(UARTDev_t *, uint32_t) {}

static bool ValidUartPins(const IOPinCfg_t *pins, unsigned dev)
{
	unsigned seen = 0;
	for (unsigned i = 0; i < 2; i++)
	{
		unsigned pin = pins[i].PinNo;
		unsigned port = ((pin >> 2) ^ (pin >> 3)) & 1U;
		if (!RpPinValid(pins[i].PortNo, pin) || pins[i].PinOp != IOPINOP_FUNC2 ||
			port != dev || (pin & 2U)) return false;
		if ((pin & 1U) && pins[i].PinDir == IOPINDIR_INPUT) seen |= 1U;
		if (!(pin & 1U) && pins[i].PinDir == IOPINDIR_OUTPUT) seen |= 2U;
	}
	return seen == 3U;
}

bool UARTInit(UARTDev_t *u, const UARTCfg_t *cfg)
{
	RpUart *p;
	DevIntrf_t *i;
	uint32_t format;
	if (!u || !cfg || (unsigned)cfg->DevNo >= 2 || !cfg->pIOPinMap || cfg->NbIOPins < 2 ||
		cfg->Parity < UART_PARITY_NONE || cfg->Parity > UART_PARITY_SPACE ||
		cfg->Rate <= 0 || cfg->DataBits < 5 || cfg->DataBits > 8 ||
		(cfg->StopBits != 1 && cfg->StopBits != 2) || cfg->Mode != UART_MODE_UART ||
		cfg->Duplex != UART_DUPLEX_FULL || cfg->bDMAMode || cfg->bIrDAMode ||
		cfg->FlowControl != UART_FLWCTRL_NONE) return false;
	if (!ValidUartPins((const IOPinCfg_t *)cfg->pIOPinMap, cfg->DevNo)) return false;
	p = &s_Port[cfg->DevNo];
	if (p->Owner && p->Owner != u) return false;
	for (unsigned n = 0; n < 2; n++)
	{
		if (s_Port[n].Owner == u)
		{
			if (u->DevIntrf.EnCnt > 0) return false;
			Off(&u->DevIntrf);
		}
	}
	u->hRxFifo = CFifoInit(cfg->pRxMem ? cfg->pRxMem : p->Rx,
		cfg->pRxMem ? cfg->RxMemSize : sizeof(p->Rx), 1, cfg->bFifoBlocking);
	u->hTxFifo = CFifoInit(cfg->pTxMem ? cfg->pTxMem : p->Tx,
		cfg->pTxMem ? cfg->TxMemSize : sizeof(p->Tx), 1, cfg->bFifoBlocking);
	if (!u->hRxFifo || !u->hTxFifo)
	{
		return false;
	}
	i = &u->DevIntrf;
	p->Owner = u;
	p->Pins = (const IOPinCfg_t *)cfg->pIOPinMap;
	i->pDevData = p;
	i->EvtCB = cfg->EvtCB;
	i->Type = DEVINTRF_TYPE_UART;
	i->EnCnt = 1;
	i->bIntEn = cfg->bIntMode;
	i->IntPrio = cfg->IntPrio;
	i->bDma = false;
	i->bTxReady = true;
	i->bNoStop = false;
	atomic_flag_clear(&i->bBusy);
	i->MaxRetry = UART_RETRY_MAX;
	i->Enable = Enable;
	i->Disable = Disable;
	i->PowerOff = Off;
	i->GetHandle = Handle;
	i->StartTx = Start;
	i->StopTx = Stop;
	i->StartRx = Start;
	i->StopRx = Stop;
	i->RxData = Rx;
	i->TxData = Tx;
	i->SetRate = SetRate;
	i->GetRate = Rate;
	u->DataBits = cfg->DataBits;
	u->Parity = cfg->Parity;
	u->StopBits = cfg->StopBits;
	RpReleaseReset(p == s_Port ? RESETS_RESET_UART0_BITS : RESETS_RESET_UART1_BITS);
	RP_REG(p->Base, UART_UARTCR_OFFSET) = 0;
	format = ((cfg->DataBits - 5U) << UART_UARTLCR_H_WLEN_LSB) | UART_UARTLCR_H_FEN_BITS;
	if (cfg->StopBits == 2) format |= UART_UARTLCR_H_STP2_BITS;
	if (cfg->Parity != UART_PARITY_NONE) format |= UART_UARTLCR_H_PEN_BITS;
	if (cfg->Parity == UART_PARITY_EVEN || cfg->Parity == UART_PARITY_SPACE) format |= UART_UARTLCR_H_EPS_BITS;
	if (cfg->Parity == UART_PARITY_MARK || cfg->Parity == UART_PARITY_SPACE) format |= UART_UARTLCR_H_SPS_BITS;
	RP_REG(p->Base, UART_UARTLCR_H_OFFSET) = format;
	p->RequestedRate = cfg->Rate;
	IOPinCfg(p->Pins, 2);
	Enable(i);
	if (!p->Enabled)
	{
		Off(i);
		return false;
	}
	if (cfg->bIntMode) RpEnableIrq(UART0_IRQ + cfg->DevNo, cfg->IntPrio);
	return true;
}
