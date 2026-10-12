/*--------------------------------------------------------------------------
File   : uart_test.cpp

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : Host regression tests for the shared legacy LPC UART

Copyright (c) 2012, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
// Legacy LPC UART regressions. Uses the vendor device types; only core IRQ
// operations and UART register side effects are modelled on the host.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <climits>
#include <deque>
#include <vector>
#include <sys/mman.h>
#include "uart_lpcxx.h"
#include "uart_lpc_target.h"

struct UartModel {
	uint32_t Ier = 0;
	uint32_t Dll = 1;
	uint32_t Dlm = 0;
	uint32_t Acr = 0;
	int InFlight = 0;
	bool Polling = true;
	bool Stalled = false;
	uint32_t Error = 0;
	std::deque<uint8_t> Rx;
	std::vector<uint8_t> Tx;
};
static UartModel s_Model[LPCUART_DEV_COUNT];
static bool s_Irq[64];
static uint32_t s_Priority[64];
static int s_Events[4];
static UARTDev_t s_Uart[LPCUART_DEV_COUNT];
static int s_PinConfigs;
static int s_RegWrites;
static const IOPinCfg_t s_Pins[4] = {
	{0, 18, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 19, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 7, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 17, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}
};
#ifndef TEST_LPC17
static const int s_RxRoutes[][3] = {
	{0, 18, IOPINOP_FUNC1},
	{1, 14, IOPINOP_FUNC3},
	{1, 17, IOPINOP_FUNC2},
	{1, 26, IOPINOP_FUNC2}
};
static const int s_TxRoutes[][3] = {
	{0, 19, IOPINOP_FUNC1},
	{1, 13, IOPINOP_FUNC3},
	{1, 18, IOPINOP_FUNC2},
	{1, 27, IOPINOP_FUNC2}
};
#endif
uint32_t SystemCoreClock = 120000000;
uint32_t SystemMainClkFreq = 48000000;

void NVIC_ClearPendingIRQ(IRQn_Type)
{
}
void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	s_Priority[Irq] = Priority;
}
void NVIC_EnableIRQ(IRQn_Type Irq)
{
	s_Irq[Irq] = true;
}
void NVIC_DisableIRQ(IRQn_Type Irq)
{
	s_Irq[Irq] = false;
}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	s_PinConfigs++;
}

#ifdef TEST_LPC17
#include "../../ARM/NXP/LPC17xx/src/uart_lpc17xx.c"
#else
#include "../../ARM/NXP/LPC11xx/src/uart_lpc11uxx.c"
#endif

static uint32_t ModelRead(int No, const char *Reg)
{
	auto &m = s_Model[No];
	if (!strcmp(Reg, "LSR"))
	{
		if (m.Polling && !m.Stalled)
		{
			m.InFlight = 0;
		}
		uint32_t value = m.Error | (m.Rx.empty() ? 0 : LPCUART_LSR_RDR);
		if (m.InFlight == 0 && !m.Stalled)
		{
			value |= LPCUART_LSR_THRE | LPCUART_LSR_TEMT;
		}
		m.Error = 0;
		return value;
	}
	if (!strcmp(Reg, "RBR"))
	{
		assert(!m.Rx.empty());
		uint8_t value = m.Rx.front();
		m.Rx.pop_front();
		return value;
	}
	if (!strcmp(Reg, "IIR"))
	{
		if (m.Error && (m.Ier & LPCUART_IER_RLS))
		{
			return LPCUART_IIR_ID_RLS;
		}
		if (!m.Rx.empty() && (m.Ier & LPCUART_IER_RBR))
		{
			return LPCUART_IIR_ID_CTIMOUT;
		}
		if (m.InFlight == 0 && (m.Ier & LPCUART_IER_THRE))
		{
			return LPCUART_IIR_ID_THRE;
		}
		return LPCUART_IIR_STATUS;
	}
	if (!strcmp(Reg, "IER"))
	{
		return m.Ier;
	}
	if (!strcmp(Reg, "DLL"))
	{
		return m.Dll;
	}
	if (!strcmp(Reg, "DLM"))
	{
		return m.Dlm;
	}
	if (!strcmp(Reg, "ACR"))
	{
		return m.Acr;
	}
	if (!strcmp(Reg, "LCR"))
	{
		return LPCUART_READ(No, LCR);
	}
	assert(false);
	return 0;
}

static void ModelWrite(int No, const char *Reg, uint32_t Value)
{
	s_RegWrites++;
	auto &m = s_Model[No];
	if (!strcmp(Reg, "THR"))
	{
		assert(m.InFlight < 16);
		m.InFlight++;
		m.Tx.push_back((uint8_t)Value);
		LPCUART_WRITE(No, THR, Value);
		return;
	}
	if (!strcmp(Reg, "DLL"))
	{
		m.Dll = Value;
		LPCUART_WRITE(No, DLL, Value);
		return;
	}
	if (!strcmp(Reg, "DLM"))
	{
		m.Dlm = Value;
		LPCUART_WRITE(No, DLM, Value);
		return;
	}
	if (!strcmp(Reg, "IER"))
	{
		m.Ier = Value;
		LPCUART_WRITE(No, IER, Value);
		return;
	}
	if (!strcmp(Reg, "ACR"))
	{
		m.Acr = Value & 7U;
		LPCUART_WRITE(No, ACR, Value);
		return;
	}
	if (!strcmp(Reg, "LCR"))
	{
		LPCUART_WRITE(No, LCR, Value);
		return;
	}
	if (!strcmp(Reg, "FDR"))
	{
		LPCUART_WRITE(No, FDR, Value);
		return;
	}
	if (!strcmp(Reg, "TER"))
	{
		LPCUART_WRITE(No, TER, Value);
		return;
	}
	if (!strcmp(Reg, "FCR"))
	{
		if (Value & LPCUART_FCR_RST_RXFIFO)
		{
			m.Rx.clear();
		}
		if (Value & LPCUART_FCR_RST_TXFIFO)
		{
			m.InFlight = 0;
		}
		LPCUART_WRITE(No, FCR, Value);
		return;
	}
	assert(false);
}

#undef LPCUART_READ
#undef LPCUART_WRITE
#define LPCUART_READ(No, Reg) ModelRead(No, #Reg)
#define LPCUART_WRITE(No, Reg, Value) ModelWrite(No, #Reg, Value)
#include "../../ARM/NXP/src/uart_lpcxx.c"
#include "../../src/cfifo.c"
#include "../../src/device_intrf.cpp"

static int Callback(UARTDev_t *, UART_EVT Event, uint8_t *, int)
{
	s_Events[Event]++;
	return 0;
}

static void RateTest()
{
	// Brute force every legal divider as an independent oracle.
	for (uint32_t clk : {12000000U, 24000000U, 48000000U, 120000000U})
	{
		for (uint32_t request : {1U, 110U, 115200U, 1000000U, 1500000U, UINT32_MAX})
		{
			auto chosen = LpcUARTRateFind(clk, request);
			uint32_t mul = chosen.Fraction >> 4;
			uint32_t add = chosen.Fraction & 15U;
			assert(mul >= 1 && mul <= 15 && add < mul);
			assert(chosen.Divisor >= (add ? 3U : 1U) && chosen.Divisor <= 65535);
			long double actual = (long double)clk * mul / (16U * chosen.Divisor * (mul + add));
			long double err = fabsl(actual - request);
			for (uint32_t m = 1; m <= 15; m++)
			{
				for (uint32_t a = 0; a < m; a++)
				{
					for (uint32_t d = a ? 3U : 1U; d <= 65535; d++)
					{
						long double other = (long double)clk * m / (16U * d * (m + a));
						assert(err <= fabsl(other - request) + 0.000001L);
					}
				}
			}
		}
	}
	assert(LpcUARTRateFind(48000000, 1000000).Rate == 1000000);
	assert(LpcUARTRateFind(24000000, 1500000).Rate == 1500000);
	assert(LpcUARTRateFind(0, 1000000).Divisor == 0);
	assert(LpcUARTRateFind(24000000, 0).Divisor == 0);
}

static void DriverTest()
{
	UARTCfg_t cfg = {};
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 4;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_EVEN;
	cfg.StopBits = 1;
	cfg.bFifoBlocking = true;
	cfg.EvtCallback = Callback;
	uint8_t data[65];
	for (int i = 0; i < 65; i++)
	{
		data[i] = (uint8_t)i;
	}

	for (int no = 0; no < LPCUART_DEV_COUNT; no++)
	{
		auto &uart = s_Uart[no];
		auto &model = s_Model[no];
		cfg.DevNo = no;
		cfg.bIntMode = false;
		model.Polling = true;
		assert(UARTInit(&uart, &cfg));
		assert(UARTGetInstance(no) == &uart);
		assert(UARTGetDevHandle(&uart.DevIntrf) == &uart);
		assert(ModelRead(no, "LCR") == 0x1BU);
		assert(!s_Irq[LpcUARTIrq(no)]);
		assert(UARTTx(&uart, data, sizeof(data)) == (int)sizeof(data));
		assert(model.Tx == std::vector<uint8_t>(data, data + sizeof(data)));
		model.Stalled = true;
		assert(UARTTx(&uart, data, 1) == 0);
		model.Stalled = false;

		cfg.bIntMode = true;
		model.InFlight = 0;
		assert(UARTInit(&uart, &cfg));
		model.Polling = false;
		model.Tx.clear();
		assert(s_Irq[LpcUARTIrq(no)]);
		int sent = 0;
		for (int attempts = 0; sent < (int)sizeof(data) && attempts < 100; attempts++)
		{
			sent += UARTTx(&uart, data + sent, sizeof(data) - sent);
			model.InFlight = 0;
			LpcUARTIrqHandler(no);
		}
		while ((model.Ier & LPCUART_IER_THRE) != 0)
		{
			model.InFlight = 0;
			LpcUARTIrqHandler(no);
		}
		assert(sent == sizeof(data));
		assert(model.Tx == std::vector<uint8_t>(data, data + sizeof(data)));
		assert(uart.bTxReady && s_Events[UART_EVT_TXREADY]);

		for (int i = 0; i < 32; i++)
		{
			model.Rx.push_back((uint8_t)i);
		}
		model.Error = LPCUART_LSR_OE | LPCUART_LSR_PE | LPCUART_LSR_FE;
		LpcUARTIrqHandler(no);
		assert(model.Rx.size() == 16);
		LpcUARTIrqHandler(no);
		assert(model.Rx.empty() && uart.RxDropCnt == 16);
		assert(uart.RxOvrErrCnt == 1 && uart.ParErrCnt == 1 && uart.FramErrCnt == 1);
		uint8_t rx[16];
		assert(UARTRx(&uart, rx, 16) == 16);
		assert(!memcmp(rx, data, 16));

		UARTDisable(&uart);
		assert(!s_Irq[LpcUARTIrq(no)] && !UARTTx(&uart, data, 1));
		UARTEnable(&uart);
		assert(s_Irq[LpcUARTIrq(no)]);
		DeviceIntrfReset(&uart.DevIntrf);
		assert(CFifoUsed(uart.hTxFifo) == 0);
		assert(CFifoUsed(uart.hRxFifo) == 0);
		model.InFlight = 0;
		assert(UARTSetRate(&uart, 0) == 0 && (model.Acr & 1U));
		model.Dll = 12;
		model.Dlm = 0;
		model.Acr = 0;
		assert(UARTGetRate(&uart) == (int)(LpcUARTClockFreq(no) / 192U));
		assert(UARTSetRate(&uart, 1500000) > 0);

		UARTDev_t other = {};
		assert(!UARTInit(&other, &cfg));
		cfg.bDMAMode = true;
		assert(!UARTInit(&uart, &cfg));
		cfg.bDMAMode = false;
	}
	for (int a = 0; a < LPCUART_DEV_COUNT; a++)
	{
		for (int b = a + 1; b < LPCUART_DEV_COUNT; b++)
		{
			assert(s_Uart[a].hRxFifo != s_Uart[b].hRxFifo);
			assert(s_Uart[a].hTxFifo != s_Uart[b].hTxFifo);
		}
	}
#ifdef TEST_LPC17
	assert(LpcUARTIrq(1) == UART1_IRQn);
	cfg.DevNo = 0;
	cfg.FlowControl = UART_FLWCTRL_HW;
	assert(!UARTInit(&s_Uart[0], &cfg));
#endif
}

#ifndef TEST_LPC17
static void RejectConfig(const UARTCfg_t &Cfg)
{
	uint8_t syscon[sizeof(*LPC_SYSCON)];
	uint8_t usart[sizeof(*LPC_USART)];
	uint8_t object[sizeof(s_Uart)];
	uint8_t driver[sizeof(s_LpcUartDev)];
	bool irq[sizeof(s_Irq) / sizeof(s_Irq[0])];
	uint32_t priority = s_Priority[UART_IRQn];
	int configs = s_PinConfigs;
	int writes = s_RegWrites;

	memcpy(syscon, LPC_SYSCON, sizeof(syscon));
	memcpy(usart, LPC_USART, sizeof(usart));
	memcpy(object, s_Uart, sizeof(object));
	memcpy(driver, s_LpcUartDev, sizeof(driver));
	memcpy(irq, s_Irq, sizeof(irq));
	assert(!UARTInit(&s_Uart[0], &Cfg));
	assert(memcmp(syscon, LPC_SYSCON, sizeof(syscon)) == 0);
	assert(memcmp(usart, LPC_USART, sizeof(usart)) == 0);
	assert(memcmp(object, s_Uart, sizeof(object)) == 0);
	assert(memcmp(driver, s_LpcUartDev, sizeof(driver)) == 0);
	assert(memcmp(irq, s_Irq, sizeof(irq)) == 0);
	assert(s_PinConfigs == configs && s_RegWrites == writes);
	assert(s_Priority[UART_IRQn] == priority);
}

#endif

static void PriorityTest()
{
	UARTCfg_t cfg = {};
	int irq = LpcUARTIrq(0);
	int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;

	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 4;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.bIntMode = true;
	cfg.bFifoBlocking = true;
	s_Model[0].Polling = true;
	for (int priority = 0; priority <= maxprio; priority++)
	{
		cfg.IntPrio = priority;
		assert(UARTInit(&s_Uart[0], &cfg));
		assert(s_Priority[irq] == (uint32_t)priority);
		assert(s_Uart[0].DevIntrf.IntPrio == priority);
		assert(s_Irq[irq]);
	}
	for (int priority : {-1, maxprio + 1, INT_MIN, INT_MAX})
	{
		cfg.IntPrio = priority;
		int expected = priority < 0 ? 0 : maxprio;

		assert(UARTInit(&s_Uart[0], &cfg));
		assert(cfg.IntPrio == priority);
		assert(s_Uart[0].DevIntrf.IntPrio == expected);
		assert(s_Priority[irq] == (uint32_t)expected);
		UARTDisable(&s_Uart[0]);
		UARTEnable(&s_Uart[0]);
		assert(s_Irq[irq]);
		assert(s_Priority[irq] == (uint32_t)expected);
	}
	// Polling never programs NVIC priority, even across Disable/Enable.
	cfg.bIntMode = false;
	for (int priority : {-1, maxprio + 1, INT_MIN, INT_MAX})
	{
		cfg.IntPrio = priority;
		assert(UARTInit(&s_Uart[0], &cfg));
		UARTDisable(&s_Uart[0]);
		UARTEnable(&s_Uart[0]);
		assert(!s_Irq[irq]);
		assert(s_Priority[irq] == (uint32_t)maxprio);
	}
}

#ifndef TEST_LPC17
static void PinTest()
{
	IOPinCfg_t pins[UART_NB_PINS];
	UARTCfg_t cfg = {};

	memcpy(pins, s_Pins, sizeof(s_Pins));
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.bFifoBlocking = true;
	s_Model[0].Polling = true;
	// RX and TX routes can be mixed independently.
	for (const auto &rx : s_RxRoutes)
	{
		for (const auto &tx : s_TxRoutes)
		{
			pins[0].PortNo = rx[0];
			pins[0].PinNo = rx[1];
			pins[0].PinOp = rx[2];
			pins[1].PortNo = tx[0];
			pins[1].PinNo = tx[1];
			pins[1].PinOp = tx[2];
			assert(UARTInit(&s_Uart[0], &cfg));
			pins[0].PinOp = IOPINOP_GPIO;
			RejectConfig(cfg);
			pins[0].PinOp = rx[2];
			pins[1].PinOp = IOPINOP_FUNC7;
			RejectConfig(cfg);
			pins[1].PinOp = tx[2];
		}
	}
	memcpy(pins, s_Pins, sizeof(s_Pins));
	cfg.NbIOPins = 4;
	cfg.FlowControl = UART_FLWCTRL_HW;
	assert(UARTInit(&s_Uart[0], &cfg));
	assert((LPC_USART->MCR & (LPCUART_MCR_RTSEN | LPCUART_MCR_CTSEN)) ==
		(LPCUART_MCR_RTSEN | LPCUART_MCR_CTSEN));
	// Rejection must preserve queued data as well as registers and ownership.
	assert(CFifoPutByte(s_Uart[0].hRxFifo, 0xA5));
	assert(CFifoPutByte(s_Uart[0].hTxFifo, 0x5A));
	for (int i = 0; i < 4; i++)
	{
		pins[i].PinOp = IOPINOP_FUNC0;
		RejectConfig(cfg);
		pins[i] = s_Pins[i];
		pins[i].PortNo = 2;
		RejectConfig(cfg);
		pins[i] = s_Pins[i];
		pins[i].PinNo = 32;
		RejectConfig(cfg);
		pins[i] = s_Pins[i];
	}
	for (int i = 2; i < 4; i++)
	{
		pins[i].PinNo = -1;
		RejectConfig(cfg);
		pins[i] = s_Pins[i];
		pins[i].PortNo = -1;
		RejectConfig(cfg);
		pins[i] = s_Pins[i];
	}
	cfg.NbIOPins = 2;
	RejectConfig(cfg);
	cfg.NbIOPins = UART_NB_PINS + 1;
	RejectConfig(cfg);
	cfg.NbIOPins = 4;
	std::swap(pins[0], pins[1]);
	RejectConfig(cfg);
	std::swap(pins[0], pins[1]);
	// Unconnected entries remain supported when hardware flow is disabled.
	cfg.FlowControl = UART_FLWCTRL_NONE;
	pins[0].PinNo = -1;
	pins[2].PinNo = -1;
	pins[3].PortNo = -1;
	assert(UARTInit(&s_Uart[0], &cfg));
	memcpy(pins, s_Pins, sizeof(s_Pins));
	cfg.NbIOPins = UART_NB_PINS;
	for (int i = 4; i < UART_NB_PINS; i++)
	{
		pins[i] = {1, 0, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	}
	pins[6].PinDir = IOPINDIR_OUTPUT;
	// All combinations of DCD, DSR, DTR and RI alternate pads.
	for (int route = 0; route < 16; route++)
	{
		pins[4].PinNo = route & 1 ? 21 : 15;
		pins[5].PinNo = route & 2 ? 20 : 14;
		pins[6].PinNo = route & 4 ? 19 : 13;
		pins[7].PinNo = route & 8 ? 22 : 16;
		assert(UARTInit(&s_Uart[0], &cfg));
		for (int i = 4; i < UART_NB_PINS; i++)
		{
			pins[i].PinOp = IOPINOP_FUNC2;
			RejectConfig(cfg);
			pins[i].PinOp = IOPINOP_FUNC1;
		}
	}
	// Individually legal routes must not reuse the same physical pad.
	pins[0].PortNo = 1;
	pins[0].PinNo = 14;
	pins[0].PinOp = IOPINOP_FUNC3;
	pins[5].PinNo = 14;
	RejectConfig(cfg);
	pins[0] = s_Pins[0];
	pins[1].PortNo = 1;
	pins[1].PinNo = 13;
	pins[1].PinOp = IOPINOP_FUNC3;
	pins[6].PinNo = 13;
	RejectConfig(cfg);
}
#endif

int main()
{
	void *mapped = mmap((void *)0x40000000, 0x200000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(mapped != MAP_FAILED);
	RateTest();
	DriverTest();
	PriorityTest();
#ifndef TEST_LPC17
	PinTest();
#endif
	printf("legacy LPC UART (%d instances): PASS\n", LPCUART_DEV_COUNT);
}
