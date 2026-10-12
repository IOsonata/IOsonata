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
static int s_Events[4];
static UARTDev_t s_Uart[LPCUART_DEV_COUNT];
static const IOPinCfg_t s_Pins[4] = {};
uint32_t SystemCoreClock = 120000000;
uint32_t SystemMainClkFreq = 48000000;

void NVIC_ClearPendingIRQ(IRQn_Type)
{
}
void NVIC_SetPriority(IRQn_Type, uint32_t)
{
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

int main()
{
	void *mapped = mmap((void *)0x40000000, 0x200000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(mapped != MAP_FAILED);
	RateTest();
	DriverTest();
	printf("legacy LPC UART (%d instances): PASS\n", LPCUART_DEV_COUNT);
}
