/**-------------------------------------------------------------------------
@file	esp32_uart_transfer_test.cpp

@brief	ESP32 UART host transfer and interrupt interleaving tests.

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
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#define __INTERRUPT_H__
static uintptr_t s_Masked;
static uintptr_t DisableInterrupt()
{
	uintptr_t state;

	state = s_Masked;
	s_Masked = 1;
	return state;
}
static void EnableInterrupt(uintptr_t State)
{
	s_Masked = State;
}

#include "esp32xx_uart.h"
#include "coredev/uart.h"

static uint32_t s_Regs[64];
static std::deque<uint8_t> s_Rx;
static std::vector<uint8_t> s_Tx;
static unsigned s_TxCount;
static UARTDev_t s_Uart;

struct FifoRegister {
	operator uint32_t() const
	{
		uint8_t data;

		assert(!s_Rx.empty());
		data = s_Rx.front();
		s_Rx.pop_front();
		return data;
	}
	void operator=(uint32_t Data)
	{
		assert(s_TxCount < ESP32_UART_HW_FIFO_LEN);
		s_TxCount++;
		s_Tx.push_back(Data);
	}
};
static FifoRegister s_Fifo;
static uint32_t Status()
{
	return (s_TxCount << ESP32_UART_STATUS_TXFIFO_CNT_Pos) | s_Rx.size();
}
static void *Copy(void *pDest, const void *pSrc, size_t Len);
#undef ESP32_UART_FIFO_REG
#undef ESP32_UART_STATUS_REG
#undef ESP32_REG32
#define ESP32_REG32(addr) s_Regs[((addr) & 255U) / 4U]
#define ESP32_UART_FIFO_REG(base) s_Fifo
#define ESP32_UART_STATUS_REG(base) Status()
#define memcpy Copy
#include "../../RISCV/Espressif/src/uart_esp32.cpp"
#undef memcpy

volatile uint32_t g_Esp32C3Uart0IrqCount;
volatile uint32_t g_Esp32C3Uart1IrqCount;

static void *Copy(void *pDest, const void *pSrc, size_t Len)
{
	if ((uintptr_t)pDest >= (uintptr_t)s_Uart.hTxFifo &&
		(uintptr_t)pDest < (uintptr_t)s_Uart.hTxFifo + CFIFO_MEMSIZE(512))
	{
		assert(s_Masked != 0);
		assert(Len <= ESP32_UART_COPY_MAX);
	}
	return std::memcpy(pDest, pSrc, Len);
}

static void Reset(bool Blocking)
{
	static uint8_t rx[CFIFO_MEMSIZE(512)] __attribute__((aligned(4)));
	static uint8_t tx[CFIFO_MEMSIZE(512)] __attribute__((aligned(4)));

	std::memset(s_Regs, 0, sizeof(s_Regs));
	s_Rx.clear();
	s_Tx.clear();
	s_TxCount = 0;
	s_Masked = 0;
	s_Esp32UartDev[0].pUartDev = &s_Uart;
	s_Uart.hRxFifo = CFifoInit(rx, sizeof(rx), 1, Blocking);
	s_Uart.hTxFifo = CFifoInit(tx, sizeof(tx), 1, Blocking);
	s_Uart.DevIntrf.pDevData = &s_Esp32UartDev[0];
	s_Uart.DevIntrf.bIntEn = true;
	s_Uart.DevIntrf.MaxRetry = 1;
	s_Uart.EvtCallback = nullptr;
	s_Uart.bTxReady = true;
	s_Uart.TxDropCnt = 0;
}

static void Drain()
{
	while (CFifoUsed(s_Uart.hTxFifo) > 0)
	{
		s_TxCount = 0;
		ESP32_UART_INT_ST_REG(0) = ESP32_UART_INT_TXFIFO_EMPTY;
		Esp32UartIRQHandler(&s_Esp32UartDev[0]);
	}
}

#include "../../src/uart_retarget.c"

int DeviceIntrfTx(DevIntrf_t * const pDev, uint32_t DevAddr,
		const uint8_t *pData, int DataLen)
{
	int count;

	(void)DevAddr;
	count = Esp32UartTxData(pDev, pData, DataLen);
	if (count == 0)
	{
		// Model the ISR draining a saturated queue before the next retry.
		Drain();
		s_TxCount = 0;
	}
	return count;
}

static int TxCallback(UARTDev_t *pDev, UART_EVT Event, uint8_t *pData, int Len)
{
	(void)pDev;
	(void)Event;
	(void)pData;
	(void)Len;
	return 0;
}

int main()
{
	uint8_t data[1000];
	uint8_t received[400];
	int count;
	int len;
	uint8_t *p;
	unsigned i;
	int mode;
	bool blocking;

	for (i = 0; i < sizeof(data); i++)
	{
		data[i] = i;
	}
	for (mode = 0; mode < 2; mode++)
	{
		blocking = mode == 0;
		Reset(blocking);
		// Exercise FIFO wrap over repeated foreground/ISR transfers.
		for (i = 0; i < 20; i++)
		{
			s_TxCount = 0;
			assert(Esp32UartTxData(&s_Uart.DevIntrf, data, 400) == 400);
			Drain();
			assert(std::memcmp(s_Tx.data() + i * 400, data, 400) == 0);
		}
		assert(s_Masked == 0);
		for (i = 0; i < 400; i++)
		{
			assert(CFifoPutByte(s_Uart.hRxFifo, data[i]));
		}
		assert(Esp32UartRxData(&s_Uart.DevIntrf, received, 400) == 400);
		assert(std::memcmp(received, data, 400) == 0);
		assert(!s_Uart.bRxReady);
		assert(s_Masked == 0);
	}

	Reset(true);
	count = Esp32UartTxData(&s_Uart.DevIntrf, data, sizeof(data));
	assert(count == 512);
	Drain();
	assert(s_Tx.size() == (unsigned)count);
	assert(std::memcmp(s_Tx.data(), data, count) == 0);

	Reset(true);
	// Pre-existing backlog must precede newly queued bytes.
	len = 100;
	p = CFifoPutMultiple(s_Uart.hTxFifo, &len);
	std::memcpy(p, data, len);
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data + 100, 400) == 400);
	Drain();
	assert(s_Tx.size() == 500);
	assert(std::memcmp(s_Tx.data(), data, 500) == 0);

	Reset(true);
	// An active transmitter still receives data through the software FIFO.
	s_Uart.bTxReady = false;
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data, 16) == 16);
	assert(s_Tx.empty());
	Drain();
	assert(s_Tx.size() == 16);

	Reset(true);
	s_Uart.hStdOut = 1;
	assert(UARTStdDevWrite(&s_Uart, 1, data, sizeof(data)) == sizeof(data));
	Drain();
	assert(s_Tx.size() == sizeof(data));
	assert(std::memcmp(s_Tx.data(), data, sizeof(data)) == 0);
	assert(s_Masked == 0);

	Reset(true);
	s_Uart.DevIntrf.bIntEn = false;
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data, 16) == 16);
	assert(s_Tx.size() == 16);
	assert(std::memcmp(s_Tx.data(), data, 16) == 0);
	assert((ESP32_UART_INT_ENA_REG(0) & ESP32_UART_INT_TXFIFO_EMPTY) == 0);

	Reset(true);
	// A full hardware FIFO leaves byte writes queued for interrupt refill.
	s_TxCount = ESP32_UART_HW_FIFO_LEN;
	for (i = 0; i < 512; i++)
	{
		assert(Esp32UartTxData(&s_Uart.DevIntrf, data + i, 1) == 1);
		assert(s_Tx.empty());
	}
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data + 512, 1) == 0);
	Drain();
	assert(s_Uart.bTxReady);
	assert(s_Tx.size() == 512);
	assert(std::memcmp(s_Tx.data(), data, 512) == 0);
	s_TxCount = 0;
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data + 512, 1) == 1);
	assert(s_Tx.size() == 513);
	assert(s_Tx.back() == data[512]);

	Reset(true);
	s_Uart.EvtCallback = TxCallback;
	// Room in the hardware FIFO: the byte goes out at once, TX stays ready
	// and the TX empty interrupt stays off, so a writer slower than the line
	// takes no interrupt per byte.
	for (i = 0; i < 300; i++)
	{
		assert(Esp32UartTxData(&s_Uart.DevIntrf, data + i, 1) == 1);
		assert(s_Uart.bTxReady);
		assert((ESP32_UART_INT_ENA_REG(0) & ESP32_UART_INT_TXFIFO_EMPTY) == 0);
		assert(CFifoUsed(s_Uart.hTxFifo) == 0);
		s_TxCount = 0;
	}
	assert(s_Tx.size() == 300);
	assert(std::memcmp(s_Tx.data(), data, 300) == 0);
	// Hardware FIFO full: the byte stays queued and arms the interrupt,
	// which sends it and leaves TX ready again.
	s_TxCount = ESP32_UART_HW_FIFO_LEN;
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data + 300, 1) == 1);
	assert(!s_Uart.bTxReady);
	assert((ESP32_UART_INT_ENA_REG(0) & ESP32_UART_INT_TXFIFO_EMPTY) != 0);
	assert(s_Tx.size() == 300);
	Drain();
	assert(s_Uart.bTxReady);
	assert((ESP32_UART_INT_ENA_REG(0) & ESP32_UART_INT_TXFIFO_EMPTY) == 0);
	assert(s_Tx.size() == 301);
	assert(s_Tx.back() == data[300]);

	Reset(true);
	s_Uart.EvtCallback = TxCallback;
	// Match PRBS callback and one-byte writes with the interrupt consumer.
	for (i = 0; i < sizeof(data); i++)
	{
		assert(Esp32UartTxData(&s_Uart.DevIntrf, data + i, 1) == 1);
		if (i % 64 == 63)
		{
			Drain();
			s_TxCount = 0;
		}
	}
	Drain();
	assert(s_Tx.size() == sizeof(data));
	assert(std::memcmp(s_Tx.data(), data, sizeof(data)) == 0);
	assert(s_Masked == 0);

	Reset(true);
	s_Uart.bTxReady = false;
	for (i = 0; i < 512; i++)
	{
		assert(Esp32UartTxData(&s_Uart.DevIntrf, data + i, 1) == 1);
	}
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data, 1) == 0);
	assert(s_Masked == 0);
	assert(s_Tx.empty());
	Drain();
	assert(s_Tx.size() == 512);

	Reset(true);
	s_Uart.DevIntrf.MaxRetry = 2;
	count = Esp32UartTxData(&s_Uart.DevIntrf, data, sizeof(data));
	assert(count == 640);
	Drain();
	assert(s_Tx.size() == (unsigned)count);
	assert(std::memcmp(s_Tx.data(), data, count) == 0);
	assert(s_Uart.TxDropCnt == 0);

	Reset(false);
	assert(Esp32UartTxData(&s_Uart.DevIntrf, data, sizeof(data)) == 1000);
	Drain();
	assert(s_Tx.size() == 512);
	assert(std::memcmp(s_Tx.data(), data + 488, 512) == 0);
	assert(s_Uart.TxDropCnt == 488);
	return 0;
}
