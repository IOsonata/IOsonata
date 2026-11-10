/**-------------------------------------------------------------------------
@file	uart_esp32.cpp

@brief	IOsonata UART driver for ESP32 RISC-V (UART0, UART1)

Shared HP UART FIFO transfers for ESP32-C2/C3/C5/C6/C61. Register layout
and peripheral clock selection follow ESP-IDF v5.5.1 per-chip UART headers.
C2/C5/C61 use the shared Vectors_esp32.c interrupt entry. C3 has hardware validation; C6 has build and host MMIO checks.

Pin routing uses the application's IOPinCfg_t map. Peripheral clock/reset
operations live in system_esp32_uart_clock.c. CPU interrupt routing lives
in esp32xx_irq.c. The UART core uses g_McuOsc.CoreOsc.Freq as its XTAL rate,
with a source pre-divider and a 12-bit plus 4-bit fractional baud divider.
SetRate returns the selected baud rate. Automatic baud detection, DMA,
IrDA, software flow control and synchronous/half-duplex modes are unsupported.

@author	Hoang Nguyen Hoan
@date	Mar. 2026

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
#include <string.h>
#include <stdint.h>

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

#include "esp32xx.h"
#include "esp32xx_gpio.h"
#include "esp32xx_intmtx.h"
#include "esp32xx_irq.h"
#include "esp32xx_uart.h"

// ===========================================================================
// External IRQ-count counters (provided elsewhere -- Vectors file or test
// harness -- for cross-module inspection).
// ===========================================================================
#if defined(ESP32C3) || defined(ESP32C6)
extern "C" volatile uint32_t g_Esp32C3Uart0IrqCount;
extern "C" volatile uint32_t g_Esp32C3Uart1IrqCount;
#endif

// ===========================================================================
// Local CPU-INT level assignment.  CPU INT 1 is reserved by IDF for Wi-Fi
// and CPU INT 6 is the permanently disabled input on C3.  Use free level-1
// inputs.  UART_CPU_INT_PRIO = 1..15, must be > THRESH (which we set to 0).
// ===========================================================================
#define UART0_CPU_INT_LEVEL             2U
#if defined(ESP32C6)
#define UART1_CPU_INT_LEVEL             5U
#else
#define UART1_CPU_INT_LEVEL             3U
#endif
#define UART_CPU_INT_PRIO               1U

/*---------------------------------------------------------------------------
 * Per-instance driver data
 *---------------------------------------------------------------------------*/
#define ESP32_UART_CFIFO_SIZE		CFIFO_MEMSIZE(64)
#define ESP32_UART_COPY_MAX			128

#pragma pack(push, 4)
typedef struct {
	int          DevNo;
	uint32_t     Base;
	UARTDev_t   *pUartDev;
	uint8_t      RxFifoMem[ESP32_UART_CFIFO_SIZE];
	uint8_t      TxFifoMem[ESP32_UART_CFIFO_SIZE];
} ESP32_UARTDEV;
#pragma pack(pop)

static ESP32_UARTDEV s_Esp32UartDev[] = {
	{ .DevNo = 0, .Base = ESP32_UART0_BASE, .pUartDev = nullptr },
	{ .DevNo = 1, .Base = ESP32_UART1_BASE, .pUartDev = nullptr },
#if defined(ESP32C61)
	{ .DevNo = 2, .Base = ESP32_UART2_BASE, .pUartDev = nullptr },
#endif
};
static const int s_NbUartDev = sizeof(s_Esp32UartDev) / sizeof(s_Esp32UartDev[0]);

volatile uint32_t g_Esp32C3UartDbgUart0Map;
volatile uint32_t g_Esp32C3UartDbgUart1Map;
volatile uint32_t g_Esp32C3UartDbgCpuEnable;
volatile uint32_t g_Esp32C3UartDbgCpuType;
volatile uint32_t g_Esp32C3UartDbgCpuEip;
volatile uint32_t g_Esp32C3UartDbgCpuPri2;
volatile uint32_t g_Esp32C3UartDbgCpuPri3;
volatile uint32_t g_Esp32C3UartDbgCpuThresh;
volatile uint32_t g_Esp32C3UartDbgSrcStatus0;
volatile uint32_t g_Esp32C3UartDbgSrcStatus1;
volatile uint32_t g_Esp32C3UartDbgUart0Raw;
volatile uint32_t g_Esp32C3UartDbgUart0Ena;
volatile uint32_t g_Esp32C3UartDbgUart0St;
volatile uint32_t g_Esp32C3UartDbgUart1Raw;
volatile uint32_t g_Esp32C3UartDbgUart1Ena;
volatile uint32_t g_Esp32C3UartDbgUart1St;

static void Esp32UartDebugSnapshot(void)
{
#if defined(ESP32C3) || defined(ESP32C6)
	g_Esp32C3UartDbgUart0Map   = ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART0);
	g_Esp32C3UartDbgUart1Map   = ESP32_INTMTX_SOURCE_MAP_REG(ESP32_UART_INTR_SOURCE_UART1);
	g_Esp32C3UartDbgCpuEnable  = ESP32_CPU_INT_ENABLE_REG;
	g_Esp32C3UartDbgCpuType    = ESP32_CPU_INT_TYPE_REG;
	g_Esp32C3UartDbgCpuEip     = ESP32_CPU_INT_EIP_STATUS_REG;
	g_Esp32C3UartDbgCpuPri2    = ESP32_CPU_INT_PRI_REG(UART0_CPU_INT_LEVEL);
	g_Esp32C3UartDbgCpuPri3    = ESP32_CPU_INT_PRI_REG(UART1_CPU_INT_LEVEL);
	g_Esp32C3UartDbgCpuThresh  = ESP32_CPU_INT_THRESH_REG;
	g_Esp32C3UartDbgSrcStatus0 = ESP32_INTMTX_INTR_STATUS_0_REG;
	g_Esp32C3UartDbgSrcStatus1 = ESP32_INTMTX_INTR_STATUS_1_REG;
	g_Esp32C3UartDbgUart0Raw   = ESP32_UART_INT_RAW_REG(ESP32_UART0_BASE);
	g_Esp32C3UartDbgUart0Ena   = ESP32_UART_INT_ENA_REG(ESP32_UART0_BASE);
	g_Esp32C3UartDbgUart0St    = ESP32_UART_INT_ST_REG(ESP32_UART0_BASE);
	g_Esp32C3UartDbgUart1Raw   = ESP32_UART_INT_RAW_REG(ESP32_UART1_BASE);
	g_Esp32C3UartDbgUart1Ena   = ESP32_UART_INT_ENA_REG(ESP32_UART1_BASE);
	g_Esp32C3UartDbgUart1St    = ESP32_UART_INT_ST_REG(ESP32_UART1_BASE);
#endif
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	if (DevNo < 0 || DevNo >= s_NbUartDev)
	{
		return nullptr;
	}
	return s_Esp32UartDev[DevNo].pUartDev;
}

/*---------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/
static inline uint32_t Esp32UartTxFree(uint32_t base)
{
	return ESP32_UART_HW_FIFO_LEN -
		   ((ESP32_UART_STATUS_REG(base) & ESP32_UART_STATUS_TXFIFO_CNT_Msk) >> ESP32_UART_STATUS_TXFIFO_CNT_Pos);
}

static inline uint32_t Esp32UartRxAvail(uint32_t base)
{
	return (ESP32_UART_STATUS_REG(base) & ESP32_UART_STATUS_RXFIFO_CNT_Msk) >> ESP32_UART_STATUS_RXFIFO_CNT_Pos;
}

static inline void Esp32UartRegUpdate(uint32_t base)
{
#if defined(ESP32C5) || defined(ESP32C6) || defined(ESP32C61)
	ESP32_UART_REG_UPDATE_REG(base) = 1U;
	while (ESP32_UART_REG_UPDATE_REG(base) & 1U)
	{
	}
#else
	// C2/C3 configuration registers synchronize automatically.
	(void)base;
#endif
}

static uint32_t Esp32UartSetBaud(ESP32_UARTDEV *dev, uint32_t baud)
{
	uint64_t source;
	uint64_t divisor;
	uint32_t scaled;
	uint32_t candidate;
	uint32_t step;
	uint32_t pre;
	uint32_t rate;
	uint32_t error;
	uint32_t bestError;
	uint32_t bestRate;
	uint32_t bestPre;
	uint32_t bestScaled;

	if (baud == 0)
	{
		// Automatic baud detection is not implemented.
		return 0;
	}
	source = (uint64_t)GetHighFreqOscFreq() * 16U;
	if (source == 0)
	{
		return 0;
	}
	bestError = UINT32_MAX;
	bestRate = 0;
	bestPre = 1;
	bestScaled = 16;
	for (pre = 1; pre <= 256; pre++)
	{
		divisor = (uint64_t)baud * pre;
		scaled = (uint32_t)(source / divisor);
		if (scaled < 16U)
		{
			scaled = 16U;
		}
		if (scaled > 65535U)
		{
			scaled = 65535U;
		}
		// Compare both adjacent divisors in baud space.
		for (step = 0; step < 2; step++)
		{
			candidate = scaled + step;
			if (candidate > 65535U)
			{
				break;
			}
			divisor = (uint64_t)pre * candidate;
			rate = (uint32_t)((source + divisor / 2U) / divisor);
			error = rate > baud ? rate - baud : baud - rate;
			if (error < bestError)
			{
				bestError = error;
				bestRate = rate;
				bestPre = pre;
				bestScaled = candidate;
			}
		}
		if (bestError == 0)
		{
			break;
		}
	}
	Esp32UartClockConfig(dev->DevNo, bestPre);
	ESP32_UART_CLKDIV_REG(dev->Base) = (bestScaled >> 4) |
		((bestScaled & 15U) << ESP32_UART_CLKDIV_FRAG_Pos);
	return bestRate;
}

static bool Esp32UartIntMtxEnable(uint32_t source, uint32_t cpu_int_id, Esp32IrqHandler handler)
{
	if (!Esp32EnableSourceIrq(source, cpu_int_id, UART_CPU_INT_PRIO, handler))
	{
		return false;
	}
#if defined(ESP32C3) || defined(ESP32C6)
	Esp32UartDebugSnapshot();
#endif
	return true;
}

/*---------------------------------------------------------------------------
 * IRQ handler - invoked from the ESP32-C3 CPU interrupt input
 *---------------------------------------------------------------------------*/
// Transfer queued bytes into free hardware TX FIFO slots.
// Called from the ISR or with interrupts disabled.
static void Esp32UartTxFill(ESP32_UARTDEV *pDev)
{
	hCFifo_t fifo;
	uint8_t *p;
	int space;
	int len;
	int i;
	bool blocking;

	fifo = pDev->pUartDev->hTxFifo;
	blocking = CFifoIsBlocking(fifo);
	space = (int)Esp32UartTxFree(pDev->Base);
	while (space > 0)
	{
		len = space;
		p = blocking ? CFifoPeekMultiple(fifo, &len) :
			CFifoGetMultiple(fifo, &len);
		if (p == nullptr || len <= 0)
		{
			break;
		}
		for (i = 0; i < len; i++)
		{
			ESP32_UART_FIFO_REG(pDev->Base) = p[i];
		}
		if (blocking)
		{
			CFifoRelease(fifo, len);
		}
		space -= len;
	}
}

static void Esp32UartIRQHandler(ESP32_UARTDEV *pDev)
{
	UARTDev_t *udev;
	uint32_t base;
	uint32_t st;
	uint8_t *p;
	int available;
	int len;
	int i;

	base = pDev->Base;
	udev = pDev->pUartDev;
	st = ESP32_UART_INT_ST_REG(base);

	if (st & ESP32_UART_INT_RXFIFO_OVF)
	{
		udev->RxOvrErrCnt++;
	}
	if (st & ESP32_UART_INT_FRM_ERR)
	{
		udev->FramErrCnt++;
	}
	if (st & ESP32_UART_INT_PARITY_ERR)
	{
		udev->ParErrCnt++;
	}
	if (st & (ESP32_UART_INT_FRM_ERR | ESP32_UART_INT_PARITY_ERR |
		ESP32_UART_INT_RXFIFO_OVF | ESP32_UART_INT_GLITCH_DET))
	{
		ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_FRM_ERR |
			ESP32_UART_INT_PARITY_ERR | ESP32_UART_INT_RXFIFO_OVF |
			ESP32_UART_INT_GLITCH_DET;
	}

	if (st & (ESP32_UART_INT_RXFIFO_FULL | ESP32_UART_INT_RXFIFO_TOUT))
	{
		// Reserve contiguous FIFO memory, read a hardware batch, then
		// publish it once. No consumer sees data before it is written.
		available = (int)Esp32UartRxAvail(base);
		while (available > 0)
		{
			len = available;
			p = CFifoResvMultiple(udev->hRxFifo, &len);
			if (p == nullptr || len <= 0)
			{
				// Even when the software FIFO is full, drain the
				// hardware FIFO so RX interrupts can make progress.
				while (available > 0)
				{
					(void)ESP32_UART_FIFO_REG(base);
					udev->RxDropCnt++;
					available--;
				}
				break;
			}
			for (i = 0; i < len; i++)
			{
				p[i] = (uint8_t)ESP32_UART_FIFO_REG(base);
			}
			(void)CFifoPutMultiple(udev->hRxFifo, &len);
			available -= len;
		}
		ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_RXFIFO_FULL |
			ESP32_UART_INT_RXFIFO_TOUT;
		udev->bRxReady = CFifoUsed(udev->hRxFifo) > 0;
		if (udev->EvtCallback != nullptr)
		{
			udev->EvtCallback(udev, UART_EVT_RXDATA, nullptr,
				CFifoUsed(udev->hRxFifo));
		}
	}

	if (st & ESP32_UART_INT_TXFIFO_EMPTY)
	{
		Esp32UartTxFill(pDev);
		if (CFifoUsed(udev->hTxFifo) == 0)
		{
			ESP32_UART_INT_ENA_REG(base) &= ~ESP32_UART_INT_TXFIFO_EMPTY;
			udev->bTxReady = true;
			if (udev->EvtCallback != nullptr)
			{
				udev->EvtCallback(udev, UART_EVT_TXREADY, nullptr, 0);
			}
		}
		ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_TXFIFO_EMPTY;
	}
}

/*---------------------------------------------------------------------------
 * IRQ entry points. Installed by Esp32UartIntMtxEnable().
 *---------------------------------------------------------------------------*/
extern "C" __attribute__((used))
void UART0_IRQHandler(void)
{
#if defined(ESP32C3) || defined(ESP32C6)
	g_Esp32C3Uart0IrqCount++;
#endif
	Esp32UartIRQHandler(&s_Esp32UartDev[0]);
}

extern "C" __attribute__((used))
void UART1_IRQHandler(void)
{
#if defined(ESP32C3) || defined(ESP32C6)
	g_Esp32C3Uart1IrqCount++;
#endif
	Esp32UartIRQHandler(&s_Esp32UartDev[1]);
}

#if defined(ESP32C61)
extern "C" void UART2_IRQHandler(void)
{
	Esp32UartIRQHandler(&s_Esp32UartDev[2]);
}
#endif

/*---------------------------------------------------------------------------
 * DevIntrf operations
 *---------------------------------------------------------------------------*/
static uint32_t Esp32UartGetRate(DevIntrf_t * const pDev)
{
	ESP32_UARTDEV *dev;

	dev = (ESP32_UARTDEV *)pDev->pDevData;
	return dev->pUartDev->Rate;
}

static uint32_t Esp32UartSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	ESP32_UARTDEV *dev;

	dev = (ESP32_UARTDEV *)pDev->pDevData;
	Rate = Esp32UartSetBaud(dev, Rate);
	if (Rate == 0)
	{
		return 0;
	}
	Esp32UartRegUpdate(dev->Base);
	dev->pUartDev->Rate = Rate;
	return Rate;
}

static void Esp32UartEnable(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void Esp32UartDisable(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void Esp32UartPowerOff(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static void Esp32UartReset(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool Esp32UartStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;
	return true;
}

static void Esp32UartStopRx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static bool Esp32UartStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	(void)pDev;
	(void)DevAddr;
	return true;
}

static void Esp32UartStopTx(DevIntrf_t * const pDev)
{
	(void)pDev;
}

static int Esp32UartRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	ESP32_UARTDEV *dev;
	UARTDev_t *udev;
	hCFifo_t fifo;
	uint8_t *p;
	uintptr_t state;
	int cnt;
	int len;

	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}

	dev = (ESP32_UARTDEV *)pDev->pDevData;
	udev = dev->pUartDev;
	fifo = udev->hRxFifo;
	cnt = 0;

	if (pDev->bIntEn)
	{
		if (CFifoIsBlocking(fifo))
		{
			// The ISR cannot overwrite an unread blocking FIFO block.
			// Keep RX interrupts enabled during the buffer copy.
			while (cnt < BuffLen)
			{
				len = BuffLen - cnt;
				p = CFifoPeekMultiple(fifo, &len);
				if (p == nullptr)
				{
					break;
				}
				memcpy(pBuff + cnt, p, len);
				CFifoRelease(fifo, len);
				cnt += len;
			}
		}
		else
		{
			// A non-blocking producer can discard unread blocks.
			// Protect the copy until the blocks have been consumed.
			while (cnt < BuffLen)
			{
				len = BuffLen - cnt;
				if (len > ESP32_UART_COPY_MAX)
				{
					len = ESP32_UART_COPY_MAX;
				}
				state = DisableInterrupt();
				p = CFifoGetMultiple(fifo, &len);
				if (p != nullptr)
				{
					memcpy(pBuff + cnt, p, len);
				}
				EnableInterrupt(state);
				if (p == nullptr)
				{
					break;
				}
				cnt += len;
			}
		}
		// Clear first so a later RX interrupt cannot be overwritten by
		// a stale empty result from the foreground reader.
		udev->bRxReady = false;
		if (CFifoUsed(fifo) > 0)
		{
			udev->bRxReady = true;
		}
	}
	else
	{
		while (cnt < BuffLen && Esp32UartRxAvail(dev->Base) > 0U)
		{
			pBuff[cnt] = (uint8_t)ESP32_UART_FIFO_REG(dev->Base);
			cnt++;
		}
	}
	return cnt;
}

// Queue each bounded copy before exposing it to the TX interrupt.
static int Esp32UartTxPut(ESP32_UARTDEV *pDev, const uint8_t *pData, int DataLen)
{
	UARTDev_t *udev = pDev->pUartDev;
	hCFifo_t fifo = udev->hTxFifo;
	uintptr_t state;
	uint32_t dropped;
	uint8_t *p;
	int cnt = 0;
	int len;

	while (DataLen > 0)
	{
		len = DataLen < ESP32_UART_COPY_MAX ? DataLen : ESP32_UART_COPY_MAX;
		state = DisableInterrupt();
		dropped = fifo->DropCnt;
		p = CFifoPutMultiple(fifo, &len);
		if (p != nullptr)
		{
			memcpy(p, pData, len);
		}
		udev->TxDropCnt += fifo->DropCnt - dropped;
		EnableInterrupt(state);
		if (p == nullptr)
		{
			break;
		}
		pData += len;
		DataLen -= len;
		cnt += len;
	}

	return cnt;
}

static int Esp32UartTxData(DevIntrf_t * const pDev, uint8_t const *pData, int DataLen)
{
	ESP32_UARTDEV *dev;
	UARTDev_t *udev;
	uintptr_t state;
	uint32_t dropped;
	int cnt;
	int space;
	bool pending;

	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	dev = (ESP32_UARTDEV *)pDev->pDevData;
	udev = dev->pUartDev;
	if (!pDev->bIntEn)
	{
		cnt = 0;
		while (DataLen > 0)
		{
			space = (int)Esp32UartTxFree(dev->Base);
			while (DataLen > 0 && space > 0)
			{
				ESP32_UART_FIFO_REG(dev->Base) = *pData;
				pData++;
				DataLen--;
				space--;
				cnt++;
			}
		}
		return cnt;
	}

	if (DataLen == 1)
	{
		state = DisableInterrupt();
		dropped = udev->hTxFifo->DropCnt;
		cnt = CFifoPutByte(udev->hTxFifo, *pData) ? 1 : 0;
		udev->TxDropCnt += udev->hTxFifo->DropCnt - dropped;
	}
	else
	{
		cnt = Esp32UartTxPut(dev, pData, DataLen);
		state = DisableInterrupt();
	}

	Esp32UartTxFill(dev);
	pending = CFifoUsed(udev->hTxFifo) > 0;
	if (pending || (udev->EvtCallback != nullptr &&
		(cnt > 0 || !udev->bTxReady)))
	{
		udev->bTxReady = false;
		ESP32_UART_INT_CLR_REG(dev->Base) = ESP32_UART_INT_TXFIFO_EMPTY;
		ESP32_UART_INT_ENA_REG(dev->Base) |= ESP32_UART_INT_TXFIFO_EMPTY;
	}
	else
	{
		ESP32_UART_INT_ENA_REG(dev->Base) &= ~ESP32_UART_INT_TXFIFO_EMPTY;
		udev->bTxReady = !pending;
	}
	EnableInterrupt(state);
	return cnt;
}


/*---------------------------------------------------------------------------
 * UARTInit - top-level driver init
 *---------------------------------------------------------------------------*/
bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	ESP32_UARTDEV *dev;
	uint32_t base;
	uint32_t cc;
	uint32_t rate;
	uint32_t conf0;
	int devno;
	int bn;
	bool irqOk;

	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}
	if (pCfg->pIOPinMap == nullptr || pCfg->NbIOPins < 2)
	{
		return false;
	}
	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	if (pCfg->Rate <= 0 || pCfg->bDMAMode || pCfg->bIrDAMode ||
		pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL ||
		pCfg->FlowControl == UART_FLWCTRL_XONXOFF ||
		(pCfg->FlowControl == UART_FLWCTRL_HW && pCfg->NbIOPins < 4) ||
		(pCfg->Parity != UART_PARITY_NONE && pCfg->Parity != UART_PARITY_EVEN &&
		pCfg->Parity != UART_PARITY_ODD))
	{
		return false;
	}

	devno = pCfg->DevNo;
	dev = &s_Esp32UartDev[devno];
	base = dev->Base;

	pDev->DevIntrf.pDevData = dev;
	dev->pUartDev = pDev;

	// Preserve UART0's running ROM console; reset other instances.
	Esp32UartClockEnable(devno);
	Esp32UartReset(devno);
	ESP32_UART_INT_ENA_REG(base) = 0U;
	ESP32_UART_INT_CLR_REG(base) = 0xFFFFFFFFUL;
	rate = Esp32UartSetBaud(dev, (uint32_t)pCfg->Rate);
	if (rate == 0)
	{
		return false;
	}

	// Frame format: data bits, parity, stop.  Read-modify-write
	//    preserves CONF0.mem_clk_en and any other bits ROM/Init set.
	conf0 = ESP32_UART_CONF0_REG(base) & ~(ESP32_UART_CONF0_BIT_NUM_Msk | ESP32_UART_CONF0_STOP_BIT_Msk
										 | ESP32_UART_CONF0_PARITY_Msk | ESP32_UART_CONF0_PARITY_EN_Msk
										 | ESP32_UART_CONF0_TX_FLOW_EN_Msk
										 | ESP32_UART_CONF0_RXD_INV_Msk | ESP32_UART_CONF0_TXD_INV_Msk);

	bn = pCfg->DataBits;
	if (bn < 5)
	{
		bn = 5;
	}
	if (bn > 8)
	{
		bn = 8;
	}
	conf0 |= ((uint32_t)(bn - 5) << ESP32_UART_CONF0_BIT_NUM_Pos);

	// STOP_BIT field: 1 = 1 bit, 2 = 1.5, 3 = 2
	conf0 |= ((pCfg->StopBits >= 2 ? 3U : 1U) << ESP32_UART_CONF0_STOP_BIT_Pos);

	switch (pCfg->Parity)
	{
		case UART_PARITY_EVEN:
			conf0 |= ESP32_UART_CONF0_PARITY_EN_Msk;        // PARITY=0 = even
			break;
		case UART_PARITY_ODD:
			conf0 |= ESP32_UART_CONF0_PARITY_EN_Msk | ESP32_UART_CONF0_PARITY_Msk;
			break;
		default:
			break;
	}

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		conf0 |= ESP32_UART_CONF0_TX_FLOW_EN_Msk;
	}

	ESP32_UART_CONF0_REG(base) = conf0;

	// RX-full asserts when the FIFO count is greater than the threshold.
	// Deliver four-byte batches to limit streaming RX latency without
	// interrupting on every byte. Short tails use the RX timeout.
	// Keep TX-empty at 16 bytes.
	ESP32_UART_CONF1_REG(base) =
		(3U << ESP32_UART_CONF1_RXFIFO_FULL_THRHD_Pos) |
		(16U << ESP32_UART_CONF1_TXFIFO_EMPTY_THRHD_Pos);

	cc = ESP32_UART_RX_TOUT_REG(base) & ~ESP32_UART_RX_TOUT_THRHD_Msk;
	ESP32_UART_RX_TOUT_REG(base) = cc | (10U << ESP32_UART_RX_TOUT_THRHD_Pos);
	ESP32_UART_RX_TOUT_EN_REG(base) |= ESP32_UART_RX_TOUT_EN_Msk;
	cc = ESP32_UART_RX_FLOW_REG(base) & ~ESP32_UART_RX_FLOW_THRHD_Msk;
	ESP32_UART_RX_FLOW_REG(base) = cc | (96U << ESP32_UART_RX_FLOW_THRHD_Pos);
	ESP32_UART_RX_FLOW_EN_REG(base) &= ~ESP32_UART_RX_FLOW_EN_Msk;
	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		ESP32_UART_RX_FLOW_EN_REG(base) |= ESP32_UART_RX_FLOW_EN_Msk;
	}

	// IDLE_CONF: clear tx_idle_num so there's no inter-byte gap.
	//     C3 default is 0x100 (256 baud cycles ~ 2.22 ms at 115200),
	//     dropping observed line rate from ~11500 B/s to ~430 B/s.
	//     IDF's uart_hal_init clears this to 0; we do the same.
	ESP32_UART_IDLE_CONF_REG(base) = 0U;

	// C5/C6/C61 require an explicit commit of synchronized registers.
	Esp32UartRegUpdate(base);

	// Pin/matrix routing
	IOPinCfg((IOPinCfg_t *)pCfg->pIOPinMap, pCfg->NbIOPins);

	// Software FIFOs
	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(dev->RxFifoMem, ESP32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(dev->TxFifoMem, ESP32_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	pDev->Rate         = rate;
	pDev->DataBits     = pCfg->DataBits;
	pDev->Parity       = pCfg->Parity;
	pDev->StopBits     = pCfg->StopBits;
	pDev->FlowControl  = pCfg->FlowControl;
	pDev->Mode         = pCfg->Mode;
	pDev->Duplex       = pCfg->Duplex;
	pDev->EvtCallback  = pCfg->EvtCallback;
	pDev->bRxReady     = false;
	pDev->bTxReady     = true;
	pDev->RxOvrErrCnt  = 0;
	pDev->ParErrCnt    = 0;
	pDev->FramErrCnt   = 0;
	pDev->RxDropCnt    = 0;
	pDev->TxDropCnt    = 0;

	pDev->DevIntrf.Type     = DEVINTRF_TYPE_UART;
	pDev->DevIntrf.bIntEn   = pCfg->bIntMode;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.EnCnt    = 1;
	pDev->DevIntrf.Disable  = Esp32UartDisable;
	pDev->DevIntrf.Enable   = Esp32UartEnable;
	pDev->DevIntrf.PowerOff = Esp32UartPowerOff;
	pDev->DevIntrf.Reset    = Esp32UartReset;
	pDev->DevIntrf.GetRate  = Esp32UartGetRate;
	pDev->DevIntrf.SetRate  = Esp32UartSetRate;
	pDev->DevIntrf.StartRx  = Esp32UartStartRx;
	pDev->DevIntrf.RxData   = Esp32UartRxData;
	pDev->DevIntrf.StopRx   = Esp32UartStopRx;
	pDev->DevIntrf.StartTx  = Esp32UartStartTx;
	pDev->DevIntrf.TxData   = Esp32UartTxData;
	pDev->DevIntrf.StopTx   = Esp32UartStopTx;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	// Optional interrupt routing
	if (pCfg->bIntMode)
	{
		ESP32_UART_INT_CLR_REG(base) = 0xFFFFFFFFUL;
		ESP32_UART_INT_ENA_REG(base) = ESP32_UART_INT_RXFIFO_FULL | ESP32_UART_INT_RXFIFO_TOUT
						   | ESP32_UART_INT_FRM_ERR | ESP32_UART_INT_PARITY_ERR
						   | ESP32_UART_INT_RXFIFO_OVF;
		// TXFIFO_EMPTY is enabled on demand by Esp32UartTxData.

		if (devno == 0)
		{
			irqOk = Esp32UartIntMtxEnable(ESP32_UART_INTR_SOURCE_UART0, UART0_CPU_INT_LEVEL, UART0_IRQHandler);
		}
		else if (devno == 1)
		{
			irqOk = Esp32UartIntMtxEnable(ESP32_UART_INTR_SOURCE_UART1, UART1_CPU_INT_LEVEL, UART1_IRQHandler);
		}
#if defined(ESP32C61)
		else
		{
			irqOk = Esp32UartIntMtxEnable(ESP32_UART_INTR_SOURCE_UART2, 8U, UART2_IRQHandler);
		}
#endif
		if (!irqOk)
		{
			ESP32_UART_INT_ENA_REG(base) = 0;
			dev->pUartDev = nullptr;
			return false;
		}
	}

	return true;
}

void UARTSetCtrlLineState(UARTDev_t * const pDev, uint32_t LineState)
{
	(void)pDev;
	(void)LineState;
}
