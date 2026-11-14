/*--------------------------------------------------------------------------
File   : uart_lpcxx.cpp

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : LPC UART implementation

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
#include <string.h>
#include <limits.h>

#include "uart_lpcxx.h"
#include "uart_lpc_target.h"
#include "coredev/interrupt.h"

#define LPCUART_FIFO_DEPTH		16
#define LPCUART_CFIFO_SIZE		CFIFO_MEMSIZE(16)
#define LPCUART_WAIT_COUNT		100000U
#define LPCUART_DIV_MAX			65535U
// Passes of the interrupt handler over the interrupt sources before it
// returns, when new ones come up while it runs
#define LPCUART_IRQ_PASSES		4

#define LPCUART_LSR_ERR			(LPCUART_LSR_OE | LPCUART_LSR_PE | LPCUART_LSR_FE | LPCUART_LSR_BI)

// One state and pair of default buffers per physical UART.
typedef struct {
	UARTDev_t *pUartDev;
	UARTCfg_t Cfg;
	bool bEnabled;
	bool bAutoBaud;
	uint8_t RxMem[LPCUART_CFIFO_SIZE] __attribute__((aligned(4)));
	uint8_t TxMem[LPCUART_CFIFO_SIZE] __attribute__((aligned(4)));
} LpcUartDev_t;

typedef struct {
	uint32_t Divisor;
	uint32_t Fraction;
	uint32_t Rate;
} LpcUartRate_t;

static LpcUartDev_t s_LpcUartDev[LPCUART_DEV_COUNT];

static LpcUartRate_t LpcUARTRateFind(uint32_t Clock, uint32_t Rate);
static uint32_t LpcUARTGetRate(DevIntrf_t *pDev);
static uint32_t LpcUARTSetRate(DevIntrf_t *pDev, uint32_t Rate);
static int LpcUARTRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen);
static int LpcUARTTxData(DevIntrf_t *pDev, const uint8_t *pData, int DataLen);
static bool LpcUARTStart(DevIntrf_t *pDev, uint32_t DevAddr);
static void LpcUARTStop(DevIntrf_t *pDev);
static void LpcUARTEnable(DevIntrf_t *pDev);
static void LpcUARTDisable(DevIntrf_t *pDev);
static void LpcUARTReset(DevIntrf_t *pDev);
static void *LpcUARTGetHandle(DevIntrf_t *pDev);
static uint32_t LpcUARTTxFill(LpcUartDev_t *pDev);
static uint32_t LpcUARTRxFill(LpcUartDev_t *pDev);

// Count the receive errors of one line status value and return them as line
// state flags. Reading LSR clears the error flags, so every read of LSR goes
// through here : a read on the transmit side would otherwise drop an overrun
// before the receive side sees it. Called only when an error flag is set.
static uint32_t __attribute__((noinline)) LpcUARTLineErr(UARTDev_t *pUart, uint32_t Lsr)
{
	uint32_t errors = 0;

	if (Lsr & LPCUART_LSR_OE)
	{
		pUart->RxOvrErrCnt++;
		errors |= UART_LINESTATE_OVR;
	}
	if (Lsr & LPCUART_LSR_PE)
	{
		pUart->ParErrCnt++;
		errors |= UART_LINESTATE_PARERR;
	}
	if (Lsr & LPCUART_LSR_FE)
	{
		pUart->FramErrCnt++;
		errors |= UART_LINESTATE_FRMERR;
	}
	if (Lsr & LPCUART_LSR_BI)
	{
		errors |= UART_LINESTATE_BRK;
	}

	return errors;
}

// Fixed 16x sampling. Compare both neighbouring divisors for every legal
// fraction. DLL must be at least 3 when DLM is zero and DIVADDVAL is nonzero.
// Compare rational errors without rounding rates or overflowing a product.
static LpcUartRate_t LpcUARTRateFind(uint32_t Clock, uint32_t Rate)
{
	LpcUartRate_t result = {0, 0, 0};
	uint64_t bestWhole = UINT64_MAX;
	uint64_t bestRem = 0;
	uint32_t bestDen = 1;

	if (Clock == 0 || Rate == 0)
	{
		return result;
	}

	for (uint32_t mul = 1; mul <= 15; mul++)
	{
		for (uint32_t add = 0; add < mul; add++)
		{
			uint64_t num = (uint64_t)Clock * mul;
			uint32_t scale = 16U * (mul + add);
			uint32_t min = add == 0 ? 1U : 3U;
			uint64_t ideal = num / ((uint64_t)Rate * scale);

			for (uint32_t next = 0; next < 2; next++)
			{
				uint64_t div = ideal + next;

				if (div < min)
				{
					div = min;
				}
				if (div > LPCUART_DIV_MAX)
				{
					div = LPCUART_DIV_MAX;
				}

				uint32_t den = scale * (uint32_t)div;
				uint64_t target = (uint64_t)Rate * den;
				uint64_t err = num > target ? num - target : target - num;
				uint64_t whole = err / den;
				uint64_t rem = err % den;

				if (whole < bestWhole ||
					(whole == bestWhole && rem * bestDen < bestRem * den))
				{
					bestWhole = whole;
					bestRem = rem;
					bestDen = den;
					result.Divisor = (uint32_t)div;
					result.Fraction = (mul << 4) | add;
					result.Rate = (uint32_t)((num + den / 2U) / den);
				}
			}
		}
	}

	return result;
}

static uint32_t LpcUARTGetRate(DevIntrf_t *pDev)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	int no = dev->Cfg.DevNo;
	uint32_t state = DisableInterrupt();

	if (dev->bEnabled && dev->bAutoBaud &&
		(LPCUART_READ(no, ACR) & LPCUART_ACR_START) == 0)
	{
		uint32_t lcr = LPCUART_READ(no, LCR);

		LPCUART_WRITE(no, LCR, lcr | LPCUART_LCR_DLAB);
		uint32_t div = (LPCUART_READ(no, DLM) << 8) | LPCUART_READ(no, DLL);
		LPCUART_WRITE(no, LCR, lcr);
		if (div != 0)
		{
			dev->pUartDev->Rate = LpcUARTClockFreq(no) / (16U * div);
			dev->bAutoBaud = false;
		}
	}
	EnableInterrupt(state);

	return dev->pUartDev->Rate;
}

static uint32_t LpcUARTSetRate(DevIntrf_t *pDev, uint32_t Rate)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	int no = dev->Cfg.DevNo;
	LpcUartRate_t rate = LpcUARTRateFind(LpcUARTClockFreq(no), Rate);

	if (Rate != 0 && rate.Divisor == 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	// Do not change the clock of bytes already queued for transmission.
	if (!dev->bEnabled || CFifoUsed(dev->pUartDev->hTxFifo) != 0)
	{
		EnableInterrupt(state);
		return dev->pUartDev->Rate;
	}
	uint32_t wait = LPCUART_WAIT_COUNT;

	while ((LPCUART_READ(no, LSR) & LPCUART_LSR_TEMT) == 0 && --wait != 0)
	{
	}
	if (wait == 0)
	{
		EnableInterrupt(state);
		return dev->pUartDev->Rate;
	}

	uint32_t lcr = LPCUART_READ(no, LCR) & ~LPCUART_LCR_DLAB;

	LPCUART_WRITE(no, ACR, LPCUART_ACR_ABEO_INTCLR | LPCUART_ACR_ABTO_INTCLR);
	LPCUART_WRITE(no, LCR, lcr | LPCUART_LCR_DLAB);
	LPCUART_WRITE(no, DLL, Rate == 0 ? 1U : rate.Divisor & 0xFFU);
	LPCUART_WRITE(no, DLM, Rate == 0 ? 0U : rate.Divisor >> 8);
	LPCUART_WRITE(no, FDR, Rate == 0 ? 0x10U : rate.Fraction);
	LPCUART_WRITE(no, LCR, lcr);
	dev->bAutoBaud = Rate == 0;
	dev->pUartDev->Rate = rate.Rate;
	if (dev->bAutoBaud)
	{
		LPCUART_WRITE(no, ACR, LPCUART_ACR_START | LPCUART_ACR_AUTORESTART);
	}
	EnableInterrupt(state);

	return rate.Rate;
}

// Called with interrupts masked, and only writes when the hardware FIFO
// is empty. THRE then guarantees room for a complete 16-byte burst.
// Returns the receive errors found in the line status it had to read.
static uint32_t LpcUARTTxFill(LpcUartDev_t *pDev)
{
	int no = pDev->Cfg.DevNo;
	UARTDev_t *uart = pDev->pUartDev;
	uint32_t lsr = LPCUART_READ(no, LSR);
	uint32_t errors = 0;

	if (lsr & LPCUART_LSR_ERR)
	{
		errors = LpcUARTLineErr(uart, lsr);
	}
	// The FIFO is known to be empty only when a get comes back empty. Until
	// then there may be more to send : the transmit interrupt stays on and
	// the next one finds out. No call to count the FIFO is needed.
	bool more = true;

	if (lsr & LPCUART_LSR_THRE)
	{
		for (int i = 0; i < LPCUART_FIFO_DEPTH; i++)
		{
			int data = CFifoGetByte(uart->hTxFifo);

			if (data < 0)
			{
				more = false;
				break;
			}
			LPCUART_WRITE(no, THR, (uint8_t)data);
			// Something is in the transmitter now
			lsr &= ~LPCUART_LSR_THRE;
		}
	}
	uint32_t ier = LPCUART_READ(no, IER);
	bool pending = more || (lsr & LPCUART_LSR_THRE) == 0;

	uart->bTxReady = !pending;
	LPCUART_WRITE(no, IER, pending ? ier | LPCUART_IER_THRE : ier & ~LPCUART_IER_THRE);

	return errors;
}

// Bound the work even if new bytes arrive continuously. Read RBR even when
// the software FIFO is full, so a full FIFO cannot pin the CPU in the ISR.
static uint32_t LpcUARTRxFill(LpcUartDev_t *pDev)
{
	UARTDev_t *uart = pDev->pUartDev;
	hCFifo_t fifo = uart->hRxFifo;
	int no = pDev->Cfg.DevNo;
	uint32_t errors = 0;
	// Per call, not per byte
	uint32_t mask = (1U << uart->DataBits) - 1U;
	uint32_t dropped = fifo->DropCnt;

	for (int i = 0; i < LPCUART_FIFO_DEPTH; i++)
	{
		uint32_t lsr = LPCUART_READ(no, LSR);

		if (lsr & LPCUART_LSR_ERR)
		{
			errors |= LpcUARTLineErr(uart, lsr);
		}
		if ((lsr & LPCUART_LSR_RDR) == 0)
		{
			break;
		}

		if (CFifoPutByte(fifo, LPCUART_READ(no, RBR) & mask))
		{
			// The reader clears it when it finds the FIFO empty
			uart->bRxReady = true;
		}
		else
		{
			// Blocking FIFO full
			uart->RxDropCnt++;
		}
	}
	// Non blocking FIFO : blocks pushed out to make room
	uart->RxDropCnt += fifo->DropCnt - dropped;

	return errors;
}

void LpcUARTIrqHandler(int DevNo)
{
	if (DevNo < 0 || DevNo >= LPCUART_DEV_COUNT)
	{
		return;
	}
	LpcUartDev_t *dev = &s_LpcUartDev[DevNo];
	UARTDev_t *uart = dev->pUartDev;

	if (uart == NULL || !dev->bEnabled || !uart->DevIntrf.bIntEn)
	{
		return;
	}
	uint32_t state = DisableInterrupt();
	uint32_t errors = 0;
	bool rx = false;
	bool rxtimeout = false;
	bool tx = false;
	bool modem = false;

	// Serve the sources that are pending now and those that come up while
	// this runs, instead of one source per interrupt entry. At high baud
	// rates receive and transmit are often pending together.
	for (int pass = 0; pass < LPCUART_IRQ_PASSES; pass++)
	{
		uint32_t iir = LPCUART_READ(DevNo, IIR);

		if (iir & LPCUART_IIR_STATUS)
		{
			// Nothing pending
			break;
		}

		// if / else, not a switch : the compiler may turn a switch into a
		// call to a helper that stays in flash
		uint32_t id = iir & LPCUART_IIR_ID_MASK;

		if (id == LPCUART_IIR_ID_THRE)
		{
			errors |= LpcUARTTxFill(dev);
			tx = true;
		}
		else if (id == LPCUART_IIR_ID_MS)
		{
			uart->LineState = LpcUARTModemGet(DevNo);
			modem = true;
		}
		else if (id == LPCUART_IIR_ID_RLS || id == LPCUART_IIR_ID_RDA ||
				 id == LPCUART_IIR_ID_CTIMOUT)
		{
			if (id == LPCUART_IIR_ID_CTIMOUT)
			{
				rxtimeout = true;
			}
			errors |= LpcUARTRxFill(dev);
			rx = true;
		}
	}
	EnableInterrupt(state);

	if (uart->EvtCallback != NULL)
	{
		if (errors != 0 || modem)
		{
			uint32_t line = uart->LineState | errors;

			uart->EvtCallback(uart, UART_EVT_LINESTATE, (uint8_t *)&line, sizeof(line));
		}
		if (rx)
		{
			UART_EVT evt = rxtimeout ? UART_EVT_RXTIMEOUT : UART_EVT_RXDATA;

			uart->EvtCallback(uart, evt, NULL, CFifoUsed(uart->hRxFifo));
		}
		if (tx)
		{
			uart->EvtCallback(uart, UART_EVT_TXREADY, NULL, CFifoAvail(uart->hTxFifo));
		}
	}
}

static int LpcUARTRxData(DevIntrf_t *pDev, uint8_t *pBuff, int BuffLen)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	int count = 0;

	if (pBuff == NULL || BuffLen <= 0 || !dev->bEnabled)
	{
		return 0;
	}
	UARTDev_t *uart = dev->pUartDev;
	hCFifo_t fifo = uart->hRxFifo;

	if (fifo->bBlocking)
	{
		// Copy in blocks, without masking interrupts for each byte. The
		// blocks are released after the copy : released before, a full FIFO
		// would let the interrupt write over them while they are copied.
		while (count < BuffLen)
		{
			int len = BuffLen - count;
			uint8_t *p = CFifoPeekMultiple(fifo, &len);

			if (p == NULL)
			{
				// Software FIFO empty, take what the hardware holds below
				// its interrupt level
				uint32_t state = DisableInterrupt();

				LpcUARTRxFill(dev);
				EnableInterrupt(state);

				len = BuffLen - count;
				p = CFifoPeekMultiple(fifo, &len);
				if (p == NULL)
				{
					break;
				}
			}
			memcpy(&pBuff[count], p, len);
			CFifoRelease(fifo, len);
			count += len;
		}
	}
	else
	{
		// The interrupt may push the oldest bytes out of a non blocking
		// FIFO at any time, each byte is taken with interrupts masked
		while (count < BuffLen)
		{
			uint32_t state = DisableInterrupt();
			int data = CFifoGetByte(fifo);

			if (data < 0)
			{
				LpcUARTRxFill(dev);
				data = CFifoGetByte(fifo);
			}
			EnableInterrupt(state);
			if (data < 0)
			{
				break;
			}
			pBuff[count++] = (uint8_t)data;
		}
	}
	uart->bRxReady = CFifoUsed(fifo) != 0;

	return count;
}

static int LpcUARTTxData(DevIntrf_t *pDev, const uint8_t *pData, int DataLen)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	UARTDev_t *uart = dev->pUartDev;
	int no = dev->Cfg.DevNo;
	int count = 0;

	if (pData == NULL || DataLen <= 0 || !dev->bEnabled)
	{
		return 0;
	}
	if (!pDev->bIntEn)
	{
		// Polling has no ISR to drain a software queue. Return the bytes
		// written to hardware, with a bounded wait for each FIFO burst.
		while (count < DataLen)
		{
			uint32_t wait = LPCUART_WAIT_COUNT;

			while ((LPCUART_READ(no, LSR) & LPCUART_LSR_THRE) == 0 && --wait != 0)
			{
			}
			if (wait == 0)
			{
				break;
			}
			for (int i = 0; i < LPCUART_FIFO_DEPTH && count < DataLen; i++)
			{
				LPCUART_WRITE(no, THR, pData[count++]);
			}
		}
		return count;
	}

	// Queue with interrupts enabled : the space is reserved, filled, and only
	// then published, so the interrupt never reads a byte that is not there
	// yet. Interrupts were masked here for every 16 bytes together with the
	// load of the hardware FIFO, which at a high baud rate is long enough to
	// overrun the receiver. Two passes at most, the free space can wrap.
	uint32_t dropped = uart->hTxFifo->DropCnt;

	while (count < DataLen)
	{
		int len = DataLen - count;
		uint8_t *p = CFifoResvMultiple(uart->hTxFifo, &len);

		if (p == NULL || len <= 0)
		{
			break;
		}
		memcpy(p, &pData[count], len);
		CFifoPutMultiple(uart->hTxFifo, &len);
		count += len;
	}

	// One short masked section to start the transmitter, the interrupt
	// sends the rest
	uint32_t state = DisableInterrupt();

	uart->TxDropCnt += uart->hTxFifo->DropCnt - dropped;
	LpcUARTTxFill(dev);
	EnableInterrupt(state);

	return count;
}

// Start sending data the caller put in the transmit FIFO itself, with
// CFifoResvMultiple and CFifoPutMultiple on hTxFifo, for example a USB packet
// read straight into the FIFO memory. Same as the end of the Tx call.
void LpcUARTTxStart(UARTDev_t *pDev)
{
	if (pDev == NULL || pDev->DevIntrf.pDevData == NULL)
	{
		return;
	}
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->DevIntrf.pDevData;

	if (dev->bEnabled && pDev->DevIntrf.bIntEn)
	{
		uint32_t state = DisableInterrupt();

		LpcUARTTxFill(dev);
		EnableInterrupt(state);
	}
}

static bool LpcUARTStart(DevIntrf_t *pDev, uint32_t DevAddr)
{
	(void)DevAddr;
	return ((LpcUartDev_t *)pDev->pDevData)->bEnabled;
}

static void LpcUARTStop(DevIntrf_t *pDev)
{
	(void)pDev;
}

static void LpcUARTDisable(DevIntrf_t *pDev)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	int no = dev->Cfg.DevNo;
	uint32_t state = DisableInterrupt();

	if (dev->bEnabled)
	{
		NVIC_DisableIRQ((IRQn_Type)LpcUARTIrq(no));
		LPCUART_WRITE(no, IER, 0);
		LPCUART_WRITE(no, TER, 0);
		LpcUARTClock(no, false);
		dev->bEnabled = false;
	}
	EnableInterrupt(state);
}

static void LpcUARTEnable(DevIntrf_t *pDev)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	int no = dev->Cfg.DevNo;
	uint32_t state = DisableInterrupt();

	LpcUARTClock(no, true);
	IOPinCfg((const IOPinCfg_t *)dev->Cfg.pIOPinMap, dev->Cfg.NbIOPins);
	LPCUART_WRITE(no, TER, LPCUART_TER_TXEN);
	dev->bEnabled = true;
	if (pDev->bIntEn)
	{
		uint32_t ier = LPCUART_IER_RBR | LPCUART_IER_RLS;

		if (dev->Cfg.FlowControl == UART_FLWCTRL_HW)
		{
			ier |= LPCUART_IER_MS;
		}
		LPCUART_WRITE(no, IER, ier);
		LpcUARTTxFill(dev);
		NVIC_ClearPendingIRQ((IRQn_Type)LpcUARTIrq(no));
		NVIC_SetPriority((IRQn_Type)LpcUARTIrq(no), pDev->IntPrio);
		NVIC_EnableIRQ((IRQn_Type)LpcUARTIrq(no));
	}
	EnableInterrupt(state);
}

// Clear queued data and errors while retaining the configured line format.
static void LpcUARTReset(DevIntrf_t *pDev)
{
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->pDevData;
	uint32_t state = DisableInterrupt();
	int no = dev->Cfg.DevNo;

	LpcUARTClock(no, true);
	LPCUART_WRITE(no, FCR, LPCUART_FCR_FIFOEN | LPCUART_FCR_RST_RXFIFO |
		LPCUART_FCR_RST_TXFIFO | LPCUART_FCR_RX_TRIG8);
	CFifoFlush(dev->pUartDev->hRxFifo);
	CFifoFlush(dev->pUartDev->hTxFifo);
	dev->pUartDev->bRxReady = false;
	dev->pUartDev->bTxReady = true;
	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;
	LPCUART_WRITE(no, IER, LPCUART_READ(no, IER) & ~LPCUART_IER_THRE);
	if (!dev->bEnabled)
	{
		LpcUARTClock(no, false);
	}
	EnableInterrupt(state);
}

static void *LpcUARTGetHandle(DevIntrf_t *pDev)
{
	return ((LpcUartDev_t *)pDev->pDevData)->pUartDev;
}

UARTDev_t const *UARTGetInstance(int DevNo)
{
	return DevNo >= 0 && DevNo < LPCUART_DEV_COUNT ? s_LpcUartDev[DevNo].pUartDev : NULL;
}

void UARTSetCtrlLineState(UARTDev_t *pDev, uint32_t LineState)
{
	if (pDev == NULL || pDev->DevIntrf.pDevData == NULL)
	{
		return;
	}
	LpcUartDev_t *dev = (LpcUartDev_t *)pDev->DevIntrf.pDevData;
	uint32_t state = DisableInterrupt();

	if (dev->bEnabled)
	{
		LpcUARTModemSet(dev->Cfg.DevNo, LineState);
	}
	EnableInterrupt(state);
}

bool UARTInit(UARTDev_t *pDev, const UARTCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= LPCUART_DEV_COUNT || pCfg->pIOPinMap == NULL ||
		pCfg->NbIOPins < 2 || pCfg->Rate < 0 || pCfg->bDMAMode ||
		pCfg->Mode != UART_MODE_UART || pCfg->Duplex != UART_DUPLEX_FULL ||
		pCfg->DataBits < 5 || pCfg->DataBits > 8 ||
		pCfg->Parity < UART_PARITY_NONE || pCfg->Parity > UART_PARITY_SPACE ||
		(pCfg->StopBits != 1 && pCfg->StopBits != 2) ||
		(pCfg->DataBits == 5 && pCfg->StopBits == 2) ||
		(pCfg->FlowControl != UART_FLWCTRL_NONE && pCfg->FlowControl != UART_FLWCTRL_HW) ||
		!LpcUARTConfigValid(pCfg))
	{
		return false;
	}
	if ((pCfg->pRxMem != NULL && (pCfg->RxMemSize < (int)CFIFO_MEMSIZE(1) ||
		((uintptr_t)pCfg->pRxMem & 3U) != 0)) ||
		(pCfg->pTxMem != NULL && (pCfg->TxMemSize < (int)CFIFO_MEMSIZE(1) ||
		((uintptr_t)pCfg->pTxMem & 3U) != 0)))
	{
		return false;
	}
	LpcUartDev_t *dev = &s_LpcUartDev[pCfg->DevNo];
	uint32_t state = DisableInterrupt();

	// Keep the physical instance bound to its object until that object is
	// reinitialized. A stale object must never operate on another UART.
	if (dev->pUartDev != NULL && dev->pUartDev != pDev)
	{
		EnableInterrupt(state);
		return false;
	}
	for (int i = 0; i < LPCUART_DEV_COUNT; i++)
	{
		if (i != pCfg->DevNo && s_LpcUartDev[i].pUartDev == pDev)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	dev->Cfg = *pCfg;
	dev->pUartDev = pDev;
	pDev->DevIntrf.pDevData = dev;
	NVIC_DisableIRQ((IRQn_Type)LpcUARTIrq(pCfg->DevNo));
	LpcUARTPrepare(pCfg->DevNo);
	LPCUART_WRITE(pCfg->DevNo, LCR, 0);
	LPCUART_WRITE(pCfg->DevNo, IER, 0);
	LPCUART_WRITE(pCfg->DevNo, TER, 0);
	LPCUART_WRITE(pCfg->DevNo, ACR, 0);
	LpcUARTConfigure(pCfg->DevNo, pCfg);

	pDev->hRxFifo = CFifoInit(pCfg->pRxMem != NULL ? pCfg->pRxMem : dev->RxMem,
		pCfg->pRxMem != NULL ? (uint32_t)pCfg->RxMemSize : sizeof(dev->RxMem), 1, pCfg->bFifoBlocking);
	pDev->hTxFifo = CFifoInit(pCfg->pTxMem != NULL ? pCfg->pTxMem : dev->TxMem,
		pCfg->pTxMem != NULL ? (uint32_t)pCfg->TxMemSize : sizeof(dev->TxMem), 1, pCfg->bFifoBlocking);
	LpcUARTReset(&pDev->DevIntrf);
	LpcUARTClock(pCfg->DevNo, true);
	dev->bEnabled = true;

	uint32_t lcr = pCfg->DataBits - 5;

	if (pCfg->Parity != UART_PARITY_NONE)
	{
		lcr |= LPCUART_LCR_PAR_EN | ((uint32_t)pCfg->Parity << 4);
	}
	if (pCfg->StopBits == 2)
	{
		lcr |= LPCUART_LCR_STOPBIT_MASK;
	}
	LPCUART_WRITE(pCfg->DevNo, LCR, lcr);
	pDev->Rate = 0;
	EnableInterrupt(state);
	uint32_t rate = LpcUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

	if (pCfg->Rate != 0 && rate == 0)
	{
		LpcUARTDisable(&pDev->DevIntrf);
		dev->pUartDev = NULL;
		pDev->DevIntrf.pDevData = NULL;
		return false;
	}

	pDev->Mode = pCfg->Mode;
	pDev->Duplex = pCfg->Duplex;
	pDev->DataBits = pCfg->DataBits;
	pDev->Parity = pCfg->Parity;
	pDev->StopBits = pCfg->StopBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->LineState = 0;
	pDev->RxOvrErrCnt = 0;
	pDev->ParErrCnt = 0;
	pDev->FramErrCnt = 0;
	pDev->RxDropCnt = 0;
	pDev->TxDropCnt = 0;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DevIntrf.IntPrio = pCfg->IntPrio;
	if (pCfg->bIntMode)
	{
		// Select the nearest implemented priority instead of NVIC truncation.
		int maxprio = (1U << __NVIC_PRIO_BITS) - 1U;

		if (pDev->DevIntrf.IntPrio < 0)
		{
			pDev->DevIntrf.IntPrio = 0;
		}
		else if (pDev->DevIntrf.IntPrio > maxprio)
		{
			pDev->DevIntrf.IntPrio = maxprio;
		}
	}
	pDev->DevIntrf.EvtCB = NULL;
	pDev->DevIntrf.MaxRetry = 0;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = pCfg->bIntMode;
	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Disable = LpcUARTDisable;
	pDev->DevIntrf.Enable = LpcUARTEnable;
	pDev->DevIntrf.GetRate = LpcUARTGetRate;
	pDev->DevIntrf.SetRate = LpcUARTSetRate;
	pDev->DevIntrf.StartRx = LpcUARTStart;
	pDev->DevIntrf.RxData = LpcUARTRxData;
	pDev->DevIntrf.StopRx = LpcUARTStop;
	pDev->DevIntrf.StartTx = LpcUARTStart;
	pDev->DevIntrf.TxData = LpcUARTTxData;
	pDev->DevIntrf.StopTx = LpcUARTStop;
	pDev->DevIntrf.TxSrData = NULL;
	pDev->DevIntrf.Reset = LpcUARTReset;
	pDev->DevIntrf.PowerOff = LpcUARTDisable;
	pDev->DevIntrf.GetHandle = LpcUARTGetHandle;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	LpcUARTEnable(&pDev->DevIntrf);

	return true;
}
