/**-------------------------------------------------------------------------
@example	qdec_demo.cpp

@brief	Quadrature decoder demo

Follows a rotary encoder on the QDEC_PHA and QDEC_PHB pins of board.h and logs
the position and the error count with SysLog on the console UART each time it
changes.

The pins have pull-ups, an encoder with common to ground connects directly.
Without an encoder, two buttons to ground on these pins can play one: press
A, press B, release A, release B moves the position by 4, the reverse order
moves it back. The default sampling period is long enough for the button
bounce, board.h can set a shorter one for a real encoder.

The board.h of the project gives the console UART pins and the encoder pins.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

@license

MIT License

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
#include <stdint.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "syslog.h"
#include "coredev/qdec.h"

#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif

// Input sampling period in usec, long enough for button bounce
#ifndef QDEC_DEMO_SAMPLE_PERIOD
#define QDEC_DEMO_SAMPLE_PERIOD	8000
#endif

// Movement report period in usec
#ifndef QDEC_DEMO_REPORT_PERIOD
#define QDEC_DEMO_REPORT_PERIOD	100000
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static const IOPinCfg_t s_QdecPins[] = {
	{QDEC_PHA_PORT, QDEC_PHA_PIN, QDEC_PHA_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{QDEC_PHB_PORT, QDEC_PHB_PIN, QDEC_PHB_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static void QdecDemoMoved(QdecDev_t * const pDev, int32_t Count);

static const QdecCfg_t s_QdecCfg = {
	.DevNo = 0,
	.pPins = s_QdecPins,
	.NbPins = sizeof(s_QdecPins) / sizeof(IOPinCfg_t),
	.usSamplePeriod = QDEC_DEMO_SAMPLE_PERIOD,
	.usReportPeriod = QDEC_DEMO_REPORT_PERIOD,
	.bDebounce = true,
	.bLedActHigh = false,
	.usLedLead = 0,
	.IntPrio = 6,
	.EvtHandler = QdecDemoMoved,
};

static UART s_Uart;

// SysLog on the console UART, a record goes out as soon as it is logged.
// 16 records of 128 bytes. A record the UART does not take in full stays in
// the store and the rest goes out at the next log call. bBlocking keeps the
// oldest records when the store is full.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

static Qdec s_Qdec;

// Interrupt context: only a flag, main logs
static volatile bool s_bQdecDemoMoved = false;

static void QdecDemoMoved(QdecDev_t * const pDev, int32_t Count)
{
	(void)pDev;
	(void)Count;

	s_bQdecDemoMoved = true;
}

int main()
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "QdecDemo\r\n");

	if (s_Qdec.Init(s_QdecCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Decoder init failed\r\n");

		while (1)
		{
			__WFE();
		}
	}

	QdecDev_t *dev = s_Qdec;

	SysLogPrintf(SysLogGet(), "Sampling %u us, report %u us\r\n", (unsigned)dev->usSamplePeriod,
				 (unsigned)dev->usReportPeriod);

	s_Qdec.Start();

	int32_t pos = s_Qdec.Position();
	uint32_t errcnt = s_Qdec.ErrCount();

	SysLogPrintf(SysLogGet(), "Position %d, errors %u\r\n", (int)pos, (unsigned)errcnt);

	while (1)
	{
		__WFE();

		if (s_bQdecDemoMoved)
		{
			s_bQdecDemoMoved = false;

			int32_t p = s_Qdec.Position();
			uint32_t e = s_Qdec.ErrCount();

			if (p != pos || e != errcnt)
			{
				pos = p;
				errcnt = e;
				SysLogPrintf(SysLogGet(), "Position %d, errors %u\r\n", (int)pos, (unsigned)errcnt);
			}
		}
	}

	return 0;
}
