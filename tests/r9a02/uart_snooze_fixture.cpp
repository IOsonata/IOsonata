/**-------------------------------------------------------------------------
@file	uart_snooze_fixture.cpp

@brief	R9A02G021 UART reception in Snooze, emulator firmware

UART0 (DevNo 0) on P302 (TxD0_B) and P303 (RxD0_B), interrupt or polling mode.
uart_snooze_emu.py loads it, calls the Emu functions, sends the characters
and delivers the interrupts through R9a02IrqDispatch. main only waits.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <string.h>
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "sau_r9a02.h"
#include "system_r9a02.h"

#define EMU_FIFO_SIZE		CFIFO_MEMSIZE(32)

UART g_Uart;
uint8_t g_RxBuf[32];
volatile int g_RxEvt;
volatile int g_CbMode;			// UART_EVT_RXDATA: 1 echo, 2 Snooze off, 3 Disable
volatile int g_FifoBlocking;		// 0 drops oldest when full, 1 refuses new data
volatile int g_PollMode;			// 0 interrupts, 1 polling

static const R9a02SnoozeCfg_t s_OtherSnz = {
	.Request = R9A02_SNZREQ_IRQ(3),
	.End = 0,
	.WakeEvt = 0,
	.bDtc = false,
	.Handler = NULL,
	.pCtx = NULL,
	.IntPrio = 0,
};

alignas(4) static uint8_t s_RxMem[EMU_FIFO_SIZE];
alignas(4) static uint8_t s_TxMem[EMU_FIFO_SIZE];

static const IOPinCfg_t s_Pins[2] = {
	{3, 3, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 2, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static int Evt(UARTDev_t * const pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	if (EvtId == UART_EVT_RXDATA)
	{
		g_RxEvt = g_RxEvt + 1;

		if (g_CbMode == 1)
		{
			uint8_t b[8];
			int n = g_Uart.Rx(b, sizeof(b));

			g_Uart.Tx(b, n);
		}
		else if (g_CbMode == 2)
		{
			R9a02UartSnooze(g_Uart, false);
		}
		else if (g_CbMode == 3)
		{
			g_Uart.Disable();
		}
	}

	return 0;
}

// UART0 at Rate, 8N1. Returns 1 when UARTInit succeeds.
extern "C" int EmuUartInit(int Rate)
{
	UARTCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = 0;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = Rate;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.bIntMode = g_PollMode == 0;
	cfg.IntPrio = 6;
	cfg.EvtCallback = Evt;
	cfg.bFifoBlocking = g_FifoBlocking != 0;
	cfg.RxMemSize = EMU_FIFO_SIZE;
	cfg.pRxMem = s_RxMem;
	cfg.TxMemSize = EMU_FIFO_SIZE;
	cfg.pTxMem = s_TxMem;
	cfg.Duplex = UART_DUPLEX_FULL;
	cfg.Mode = UART_MODE_UART;

	return g_Uart.Init(cfg) ? 1 : 0;
}

// Reception in Snooze on or off: returns 1 on success
extern "C" int EmuUartSnooze(int bEnable)
{
	return R9a02UartSnooze(g_Uart, bEnable != 0) ? 1 : 0;
}

// Software Standby, the harness plays the time in WFI
extern "C" int EmuStandby(void)
{
	return R9a02LowPowerEnter(true) ? 1 : 0;
}

// Characters of the RX CFIFO to g_RxBuf, returns their number
extern "C" int EmuUartRx(void)
{
	return g_Uart.Rx(g_RxBuf, sizeof(g_RxBuf));
}

extern "C" int EmuUartRate(int Rate)
{
	return (int)g_Uart.Rate((uint32_t)Rate);
}

// Len bytes 0x30, 0x31... to the TX CFIFO, returns the number queued
extern "C" int EmuUartTx(int Len)
{
	uint8_t b[32];

	for (int i = 0; i < Len && i < (int)sizeof(b); i++)
	{
		b[i] = (uint8_t)(0x30 + i);
	}

	return g_Uart.Tx(b, (uint32_t)Len);
}

extern "C" int EmuUartEnable(void)
{
	g_Uart.Enable();

	return 1;
}

// Another Snooze set up (IRQ3 request) on or off
extern "C" int EmuOtherSnooze(int bEnable)
{
	return R9a02SnoozeSet(bEnable ? &s_OtherSnz : NULL) ? 1 : 0;
}

extern "C" int EmuUartPowerOff(void)
{
	g_Uart.PowerOff();

	return 1;
}

int main()
{
	while (1)
	{
		__asm volatile ("nop");
	}

	return 0;
}
