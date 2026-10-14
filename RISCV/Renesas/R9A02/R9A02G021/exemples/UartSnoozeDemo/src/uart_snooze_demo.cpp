/**-------------------------------------------------------------------------
@example	uart_snooze_demo.cpp

@brief	UART reception in Snooze on the FPB-R9A02G021

The MCU waits in Software Standby. A character sent on UART0 at 9600 bps
starts Snooze, the SAU receives it without the CPU, then wakes the CPU: the
character is printed with the number of wake-ups and LED1 toggles. A
character with an error (a terminal at another rate) sends the MCU back to
Software Standby without waking the CPU. Characters received while the CPU
runs are printed after "Received:", the MCU goes back to Software Standby
once none is left.

UART0 is on the J-Link OB virtual COM port, terminal at 9600 8N1. Reception
in Snooze needs PCLKB from the HOCO at 24 or 32 MHz (table 23.95), board.h
selects 32 MHz. The UART stops transmitting in Software Standby, the
transmission ends before. The debugger connection may be lost in Software
Standby.

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
#include <stdio.h>
#include <stdint.h>

#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "system_r9a02.h"
#include "sau_r9a02.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_UartPins[] = UART_PINS;

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

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
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;

// The TX CFIFO is emptied first, the Snooze standby hook then waits for the
// last character to leave the shift register
static void UartWaitIdle(void)
{
	UARTDev_t *dev = s_Uart;
	uint32_t n = 10000000;

	while (dev->bTxReady == false && --n > 0U)
	{
	}
}

int main()
{
	IOPinCfg(s_Leds, s_NbLeds);

	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1) __WFE();
	}
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, NULL, _IONBF, 0);

	printf("\r\nUART reception in Snooze, UART0 at %lu bps\r\n", (unsigned long)s_Uart.Rate());

	if (R9a02UartSnooze(s_Uart, true) == false)
	{
		printf("Snooze reception refused: HOCO at 24 or 32 MHz, 4800 or 9600 bps\r\n");
		while (1) __WFE();
	}

	uint32_t wakes = 0;

	while (1)
	{
		uint8_t buf[16];
		int n = s_Uart.Rx(buf, sizeof(buf));

		if (n > 0)
		{
			// Received in normal operation, after the wake-up
			printf("Received:");
		}
		else
		{
			printf("Software Standby, send a character\r\n");
			UartWaitIdle();
			R9a02LowPowerEnter(true);

			wakes++;
			IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);

			n = s_Uart.Rx(buf, sizeof(buf));
			if (n <= 0)
			{
				printf("Wake-up %lu, no character\r\n", (unsigned long)wakes);
				continue;
			}
			printf("Wake-up %lu:", (unsigned long)wakes);
		}

		for (int i = 0; i < n; i++)
		{
			printf(" 0x%02X", buf[i]);
			if (buf[i] >= 0x20 && buf[i] < 0x7F)
			{
				printf(" '%c'", buf[i]);
			}
		}
		printf("\r\n");
	}

	return 0;
}
