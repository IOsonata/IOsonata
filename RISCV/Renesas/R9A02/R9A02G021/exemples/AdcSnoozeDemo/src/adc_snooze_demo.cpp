/**-------------------------------------------------------------------------
@example	adc_snooze_demo.cpp

@brief	A/D conversion in Snooze on the FPB-R9A02G021

The MCU stays in Software Standby. Every ADC_SNOOZE_PERIOD_MS the TML32,
counting the LOCO, requests Snooze. The Snooze entry starts the conversion
of ADC_SNOOZE_IN through the ELC, and the DTC clears the TML32 interval
flag. A result below ADC_SNOOZE_LIMIT sends the MCU back to Software Standby
without the CPU (SNZEDCR0.ADNCRED), at or above it ADC_ENDI wakes the CPU
through SELSR0: the voltage is printed and LED1 toggles. R9A02G021 User's
Manual R01UH1036EJ0110, 10.8.4 and 19.3.7.

The CPU sleeps at least one second after each result, so that an input
staying above the limit does not wake it at every interval.

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

#include "idelay.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "r9a02g021.h"
#include "system_r9a02.h"
#include "interrupt_r9a02.h"
#include "adc_r9a02.h"
#include "tml32_r9a02.h"
#include "board.h"

static void AdcEvtHandler(Device * const pDev, DEV_EVT Evt);

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

static const AdcRefVolt_t s_AdcRefVolt[] = {
	{ .Type = ADC_REFVOLT_TYPE_SUPPLY, .Voltage = ADC_SNOOZE_VCC, .Pin = -1 },
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_AdcRefVolt,
	.NbRefVolt = sizeof(s_AdcRefVolt) / sizeof(AdcRefVolt_t),
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = AdcEvtHandler,
};

static const AdcChanCfg_t s_AdcChanCfg = {
	.Chan = 0, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
	.AcqTime = 1, .BurstMode = false, .PinP = { ADC_SNOOZE_IN, ADC_PIN_CONN_NONE },
	.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
};

// The TML32 interval requests Snooze, its flag cleared by the DTC
static const R9a02Tml32Cfg_t s_Tml32Cfg = {
	.ClkSrc = R9A02_TML32_CLK_LF,
	.Mode = R9A02_TML32_MODE_SNOOZE,
	.nsPeriod = ADC_SNOOZE_PERIOD_MS * 1000000ULL,
	.Handler = NULL,
	.pCtx = NULL,
	.IntPrio = 0,
	.bWakeup = false,
};

// Out of the window back to Software Standby, in it to Normal mode
static const R9a02SnoozeCfg_t s_SnoozeCfg = {
	.Request = R9A02_SNZREQ_TML32,
	.End = R9A02_SNZEND_ADC_NOMATCH,
	.WakeEvt = R9A02_EVT_ADC_ENDI,
	.bDtc = true,
	.Handler = NULL,
	.pCtx = NULL,
	.IntPrio = IRQ_PRIO_NORMAL,
};

static UART s_Uart;
static AdcR9a02 s_Adc;
static volatile bool s_bAdcDone = false;

static void AdcEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_bAdcDone = true;
	}
}

static void Fail(const char *pMsg)
{
	printf("%s\r\n", pMsg);

	while (1)
	{
		__WFE();
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

	printf("\r\nA/D conversion in Snooze, wake-up at %d mV and above\r\n", (int)(ADC_SNOOZE_LIMIT * 1000.0f));

	if (s_Adc.Init(s_AdcCfg) == false || s_Adc.OpenChannel(&s_AdcChanCfg, 1) == false ||
		s_Adc.Window(ADC_SNOOZE_LIMIT, ADC_SNOOZE_VCC, false) == false ||
		s_Adc.Snooze(true) == false || s_Adc.StartConversion() == false)
	{
		Fail("ADC init failed");
	}

	if (R9a02Tml32Init(&s_Tml32Cfg) == false)
	{
		Fail("TML32 init failed");
	}

	if (R9a02SnoozeSet(&s_SnoozeCfg) == false)
	{
		Fail("Snooze set up failed");
	}

	while (1)
	{
		// The UART output ends before its clock stops
		msDelay(5);

		s_bAdcDone = false;
		R9a02LowPowerEnter(true);

		// ADC_ENDI follows the wake-up
		for (int i = 0; i < 100 && s_bAdcDone == false; i++)
		{
			usDelay(10);
		}

		AdcData_t data;

		if (s_Adc.Read(0, &data))
		{
			printf("%d mV\r\n", (int)(data.Data * 1000.0f + 0.5f));
			IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
			msDelay(1000);
		}
	}

	return 0;
}
