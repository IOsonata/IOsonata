/**-------------------------------------------------------------------------
@example	adc_demo.cpp

@brief	ADC inputs read once per second

Opens the ADC inputs that board.h lists, converts them all once per second
and prints each in volt. board.h gives:

	ADC_DRIVER_HEADER	header of the ADC implementation of the MCU
	ADC_DRIVER			its class
	ADC_REF_VOLTAGE		voltage of the ADC reference, the supply in most
						boards
	ADC_INPUTS			list of the inputs to read, the input numbers of the
						implementation (PinP.PinNo)
	ADC_OVERSAMPLE		optional, oversampling ratio, 0 when not given

With UART_RETARGET and the UART pins, the Release build prints on the UART at
115200 8N1, the Debug build uses semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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

#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "converters/adc_device.h"
#include "idelay.h"
#include "stddev.h"
#include "board.h"
#include ADC_DRIVER_HEADER

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef ADC_OVERSAMPLE
#define ADC_OVERSAMPLE			0
#endif

#define ADC_DEMO_PERIOD_MS		1000

#ifdef UART_RETARGET
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
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
#endif

static const int s_Inputs[] = ADC_INPUTS;

#define ADC_DEMO_NB_INPUT		((int)(sizeof(s_Inputs) / sizeof(s_Inputs[0])))

static const AdcRefVolt_t s_AdcRef[] = {
	{ ADC_REFVOLT_TYPE_SUPPLY, ADC_REF_VOLTAGE, -1 },
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_AdcRef,
	.NbRefVolt = 1,
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = ADC_OVERSAMPLE,
	.bInterrupt = false,
	.IntPrio = 0,
	.EvtHandler = NULL,
};

static ADC_DRIVER s_Adc;
static AdcChanCfg_t s_AdcChan[ADC_DEMO_NB_INPUT];
static AdcData_t s_AdcData[ADC_DEMO_NB_INPUT];

// Prints a voltage with 3 decimals, without float printf
static void PrintVolt(float V)
{
	int mv = (int)(V * 1000.0f + (V < 0 ? -0.5f : 0.5f));

	if (mv < 0)
	{
		printf("-");
		mv = -mv;
	}
	printf("%d.%03d V", mv / 1000, mv % 1000);
}

int main()
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
#endif

	printf("\r\nADC, %d inputs, reference ", ADC_DEMO_NB_INPUT);
	PrintVolt(ADC_REF_VOLTAGE);
	printf(", oversampling %d\r\n", ADC_OVERSAMPLE);

	if (s_Adc.Init(s_AdcCfg, nullptr, nullptr) == false)
	{
		printf("ADC init failed\r\n");

		return 1;
	}

	for (int i = 0; i < ADC_DEMO_NB_INPUT; i++)
	{
		s_AdcChan[i] = {
			.Chan = i,
			.RefVoltIdx = 0,
			.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
			.Gain = 0x100,
			.AcqTime = 0,
			.BurstMode = false,
			.PinP = { .PinNo = s_Inputs[i], .Conn = ADC_PIN_CONN_NONE },
			.PinN = { .PinNo = -1, .Conn = ADC_PIN_CONN_NONE },
			.FifoMemSize = 0,
			.pFifoMem = nullptr,
		};
	}

	if (s_Adc.OpenChannel(s_AdcChan, ADC_DEMO_NB_INPUT) == false)
	{
		printf("ADC channels refused\r\n");

		return 1;
	}

	while (1)
	{
		if (s_Adc.StartConversion() == false)
		{
			printf("Conversion failed\r\n");
		}
		else
		{
			int n = s_Adc.Read(s_AdcData, ADC_DEMO_NB_INPUT);

			for (int i = 0; i < n; i++)
			{
				printf("%sinput %d ", i ? ", " : "", s_Inputs[s_AdcData[i].Chan]);
				PrintVolt(s_AdcData[i].Data);
			}
			printf("\r\n");
		}

		msDelay(ADC_DEMO_PERIOD_MS);
	}

	return 0;
}
