/**-------------------------------------------------------------------------
@example	dac_demo.cpp

@brief	SAM D11/D21 DAC demo, the output read back by the ADC

Steps the DAC through five voltages from 0 to its reference, one per
second. The output buffer drives the VOUT pin (PA02) and the ADC DAC input:
the ADC reads the output back, no wiring needed. The output is linear from
0.05 V to VDDANA - 0.05 V: on the board, 0 and the reference read a few tens
of mV inside. Each line gives the
voltage set and the voltage read. board.h gives:

	DAC_REF_VOLTAGE		VDDANA, the reference of the DAC and the ADC

With UART_RETARGET and the UART pins, the Release build prints on the UART at
1000000 baud 8N1, the Debug build uses semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include "adc_samd.h"
#include "dac_samd.h"
#include "idelay.h"
#include "stddev.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define DAC_DEMO_PERIOD_MS		1000
#define DAC_DEMO_STEPS			5

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
	.Rate = 1000000,
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

static const AdcRefVolt_t s_Ref = { ADC_REFVOLT_TYPE_SUPPLY, DAC_REF_VOLTAGE, -1 };

static const DAC_CFG s_DacCfg = {
	.Mode = DAC_MODE_SINGLE_ENDED,
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = &s_Ref,
	.NbRefVolt = 1,
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = false,
	.IntPrio = 0,
	.EvtHandler = NULL,
};

static const AdcChanCfg_t s_AdcChan = {
	.Chan = 0,
	.RefVoltIdx = 0,
	.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
	.Gain = 0x100,
	.AcqTime = 0,
	.BurstMode = false,
	.PinP = { .PinNo = ADC_INPUTCTRL_MUXPOS_DAC_Val, .Conn = ADC_PIN_CONN_NONE },
	.PinN = { .PinNo = -1, .Conn = ADC_PIN_CONN_NONE },
	.FifoMemSize = 0,
	.pFifoMem = nullptr,
};

static DacSamd s_Dac;
static AdcSamd s_Adc;

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

	printf("\r\nDAC on PA02 read back by the ADC, reference ");
	PrintVolt(DAC_REF_VOLTAGE);
	printf("\r\n");

	if (s_Dac.Init(s_DacCfg) == false || s_Dac.Reference(s_Ref) == false ||
		s_Dac.OpenChannel(0, true, false) == false)
	{
		printf("DAC init failed\r\n");

		return 1;
	}

	if (s_Adc.Init(s_AdcCfg, nullptr, nullptr) == false || s_Adc.OpenChannel(&s_AdcChan, 1) == false)
	{
		printf("ADC init failed\r\n");

		return 1;
	}

	int step = 0;

	while (1)
	{
		float set = s_Dac.WriteVolt(0, DAC_REF_VOLTAGE * step / (DAC_DEMO_STEPS - 1));
		AdcData_t d;

		printf("DAC ");
		PrintVolt(set);
		if (s_Adc.Read(0, &d))
		{
			printf(", ADC ");
			PrintVolt(d.Data);
			printf("\r\n");
		}
		else
		{
			printf(", ADC read failed\r\n");
		}

		step = step + 1 < DAC_DEMO_STEPS ? step + 1 : 0;
		msDelay(DAC_DEMO_PERIOD_MS);
	}

	return 0;
}
