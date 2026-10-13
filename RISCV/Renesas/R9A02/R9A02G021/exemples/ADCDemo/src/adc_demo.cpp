/**-------------------------------------------------------------------------
@file	adc_demo.cpp

@brief	ADC12 demo on the FPB-R9A02G021

Converts two analog inputs, the internal reference and the temperature sensor
once per second and prints the results on the UART:

	IN0, IN1	: voltage of ADC_DEMO_IN0 and ADC_DEMO_IN1, VCC as the reference
	VREF		: internal reference, gives an estimate of VCC
	TEMP		: die temperature from the factory calibration point (32.3.1)

AdcR9a02::Calibrate is not used: the factory point CAL140 of the temperature
sensor is a result without zero and full scale correction, the measured value
must be the same. Define ADC_DEMO_INT_ENABLE true in board.h for the
interrupt mode, the conversions then run in the background and the event
handler marks the end.

The values are printed as integers, newlib-nano printf has no float format.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include "adc_r9a02.h"
#include "board.h"

#ifndef ADC_DEMO_INT_ENABLE
#define ADC_DEMO_INT_ENABLE		false
#endif

#define ADC_DEMO_VCC			3.3f		// Reference voltage, VCC of the board
#define ADC_DEMO_VREF_TYP		1.47f		// Internal reference, table 37.49
#define ADC_DEMO_TSN_SLOPE		-0.0033f	// Temperature sensor slope in V/C, table 37.52
#define ADC_DEMO_TSN_CAL_T		140.0f		// TSCDR temperature, 32.3.1
#define ADC_DEMO_TSN_CAL_VCC	3.3f		// TSCDR supply voltage, 32.3.1

typedef enum __Adc_Demo_Chan {
	ADC_DEMO_CHAN_IN0,
	ADC_DEMO_CHAN_IN1,
	ADC_DEMO_CHAN_VREF,
	ADC_DEMO_CHAN_TEMP,
	ADC_DEMO_CHAN_CNT
} ADC_DEMO_CHAN;

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

// VCC as the reference, one entry, Init fails with more
static const AdcRefVolt_t s_AdcRefVolt[] = {
	{ .Type = ADC_REFVOLT_TYPE_SUPPLY, .Voltage = ADC_DEMO_VCC, .Pin = -1 },
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_AdcRefVolt,
	.NbRefVolt = sizeof(s_AdcRefVolt) / sizeof(AdcRefVolt_t),
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = ADC_DEMO_INT_ENABLE,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = AdcEvtHandler,
};

// AcqTime 1 us on the pins, the internal inputs get the 5 us they need
static const AdcChanCfg_t s_AdcChanCfg[] = {
	{
		.Chan = ADC_DEMO_CHAN_IN0, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
		.AcqTime = 1, .BurstMode = false, .PinP = { ADC_DEMO_IN0, ADC_PIN_CONN_NONE },
		.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
	},
	{
		.Chan = ADC_DEMO_CHAN_IN1, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
		.AcqTime = 1, .BurstMode = false, .PinP = { ADC_DEMO_IN1, ADC_PIN_CONN_NONE },
		.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
	},
	{
		.Chan = ADC_DEMO_CHAN_VREF, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
		.AcqTime = 5, .BurstMode = false, .PinP = { R9A02_ADC_INPUT_VREF, ADC_PIN_CONN_NONE },
		.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
	},
	{
		.Chan = ADC_DEMO_CHAN_TEMP, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
		.AcqTime = 5, .BurstMode = false, .PinP = { R9A02_ADC_INPUT_TEMP, ADC_PIN_CONN_NONE },
		.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
	},
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

static int ToMilli(float Val)
{
	return (int)(Val * 1000.0f + (Val < 0.0f ? -0.5f : 0.5f));
}

// T = (Vs - V1) / slope + 140, V1 = 3.3 x CAL140 / 4096 (32.3.1). Vs is
// measured with VCC as the reference, ADC_DEMO_VCC is taken as 3.3 V.
static float TempDegC(float Vs)
{
	float v1 = ADC_DEMO_TSN_CAL_VCC * (float)(R9A02_FLCN_TSCDR & R9A02_TSCDR_CAL140_Msk) / 4096.0f;

	return (Vs - v1) / ADC_DEMO_TSN_SLOPE + ADC_DEMO_TSN_CAL_T;
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

	printf("\r\nADC12 demo, %s mode\r\n", ADC_DEMO_INT_ENABLE ? "interrupt" : "polling");

	if (s_Adc.Init(s_AdcCfg) == false)
	{
		printf("ADC init failed\r\n");
		while (1) __WFE();
	}

	if (s_Adc.OpenChannel(s_AdcChanCfg, sizeof(s_AdcChanCfg) / sizeof(AdcChanCfg_t)) == false)
	{
		printf("ADC channel config failed\r\n");
		while (1) __WFE();
	}

	printf("TSCDR CAL140 %u\r\n", (unsigned)(R9A02_FLCN_TSCDR & R9A02_TSCDR_CAL140_Msk));

	while (1)
	{
		AdcData_t data[ADC_DEMO_CHAN_CNT];
		int mv[ADC_DEMO_CHAN_CNT] = { 0, };

		s_bAdcDone = false;

		if (s_Adc.StartConversion() == false)
		{
			printf("Conversion failed\r\n");
		}
		else
		{
			// A few us per channel
			while (ADC_DEMO_INT_ENABLE && s_bAdcDone == false)
			{
			}

			int cnt = s_Adc.Read(data, ADC_DEMO_CHAN_CNT);
			float vref = 0.0f;
			float temp = 0.0f;

			for (int i = 0; i < cnt; i++)
			{
				if (data[i].Chan >= 0 && data[i].Chan < ADC_DEMO_CHAN_CNT)
				{
					mv[data[i].Chan] = ToMilli(data[i].Data);
				}
				if (data[i].Chan == ADC_DEMO_CHAN_VREF)
				{
					vref = data[i].Data;
				}
				else if (data[i].Chan == ADC_DEMO_CHAN_TEMP)
				{
					temp = TempDegC(data[i].Data);
				}
			}

			// VCC = VREF typical x VCC assumed / VREF measured
			int vcc = vref > 0.0f ? ToMilli(ADC_DEMO_VREF_TYP * ADC_DEMO_VCC / vref) : 0;
			int t10 = (int)(temp * 10.0f + (temp < 0.0f ? -0.5f : 0.5f));
			int t10abs = t10 < 0 ? -t10 : t10;

			printf("IN0 %d mV, IN1 %d mV, VREF %d mV (VCC about %d mV), TEMP %d mV %s%d.%d C\r\n",
				   mv[ADC_DEMO_CHAN_IN0], mv[ADC_DEMO_CHAN_IN1], mv[ADC_DEMO_CHAN_VREF], vcc,
				   mv[ADC_DEMO_CHAN_TEMP], t10 < 0 ? "-" : "", t10abs / 10, t10abs % 10);
		}

		IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
		msDelay(1000);
	}

	return 0;
}
