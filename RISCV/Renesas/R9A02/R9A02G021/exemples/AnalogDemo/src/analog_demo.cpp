/**-------------------------------------------------------------------------
@example	analog_demo.cpp

@brief	DAC8, comparator and TML32 demo on the FPB-R9A02G021

TML32 steps DAC8 channel 1 on P001, connected to CMP0 input P106.
The FPB external pull-up R23 shifts the DAC1 voltage range upward; its
minimum is approximately VCC/2, so CMP0's 1.44 V internal reference cannot
be crossed. DAC0 on P000 drives CMP0's external IVREF0 reference instead.
The DAC0 reference code is selected to put its voltage inside the measured
DAC1 range; comparator events indicate each crossing. LED2 indicates state.

Keep P001 to P106 wired; P000 uses its on-chip DAC0/IVREF0 connection.
Do not treat the raw DAC1 code as the physical output voltage on this board.

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
#include "coredev/interrupt.h"
#include "converters/analog_comp.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "r9a02g021.h"
#include "dac_r9a02.h"
#include "tml32_r9a02.h"
#include "board.h"

static void Tml32Handler(void *pCtx);
static void CompHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt);

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

static const R9a02Tml32Cfg_t s_Tml32Cfg = {
	.ClkSrc = R9A02_TML32_CLK_LF,
	.Mode = R9A02_TML32_MODE_INT,
	.nsPeriod = ANALOG_DEMO_PERIOD_MS * 1000000ULL,
	.Handler = Tml32Handler,
	.pCtx = NULL,
	.IntPrio = IRQ_PRIO_LOW,
	.bWakeup = false,
};

// Internal reference, CompVolt is not used with it
static const AnalogCompCfg_t s_CompCfg = {
	.DevNo = ANALOG_DEMO_COMP_DEVNO,
	.Mode = ANALOG_COMP_MODE_CONTINUOUS,
	.RefSrc = 1,
	.RefVolt = ANALOG_DEMO_VCC_MV,
	.CompVolt = ANALOG_DEMO_REF_CODE * ANALOG_DEMO_VCC_MV / 256,
	.AnalogIn = 0,
	.bHystersys = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = CompHandler,
};

static const DAC_CFG s_DacCfg = {
	.Mode = DAC_MODE_SINGLE_ENDED,
};

static UART s_Uart;
static DacR9a02 s_Dac;
static AnalogComp s_Comp;
static volatile uint8_t s_DacCode = 0;
static volatile uint32_t s_Tml32Ticks = 0;
static volatile uint32_t s_CompEvents = 0;
static volatile int s_CompEvt = 0;			// 1 above, -1 below, 0 none
static volatile uint8_t s_CompCode = 0;

static void Tml32Handler(void *pCtx)
{
	uint8_t code = (uint8_t)(s_DacCode + ANALOG_DEMO_STEP);

	s_DacCode = code;
	s_Tml32Ticks++;
	s_Dac.WriteCode(ANALOG_DEMO_DAC_CHAN, code);
}

static void CompHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt)
{
	s_CompCode = s_DacCode;
	s_CompEvents++;

	if (Evt == ANALOG_COMP_EVT_HIGHER)
	{
		s_CompEvt = 1;
		IOPinSet(s_Leds[1].PortNo, s_Leds[1].PinNo);
	}
	else
	{
		s_CompEvt = -1;
		IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);
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
	uintptr_t state;
	int evt;
	uint8_t code;
	uint8_t dacCode;
	uint32_t ticks;
	uint32_t compEvents;

	IOPinCfg(s_Leds, s_NbLeds);

	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1)
		{
			__WFE();
		}
	}
#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
	setvbuf(stdout, NULL, _IONBF, 0);

	printf("\r\nDAC8, comparator and TML32 demo, wire P001 to P106\r\n");
	printf("CMP0 IVREF0 driven by DAC0 P000, code %d\r\n", ANALOG_DEMO_REF_CODE);

	if (s_Dac.Init(s_DacCfg) == false || s_Dac.OpenChannel(ANALOG_DEMO_DAC_CHAN, true) == false ||
		s_Dac.OpenChannel(0, true) == false || s_Dac.WriteCode(0, ANALOG_DEMO_REF_CODE) == false)
	{
		Fail("DAC init failed");
	}
	s_Dac.Vcc((float)ANALOG_DEMO_VCC_MV / 1000.0f);

	if (s_Comp.Init(s_CompCfg) == false || s_Comp.Start() == false)
	{
		Fail("Comparator init failed");
	}

	if (R9a02Tml32Init(&s_Tml32Cfg) == false)
	{
		Fail("TML32 init failed");
	}

	printf("TML32 period %lu us, DAC step %d\r\n", (unsigned long)(R9a02Tml32Period() / 1000U),
		   ANALOG_DEMO_STEP);

	while (1)
	{
		msDelay(1000);

		state = DisableInterrupt();
		evt = s_CompEvt;
		code = s_CompCode;
		dacCode = s_DacCode;
		ticks = s_Tml32Ticks;
		compEvents = s_CompEvents;

		s_CompEvt = 0;
		EnableInterrupt(state);

		printf("TML32 ticks %lu, DAC code %u, comparator events %lu\r\n",
			   (unsigned long)ticks, (unsigned)dacCode, (unsigned long)compEvents);

		if (evt != 0)
		{
			printf("%s at DAC1 code %u\r\n", evt > 0 ? "Above" : "Below", (unsigned)code);
			IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
		}
	}

	return 0;
}
