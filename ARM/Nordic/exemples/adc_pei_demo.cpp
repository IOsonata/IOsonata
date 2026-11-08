/**-------------------------------------------------------------------------
@example	adc_pei_demo.cpp

@brief	SAADC rounds started by a timer through PEI, with limits

The compare event of a Timer starts each SAADC round through PEI (PPI on the
nRF52), the CPU takes no part in the sampling. Two channels:
	- channel 0, the analog input ADC_PEI_DEMO_AIN of board.h with the SAADC
	  pull-up: open it reads the supply, pulled to ground (a button to ground)
	  it reads 0 V
	- channel 1, the supply VDD

The results come in blocks of ADC_PEI_DEMO_BLOCK rounds, double buffered. At
each block the demo logs the average, minimum and maximum of each channel in
millivolts with SysLog on the console UART.

Channel 0 has limits, ADC_PEI_DEMO_LOW_MV and ADC_PEI_DEMO_HIGH_MV. Each result
at or below the low one or at or above the high one is a limit event; the
counts are logged with each block.

The board.h of the project gives the console UART pins, the analog input and
the Timer.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#include "coredev/timer.h"
#include "coredev/pei.h"
#include "syslog.h"
#include "adc_nrf52_saadc.h"

#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE				1000000
#endif

// Round period in msec, from the Timer compare
#ifndef ADC_PEI_DEMO_PERIOD_MS
#define ADC_PEI_DEMO_PERIOD_MS		10
#endif

// Rounds per block, one log line per block
#ifndef ADC_PEI_DEMO_BLOCK
#define ADC_PEI_DEMO_BLOCK			50
#endif

// Limits of channel 0 in mV
#ifndef ADC_PEI_DEMO_LOW_MV
#define ADC_PEI_DEMO_LOW_MV			1000
#endif
#ifndef ADC_PEI_DEMO_HIGH_MV
#define ADC_PEI_DEMO_HIGH_MV		2000
#endif

#define ADC_PEI_DEMO_NBCHAN			2
#define ADC_PEI_DEMO_VDD			8		// PinNo of VDD, PSELP 9

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

static void AdcPeiDemoEvt(Device * const pDev, DEV_EVT Evt);
static void AdcPeiDemoLimit(AdcnRF52 * const pAdc, int Chan, bool bHigh);

// Internal 0.6 V reference, gain 1/6 below: 3.6 V full scale
static const AdcRefVolt_t s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6, .Pin = 0},
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_CONTINUOUS,
	.pRefVolt = s_RefVolt,
	.NbRefVolt = sizeof(s_RefVolt) / sizeof(AdcRefVolt_t),
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,					// Rounds from the Timer, not the internal timer
	.OvrSample = 0,
	.bInterrupt = true,
	.IntPrio = 6,
	.EvtHandler = AdcPeiDemoEvt,
};

// Each channel FIFO holds two blocks
#define ADC_PEI_DEMO_FIFO_SIZE		CFIFO_TOTAL_MEMSIZE(2 * ADC_PEI_DEMO_BLOCK, sizeof(AdcData_t))

alignas(4) static uint8_t s_AdcFifo[ADC_PEI_DEMO_NBCHAN][ADC_PEI_DEMO_FIFO_SIZE];

// DMA memory of the two blocks
alignas(4) static int16_t s_AdcBlk[2 * ADC_PEI_DEMO_BLOCK * ADC_PEI_DEMO_NBCHAN];

static const AdcChanCfg_t s_AdcChan[ADC_PEI_DEMO_NBCHAN] = {
	{
		.Chan = 0,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,
		.AcqTime = 10,
		.BurstMode = false,
		.PinP = { .PinNo = ADC_PEI_DEMO_AIN, .Conn = ADC_PIN_CONN_PULLUP },
		.PinN = { .PinNo = 0, .Conn = ADC_PIN_CONN_NONE },
		.FifoMemSize = ADC_PEI_DEMO_FIFO_SIZE,
		.pFifoMem = s_AdcFifo[0],
	},
	{
		.Chan = 1,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,
		.AcqTime = 10,
		.BurstMode = false,
		.PinP = { .PinNo = ADC_PEI_DEMO_VDD, .Conn = ADC_PIN_CONN_NONE },
		.PinN = { .PinNo = 0, .Conn = ADC_PIN_CONN_NONE },
		.FifoMemSize = ADC_PEI_DEMO_FIFO_SIZE,
		.pFifoMem = s_AdcFifo[1],
	},
};

static AdcnRF52 s_Adc;
static Timer s_Timer;
static Pei s_Pei;

// Interrupt context: flag and counts only, main logs
static volatile bool s_bAdcPeiDemoBlock = false;
static volatile uint32_t s_LimitLow = 0;
static volatile uint32_t s_LimitHigh = 0;

static void AdcPeiDemoEvt(Device * const pDev, DEV_EVT Evt)
{
	(void)pDev;

	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_bAdcPeiDemoBlock = true;
	}
}

static void AdcPeiDemoLimit(AdcnRF52 * const pAdc, int Chan, bool bHigh)
{
	(void)pAdc;
	(void)Chan;

	if (bHigh)
	{
		s_LimitHigh = s_LimitHigh + 1;
	}
	else
	{
		s_LimitLow = s_LimitLow + 1;
	}
}

static void AdcPeiDemoFail(const char *pMsg)
{
	SysLogPrintf(SysLogGet(), "%s failed\r\n", pMsg);

	while (1)
	{
		__WFE();
	}
}

int main()
{
	TimerCfg_t timerCfg = {};
	PeiEvent_t evt = 0U;

	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "AdcPeiDemo\r\n");

	if (s_Adc.Init(s_AdcCfg) == false)
	{
		AdcPeiDemoFail("ADC init");
	}
	if (s_Adc.OpenChannel(s_AdcChan, ADC_PEI_DEMO_NBCHAN) == false)
	{
		AdcPeiDemoFail("ADC channels");
	}
	s_Adc.Calibrate();
	s_Adc.BlockRounds(ADC_PEI_DEMO_BLOCK, s_AdcBlk, sizeof(s_AdcBlk));
	s_Adc.LimitHandler(AdcPeiDemoLimit);
	if (s_Adc.Limit(0, ADC_PEI_DEMO_LOW_MV / 1000.0f, ADC_PEI_DEMO_HIGH_MV / 1000.0f) == false)
	{
		AdcPeiDemoFail("ADC limits");
	}

	// The SAMPLE task can be connected once the conversion is started
	if (s_Adc.StartConversion() == false)
	{
		AdcPeiDemoFail("ADC start");
	}

	timerCfg.DevNo = ADC_PEI_DEMO_TIMER_DEVNO;
	timerCfg.Freq = 0;
	timerCfg.IntPrio = IRQ_PRIO_NORMAL;
	if (s_Timer.Init(timerCfg) == false || s_Timer.Enable() == false)
	{
		AdcPeiDemoFail("Timer init");
	}

	if (s_Timer.EnableTimerTrigger(s_Pei, evt, ADC_PEI_DEMO_PERIOD_MS, TIMER_TRIG_TYPE_CONTINUOUS) < 0)
	{
		AdcPeiDemoFail("Timer trigger");
	}
	if (s_Adc.ConnectTask(s_Pei, evt, ADC_NRF52_PEI_TASK_SAMPLE) == false)
	{
		AdcPeiDemoFail("PEI route");
	}
	s_Pei.Enable(evt);

	SysLogPrintf(SysLogGet(), "Round every %u ms, %d rounds per block, limits %d to %d mV\r\n",
				 (unsigned)ADC_PEI_DEMO_PERIOD_MS, s_Adc.BlockRounds(),
				 ADC_PEI_DEMO_LOW_MV, ADC_PEI_DEMO_HIGH_MV);

	while (1)
	{
		__WFE();

		if (s_bAdcPeiDemoBlock == false)
		{
			continue;
		}
		s_bAdcPeiDemoBlock = false;

		int32_t avg[ADC_PEI_DEMO_NBCHAN];
		int32_t min[ADC_PEI_DEMO_NBCHAN];
		int32_t max[ADC_PEI_DEMO_NBCHAN];
		uint32_t ts = 0;

		for (int c = 0; c < ADC_PEI_DEMO_NBCHAN; c++)
		{
			AdcData_t d;
			int32_t sum = 0;
			int n = 0;

			min[c] = INT32_MAX;
			max[c] = INT32_MIN;

			while (s_Adc.Read(c, &d))
			{
				int32_t mv = (int32_t)(d.Data * 1000.0f + 0.5f);

				sum += mv;
				if (mv < min[c])
					min[c] = mv;
				if (mv > max[c])
					max[c] = mv;
				ts = d.Timestamp;
				n++;
			}
			avg[c] = n > 0 ? sum / n : 0;
		}

		SysLogPrintf(SysLogGet(), "#%u AIN %d mV (%d..%d), VDD %d mV, limit low %u high %u\r\n",
					 (unsigned)ts, (int)avg[0], (int)min[0], (int)max[0], (int)avg[1],
					 (unsigned)s_LimitLow, (unsigned)s_LimitHigh);
	}

	return 0;
}
