/**-------------------------------------------------------------------------
@example	ADCDemo.cpp

@brief	ADC example for nRF52


@author	Hoang Nguyen Hoan
@date	June 16, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#include "adc_nrf52_saadc.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "iopinctrl.h"
#include "idelay.h"

// This include contain i/o definition the board in use
#include "board.h"

//#define ADC_DEMO_INTERRUPT_ENABLE

#define AIN0_PORT	0
#define AIN0_PIN	2

#define AIN1_PORT	0
#define AIN1_PIN	3

#define AIN2_PORT	0
#define AIN2_PIN	4

#define AIN3_PORT	0
#define AIN3_PIN	5

#define AIN4_PORT	0
#define AIN4_PIN	28

#define AIN5_PORT	0
#define AIN5_PIN	29

#define AIN6_PORT	0
#define AIN6_PIN	30

#define AIN7_PORT	0
#define AIN7_PIN	31


int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void ADVEventHandler(Device *pDevObj, DEV_EVT Evt);

typedef enum __ADC_Pins {
	AIN0,
	AIN1,
	AIN2,
	AIN3,
	AIN4,
	AIN5,
	AIN6,
	AIN7,
	AVdd,
}ADC_PINS;

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_TxBuff[FIFOSIZE];

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	115200,			// Baud Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

// Define available voltage sources
static const AdcRefVolt_t s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6 },
	{.Type = ADC_REFVOLT_TYPE_SUPPLY, .Voltage = 3.3 / 4.0},
};

static const int s_NbRefVolt = sizeof(s_RefVolt) / sizeof(AdcRefVolt_t);

#define ADC_CFIFO_SIZE		CFIFO_TOTAL_MEMSIZE(200, sizeof(AdcData_t))

static uint8_t s_AdcFifoMem[ADC_CFIFO_SIZE];

// Define ADC device
static const AdcCfg_t s_AdcCfg = {
#ifdef ADC_DEMO_SINGLE_SHOT
	.Mode = ADC_CONV_MODE_SINGLE,
#else
	.Mode = ADC_CONV_MODE_CONTINUOUS,
#endif
	.pRefVolt = s_RefVolt,
	.NbRefVolt = s_NbRefVolt,
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 200000,
	.OvrSample = 0,
#ifdef ADC_DEMO_INTERRUPT_ENABLE
	.bInterrupt = true,
#else
	.bInterrupt = false,
#endif
	.IntPrio = 6,
	.EvtHandler = ADVEventHandler
};

AdcnRF52 g_Adc;

// Define ADC channel
static const AdcChanCfg_t s_ChanCfg[] = {
	{
		.Chan = 0,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_DIFFERENTIAL,
		.Gain = 6,//1 << 8,
		.AcqTime = 10,
		.BurstMode = false,
		.PinP = { .PinNo = AVdd, .Conn = ADC_PIN_CONN_NONE },
		.PinN = { .PinNo = AIN5, .Conn = ADC_PIN_CONN_PULLDOWN },
//		.FifoMemSize = ADC_CFIFO_SIZE,
//		.pFifoMem = s_AdcFifoMem,
	},
	{
		.Chan = 1,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,//1 << 8,
		.AcqTime = 3,
		.BurstMode = false,
		.PinP = { .PinNo = AIN0, .Conn = ADC_PIN_CONN_NONE },
		.FifoMemSize = ADC_CFIFO_SIZE,
		.pFifoMem = s_AdcFifoMem,
	},
	{
		.Chan = 2,
		.RefVoltIdx =0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,//1 << 8,
		.AcqTime = 3,
		.BurstMode = false,
		.PinP = { .PinNo = AIN1, .Conn = ADC_PIN_CONN_NONE },
//		.FifoMemSize = ADC_CFIFO_SIZE,
//		.pFifoMem = s_AdcFifoMem,
	},
	{
		.Chan = 3,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 6,//1 << 8,
		.AcqTime = 0,
		.BurstMode = false,
		.PinP = { .PinNo = AVdd, .Conn = ADC_PIN_CONN_NONE },
	}
};

static const int s_NbChan = sizeof(s_ChanCfg) / sizeof(AdcChanCfg_t);

volatile bool g_bDataReady = false;

void ADVEventHandler(Device *pAdcDev, DEV_EVT Evt)
{
	if (Evt == DEV_EVT_DATA_RDY)
	{
		g_bDataReady = true;
#ifdef ADC_DEMO_INTERRUPT_ENABLE
		int cnt = 0;

		ADC_DATA df[s_NbChan];
		cnt = g_Adc.Read(df, s_NbChan);
		if (cnt > 0)
		{
			g_Uart.printf("%d ADC[0] = %.2fV, ADC[1] = %.2fV, ADC[2] = %.2fV, ADC[3] = %.2fV\r\n",
					df[0].Timestamp, df[0].Data, df[1].Data, df[2].Data, df[3].Data);
		}

		if (g_Adc.Mode() == ADC_CONV_MODE_SINGLE)
		{
			g_Adc.StartConversion();
		}
#endif
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
//	UARTRetargetEnable(g_Uart, STDIN_FILENO);
//	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	g_Uart.printf("Init ADC\r\n");
	IOPinDisable(AIN0_PORT, AIN0_PIN);//AIN0
	IOPinDisable(AIN1_PORT, AIN1_PIN);//AIN1

	//IOPinSet(0, 2);
	//IOPinConfig(0, 2, 0, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	//IOPinClear(0, 3);
	g_Adc.Init(s_AdcCfg);
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
}
//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	HardwareInit();
	g_Adc.StartConversion();

	while (1)
	{
		__WFE();
#ifdef ADC_DEMO_INTERRUPT_ENABLE
		if (g_bDataReady == true)
		{
			g_bDataReady = false;
		}
#else
		int cnt = 0;

		AdcData_t df[s_NbChan];
		memset(df, 0, sizeof(df));
		cnt = g_Adc.Read(df, s_NbChan);
		if (cnt > 0)
		{
			for (int i=0; i<cnt; i++)
			{
				g_Uart.printf("#%d ADC[%d] = %.2fV | ", df[i].Timestamp, i, df[i].Data);
			}
			g_Uart.printf("\r\n");
//			g_Uart.printf("#%d | ADC[0] = %.2fV, ADC[1] = %.2fV, ADC[2] = %.2fV, ADC[3] = %.2fV\r\n",
//					df[0].Timestamp, df[0].Data, df[1].Data, df[2].Data, df[3].Data);
		}
		g_Adc.StartConversion();
		msDelay(1000);
#endif
	}

	return 0;
}
