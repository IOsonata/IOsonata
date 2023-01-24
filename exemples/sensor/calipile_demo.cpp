/**-------------------------------------------------------------------------
@example	calipile_demo.c

@brief	Excelitas Calipile example


@author	Hoang Nguyen Hoan
@date	Apr. 14, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#include <stdbool.h>

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "sensors/tir_calipile.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "pulse_train.h"

#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOMEMSIZE			CFIFO_MEMSIZE(256)

static uint8_t g_UartTxFifoMem[FIFOMEMSIZE];

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// UART configuration data
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
	.IntPrio = 1,
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = FIFOMEMSIZE,
	.pTxMem = g_UartTxFifoMem,
	.bDMAMode = false,
};

UART g_Uart;

static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,		// Rate in Hz
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},		// Slave addresses
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,			// Interrupt prio
	.EvtCB = NULL		// Event callback
};

I2C g_I2c;

static const CalipileCfg_t s_CalipileCfg = {
	CALIPILE_I2C_DEVADDR,
	true,
};

Calipile g_Calipile;
volatile bool g_bIntFlag = false;

void CalipilePinIntHandler(int IntNo, void *pCtx)
{
	if (IntNo == CALIPILE_INT_NO)
	{
		g_I2c.Init(s_I2cCfg);
		g_Calipile.IntHandler();
		g_bIntFlag = true;
		g_I2c.PowerOff();
	}
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
	// Configure Leds
	IOPinCfg(s_Leds, s_NbLeds);

	// Clear all leds
	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	//g_Uart.Init(s_UartCfg);
	g_I2c.Init(s_I2cCfg);

	msDelay(1);

	IOPinSet(CALIPILE_INT_PORT, CALIPILE_INT_PIN);
	IOPinConfig(CALIPILE_INT_PORT, CALIPILE_INT_PIN, CALIPILE_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(CALIPILE_INT_NO, CALIPILE_INT_PRIO, CALIPILE_INT_PORT, CALIPILE_INT_PIN, CALIPILE_INT_SENSE, CalipilePinIntHandler, NULL);

	g_Calipile.Init(s_CalipileCfg, &g_I2c);
	g_I2c.PowerOff();

	msDelay(1000);


	uint8_t reg = 4;
	uint8_t d[2];

//	g_I2c.Tx(0, &reg, 1);
//	g_I2c.Tx(4, &reg, 1);
//	msDelay(1);

/*	reg = 31;
	d = 0x80;
	g_I2c.Write(CALIPILE_I2C_DEVADDR, &reg, 1, &d,1);

	reg = 63;

	g_I2c.Read(CALIPILE_I2C_DEVADDR, &reg, 1, &d, 1);
	printf("%x\r\n", d);

	int i = 0;
*/
//	reg = CALIPILE_INT_STATUS_REG;
//	g_Calipile.Sensor::Read(&reg,1, &d, 1);

	// Loop until button pressed
	while (1)
	{
		__WFE();
//		reg = CALIPILE_CHIP_STATUS_REG;
//		g_Calipile.Sensor::Read(&reg,1, d, 2);
		//if (d[0])// && d[1])
		{
		//	g_Uart.printf("int:%x, %x\r\n", d[0], d[1]);
		}
		reg = CALIPILE_TP_PRESENCE_REG;
		//g_Calipile.Sensor::Read(&reg,1, d, 2);
		//if (d[0] > 250)
		{
		//	g_Uart.printf("pres:%d, %d\r\n", d[0], d[1]);
		}
		//g_Calipile.UpdateData();
		if (g_bIntFlag)
		{
			g_bIntFlag = false;
			//g_Uart.printf("T = %.2f\r\n", g_Calipile.ReadTemperature());
		}
//		msDelay(1000);
	}

	// 68 58 3c d0b0
	// 0110 1000 0101 1000 0011 1100
	// 0 1101 0000 1011 0000
	return 0;
}


