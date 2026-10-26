/**-------------------------------------------------------------------------
@example main.cpp

@brief Thingy:91 X board power and user-I/O integration test.

The default test restores the documented nPM1300 board routing without
switching the nPM6001 or sensor rails, keeps expansion-board power off,
cycles the active-high RGB LED and reports button and nPM6001 READY state.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "pwrmgnt/pm_npm1300.h"

#include "board.h"
#include "thingy91x_board.h"

#define UART_FIFO_SIZE	CFIFO_MEMSIZE(256)

static uint8_t s_UartTxBuffer[UART_FIFO_SIZE];

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	UART_DEVNO,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	115200,
	8,
	UART_PARITY_NONE,
	1,
	UART_FLWCTRL_NONE,
	true,
	1,
	NULL,
	true,
	0,
	NULL,
	UART_FIFO_SIZE,
	s_UartTxBuffer,
};

static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP,
	 IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(s_I2cPins[0]),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,
	.EvtCB = NULL,
};

UART g_Uart;
I2C g_I2c;
PmnPM1300 g_Pmic;

int main()
{
	unsigned int step;
	bool ready;

	if (!g_Uart.Init(s_UartCfg))
		return 1;

	g_Uart.printf("Thingy:91 X board integration test\r\n");

	if (!g_I2c.Init(s_I2cCfg))
	{
		g_Uart.printf("I2C initialization failed\r\n");
		return 1;
	}

	if (!g_Pmic.Init(&g_I2c) || !Thingy91XPmicConfigure(g_Pmic))
	{
		g_Uart.printf("nPM1300 board routing failed\r\n");
		return 1;
	}

	Thingy91XUserIoInit();
	Thingy91XExpansionPowerInit(false);

#if THINGY91X_TEST_EXPANSION_POWER
	Thingy91XExpansionPower(true);
	g_Uart.printf("Expansion 3.3 V enabled\r\n");
#else
	g_Uart.printf("Expansion 3.3 V disabled\r\n");
#endif

	step = 0;
	while (1)
	{
		switch (step)
		{
		case 0:
			Thingy91XLedSet(true, false, false);
			break;
		case 1:
			Thingy91XLedSet(false, true, false);
			break;
		case 2:
			Thingy91XLedSet(false, false, true);
			break;
		default:
			Thingy91XLedSet(false, false, false);
			break;
		}

		if (!Thingy91XNpm6001Ready(g_Pmic, ready))
		{
			g_Uart.printf("nPM6001 READY read failed\r\n");
			return 1;
		}

		g_Uart.printf("rgb_step=%u button=%u npm6001_ready=%u\r\n",
			step, Thingy91XButtonPressed() ? 1U : 0U, ready ? 1U : 0U);

		step = (step + 1U) & 3U;
		msDelay(500);
	}

	return 0;
}
