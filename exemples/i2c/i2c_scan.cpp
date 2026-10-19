/**-------------------------------------------------------------------------
@example	i2c_scan.cpp

@brief	I2C master bus scan.

Probes every 7 bit address from 0x08 to 0x77 with a START and the address
(or a one-byte read when I2C_SCAN_READ_PROBE is selected),
and prints the addresses acknowledged. Then, when I2C_SCAN_REG_DEVADDR is
defined, reads the register I2C_SCAN_REG of that device, for example an ID
register, and prints its value.

Required in board.h : the UART pins for the output, I2C_DEVNO and I2C_PINS,
the SDA then SCL pin configuration. Optional: I2C_SCAN_RATE, and
I2C_SCAN_INT_ENABLE true for a driver that moves the data in the background
(bIntEn), the register read then ends with DEVINTRF_EVT_COMPLETED.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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

#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/i2c.h"
#include "stddev.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef I2C_SCAN_RATE
#define I2C_SCAN_RATE		100000
#endif

#ifndef I2C_SCAN_INT_ENABLE
#define I2C_SCAN_INT_ENABLE	false
#endif

#ifndef I2C_SCAN_READ_PROBE
#define I2C_SCAN_READ_PROBE false
#endif

#define I2C_SCAN_ADDR_FIRST	0x08
#define I2C_SCAN_ADDR_LAST	0x77
#define I2C_SCAN_WAIT_LOOPS	10000000U

static int I2cEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuffer, int Len);

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
#endif

static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = I2C_SCAN_RATE,
	.MaxRetry = 1,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = false,
	.bIntEn = I2C_SCAN_INT_ENABLE,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCB = I2cEvent,
};

static UART s_Uart;
static I2C s_I2c;
static volatile int s_I2cCount = -1;

static int I2cEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuffer, int Len)
{
	if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_I2cCount = Len;
	}

	return 0;
}

#if defined(I2C_SCAN_REG_DEVADDR) || I2C_SCAN_READ_PROBE
// A negative count is a transfer running in the background, its end comes
// with DEVINTRF_EVT_COMPLETED
static int I2cWait(int Count)
{
	uint32_t n = I2C_SCAN_WAIT_LOOPS;

	if (Count >= 0)
	{
		return Count;
	}

	while (s_I2cCount < 0 && --n > 0U)
	{
	}

	if (s_I2cCount < 0)
	{
		s_I2c.Reset();

		return 0;
	}

	return s_I2cCount;
}
#endif

int main()
{
	int found;
	int addr;
#ifdef I2C_SCAN_REG_DEVADDR
	uint8_t reg = I2C_SCAN_REG;
	uint8_t val = 0;
#endif

#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
	setvbuf(stdout, NULL, _IONBF, 0);
#endif

	printf("\r\nI2C scan, I2C %d%s\r\n", I2C_DEVNO, I2C_SCAN_INT_ENABLE ? ", interrupt" : "");

	if (s_I2c.Init(s_I2cCfg) == false)
	{
		printf("I2C init failed\r\n");

		return 1;
	}

	printf("Rate %lu Hz\r\n", (unsigned long)s_I2c.Rate());

	found = 0;

	for (addr = I2C_SCAN_ADDR_FIRST; addr <= I2C_SCAN_ADDR_LAST; addr++)
	{
#if I2C_SCAN_READ_PROBE
		uint8_t probe;
		s_I2cCount = -1;
		bool present = I2cWait(s_I2c.Rx(addr, &probe, 1)) == 1;
#else
		bool present = s_I2c.StartTx(addr);
		if (present) { s_I2c.StopTx(); }
#endif
		if (present)
		{
			printf("Device at 0x%02X\r\n", addr);
			found++;
		}
	}

	printf("%d devices found\r\n", found);

#ifdef I2C_SCAN_REG_DEVADDR
	s_I2cCount = -1;
	if (I2cWait(s_I2c.Read(I2C_SCAN_REG_DEVADDR, &reg, 1, &val, 1)) == 1)
	{
		printf("Device 0x%02X register 0x%02X = 0x%02X\r\n", I2C_SCAN_REG_DEVADDR, reg, val);
	}
	else
	{
		printf("Device 0x%02X register 0x%02X read failed\r\n", I2C_SCAN_REG_DEVADDR, reg);
	}
#endif

	while (1)
	{
		__WFE();
	}

	return 0;
}

