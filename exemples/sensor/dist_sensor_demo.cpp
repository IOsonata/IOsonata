/**-------------------------------------------------------------------------
@example	dist_sensor_demo.cpp

@brief	Distance sensor on an I2C bus

Reads the distance sensor that board.h selects and prints the distance, the
return signal and the ambient light rates, 5 times per second. board.h selects
the sensor by its name:

	VL53L0X		ST VL53L0X time of flight, at 0x29

and gives the I2C bus with I2C_DEVNO and I2C_PINS. When the shutdown pin of
the sensor is on the MCU, DIST_XSHUT_PORT, DIST_XSHUT_PIN and DIST_XSHUT_PINOP
give it and the demo takes the sensor out of shutdown. With UART_RETARGET and
the UART pins, the Release build prints on the UART at 1000000 baud 8N1,
the Debug build uses semihosting.

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
#include "coredev/i2c.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "stddev.h"
#include "sensors/dist_vl53l0x.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Status poll period, a fraction of the 200 ms sample period
#define DIST_DEMO_POLL_MS		20

// Boot time of the sensor after its shutdown pin is released
#define DIST_DEMO_BOOT_MS		2

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

static UART s_Uart;
#endif

static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCB = NULL,
};

static I2C s_I2c;

// Device default timing budget
static const DistSensorCfg_t s_DistCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 5000,
	.TimingBudget = 0,
	.IntPrio = 0,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.DataRdyCB = NULL,
};

#if defined(VL53L0X)
static DistVl53l0x s_Dist;
static const char s_DistName[] = "VL53L0X";
#else
#error "board.h selects no distance sensor"
#endif

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

	printf("\r\nDistance sensor, I2C %d\r\n", I2C_DEVNO);

#ifdef DIST_XSHUT_PORT
	// Out of shutdown
	IOPinConfig(DIST_XSHUT_PORT, DIST_XSHUT_PIN, DIST_XSHUT_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(DIST_XSHUT_PORT, DIST_XSHUT_PIN);
	msDelay(DIST_DEMO_BOOT_MS);
#endif

	if (s_I2c.Init(s_I2cCfg) == false)
	{
		printf("I2C init failed\r\n");

		return 1;
	}

	if (s_Dist.Init(s_DistCfg, &s_I2c) == false)
	{
		printf("%s: not found\r\n", s_DistName);

		return 1;
	}

	printf("%s at 0x%02lX: found, %lu us per measurement\r\n", s_DistName, (unsigned long)s_Dist.DeviceAddress(),
		   (unsigned long)s_Dist.TimingBudget());

	s_Dist.Enable();

	while (1)
	{
		if (s_Dist.UpdateData())
		{
			DistSensorData_t d;

			s_Dist.Read(d);

			if (d.Status == DISTSENSOR_STATUS_VALID)
			{
				printf("Distance %lu mm, ", (unsigned long)d.Distance);
			}
			else
			{
				printf("No distance, status %u, ", (unsigned)d.Status);
			}
			printf("signal %lu kcps, ambient %lu kcps\r\n", (unsigned long)d.SignalRate, (unsigned long)d.AmbientRate);
		}
		msDelay(DIST_DEMO_POLL_MS);
	}

	return 0;
}
