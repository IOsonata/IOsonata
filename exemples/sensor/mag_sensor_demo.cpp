/**-------------------------------------------------------------------------
@example	mag_sensor_demo.cpp

@brief	Magnetometer on an I2C bus

Reads the magnetometer that board.h selects and prints the field on the 3 axes
and its magnitude in uT, with the temperature of the device, 2.5 times per
second. board.h selects the magnetometer by its name:

	LIS3MDL		ST LIS3MDL, at either I2C address

and gives the I2C bus with I2C_DEVNO and I2C_PINS. With UART_RETARGET and the
UART pins, the Release build prints on the UART at 115200 8N1, the Debug build
uses semihosting.

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
#include <math.h>

#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/i2c.h"
#include "idelay.h"
#include "stddev.h"
#include "sensors/mag_lis3mdl.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Status poll period, a fraction of the 400 ms sample period
#define MAG_DEMO_POLL_MS		50

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

// Address 0 finds the device at either address
static const MagSensorCfg_t s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 2500,
	.Precision = MAGSENSOR_PRECISION_HIGH,
	.Inter = 0,
	.IntPol = DEVINTR_POL_HIGH,
	.bFifoEn = false,
};

#if defined(LIS3MDL)
static MagLis3mdl s_Mag;
static const char s_MagName[] = "LIS3MDL";
#else
#error "board.h selects no magnetometer"
#endif

// Prints a value in hundredths with 2 decimals, without floating point printf
static void PrintCenti(const char *pName, int32_t Val, const char *pUnit)
{
	const char *sign = Val < 0 ? "-" : "";
	uint32_t v = Val < 0 ? 0U - (uint32_t)Val : (uint32_t)Val;

	printf("%s %s%lu.%02lu %s", pName, sign, (unsigned long)(v / 100), (unsigned long)(v % 100), pUnit);
}

// Hundredths, rounded
static int32_t Centi(float Val)
{
	return (int32_t)(Val * 100.0f + (Val < 0 ? -0.5f : 0.5f));
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
	setvbuf(stdout, NULL, _IONBF, 0);
#endif

	printf("\r\nMagnetometer, I2C %d\r\n", I2C_DEVNO);

	if (s_I2c.Init(s_I2cCfg) == false)
	{
		printf("I2C init failed\r\n");

		return 1;
	}

	if (s_Mag.Init(s_MagCfg, &s_I2c) == false)
	{
		printf("%s: not found\r\n", s_MagName);

		return 1;
	}

	printf("%s at 0x%02lX: found, %u gauss full scale\r\n", s_MagName, (unsigned long)s_Mag.DeviceAddress(),
		   (unsigned)s_Mag.Scale());

	s_Mag.Enable();

	while (1)
	{
		if (s_Mag.UpdateData())
		{
			MagSensorData_t d;

			s_Mag.Read(d);

			float b = sqrtf(d.X * d.X + d.Y * d.Y + d.Z * d.Z);

			PrintCenti("X", Centi(d.X), "uT, ");
			PrintCenti("Y", Centi(d.Y), "uT, ");
			PrintCenti("Z", Centi(d.Z), "uT, ");
			PrintCenti("magnitude", Centi(b), "uT, ");
			PrintCenti("temperature", Centi(d.Temp), "C\r\n");
		}
		msDelay(MAG_DEMO_POLL_MS);
	}

	return 0;
}
