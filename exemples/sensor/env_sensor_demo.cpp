/**-------------------------------------------------------------------------
@example	env_sensor_demo.cpp

@brief	Environmental sensors on one I2C bus

Reads the environmental sensors that board.h selects and prints their values
as they come, once per second. board.h selects a sensor by its name:

	HTS221		ST HTS221 humidity and temperature
	LPS22HB		ST LPS22HB pressure and temperature, at either I2C address

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

#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/i2c.h"
#include "idelay.h"
#include "stddev.h"
#include "sensors/humi_hts221.h"
#include "sensors/press_lps22hb.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Status poll period, a fraction of the 1 Hz sample period
#define ENV_DEMO_POLL_MS		100

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

#ifdef HTS221
static const HumiSensorCfg_t s_Hts221HumiCfg = {
	.DevAddr = HTS221_I2C_DEV_ADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 1000,
	.HumOvrs = 0,
	.FilterCoeff = 0,
	.EvtHandler = NULL,
};

static const TempSensorCfg_t s_Hts221TempCfg = {
	.DevAddr = HTS221_I2C_DEV_ADDR,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 1000,
	.IntPrio = 0,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.TempOvrs = 0,
	.FilterCoeff = 0,
	.DataRdyCB = NULL,
};

static HumiHts221 s_Hts221;
static bool s_bHts221 = false;
#endif

#ifdef LPS22HB
static const PressureSensorCfg_t s_Lps22hbPressCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 1000,
	.PresOvrs = 0,
	.FilterCoeff = 0,
	.EvtHandler = NULL,
};

static const TempSensorCfg_t s_Lps22hbTempCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 1000,
	.IntPrio = 0,
	.Inter = 0,
	.IntPol = DEVINTR_POL_LOW,
	.TempOvrs = 0,
	.FilterCoeff = 0,
	.DataRdyCB = NULL,
};

static PressLps22hb s_Lps22hb;
static bool s_bLps22hb = false;
#endif

// Prints a value in hundredths with 2 decimals, without floating point
static void PrintCenti(const char *pName, int32_t Val, const char *pUnit)
{
	const char *sign = Val < 0 ? "-" : "";
	uint32_t v = Val < 0 ? 0U - (uint32_t)Val : (uint32_t)Val;

	printf("%s %s%lu.%02lu %s", pName, sign, (unsigned long)(v / 100), (unsigned long)(v % 100), pUnit);
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

	printf("\r\nEnvironmental sensors, I2C %d\r\n", I2C_DEVNO);

	if (s_I2c.Init(s_I2cCfg) == false)
	{
		printf("I2C init failed\r\n");

		return 1;
	}

#ifdef HTS221
	s_bHts221 = s_Hts221.Init(s_Hts221HumiCfg, &s_I2c) && s_Hts221.Init(s_Hts221TempCfg, &s_I2c);
	printf("HTS221 at 0x%02X: %s\r\n", HTS221_I2C_DEV_ADDR, s_bHts221 ? "found" : "not found");
	if (s_bHts221)
	{
		s_Hts221.Enable();
	}
#endif

#ifdef LPS22HB
	s_bLps22hb = s_Lps22hb.Init(s_Lps22hbPressCfg, &s_I2c) && s_Lps22hb.Init(s_Lps22hbTempCfg, &s_I2c);
	if (s_bLps22hb)
	{
		printf("LPS22HB at 0x%02lX: found\r\n", (unsigned long)s_Lps22hb.DeviceAddress());
		s_Lps22hb.Enable();
	}
	else
	{
		printf("LPS22HB: not found\r\n");
	}
#endif

	while (1)
	{
#ifdef HTS221
		if (s_bHts221 && s_Hts221.UpdateData())
		{
			HumiSensorData_t h;
			TempSensorData_t t;

			s_Hts221.Read(h);
			s_Hts221.Read(t);
			printf("HTS221: ");
			PrintCenti("humidity", h.Humidity, "%, ");
			PrintCenti("temperature", t.Temperature, "C\r\n");
		}
#endif
#ifdef LPS22HB
		if (s_bLps22hb && s_Lps22hb.UpdateData())
		{
			PressureSensorData_t p;
			TempSensorData_t t;

			s_Lps22hb.Read(p);
			s_Lps22hb.Read(t);
			printf("LPS22HB: ");
			// hPa x 100 is Pa
			PrintCenti("pressure", (int32_t)p.Pressure, "hPa, ");
			PrintCenti("temperature", t.Temperature, "C\r\n");
		}
#endif
		msDelay(ENV_DEMO_POLL_MS);
	}

	return 0;
}
