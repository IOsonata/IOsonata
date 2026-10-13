/**-------------------------------------------------------------------------
@example	rtc_demo.cpp

@brief	RTC and low power demo on the FPB-R9A02G021

The RTC wakes the MCU every second from Software Standby (or Sleep with
RTC_DEMO_STANDBY false in board.h). At each wake-up the time is printed on
the UART and LED1 toggles. An alarm is set RTC_DEMO_ALARM_MIN minutes ahead,
LED2 toggles when it matches and the next one is set.

The calendar needs the 32.768 kHz crystal as the low frequency clock (MCUOSC
in board.h). After a power-on reset it starts at the build date and time.
After another reset the RTC keeps counting and the time is kept. On the LOCO
only the fixed-cycle interrupt works, the seconds since start are printed.

The UART stops in Software Standby, the transmission is finished before
entering it. The debugger connection may be lost in Software Standby.

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
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "idelay.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "system_r9a02.h"
#include "rtc_r9a02.h"
#include "board.h"

#ifndef RTC_DEMO_STANDBY
#define RTC_DEMO_STANDBY		true
#endif

#ifndef RTC_DEMO_ALARM_MIN
#define RTC_DEMO_ALARM_MIN		2
#endif

#define RTC_DEMO_TX_WAIT_US		200		// Last character out of the shift register

static void RtcEvtHandler(uint32_t Evt, void *pCtx);

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const char s_MonthNames[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
static const char * const s_DayNames[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

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

static UART s_Uart;
static volatile uint32_t s_RtcEvt = 0;

static void RtcEvtHandler(uint32_t Evt, void *pCtx)
{
	s_RtcEvt |= Evt;
}

static int Digits2(const char *pStr)
{
	return (pStr[0] == ' ' ? 0 : (pStr[0] - '0') * 10) + (pStr[1] - '0');
}

// __DATE__ is "Mmm dd yyyy", __TIME__ is "hh:mm:ss"
static void BuildTime(struct tm *pTime)
{
	const char *d = __DATE__;
	const char *t = __TIME__;

	memset(pTime, 0, sizeof(struct tm));

	for (int i = 0; i < 12; i++)
	{
		if (memcmp(d, &s_MonthNames[i * 3], 3) == 0)
		{
			pTime->tm_mon = i;
			break;
		}
	}
	pTime->tm_mday = Digits2(&d[4]);
	pTime->tm_year = atoi(&d[7]) - 1900;
	pTime->tm_hour = Digits2(&t[0]);
	pTime->tm_min = Digits2(&t[3]);
	pTime->tm_sec = Digits2(&t[6]);
}

static void PrintTime(const struct tm *pTime)
{
	printf("%04d-%02d-%02d %02d:%02d:%02d %s\r\n", pTime->tm_year + 1900, pTime->tm_mon + 1,
		   pTime->tm_mday, pTime->tm_hour, pTime->tm_min, pTime->tm_sec, s_DayNames[pTime->tm_wday % 7]);
}

// Alarm Min minutes after the current time, any weekday
static bool AlarmSet(int Min)
{
	struct tm t;

	if (R9a02RtcGetTime(&t) == false)
	{
		return false;
	}

	int m = t.tm_hour * 60 + t.tm_min + Min;

	return R9a02RtcSetAlarm(0x7F, (m / 60) % 24, m % 60);
}

// The UART stops in Software Standby, the TX CFIFO and the shift register are
// emptied first
static void UartWaitIdle(void)
{
	UARTDev_t *dev = s_Uart;
	uint32_t n = 1000000;

	while (dev->bTxReady == false && --n > 0U)
	{
	}
	usDelay(RTC_DEMO_TX_WAIT_US);
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

	printf("\r\nRTC demo, %s between events\r\n", RTC_DEMO_STANDBY ? "Software Standby" : "Sleep");

	if (R9a02RtcInit(RtcEvtHandler, NULL, IRQ_PRIO_NORMAL, true) == false)
	{
		printf("RTC init failed, low frequency clock not running\r\n");
		while (1) __WFE();
	}

	bool calendar = R9a02RtcCalendar();
	bool setup = calendar;
	struct tm t;

	printf("%s\r\n", calendar ? "Calendar on SOSC" : "RTC on LOCO, fixed-cycle interrupt only");

	// With the alarm enabled the counters are read and written right after
	// the 1 s event, the calendar is set up at the first one
	R9a02RtcSetPeriod(R9A02_RTC_PERIOD_SEC);

	uint32_t secs = 0;

	while (1)
	{
		uintptr_t state = DisableInterrupt();
		uint32_t evt = s_RtcEvt;

		s_RtcEvt = 0;
		EnableInterrupt(state);

		if (evt & R9A02_RTC_EVT_PERIOD)
		{
			secs++;
			IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);

			if (setup)
			{
				// 2000 is the start-up value, a running RTC keeps its time
				setup = false;
				if (R9a02RtcGetTime(&t) && t.tm_year == 100)
				{
					BuildTime(&t);
					R9a02RtcSetTime(&t);
				}
				printf("Alarm in %d min %s\r\n", RTC_DEMO_ALARM_MIN, AlarmSet(RTC_DEMO_ALARM_MIN) ? "set" : "failed");
			}

			if (calendar && R9a02RtcGetTime(&t))
			{
				PrintTime(&t);
			}
			else
			{
				printf("%lu s\r\n", (unsigned long)secs);
			}
		}

		if (evt & R9A02_RTC_EVT_ALARM)
		{
			IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
			printf("Alarm, next in %d min %s\r\n", RTC_DEMO_ALARM_MIN, AlarmSet(RTC_DEMO_ALARM_MIN) ? "set" : "failed");
		}

		if (evt == 0U)
		{
			// An event between the check and the WFI is handled at the next
			// one, one second later
			UartWaitIdle();
			R9a02LowPowerEnter(RTC_DEMO_STANDBY);
		}
	}

	return 0;
}
