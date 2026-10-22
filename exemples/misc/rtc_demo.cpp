/**-------------------------------------------------------------------------
@example	rtc_demo.cpp

@brief	Realtime clock demo

Starts the RTC, on the low frequency crystal when it starts, on the low
frequency RC otherwise, and logs with SysLog on the console UART the clock
in use, whether the calendar was kept from before the reset, and a boot count
in backup register 0, which a reset keeps. Then it logs the date and time
with the milliseconds and the seconds since the epoch at each periodic wakeup
event (RTC_DEMO_WAKEUP msec), or every second with msDelay on an RTC without
wakeup timer, and an alarm event at the start of each minute.

A calendar not set since the RTC started takes the build date and time of
this file, as UTC.

The board.h of the project gives the console UART pins.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <time.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "syslog.h"
#include "miscdev/rtc.h"
#include "idelay.h"

#include "board.h"

// Wakeup period in msec
#ifndef RTC_DEMO_WAKEUP
#define RTC_DEMO_WAKEUP			1000
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

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
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static void RtcDemoEvent(RtcDev_t * const pDev, uint32_t Evt);

static const RtcCfg_t s_RtcCfg = {
	.DevNo = 0,
	.ClkSrc = RTC_CLKSRC_DEFAULT,
	.IntPrio = 6,
	.EvtHandler = RtcDemoEvent,
};

// Alarm 0 at second 0 of every minute
static const RtcAlarm_t s_Alarm = {
	.Sec = 0,
	.Min = -1,
	.Hour = -1,
	.MDay = -1,
	.WDay = -1,
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

static Rtc s_Rtc;

// RTC_EVT_xxx bits of the events not yet logged
static volatile uint32_t s_RtcDemoEvt = 0;

// Interrupt context: only the event bits
static void RtcDemoEvent(RtcDev_t * const pDev, uint32_t Evt)
{
	(void)pDev;
	s_RtcDemoEvt |= Evt;
}

static int RtcDemoDigits(const char *p, int n)
{
	int v = 0;

	for (int i = 0; i < n; i++)
	{
		v = v * 10 + (p[i] == ' ' ? 0 : p[i] - '0');
	}

	return v;
}

// Build date and time of this file: __DATE__ "Mmm dd yyyy", __TIME__
// "hh:mm:ss"
static struct tm RtcDemoBuildTime(void)
{
	static const char s_Mon[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	const char *d = __DATE__;
	const char *t = __TIME__;
	struct tm tm = {};

	for (int i = 0; i < 12; i++)
	{
		if (d[0] == s_Mon[i * 3] && d[1] == s_Mon[i * 3 + 1] && d[2] == s_Mon[i * 3 + 2])
		{
			tm.tm_mon = i;
		}
	}
	tm.tm_mday = RtcDemoDigits(&d[4], 2);
	tm.tm_year = RtcDemoDigits(&d[7], 4) - 1900;
	tm.tm_hour = RtcDemoDigits(&t[0], 2);
	tm.tm_min = RtcDemoDigits(&t[3], 2);
	tm.tm_sec = RtcDemoDigits(&t[6], 2);
	tm.tm_isdst = -1;

	return tm;
}

static void RtcDemoPrintTime(void)
{
	static const char *const s_Day[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
	struct tm tm;
	struct timespec ts;
	uint32_t us = 0;

	if (s_Rtc.GetTime(tm, &us) && s_Rtc.GetTime(ts))
	{
		SysLogPrintf(SysLogGet(), "%04d-%02d-%02d %s %02d:%02d:%02d.%03u, %lu s since the epoch\r\n",
					 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, s_Day[tm.tm_wday], tm.tm_hour, tm.tm_min,
					 tm.tm_sec, (unsigned)(us / 1000U), (unsigned long)ts.tv_sec);
	}
}

int main()
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "RtcDemo\r\n");

	if (s_Rtc.Init(s_RtcCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "RTC init failed\r\n");

		while (1)
		{
			__WFE();
		}
	}

	SysLogPrintf(SysLogGet(), "RTC on the %s, %u Hz\r\n",
				 s_Rtc.ClockSource() == RTC_CLKSRC_LFXTAL ? "LSE" : "LSI", (unsigned)s_Rtc.ClockFrequency());

	if (s_Rtc.TimeValid())
	{
		SysLogPrintf(SysLogGet(), "Calendar kept from before the reset\r\n");
	}
	else
	{
		struct tm tm = RtcDemoBuildTime();

		s_Rtc.SetTime(tm);
		SysLogPrintf(SysLogGet(), "Calendar set to the build time\r\n");
	}

	if (s_Rtc.NbBackup() > 0)
	{
		uint32_t boot = 0;

		s_Rtc.ReadBackup(0, boot);
		boot++;
		s_Rtc.WriteBackup(0, boot);
		SysLogPrintf(SysLogGet(), "Boot %u, backup register 0\r\n", (unsigned)boot);
	}

	s_Rtc.SetAlarm(0, s_Alarm);

	bool wakeup = s_Rtc.SetWakeup(RTC_DEMO_WAKEUP);

	if (wakeup)
	{
		SysLogPrintf(SysLogGet(), "Wakeup every %u ms\r\n", (unsigned)s_Rtc.Wakeup());
	}
	else
	{
		SysLogPrintf(SysLogGet(), "No wakeup timer, 1 s delay\r\n");
	}

	RtcDemoPrintTime();

	while (1)
	{
		if (wakeup)
		{
			while (s_RtcDemoEvt == 0)
			{
				__WFE();
			}
		}
		else
		{
			msDelay(1000);
		}

		__disable_irq();
		uint32_t evt = s_RtcDemoEvt;
		s_RtcDemoEvt = 0;
		__enable_irq();

		if (evt & RTC_EVT_ALARM0)
		{
			SysLogPrintf(SysLogGet(), "Alarm 0\r\n");
		}
		if (evt & RTC_EVT_WAKEUP || wakeup == false)
		{
			RtcDemoPrintTime();
		}
	}

	return 0;
}
