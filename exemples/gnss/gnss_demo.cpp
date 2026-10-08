/**-------------------------------------------------------------------------
@example	gnss_demo.cpp

@brief	GNSS demo

Starts the GNSS receiver for a fix every second and prints each update on the
console UART. While searching: the time searched and the satellites tracked
with their signal strength (C/N0 in dB-Hz, system letter G GPS, E Galileo,
J QZSS, R GLONASS, C BeiDou). With a fix: the UTC time, the position, its
accuracy, the speed and the satellites used. The time to the first fix is
printed once.

The board.h of the project gives the console UART pins, the receiver and its
interface (GNSS_RECEIVER, GNSS_INTRF, GNSS_INTRF_CFG_T, GNSS_INTRF_CFG), the
timer (GNSS_TIMER_DEVNO) and the board specific receiver commands
(GNSS_CMD_LIST), for example the antenna LNA control of a DK.

The receiver needs a view of the sky. Without assistance data the first fix
takes from about 30 s to a few minutes.

@author	Hoang Nguyen Hoan
@date	Oct. 8, 2026

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
#include <stdio.h>
#include <math.h>

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "gnss/gnss.h"

#include "board.h"

#if !defined(GNSS_RECEIVER) || !defined(GNSS_INTRF) || !defined(GNSS_INTRF_CFG_T) || !defined(GNSS_INTRF_CFG)
#error "board.h gives the receiver: GNSS_RECEIVER, GNSS_INTRF, GNSS_INTRF_CFG_T and GNSS_INTRF_CFG"
#endif

// Timer of the time stamps, and of the waits of a receiver interface that has
// some
#ifndef GNSS_TIMER_DEVNO
#define GNSS_TIMER_DEVNO		0
#endif

// Trigger of the timer kept for the waits of the receiver interface
#define GNSS_DEMO_TIMER_TRIG	0

// Satellites printed per update
#define GNSS_DEMO_SAT_MAX		24

// Longest number GnssDemoNum writes
#define GNSS_DEMO_NUM_LEN		24

// Decimals GnssDemoNum takes
#define GNSS_DEMO_DEC_MAX		7

// Application event queue: the GNSS work
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(4)];

static void GnssDemoEvtHandler(Gnss * const pGnss, GNSS_EVT Evt, const GnssStatus_t * const pStatus);
static int GnssDemoIntrfEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

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

// Low frequency timer
static const TimerCfg_t s_TimerCfg = {
	.DevNo = GNSS_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

#ifdef GNSS_CMD_LIST
static const char * const s_GnssCmd[] = GNSS_CMD_LIST;
#endif

// A fix every second, all the systems the receiver is set for
static const GnssCfg_t s_GnssCfg = {
	.DevAddr = 0,
	.SysMask = 0,
	.FixInterval = 1,
	.FixTimeout = 0,
	.Dyn = GNSS_DYN_GENERAL,
#ifdef GNSS_CMD_LIST
	.ppCmd = s_GnssCmd,
	.NbCmd = sizeof(s_GnssCmd) / sizeof(s_GnssCmd[0]),
#else
	.ppCmd = nullptr,
	.NbCmd = 0,
#endif
	.EvtHandler = GnssDemoEvtHandler,
};

// Decimal formats and scale of GnssDemoNum, indexed by the decimals
static const char * const s_NumFmt[GNSS_DEMO_DEC_MAX + 1] = {
	"%s%u", "%s%u.%01u", "%s%u.%02u", "%s%u.%03u",
	"%s%u.%04u", "%s%u.%05u", "%s%u.%06u", "%s%u.%07u"
};

static const uint32_t s_NumScale[GNSS_DEMO_DEC_MAX + 1] = {
	1, 10, 100, 1000, 10000, 100000, 1000000, 10000000
};

static UART s_Uart;
static Timer s_Timer;
static const GNSS_INTRF_CFG_T s_GnssIntrfCfg = GNSS_INTRF_CFG(s_Timer, GNSS_DEMO_TIMER_TRIG,
																GnssDemoIntrfEvt);
static GNSS_INTRF s_GnssIntrf;
static GNSS_RECEIVER s_Gnss;
static GnssSat_t s_Sat[GNSS_DEMO_SAT_MAX];
static uint64_t s_StartTime = 0;		// Time of the start, ns
static bool s_bFirstFix = true;

// Val with Dec decimals into pBuf, without the float support of printf
static const char *GnssDemoNum(char *pBuf, double Val, int Dec)
{
	double v = Val * (double)s_NumScale[Dec];
	bool neg = v < 0.0;
	uint64_t a = (uint64_t)((neg ? -v : v) + 0.5);

	snprintf(pBuf, GNSS_DEMO_NUM_LEN, s_NumFmt[Dec], neg && a != 0 ? "-" : "",
			 (unsigned)(a / s_NumScale[Dec]), (unsigned)(a % s_NumScale[Dec]));

	return pBuf;
}

static char GnssDemoSysLetter(GNSS_SYS Sys)
{
	switch (Sys)
	{
		case GNSS_SYS_GPS:
			return 'G';
		case GNSS_SYS_GALILEO:
			return 'E';
		case GNSS_SYS_QZSS:
			return 'J';
		case GNSS_SYS_GLONASS:
			return 'R';
		case GNSS_SYS_BEIDOU:
			return 'C';
	}

	return '?';
}

// Seconds since the start
static unsigned GnssDemoSec(void)
{
	return (unsigned)((s_Timer.nSecond() - s_StartTime) / 1000000000ULL);
}

static bool GnssDemoStart(void)
{
	if (s_GnssIntrf.Init(s_GnssIntrfCfg) == false || s_Gnss.Init(s_GnssCfg, &s_GnssIntrf, &s_Timer) == false)
	{
		return false;
	}

	s_StartTime = s_Timer.nSecond();
	s_bFirstFix = true;

	return true;
}

// Interface interrupt: what the receiver sends goes to its driver
static int GnssDemoIntrfEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev;

	return s_Gnss.IntrfEvtHandler(EvtId, pBuffer, Len);
}

// Called from the queued GNSS work
static void GnssDemoEvtHandler(Gnss * const pGnss, GNSS_EVT Evt, const GnssStatus_t * const pStatus)
{
	char n1[GNSS_DEMO_NUM_LEN];
	char n2[GNSS_DEMO_NUM_LEN];
	char n3[GNSS_DEMO_NUM_LEN];
	char n4[GNSS_DEMO_NUM_LEN];
	char n5[GNSS_DEMO_NUM_LEN];

	switch (Evt)
	{
		case GNSS_EVT_FIX:
			{
				const GnssFix_t &f = pStatus->Fix;

				if (s_bFirstFix)
				{
					s_bFirstFix = false;
					s_Uart.printf("First fix after %u s\r\n", GnssDemoSec());
				}

				s_Uart.printf("%04u-%02u-%02u %02u:%02u:%02u UTC lat %s lon %s alt %s m acc %s m",
							  f.Time.Year, f.Time.Month, f.Time.Day, f.Time.Hour, f.Time.Min,
							  f.Time.Sec, GnssDemoNum(n1, f.Lat, 7), GnssDemoNum(n2, f.Lon, 7),
							  GnssDemoNum(n3, f.Alt, 1), GnssDemoNum(n4, f.PosAccH, 1));
				if (f.bVelValid)
				{
					s_Uart.printf(" speed %s m/s",
								  GnssDemoNum(n5, sqrtf(f.Vel[0] * f.Vel[0] + f.Vel[1] * f.Vel[1]), 1));
				}
				s_Uart.printf(" sats %u/%u\r\n", pStatus->NbSatUsed, pStatus->NbSatTracked);
			}
			break;

		case GNSS_EVT_NOFIX:
			{
				int nb = pGnss->GetSat(s_Sat, GNSS_DEMO_SAT_MAX);

				s_Uart.printf("No fix, %u s since start, tracked %u:", GnssDemoSec(), pStatus->NbSatTracked);
				for (int i = 0; i < nb; i++)
				{
					s_Uart.printf(" %c%u:%u", GnssDemoSysLetter(s_Sat[i].Sys), s_Sat[i].Id,
								  s_Sat[i].Cn0 / 10U);
				}
				s_Uart.printf("\r\n");
			}
			break;

		case GNSS_EVT_TIMEOUT:
			s_Uart.printf("No fix within the timeout\r\n");
			break;

		case GNSS_EVT_BLOCKED:
			s_Uart.printf("Receiver held off, its radio is in use\r\n");
			break;

		case GNSS_EVT_UNBLOCKED:
			s_Uart.printf("Receiver running again\r\n");
			break;

		case GNSS_EVT_FAULT:
			s_Uart.printf("Receiver fault, restarting\r\n");
			if (GnssDemoStart() == false)
			{
				s_Uart.printf("GNSS init failed\r\n");
			}
			break;
	}
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));

	s_Uart.Init(s_UartCfg);
	s_Uart.printf("GnssDemo\r\n");

	if (s_Timer.Init(s_TimerCfg) == false)
	{
		s_Uart.printf("Timer init failed\r\n");
	}
	else if (GnssDemoStart() == false)
	{
		s_Uart.printf("GNSS init failed\r\n");
	}
	else
	{
		s_Uart.printf("Searching, a fix needs a view of the sky\r\n");
	}

	AppRun();

	return 0;
}
