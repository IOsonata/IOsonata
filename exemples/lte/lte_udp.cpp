/**-------------------------------------------------------------------------
@example	lte_udp.cpp

@brief	LTE UDP: attach, then send a UDP packet to a server at an interval.

Starts the LTE subsystem (lte.h) with PSM requested, opens a UDP socket
(net/sock_intrf.h) once registered and sends a short message every
LTE_UDP_INTERVAL seconds, with a release assistance hint so the network
lets the modem sleep right after it. Network events and anything the
server sends back are printed on the console UART. Between packets the
modem is in PSM and the application waits in AppRun. For an average current
measurement, leave the console out: its receiver draws far more than the
modem in PSM. After a modem fault the example starts LTE again.

Everything runs from the application event queue: the LTE events, the
received data and the send timer.

Configure the server here or from board.h:

	LTE_UDP_HOST		server name or address
	LTE_UDP_PORT		server port
	LTE_UDP_APN			APN, NULL for the network default
	LTE_UDP_INTERVAL	seconds between packets
	LTE_UDP_RAI			1 to use release assistance (modem support needed)

The board.h of the project gives the console UART pins and the timer used
by the modem waits and the send interval.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "lte/lte.h"
#include "net/sock_intrf.h"

#include "board.h"

#ifndef LTE_UDP_HOST
#define LTE_UDP_HOST			"udp.example.com"
#endif
#ifndef LTE_UDP_PORT
#define LTE_UDP_PORT			2469
#endif
#ifndef LTE_UDP_APN
#define LTE_UDP_APN				nullptr
#endif
#ifndef LTE_UDP_INTERVAL
#define LTE_UDP_INTERVAL		60
#endif
#ifndef LTE_UDP_RAI
#define LTE_UDP_RAI				1
#endif

// Timer triggers: one kept for the modem waits, one for the send interval
#define LTE_UDP_TRIG_MODEM		0
#define LTE_UDP_TRIG_SEND		1

// Longest message and longest received datagram printed
#define LTE_UDP_MSG_LEN			64
#define LTE_UDP_RX_LEN			128

// Application events
#define LTE_UDP_EVT_SEND		1
#define LTE_UDP_EVT_RX			2

// Application event queue: LTE work, received data and the send timer
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(8)];

static void LteUdpEvtHandler(LTE_EVT Evt, const LteStatus_t * const pStatus);
static void LteUdpUrcHandler(const char *pUrc);
static int LteUdpSockEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

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
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

// Low frequency timer: modem waits and the send interval
static const TimerCfg_t s_TimerCfg = {
	.DevNo = LTE_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static TimerDev_t s_TimerDev;

static const LteCfg_t s_LteCfg = {
	.Rat = LTE_RAT_LTEM_NBIOT,
	.RatPref = LTE_RAT_LTEM,
	.bGnss = false,
	.pApn = LTE_UDP_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = true,
	.PsmTau = 3600,				// Periodic TAU requested: 1 h
	.PsmActive = 10,			// Active time requested: 10 s
	.EdrxCycle = 0,
	.bRai = LTE_UDP_RAI != 0,
	.IntPrio = IRQ_PRIO_LOW,
	.pTimer = &s_TimerDev,
	.TimerTrigNo = LTE_UDP_TRIG_MODEM,
	.UrcMemSize = 0,
	.EvtHandler = LteUdpEvtHandler,
	.UrcHandler = LteUdpUrcHandler,
};

static const SockIntrfCfg_t s_SockCfg = {
	.Proto = SOCKINTRF_PROTO_UDP,
	.pHost = LTE_UDP_HOST,
	.Port = LTE_UDP_PORT,
	.LocalPort = 0,
	.SecTag = -1,
	.bPeerVerify = false,
	.Timeout = 0,
	.EvtCB = LteUdpSockEvt,
};

static UART s_Uart;
static SockIntrf s_Sock;
static uint32_t s_TxCount = 0;

static void LteUdpSend(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (LteRegistered() == false)
	{
		return;
	}

	if (s_Sock.Connected() == false && s_Sock.Init(s_SockCfg) == false)
	{
		s_Uart.printf("Socket to %s:%d failed\r\n", LTE_UDP_HOST, LTE_UDP_PORT);
		return;
	}

	char msg[LTE_UDP_MSG_LEN];
	int rsrp = 0;

	if (LteGetSignal(&rsrp, nullptr) == false)
	{
		rsrp = 0;
	}

	int len = snprintf(msg, sizeof(msg), "IOsonata LteUdp %u rsrp %d", (unsigned)s_TxCount, rsrp);

	// The send is the last of the exchange: the network releases the radio
	// connection right after it
	if (LTE_UDP_RAI)
	{
		s_Sock.Rai(SOCKINTRF_RAI_LAST);
	}

	if (s_Sock.Tx(0, (uint8_t *)msg, len) == len)
	{
		s_Uart.printf("Sent %d: %s\r\n", len, msg);
		s_TxCount++;
	}
	else
	{
		s_Uart.printf("Send failed\r\n");
	}
}

static void LteUdpRecv(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	uint8_t buf[LTE_UDP_RX_LEN];
	int n;

	while ((n = s_Sock.Rx(0, buf, sizeof(buf) - 1)) > 0)
	{
		buf[n] = 0;
		s_Uart.printf("Received %d: %s\r\n", n, (char *)buf);
	}
}

// Interrupt context: queue the read
static int LteUdpSockEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	(void)Len;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		AppEvtHandlerQue(LTE_UDP_EVT_RX, nullptr, LteUdpRecv);
	}

	return 0;
}

// Interrupt context: queue the send
static void LteUdpTimerTrig(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pContext;

	AppEvtHandlerQue(LTE_UDP_EVT_SEND, nullptr, LteUdpSend);
}

static void LteUdpEvtHandler(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			s_Uart.printf("Registered %s, %s, TAC %04X, cell %08X\r\n",
						  pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
						  pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M",
						  (unsigned)pStatus->Tac, (unsigned)pStatus->CellId);
			if (pStatus->bPsm)
			{
				s_Uart.printf("PSM TAU %u s, active %u s\r\n",
							  (unsigned)pStatus->PsmTau, (unsigned)pStatus->PsmActive);
			}
			LteUdpSend(0, nullptr);
			break;

		case LTE_EVT_UNREGISTERED:
			s_Uart.printf("Unregistered, state %d\r\n", (int)pStatus->RegStat);
			break;

		case LTE_EVT_REG_STATE:
			s_Uart.printf("Registration state %d\r\n", (int)pStatus->RegStat);
			break;

		case LTE_EVT_CELL:
			s_Uart.printf("Cell TAC %04X, cell %08X\r\n", (unsigned)pStatus->Tac, (unsigned)pStatus->CellId);
			break;

		case LTE_EVT_RRC_CONNECTED:
			s_Uart.printf("RRC connected\r\n");
			break;

		case LTE_EVT_RRC_IDLE:
			s_Uart.printf("RRC idle\r\n");
			break;

		case LTE_EVT_PSM:
			s_Uart.printf("PSM %s, TAU %u s, active %u s\r\n", pStatus->bPsm ? "on" : "off",
						  (unsigned)pStatus->PsmTau, (unsigned)pStatus->PsmActive);
			break;

		case LTE_EVT_EDRX:
			s_Uart.printf("eDRX %u ms, PTW %u ms\r\n", (unsigned)pStatus->EdrxCycle, (unsigned)pStatus->EdrxPtw);
			break;

		case LTE_EVT_MODEM_FAULT:
			s_Uart.printf("Modem fault, restarting\r\n");
			s_Sock.Close();
			if (LteInit(&s_LteCfg) == false)
			{
				s_Uart.printf("LTE init failed\r\n");
			}
			break;
	}
}

static void LteUdpUrcHandler(const char *pUrc)
{
	s_Uart.printf("URC %s\r\n", pUrc);
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));

	s_Uart.Init(s_UartCfg);
	s_Uart.printf("LteUdp\r\n");

	char info[48];

	if (TimerInit(&s_TimerDev, &s_TimerCfg) == false)
	{
		s_Uart.printf("Timer init failed\r\n");
	}
	else if (LteInit(&s_LteCfg) == false)
	{
		s_Uart.printf("LTE init failed\r\n");
	}
	else
	{
		if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
		{
			s_Uart.printf("Modem firmware %s\r\n", info);
		}
		if (LteGetInfo(LTE_INFO_IMEI, info, sizeof(info)))
		{
			s_Uart.printf("IMEI %s\r\n", info);
		}
		msTimerEnableTrigger(&s_TimerDev, LTE_UDP_TRIG_SEND, LTE_UDP_INTERVAL * 1000U,
							 TIMER_TRIG_TYPE_CONTINUOUS, LteUdpTimerTrig, nullptr);
		s_Uart.printf("Attaching\r\n");
	}

	AppRun();

	return 0;
}
