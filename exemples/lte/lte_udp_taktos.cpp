/**-------------------------------------------------------------------------
@example	lte_udp_taktos.cpp

@brief	LTE UDP with TaktOS: the LTE work runs in its own thread.

The same as lte_udp.cpp, with TaktOS. The application overrides LteEvtQue:
each piece of LTE work becomes a message to the LTE thread, which runs it,
the way uart_ble_taktos.cpp serves Bluetooth. The thread starts LTE, runs
its messages and sends a UDP packet every LTE_UDP_INTERVAL seconds once
registered. Data the server sends back comes in as a message too.

The modem waits use TaktOS: the project links the TaktOS modem glue of the
port, so LTE gets no timer. LteInit and every AT command run in the LTE
thread, never before TaktOSStart.

Configure the server here or from board.h:

	LTE_UDP_HOST		server name or address
	LTE_UDP_PORT		server port
	LTE_UDP_APN			APN, NULL for the network default
	LTE_UDP_INTERVAL	seconds between packets
	LTE_UDP_RAI			1 to use release assistance (modem support needed)

The board.h of the project gives the console UART pins.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

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

#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSQueue.h"

#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
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

#ifndef TAKTOS_APP_TICK_HZ
#define TAKTOS_APP_TICK_HZ		1000u
#endif

// LTE thread: AT commands, name lookup and the console printing
#define LTE_UDP_THREAD_STACK	3072u

// Messages to the LTE thread
#define LTE_UDP_WORK_QUE_SIZE	16u

// Longest message and longest received datagram printed
#define LTE_UDP_MSG_LEN			64
#define LTE_UDP_RX_LEN			128

// Application work queued to the LTE thread
#define LTE_UDP_EVT_RX			2

// LTE work: one message per LteEvtQue call
typedef struct {
	uint32_t EvtId;
	void *pCtx;
	LteEvtQueHandler_t Handler;
} LteWork_t;

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

// No timer: the TaktOS modem glue does the waits
static const LteCfg_t s_LteCfg = {
	.Rat = LTE_RAT_LTEM_NBIOT,
	.RatPref = LTE_RAT_LTEM,
	.bGnss = false,
	.pBand = nullptr,			// Band setting of the modem kept
	.NbBand = 0,
	.pApn = LTE_UDP_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = true,
	.PsmTau = 3600,				// Periodic TAU requested: 1 h
	.PsmActive = 10,			// Active time requested: 10 s
	.EdrxCycle = 0,
	.bRai = LTE_UDP_RAI != 0,
	.IntPrio = IRQ_PRIO_LOW,
	.pTimer = nullptr,
	.TimerTrigNo = 0,
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

static uint8_t s_LteWorkQueMem[LTE_UDP_WORK_QUE_SIZE * sizeof(LteWork_t)] TAKT_ALIGNED(4);
static TaktOSQueue_t s_LteWorkQue;
static uint8_t s_LteThreadMem[TAKTOS_THREAD_MEM_SIZE(LTE_UDP_THREAD_STACK)] TAKT_ALIGNED(4);
static hTaktOSThread_t s_LteThread = nullptr;

static UART s_Uart;
static SockIntrf s_Sock;
static uint32_t s_TxCount = 0;

// Data arrived and not read yet: queued once, kept when the queue refused it
static volatile bool s_bRxPending = false;

// RTOS bridge: link-time override of LteEvtQue, see lte/lte.h. Called from
// the modem interrupt and by LteCheckStatus in the LTE thread. Each piece of
// work becomes a message to the LTE thread. The application event queue is
// not used.
bool LteEvtQue(uint32_t EvtId, void *pCtx, LteEvtQueHandler_t Handler)
{
	const LteWork_t work = { EvtId, pCtx, Handler };

	return TaktOSQueueSend(&s_LteWorkQue, &work, false, 0) == TAKTOS_OK;
}

static void LteUdpSend(void)
{
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

	int len = snprintf(msg, sizeof(msg), "IOsonata LteUdpTaktOS %u rsrp %d", (unsigned)s_TxCount, rsrp);

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

	// Data arriving from here on queues the next read
	s_bRxPending = false;

	uint8_t buf[LTE_UDP_RX_LEN];
	int n;

	while ((n = s_Sock.Rx(0, buf, sizeof(buf) - 1)) > 0)
	{
		buf[n] = 0;
		s_Uart.printf("Received %d: %s\r\n", n, (char *)buf);
	}
}

// Interrupt context: the read runs in the LTE thread
static int LteUdpSockEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	(void)Len;

	if (EvtId == DEVINTRF_EVT_RX_DATA && s_bRxPending == false)
	{
		s_bRxPending = true;
		LteEvtQue(LTE_UDP_EVT_RX, nullptr, LteUdpRecv);
	}

	return 0;
}

// LTE thread
static void LteUdpEvtHandler(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			s_Uart.printf("Registered %s, %s, TAC %04X, cell %08X\r\n",
						  pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
						  pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M",
						  (unsigned)pStatus->Tac, (unsigned)pStatus->CellId);
			LteUdpSend();
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

// Starts LTE, then runs the LTE work as its messages arrive and sends a
// packet at each interval
static void LteThread(void *pArg)
{
	(void)pArg;

	if (LteInit(&s_LteCfg) == false)
	{
		s_Uart.printf("LTE init failed\r\n");
	}
	else
	{
		char info[48];

		if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
		{
			s_Uart.printf("Modem firmware %s\r\n", info);
		}
		s_Uart.printf("Attaching\r\n");
	}

	const uint32_t period = LTE_UDP_INTERVAL * TaktOSGetTickHz();
	uint32_t next = TaktOSTickCount() + period;

	while (1)
	{
		int32_t wait = (int32_t)(next - TaktOSTickCount());

		if (wait <= 0)
		{
			LteUdpSend();
			next += period;
			if ((int32_t)(next - TaktOSTickCount()) <= 0)
			{
				// The send took longer than a period: no catching up
				next = TaktOSTickCount() + period;
			}
			continue;
		}

		LteWork_t work;

		if (TaktOSQueueReceive(&s_LteWorkQue, &work, false, 0) != TAKTOS_OK)
		{
			// Queue empty: queue again the work it refused, read what a
			// refused read left, then wait for a message or the next send
			LteCheckStatus();
			if (s_bRxPending)
			{
				LteUdpRecv(LTE_UDP_EVT_RX, nullptr);
			}
			if (TaktOSQueueReceive(&s_LteWorkQue, &work, true, (uint32_t)wait) != TAKTOS_OK)
			{
				continue;
			}
		}
		work.Handler(work.EvtId, work.pCtx);
	}
}

int main()
{
	s_Uart.Init(s_UartCfg);
	s_Uart.printf("LteUdpTaktOS\r\n");

	// The work queue exists before anything can queue LTE work
	if (TaktOSQueueInit(&s_LteWorkQue, s_LteWorkQueMem, sizeof(LteWork_t),
						LTE_UDP_WORK_QUE_SIZE) != TAKTOS_OK)
	{
		s_Uart.printf("Queue init failed\r\n");
		while (1)
		{
			__WFE();
		}
	}

	TaktOSCfg_t cfg = {
		.KernClockHz = SystemCoreClockGet(),
		.TickHz = TAKTOS_APP_TICK_HZ,
	};

	if (TaktOSInit(&cfg) != TAKTOS_OK)
	{
		s_Uart.printf("TaktOS init failed\r\n");
		while (1)
		{
			__WFE();
		}
	}

	s_LteThread = TaktOSThreadCreate(s_LteThreadMem, sizeof(s_LteThreadMem), LteThread, nullptr,
									 TAKTOS_PRIORITY_NORMAL);
	if (s_LteThread == nullptr)
	{
		s_Uart.printf("Thread create failed\r\n");
		while (1)
		{
			__WFE();
		}
	}

	TaktOSStart();

	return 0;
}
