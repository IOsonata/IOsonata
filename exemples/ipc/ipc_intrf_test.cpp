/**-------------------------------------------------------------------------
@example	ipc_intrf_test.cpp

@brief	IpcIntrf test between two cores: the owner echoes, the peer checks.

One source for both cores, the role comes from board.h.

Owner (IPCTEST_OWNER 1, nRF5340 network core): formats the shared memory block
and echoes every byte or packet it receives. It keeps what it could not send
back yet, so a full ring only slows it down.

Peer (IPCTEST_OWNER 0, nRF5340 application core): holds the network core in
reset, clears the block, attaches, releases the network core, then sends a
pseudo random stream and checks what comes back. In packet mode every message
is one ring slot of a length that changes from message to message, and the
echo must keep both the length and the bytes. In byte mode the stream has no
boundaries and only the bytes are compared. Results go to the UART and the
LEDs:
	LED1 on     link is up
	LED2 toggles at each report while messages keep coming back
	LED3 on     an error was seen (stays on)
	LED4 on     the run of IPCTEST_COUNT messages finished without error

The test does not use interrupts for reading, the ring is polled, so it checks
the ring and signal paths of IpcIntrf alone. Run it first in IPCTEST_WINDOW 1
(one message in flight), then with a larger window to fill the rings.

Both images must use the same board.h values for the block address, ring
geometry and channels.

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
#include <string.h>

#include "nrf.h"

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "ipc/ipc_intrf.h"

// This include holds the role and the I/O definitions of the board in use
#include "board.h"

#include "coredev/uart.h"
#include "syslog.h"

// Defaults, board.h may override them before this point by defining them
#ifndef IPCTEST_PACKET
#define IPCTEST_PACKET			1		// 1 packet slots, 0 byte stream
#endif
#ifndef IPCTEST_PKT_MAX
#define IPCTEST_PKT_MAX			60U		// Largest payload of one packet
#endif
#ifndef IPCTEST_NB_PKT
#define IPCTEST_NB_PKT			8U		// Slots per ring
#endif
#ifndef IPCTEST_RING_BYTES
#define IPCTEST_RING_BYTES		256U	// Bytes per ring in byte mode
#endif
#ifndef IPCTEST_WINDOW
#define IPCTEST_WINDOW			6U		// Messages in flight, 1 for lock step
#endif
#ifndef IPCTEST_COUNT
#define IPCTEST_COUNT			200000UL // Messages of one run
#endif
#ifndef IPCTEST_STALL_MS
#define IPCTEST_STALL_MS		2000U	// No echo for this long is an error
#endif

#if IPCTEST_PACKET
#define RING_NB					IPCTEST_NB_PKT
#define RING_BLK				(IPCTEST_PKT_MAX + IPC_INTRF_PKT_HDR_SIZE)
#define MSG_MAX					IPCTEST_PKT_MAX
#else
#define RING_NB					IPCTEST_RING_BYTES
#define RING_BLK				0U
#define MSG_MAX					64U
#endif

#define BLOCK_SIZE				IPC_INTRF_MEMSIZE(RING_NB, RING_BLK, RING_NB, RING_BLK)

static_assert(BLOCK_SIZE <= IPCTEST_MEM_SIZE, "IPCTEST_MEM_SIZE is too small for the ring geometry");

static IpcIntrfDev_t s_Ipc;

#ifdef CORE_PINS
static const IOPinCoreCfg_t s_CorePins[] = CORE_PINS;
#endif

/*---------------------------------------------------------------------------
 * Trace, both cores: SysLog over the UART of the board, the application
 * core on its console port, the network core on its own port.
 *-------------------------------------------------------------------------*/

#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif

alignas(4) static uint8_t s_UartRxFifo[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_UartTxFifo[CFIFO_MEMSIZE(1024)];

static IOPinCfg_t s_UartPins[] = UART_PINS;

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
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = sizeof(s_UartRxFifo),
	.pRxMem = s_UartRxFifo,
	.TxMemSize = sizeof(s_UartTxFifo),
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART,
};

static UART s_Uart;

alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 96)];

static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 96,
	.bBlocking = true,
};

// Trace line. Records are kept while the UART is not up, so a boot message
// logged before the UART is reaches the terminal once it is.
#define LOG(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)

static bool TraceInit(void)
{
	SysLogInit(SysLogGet(), &s_LogCfg, nullptr, 0, nullptr, 0);

	if (s_Uart.Init(s_UartCfg) == false)
	{
		return false;
	}

	SysLogSetSink(SysLogGet(), (DevIntrf_t *)s_Uart, 0);
	SysLogFlush(SysLogGet());

	return true;
}

// Pseudo random bytes, the same on both sides of a message
static inline uint32_t Rng(uint32_t x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

#if IPCTEST_OWNER

/*---------------------------------------------------------------------------
 * Owner: echo
 *-------------------------------------------------------------------------*/

// Counters, also for a debugger
volatile struct {
	uint32_t Up;
	uint32_t RxMsg;
	uint32_t RxBytes;
	uint32_t TxBytes;
	uint32_t Pending;
	uint32_t Stalls;
} g_IpcTestStat;

static uint8_t s_EchoBuf[MSG_MAX];
static int s_EchoLen;
static int s_EchoOff;

bool IpcTestOwnerInit(void)
{
	IpcIntrfCfg_t cfg = {};

	cfg.pMemBlk = reinterpret_cast<uint8_t*>(IPCTEST_MEM_ADDR);
	cfg.MemSize = IPCTEST_MEM_SIZE;
	cfg.bOwner = true;
	cfg.TxNbBlk = RING_NB;
	cfg.TxBlkSize = RING_BLK;
	cfg.RxNbBlk = RING_NB;
	cfg.RxBlkSize = RING_BLK;
	cfg.TxChan = IPCTEST_OWNER_TX_CHAN;
	cfg.RxChan = IPCTEST_OWNER_RX_CHAN;
	cfg.IntPrio = 6;
	cfg.EvtCB = nullptr;

	s_EchoLen = 0;
	s_EchoOff = 0;

	return IpcIntrfInit(&s_Ipc, &cfg);
}

// One pass of the echo: take a message if none is held, send what is held.
void IpcTestOwnerStep(void)
{
	if (s_EchoLen == 0)
	{
		int n = DeviceIntrfRx(&s_Ipc.DevIntrf, 0, s_EchoBuf, sizeof(s_EchoBuf));

		if (n > 0)
		{
			s_EchoLen = n;
			s_EchoOff = 0;
			g_IpcTestStat.RxMsg++;
			g_IpcTestStat.RxBytes += n;
		}
	}

	if (s_EchoLen > 0)
	{
		int w = DeviceIntrfTx(&s_Ipc.DevIntrf, 0, s_EchoBuf + s_EchoOff, s_EchoLen - s_EchoOff);

		if (w > 0)
		{
			s_EchoOff += w;
			g_IpcTestStat.TxBytes += w;
			if (s_EchoOff >= s_EchoLen)
			{
				s_EchoLen = 0;
				s_EchoOff = 0;
			}
		}
		else
		{
			g_IpcTestStat.Stalls++;
		}
		g_IpcTestStat.Pending = s_EchoLen - s_EchoOff;
	}
}

int main()
{
	// The UART works only once the application core has handed its pins
	// over; the records are kept either way.
	TraceInit();
	LOG("\r\nIpcIntrf test, owner, %s mode, block %08lx\r\n",
		IPCTEST_PACKET ? "packet" : "byte", (unsigned long)IPCTEST_MEM_ADDR);

	if (IpcTestOwnerInit() == false)
	{
		LOG("IpcIntrfInit failed\r\n");
		while (1)
		{
			__WFE();
		}
	}

	g_IpcTestStat.Up = 1;
	LOG("block formatted, echoing\r\n");

	// The ring is polled. A WFE in the loop could sleep through a signal that
	// came after the ring was checked and before the instruction.
	uint32_t reported = 0;
	bool bFirst = false;

	while (1)
	{
		IpcTestOwnerStep();

		if (bFirst == false && g_IpcTestStat.RxMsg != 0)
		{
			bFirst = true;
			LOG("first message from the peer\r\n");
		}
		if (g_IpcTestStat.RxMsg - reported >= 20000U)
		{
			reported = g_IpcTestStat.RxMsg;
			LOG("echo rx=%lu rxb=%lu txb=%lu stall=%lu\r\n",
				(unsigned long)g_IpcTestStat.RxMsg, (unsigned long)g_IpcTestStat.RxBytes,
				(unsigned long)g_IpcTestStat.TxBytes, (unsigned long)g_IpcTestStat.Stalls);
		}
	}

	return 0;
}

#else

/*---------------------------------------------------------------------------
 * Peer: drive and check
 *-------------------------------------------------------------------------*/


typedef struct {
	uint32_t Sent;			// Messages written
	uint32_t Echoed;		// Messages that came back whole
	uint32_t TxBytes;
	uint32_t RxBytes;
	uint32_t BadLen;		// Packet length changed (packet mode)
	uint32_t BadData;		// Byte differs from the stream
	uint32_t TxFull;		// Writes refused or cut short by a full ring
	uint32_t Stall;			// Times nothing came back for IPCTEST_STALL_MS
	uint32_t MaxInflight;	// Most messages (bytes in byte mode) outstanding
} Stat_t;

static Stat_t s_Stat;

// Length of message Seq, 1 to MSG_MAX
static inline uint32_t MsgLen(uint32_t Seq)
{
	return 1U + (Rng(Seq * 2654435761U + 12345U) % MSG_MAX);
}

// Fill buffer with the bytes of the stream that start at State
static uint32_t Fill(uint8_t *pBuf, uint32_t Len, uint32_t State)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		State = Rng(State);
		pBuf[i] = (uint8_t)(State >> 11);
	}
	return State;
}

static inline uint32_t PktSeed(uint32_t Seq)
{
	return Rng(Seq + 0x9E3779B9U) | 1U;
}

static void Led(int Idx, bool bOn)
{
	static const struct { int Port; int Pin; } leds[] = {
		{ LED1_PORT, LED1_PIN }, { LED2_PORT, LED2_PIN },
		{ LED3_PORT, LED3_PIN }, { LED4_PORT, LED4_PIN },
	};

	// The board says which level lights its LEDs
#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW	1
#endif
	if (bOn == (LED_ACTIVE_LOW == 0))
	{
		IOPinSet(leds[Idx].Port, leds[Idx].Pin);
	}
	else
	{
		IOPinClear(leds[Idx].Port, leds[Idx].Pin);
	}
}

static void LedInit(void)
{
	IOPinConfig(LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	for (int i = 0; i < 4; i++)
	{
		Led(i, false);
	}
}

static void NetCore(bool bRun)
{
#if defined(NRF_RESET_S) && !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_RESET_S->NETWORK.FORCEOFF = bRun ? RESET_NETWORK_FORCEOFF_FORCEOFF_Release
										  : RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
#else
	NRF_RESET_NS->NETWORK.FORCEOFF = bRun ? RESET_NETWORK_FORCEOFF_FORCEOFF_Release
										   : RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
#endif
}

static void Report(const char *pTag)
{
	LOG("%s sent=%lu echo=%lu txb=%lu rxb=%lu badlen=%lu baddata=%lu full=%lu stall=%lu maxin=%lu\r\n",
				  pTag, (unsigned long)s_Stat.Sent, (unsigned long)s_Stat.Echoed,
				  (unsigned long)s_Stat.TxBytes, (unsigned long)s_Stat.RxBytes,
				  (unsigned long)s_Stat.BadLen, (unsigned long)s_Stat.BadData,
				  (unsigned long)s_Stat.TxFull, (unsigned long)s_Stat.Stall,
				  (unsigned long)s_Stat.MaxInflight);
}

static inline bool HasError(void)
{
	return s_Stat.BadLen != 0U || s_Stat.BadData != 0U || s_Stat.Stall != 0U;
}

int main()
{
	LedInit();

	if (TraceInit() == false)
	{
		Led(2, true);
		while (1)
		{
			__WFE();
		}
	}

	LOG("\r\nIpcIntrf test, peer, %s mode, window %u, %lu messages\r\n",
		IPCTEST_PACKET ? "packet" : "byte", (unsigned)IPCTEST_WINDOW,
		(unsigned long)IPCTEST_COUNT);

	// The network core stays off while the block is cleared, so the owner
	// never meets leftovers of an earlier run.
	NetCore(false);
	memset(reinterpret_cast<void*>(IPCTEST_MEM_ADDR), 0, IPCTEST_MEM_SIZE);
#ifdef CORE_PINS
	if (!IOPinCoreCfg(s_CorePins, sizeof(s_CorePins) / sizeof(s_CorePins[0])))
	{
		LOG("core pin assignment failed\r\n");
		Led(2, true);
		while (1)
		{
			__WFE();
		}
	}
#endif
	LOG("block cleared, network core pins handed over\r\n");

	IpcIntrfCfg_t cfg = {};

	cfg.pMemBlk = reinterpret_cast<uint8_t*>(IPCTEST_MEM_ADDR);
	cfg.MemSize = IPCTEST_MEM_SIZE;
	cfg.bOwner = false;
	cfg.TxChan = IPCTEST_PEER_TX_CHAN;
	cfg.RxChan = IPCTEST_PEER_RX_CHAN;
	cfg.IntPrio = 6;
	cfg.EvtCB = nullptr;

	if (IpcIntrfInit(&s_Ipc, &cfg) == false)
	{
		LOG("IpcIntrfInit failed\r\n");
		Led(2, true);
		while (1)
		{
			__WFE();
		}
	}
	s_Ipc.DevIntrf.MaxRetry = 1;

	// Do not dereference network-core flash from the application core.
	// Flash availability is not guaranteed by this core's memory permissions.
	// The IPC handshake below checks whether the network image is running.

	NetCore(true);
	LOG("network core released, FORCEOFF reads %lu\r\n",
#if defined(NRF_RESET_S) && !defined(NRF_TRUSTZONE_NONSECURE)
		(unsigned long)NRF_RESET_S->NETWORK.FORCEOFF);
#else
		(unsigned long)NRF_RESET_NS->NETWORK.FORCEOFF);
#endif

	// Wait for the owner to format the block
	uint32_t waitMs = 0;

	while (IpcIntrfIsUp(&s_Ipc) == false)
	{
		msDelay(1);
		if (++waitMs >= 3000U)
		{
			LOG("link did not come up, is the network core image running?\r\n");
			Led(2, true);
			waitMs = 0;
		}
	}
	Led(0, true);
	LOG("link up\r\n");

	uint8_t txbuf[MSG_MAX];
	uint8_t rxbuf[MSG_MAX];
	uint32_t txSeq = 0;			// Next message to send
	uint32_t rxSeq = 0;			// Next message expected back
#if !IPCTEST_PACKET
	uint32_t txState = 0x1234567U;	// Byte mode stream generator
	uint32_t rxState = 0x1234567U;
	uint32_t rxPos = 0;			// Byte mode: bytes of the current message already checked
	uint32_t txOff = 0;			// Byte mode: bytes of the current message already sent
#endif
	uint32_t idleUs = 0;
	uint32_t reportUs = 0;
	uint32_t lastEchoed = 0;
	bool bDone = false;

	while (1)
	{
		bool bProgress = false;

		// Send as long as the window allows
		while (txSeq < IPCTEST_COUNT)
		{
#if IPCTEST_PACKET
			uint32_t inflight = txSeq - rxSeq;
			uint32_t len = MsgLen(txSeq);

			if (inflight >= IPCTEST_WINDOW)
			{
				break;
			}

			Fill(txbuf, len, PktSeed(txSeq));

			int w = DeviceIntrfTx(&s_Ipc.DevIntrf, 0, txbuf, len);

			if (w != (int)len)
			{
				// A packet ring takes the whole message or none of it
				s_Stat.TxFull++;
				break;
			}
			s_Stat.TxBytes += len;
			txSeq++;
			s_Stat.Sent++;
			bProgress = true;
			if (inflight + 1U > s_Stat.MaxInflight)
			{
				s_Stat.MaxInflight = inflight + 1U;
			}
#else
			// Byte stream: a message is a run of bytes, window counts bytes
			uint32_t len = MsgLen(txSeq);
			uint32_t inflightBytes = s_Stat.TxBytes - s_Stat.RxBytes;

			if (inflightBytes >= IPCTEST_WINDOW * MSG_MAX)
			{
				break;
			}

			uint32_t left = len - txOff;
			uint32_t st = txState;

			Fill(txbuf, left, st);

			int w = DeviceIntrfTx(&s_Ipc.DevIntrf, 0, txbuf, left);

			if (w <= 0)
			{
				s_Stat.TxFull++;
				break;
			}
			if ((uint32_t)w < left)
			{
				s_Stat.TxFull++;
			}
			// Advance the generator by the bytes accepted
			txState = Fill(txbuf, (uint32_t)w, st);
			s_Stat.TxBytes += (uint32_t)w;
			txOff += (uint32_t)w;
			bProgress = true;
			if (txOff >= len)
			{
				txOff = 0;
				txSeq++;
				s_Stat.Sent++;
			}
			if (inflightBytes + (uint32_t)w > s_Stat.MaxInflight)
			{
				s_Stat.MaxInflight = inflightBytes + (uint32_t)w;
			}
#endif
		}

		// Read what came back
		int n = DeviceIntrfRx(&s_Ipc.DevIntrf, 0, rxbuf, sizeof(rxbuf));

		while (n > 0)
		{
			bProgress = true;
			s_Stat.RxBytes += (uint32_t)n;
#if IPCTEST_PACKET
			uint32_t want = MsgLen(rxSeq);

			if ((uint32_t)n != want)
			{
				s_Stat.BadLen++;
			}
			else
			{
				uint8_t ref[MSG_MAX];

				Fill(ref, want, PktSeed(rxSeq));
				if (memcmp(ref, rxbuf, want) != 0)
				{
					s_Stat.BadData++;
				}
				else
				{
					s_Stat.Echoed++;
				}
			}
			rxSeq++;
#else
			{
				uint8_t ref[MSG_MAX];
				int left = n;
				const uint8_t *p = rxbuf;
				bool bBad = false;

				while (left > 0)
				{
					int c = left > (int)sizeof(ref) ? (int)sizeof(ref) : left;

					rxState = Fill(ref, (uint32_t)c, rxState);
					if (memcmp(ref, p, (size_t)c) != 0)
					{
						bBad = true;
					}
					p += c;
					left -= c;
				}
				if (bBad)
				{
					s_Stat.BadData++;
				}
				// A message is complete when its last byte is back
				rxPos += (uint32_t)n;
				while (rxSeq < txSeq && rxPos >= MsgLen(rxSeq))
				{
					rxPos -= MsgLen(rxSeq);
					rxSeq++;
					s_Stat.Echoed++;
				}
			}
#endif
			n = DeviceIntrfRx(&s_Ipc.DevIntrf, 0, rxbuf, sizeof(rxbuf));
		}

		if (bProgress)
		{
			idleUs = 0;
		}
		else
		{
			usDelay(50);
			idleUs += 50U;
			reportUs += 50U;
		}

		if (idleUs >= IPCTEST_STALL_MS * 1000U && bDone == false &&
			(txSeq != rxSeq || txSeq < IPCTEST_COUNT))
		{
			// Nothing came back although messages are outstanding
			s_Stat.Stall++;
			idleUs = 0;
			Led(2, true);
			Report("STALL");
		}

		if (reportUs >= 1000000U || (bProgress && s_Stat.Echoed - lastEchoed >= 20000U))
		{
			reportUs = 0;
			if (s_Stat.Echoed != lastEchoed)
			{
				IOPinToggle(LED2_PORT, LED2_PIN);
			}
			lastEchoed = s_Stat.Echoed;
			Report("run  ");
		}

		if (HasError())
		{
			Led(2, true);
		}

		// Every message came back, good or not
		if (bDone == false && rxSeq >= IPCTEST_COUNT)
		{
			bDone = true;
			Report("done ");
			if (HasError())
			{
				LOG("RESULT FAIL\r\n");
			}
			else
			{
				LOG("RESULT PASS\r\n");
				Led(3, true);
			}
		}
	}

	return 0;
}

#endif
