/**-------------------------------------------------------------------------
@file	ipc_intrf_test.cpp

@brief	Host tests for IpcIntrf, two cores simulated by threads.

The target port is replaced by a small fake: the signal of one core sets a
pending bit for the other, and one thread per core plays the interrupt, so the
interrupt handler, the transfer functions and the other core all run at the
same time. The shared region is one array.

Cases:
	link and configuration checks
	peer started before the owner
	small round trips, both directions, with the events
	ring wrap around
	full ring: short write, wait flag, ready event, bTxReady
	owner formatting again, seen by the peer as a new epoch
	geometry mismatch keeps the peer down
	stress: both directions at once, tiny rings, random chunk sizes, every
	byte checked in order; a lost wake up shows as a timeout

Usage:
	./ipc_intrf_test [bytes per direction in the stress case]

Build the thread sanitizer variant with the tsan target of the Makefile.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "ipc/ipc_intrf.h"

// Fake target port ----------------------------------------------------------

namespace {

const int CORE_OWNER = 0;
const int CORE_PEER = 1;
const int CORE_CNT = 2;

// Channel numbers: owner raises 0 and listens on 1, the peer the other way.
const int CHAN_OWNER_TX = 0;
const int CHAN_PEER_TX = 1;

thread_local int t_Core = CORE_OWNER;

IpcIntrfDev_t *g_pDev[CORE_CNT];
std::atomic<uint32_t> g_Pending[CORE_CNT];
std::atomic<bool> g_RxEn[CORE_CNT];
std::atomic<int> g_RxEvt[CORE_CNT];
std::atomic<int> g_TxReadyEvt[CORE_CNT];
std::atomic<int> g_SendCnt[CORE_CNT];

int CoreOf(const IpcIntrfDev_t *pDev)
{
	return pDev->bOwner ? CORE_OWNER : CORE_PEER;
}

void PortReset(void)
{
	for (int i = 0; i < CORE_CNT; i++)
	{
		g_pDev[i] = nullptr;
		g_Pending[i] = 0U;
		g_RxEn[i] = false;
		g_RxEvt[i] = 0;
		g_TxReadyEvt[i] = 0;
		g_SendCnt[i] = 0;
	}
}

}

bool IpcCtrlrInit(IpcIntrfDev_t * const pDev)
{
	if (pDev->TxChan < 0 || pDev->TxChan > 15 || pDev->RxChan < 0 || pDev->RxChan > 15)
	{
		return false;
	}

	int c = CoreOf(pDev);

	g_pDev[c] = pDev;
	g_RxEn[c] = true;

	return true;
}

void IpcCtrlrSend(int Chan)
{
	g_SendCnt[t_Core]++;
	g_Pending[1 - t_Core].fetch_or(1U << Chan, std::memory_order_release);
}

void IpcCtrlrRxEnable(IpcIntrfDev_t * const pDev, bool bEnable)
{
	g_RxEn[CoreOf(pDev)] = bEnable;
}

namespace {

// The interrupt of one core
void IrqPump(int Core)
{
	uint32_t pend = g_Pending[Core].load(std::memory_order_acquire);

	if (pend != 0U && g_RxEn[Core] && g_pDev[Core] != nullptr)
	{
		g_Pending[Core].fetch_and(~pend, std::memory_order_acq_rel);
		IpcIntrfIrqHandler(g_pDev[Core]);
	}
}

// Test harness ---------------------------------------------------------------

int g_Fail = 0;
int g_Pass = 0;

#define CHECK(Cond) \
	do { \
		if (Cond) { g_Pass++; } \
		else { g_Fail++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #Cond); } \
	} while (0)

int EvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *, int)
{
	int c = CoreOf(static_cast<IpcIntrfDev_t *>(pDev->pDevData));

	if (Evt == DEVINTRF_EVT_RX_DATA)
	{
		g_RxEvt[c]++;
	}
	else if (Evt == DEVINTRF_EVT_TX_READY)
	{
		g_TxReadyEvt[c]++;
	}

	return 0;
}

const uint32_t RING_BYTES = 64;
const uint32_t SHM_BYTES = 4096;

alignas(8) uint8_t g_Shm[SHM_BYTES];

IpcIntrfCfg_t MakeCfg(bool bOwner, uint32_t TxBytes, uint32_t RxBytes)
{
	IpcIntrfCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.pMemBlk = g_Shm;
	cfg.MemSize = SHM_BYTES;
	cfg.bOwner = bOwner;
	if (bOwner)
	{
		// the peer takes the geometry from the device data
		cfg.TxNbBlk = TxBytes;
		cfg.RxNbBlk = RxBytes;
	}
	cfg.TxChan = bOwner ? CHAN_OWNER_TX : CHAN_PEER_TX;
	cfg.RxChan = bOwner ? CHAN_PEER_TX : CHAN_OWNER_TX;
	cfg.IntPrio = 3;
	cfg.EvtCB = EvtHandler;

	return cfg;
}

// Stale RAM, as after a reset
void FillShm(uint8_t Val)
{
	memset(g_Shm, Val, sizeof(g_Shm));
}

int Tx(IpcIntrfDev_t &Dev, const uint8_t *pData, int Len)
{
	return DeviceIntrfTxData(&Dev.DevIntrf, pData, Len);
}

int Rx(IpcIntrfDev_t &Dev, uint8_t *pBuf, int Len)
{
	return DeviceIntrfRxData(&Dev.DevIntrf, pBuf, Len);
}

// Cases ----------------------------------------------------------------------

void TestConfig(void)
{
	printf("config checks\n");
	PortReset();

	IpcIntrfDev_t d;
	IpcIntrfCfg_t cfg = MakeCfg(true, RING_BYTES, RING_BYTES);

	CHECK(!IpcIntrfInit(nullptr, &cfg));
	CHECK(!IpcIntrfInit(&d, nullptr));

	IpcIntrfCfg_t bad = cfg;
	bad.pMemBlk = g_Shm + 1;
	CHECK(!IpcIntrfInit(&d, &bad));

	bad = cfg;
	bad.pMemBlk = nullptr;
	CHECK(!IpcIntrfInit(&d, &bad));

	bad = cfg;
	bad.TxNbBlk = 0;
	CHECK(!IpcIntrfInit(&d, &bad));

	bad = cfg;
	bad.RxNbBlk = 0;
	CHECK(!IpcIntrfInit(&d, &bad));

	bad = cfg;
	bad.MemSize = IPC_INTRF_MEMSIZE(RING_BYTES, 0, RING_BYTES, 0) - 4U;
	CHECK(!IpcIntrfInit(&d, &bad));

	bad = cfg;
	bad.MemSize = IPC_INTRF_MEMSIZE(RING_BYTES, 0, RING_BYTES, 0);
	CHECK(IpcIntrfInit(&d, &bad));
	CHECK(d.hTxFifo != nullptr && d.hRxFifo != nullptr);
	CHECK(CFifoAvail(d.hTxFifo) == (int)RING_BYTES);
	CHECK(CFifoAvail(d.hRxFifo) == (int)RING_BYTES);

	// a peer needs at least the device data
	bad = MakeCfg(false, 0, 0);
	bad.MemSize = 8;
	CHECK(!IpcIntrfInit(&d, &bad));

	PortReset();
	bad = cfg;
	bad.TxChan = 16;
	CHECK(!IpcIntrfInit(&d, &bad));
}

void TestPeerFirst(void)
{
	printf("peer started before the owner\n");
	PortReset();
	FillShm(0xFF);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	uint8_t b[16] = { 1, 2, 3 };

	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));
	CHECK(!IpcIntrfIsUp(&peer));
	CHECK(Tx(peer, b, 3) == 0);
	CHECK(Rx(peer, b, 3) == 0);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	CHECK(IpcIntrfIsUp(&own));
	CHECK(g_SendCnt[CORE_OWNER] == 1);
	CHECK((g_Pending[CORE_PEER] & (1U << CHAN_OWNER_TX)) != 0U);

	CHECK(IpcIntrfIsUp(&peer));
	CHECK(Tx(peer, b, 3) == 3);
}

void TestRoundTrip(void)
{
	printf("round trip\n");
	PortReset();
	FillShm(0xA5);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));
	CHECK(IpcIntrfIsUp(&peer));

	uint8_t out[32], in[64];
	for (int i = 0; i < 32; i++)
	{
		out[i] = (uint8_t)(i * 7 + 1);
	}

	// owner to peer
	t_Core = CORE_OWNER;
	CHECK(Tx(own, out, 20) == 20);
	g_Pending[CORE_OWNER] = 0U;
	t_Core = CORE_PEER;
	IrqPump(CORE_PEER);
	CHECK(g_RxEvt[CORE_PEER] == 1);
	memset(in, 0, sizeof(in));
	CHECK(Rx(peer, in, 64) == 20);
	CHECK(memcmp(in, out, 20) == 0);
	CHECK(Rx(peer, in, 64) == 0);

	// peer to owner
	CHECK(Tx(peer, out, 32) == 32);
	t_Core = CORE_OWNER;
	IrqPump(CORE_OWNER);
	CHECK(g_RxEvt[CORE_OWNER] >= 1);
	memset(in, 0, sizeof(in));
	CHECK(Rx(own, in, 10) == 10);
	CHECK(Rx(own, in + 10, 54) == 22);
	CHECK(memcmp(in, out, 32) == 0);

	// rings are independent: nothing leaked into the other direction
	CHECK(Rx(own, in, 64) == 0);
	CHECK(Rx(peer, in, 64) == 0);

	// zero and bad arguments
	CHECK(Tx(own, out, 0) == 0);
	CHECK(Tx(own, nullptr, 4) == 0);
	CHECK(Rx(own, in, 0) == 0);
	CHECK(Rx(own, nullptr, 4) == 0);
}

void TestWrap(void)
{
	printf("wrap around\n");
	PortReset();
	FillShm(0x00);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	uint8_t out[RING_BYTES], in[RING_BYTES];
	uint32_t seq = 0;
	bool ok = true;

	for (int round = 0; round < 500; round++)
	{
		int len = 1 + (round * 13) % (int)RING_BYTES;

		for (int i = 0; i < len; i++)
		{
			out[i] = (uint8_t)(seq + (uint32_t)i);
		}

		t_Core = CORE_OWNER;
		if (Tx(own, out, len) != len)
		{
			ok = false;
		}
		t_Core = CORE_PEER;
		memset(in, 0, sizeof(in));
		if (Rx(peer, in, (int)RING_BYTES) != len || memcmp(in, out, (size_t)len) != 0)
		{
			ok = false;
		}
		seq += (uint32_t)len;
	}

	CHECK(ok);
}

void TestFull(void)
{
	printf("full ring, wait flag, ready event\n");
	PortReset();
	FillShm(0x5A);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	uint8_t out[200], in[200];
	for (int i = 0; i < 200; i++)
	{
		out[i] = (uint8_t)i;
	}

	int cap = (int)oc.TxNbBlk;

	t_Core = CORE_OWNER;
	int n = Tx(own, out, 200);
	CHECK(n == cap);
	CHECK(!own.DevIntrf.bTxReady);
	CHECK(own.pHdr->Wait[0] == 1U);

	// a second write finds no room at all
	CHECK(Tx(own, out + n, 10) == 0);

	// the consumer frees room: the producer is told, it is not told twice
	t_Core = CORE_PEER;
	int sends = g_SendCnt[CORE_PEER];
	CHECK(Rx(peer, in, 10) == 10);
	CHECK(g_SendCnt[CORE_PEER] == sends + 1);

	t_Core = CORE_OWNER;
	g_TxReadyEvt[CORE_OWNER] = 0;
	IrqPump(CORE_OWNER);
	CHECK(g_TxReadyEvt[CORE_OWNER] == 1);
	CHECK(own.DevIntrf.bTxReady);
	CHECK(own.pHdr->Wait[0] == 0U);

	// no wait flag: a read does not signal
	t_Core = CORE_PEER;
	sends = g_SendCnt[CORE_PEER];
	CHECK(Rx(peer, in + 10, 50) == 50);
	CHECK(g_SendCnt[CORE_PEER] == sends);

	// everything read is the stream in order
	CHECK(memcmp(in, out, 60) == 0);

	// the rest goes in
	t_Core = CORE_OWNER;
	n = Tx(own, out + cap, 200 - cap);
	CHECK(n > 0);
}

void TestEpoch(void)
{
	printf("owner formats again\n");
	PortReset();
	FillShm(0x33);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	uint8_t b[32] = { 9, 8, 7, 6, 5, 4, 3, 2, 1 };
	t_Core = CORE_OWNER;
	CHECK(Tx(own, b, 9) == 9);
	uint32_t epoch = own.pHdr->Epoch;

	// formatting again drops what was in the rings
	DeviceIntrfReset(&own.DevIntrf);
	CHECK(own.pHdr->Epoch == epoch + 1U);
	t_Core = CORE_PEER;
	CHECK(IpcIntrfIsUp(&peer));
	CHECK(peer.Epoch == own.pHdr->Epoch);
	CHECK(Rx(peer, b, 32) == 0);

	// and the link works afterwards, both ways
	t_Core = CORE_OWNER;
	CHECK(Tx(own, b, 4) == 4);
	t_Core = CORE_PEER;
	uint8_t in[8];
	CHECK(Rx(peer, in, 8) == 4);
	CHECK(Tx(peer, b, 5) == 5);
	t_Core = CORE_OWNER;
	CHECK(Rx(own, in, 8) == 5);

	// owner down: the peer sees the link down
	__atomic_store_n(&own.pHdr->Magic, 0U, __ATOMIC_RELEASE);
	t_Core = CORE_PEER;
	CHECK(!IpcIntrfIsUp(&peer));
	CHECK(Tx(peer, b, 5) == 0);
	CHECK(Rx(peer, in, 8) == 0);
}

void TestGeometry(void)
{
	printf("peer takes the geometry from the device data\n");
	PortReset();
	FillShm(0x77);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, 32);
	IpcIntrfCfg_t pc = MakeCfg(false, 0, 0);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));
	CHECK(IpcIntrfIsUp(&peer));

	// unequal rings: each takes bytes at its own size, the peer sees them swapped
	uint8_t out[128];
	memset(out, 0xC3, sizeof(out));
	t_Core = CORE_OWNER;
	CHECK(Tx(own, out, 128) == (int)RING_BYTES);
	t_Core = CORE_PEER;
	CHECK(Tx(peer, out, 128) == 32);
	CHECK(IpcIntrfTxRoom(&peer) == 0);

	// the device data and both rings sit in the owner block, in order
	CHECK((uint8_t *)own.hTxFifo == g_Shm + sizeof(IpcIntrfHdr_t));
	CHECK((uint8_t *)peer.hRxFifo == (uint8_t *)own.hTxFifo);
	CHECK((uint8_t *)peer.hTxFifo == (uint8_t *)own.hRxFifo);
	CHECK((uint8_t *)own.hRxFifo ==
		  g_Shm + sizeof(IpcIntrfHdr_t) + IPC_INTRF_RING_MEMSIZE(RING_BYTES, 0));

	// a peer whose block is smaller than the owner geometry stays down
	PortReset();
	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	IpcIntrfCfg_t small = pc;
	small.MemSize = IPC_INTRF_MEMSIZE(RING_BYTES, 0, 32, 0) - 4U;
	CHECK(IpcIntrfInit(&peer, &small));
	CHECK(!IpcIntrfIsUp(&peer));

	// a layout version it does not know stays down
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));
	CHECK(IpcIntrfIsUp(&peer));
	t_Core = CORE_OWNER;
	own.pHdr->Version = IPC_INTRF_VERSION + 1U;
	DeviceIntrfReset(&own.DevIntrf);	// formats again, version restored
	t_Core = CORE_PEER;
	CHECK(IpcIntrfIsUp(&peer));
	__atomic_store_n(&own.pHdr->Epoch, own.pHdr->Epoch + 1U, __ATOMIC_RELAXED);
	own.pHdr->Version = IPC_INTRF_VERSION + 1U;
	CHECK(!IpcIntrfIsUp(&peer));

	// ring offsets outside the block are refused
	own.pHdr->Version = IPC_INTRF_VERSION;
	own.pHdr->Ring[1].Off = SHM_BYTES;
	CHECK(!IpcIntrfIsUp(&peer));
}

// Stress ---------------------------------------------------------------------

uint8_t Prbs(uint64_t i)
{
	uint64_t z = i + 0x9E3779B97F4A7C15ULL;

	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
	z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
	z = z ^ (z >> 31);

	return (uint8_t)(z >> 24);
}

uint32_t Rng(uint32_t &s)
{
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5;

	return s;
}

struct Stream {
	IpcIntrfDev_t *pTx;
	IpcIntrfDev_t *pRx;
	int TxCore;
	int RxCore;
	uint64_t Total;
	std::atomic<uint64_t> Sent;
	std::atomic<uint64_t> Rcvd;
	std::atomic<int> Err;
	std::atomic<int> LostWake;
	std::atomic<uint64_t> Short;
};

void TxThread(Stream *pS, uint32_t Seed)
{
	t_Core = pS->TxCore;
	uint32_t rng = Seed;
	std::vector<uint8_t> buf(700);
	uint64_t sent = 0;

	while (sent < pS->Total)
	{
		int len = 1 + (int)(Rng(rng) % 700U);

		if ((uint64_t)len > pS->Total - sent)
		{
			len = (int)(pS->Total - sent);
		}
		for (int i = 0; i < len; i++)
		{
			buf[(size_t)i] = Prbs(sent + (uint64_t)i);
		}

		int n = DeviceIntrfTxData(&pS->pTx->DevIntrf, buf.data(), len);

		if (n < 0 || n > len)
		{
			pS->Err++;
			return;
		}

		sent += (uint64_t)n;
		pS->Sent = sent;

		if (n < len)
		{
			pS->Short++;

			// wait for the ready event of the interrupt
			auto t0 = std::chrono::steady_clock::now();

			while (!pS->pTx->DevIntrf.bTxReady)
			{
				if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10))
				{
					pS->LostWake++;
					return;
				}
				std::this_thread::yield();
			}
		}
	}
}

void RxThread(Stream *pS, uint32_t Seed)
{
	t_Core = pS->RxCore;
	uint32_t rng = Seed;
	std::vector<uint8_t> buf(512);
	uint64_t rcvd = 0;
	auto last = std::chrono::steady_clock::now();

	while (rcvd < pS->Total)
	{
		int want = 1 + (int)(Rng(rng) % 500U);
		int n = DeviceIntrfRxData(&pS->pRx->DevIntrf, buf.data(), want);

		if (n < 0 || n > want)
		{
			pS->Err++;
			return;
		}

		if (n == 0)
		{
			if (std::chrono::steady_clock::now() - last > std::chrono::seconds(20))
			{
				pS->LostWake++;
				return;
			}
			std::this_thread::yield();
			continue;
		}

		last = std::chrono::steady_clock::now();

		for (int i = 0; i < n; i++)
		{
			if (buf[(size_t)i] != Prbs(rcvd + (uint64_t)i))
			{
				pS->Err++;
				return;
			}
		}

		rcvd += (uint64_t)n;
		pS->Rcvd = rcvd;
	}
}

std::atomic<bool> g_Run;

void IrqThread(int Core)
{
	t_Core = Core;

	while (g_Run)
	{
		IrqPump(Core);
		std::this_thread::yield();
	}
}

void TestStress(uint64_t Bytes)
{
	printf("stress, %llu bytes each way, ring 48 and 80 bytes\n", (unsigned long long)Bytes);
	PortReset();
	FillShm(0xE7);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, 48, 80);
	IpcIntrfCfg_t pc = MakeCfg(false, 80, 48);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	Stream a2p, p2a;

	a2p.pTx = &own;
	a2p.pRx = &peer;
	a2p.TxCore = CORE_OWNER;
	a2p.RxCore = CORE_PEER;
	a2p.Total = Bytes;
	a2p.Sent = 0;
	a2p.Rcvd = 0;
	a2p.Err = 0;
	a2p.LostWake = 0;
	a2p.Short = 0;

	p2a.pTx = &peer;
	p2a.pRx = &own;
	p2a.TxCore = CORE_PEER;
	p2a.RxCore = CORE_OWNER;
	p2a.Total = Bytes;
	p2a.Sent = 0;
	p2a.Rcvd = 0;
	p2a.Err = 0;
	p2a.LostWake = 0;
	p2a.Short = 0;

	g_Run = true;

	std::thread irqO(IrqThread, CORE_OWNER);
	std::thread irqP(IrqThread, CORE_PEER);
	std::thread tx1(TxThread, &a2p, 0x12345678U);
	std::thread rx1(RxThread, &a2p, 0x9ABCDEF1U);
	std::thread tx2(TxThread, &p2a, 0x0BADF00DU);
	std::thread rx2(RxThread, &p2a, 0xC0FFEE11U);

	auto t0 = std::chrono::steady_clock::now();

	tx1.join();
	rx1.join();
	tx2.join();
	rx2.join();
	g_Run = false;
	irqO.join();
	irqP.join();

	double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

	printf("  owner to peer: sent %llu rcvd %llu short writes %llu\n",
		   (unsigned long long)a2p.Sent.load(), (unsigned long long)a2p.Rcvd.load(),
		   (unsigned long long)a2p.Short.load());
	printf("  peer to owner: sent %llu rcvd %llu short writes %llu\n",
		   (unsigned long long)p2a.Sent.load(), (unsigned long long)p2a.Rcvd.load(),
		   (unsigned long long)p2a.Short.load());
	printf("  ready events %d %d, receive events %d %d, %.2f s\n",
		   g_TxReadyEvt[CORE_OWNER].load(), g_TxReadyEvt[CORE_PEER].load(),
		   g_RxEvt[CORE_OWNER].load(), g_RxEvt[CORE_PEER].load(), sec);

	CHECK(a2p.Err == 0);
	CHECK(p2a.Err == 0);
	CHECK(a2p.LostWake == 0);
	CHECK(p2a.LostWake == 0);
	CHECK(a2p.Rcvd == Bytes);
	CHECK(p2a.Rcvd == Bytes);
	CHECK(a2p.Short > 0);
	CHECK(p2a.Short > 0);
}

}

// Packet stream -----------------------------------------------------------

IpcIntrfCfg_t MakePktCfg(bool bOwner, uint32_t TxBlk, uint32_t TxNb, uint32_t RxBlk, uint32_t RxNb)
{
	IpcIntrfCfg_t cfg = MakeCfg(bOwner, TxNb, RxNb);

	if (bOwner)
	{
		cfg.TxBlkSize = TxBlk;
		cfg.RxBlkSize = RxBlk;
	}

	return cfg;
}

void TestPktConfig(void)
{
	printf("packet stream config checks\n");
	PortReset();

	IpcIntrfDev_t d;
	IpcIntrfCfg_t cfg = MakePktCfg(true, 32, 4, 32, 4);

	CHECK(IpcIntrfInit(&d, &cfg));
	CHECK(d.TxBlk == 32U && d.RxBlk == 32U);

	IpcIntrfCfg_t bad = cfg;
	bad.TxBlkSize = 2;
	CHECK(!IpcIntrfInit(&d, &bad));
	bad = cfg;
	bad.RxBlkSize = IPC_INTRF_PKT_BLK_MAX + 1U;
	CHECK(!IpcIntrfInit(&d, &bad));

	// block too small for the slots
	bad = cfg;
	bad.MemSize = IPC_INTRF_MEMSIZE(4, 32, 4, 32) - 4U;
	CHECK(!IpcIntrfInit(&d, &bad));
	bad.MemSize = IPC_INTRF_MEMSIZE(4, 32, 4, 32);
	CHECK(IpcIntrfInit(&d, &bad));
	CHECK(CFifoAvail(d.hTxFifo) == 4);

	// block size 0 and 1 are both a byte stream
	PortReset();
	bad = MakeCfg(true, 64, 64);
	bad.TxBlkSize = 1;
	CHECK(IpcIntrfInit(&d, &bad));
	CHECK(d.TxBlk == 1U && d.RxBlk == 1U);
}

void TestPktBoundaries(void)
{
	printf("packet boundaries, mixed with a byte stream\n");
	PortReset();
	FillShm(0xC1);

	IpcIntrfDev_t own, peer;
	// owner to peer: packets of 32 byte slots. peer to owner: bytes
	IpcIntrfCfg_t oc = MakePktCfg(true, 32, 4, 1, 64);
	IpcIntrfCfg_t pc = MakePktCfg(false, 1, 64, 32, 4);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));
	CHECK(IpcIntrfIsUp(&peer));

	uint8_t out[64], in[64];
	for (int i = 0; i < 64; i++)
	{
		out[i] = (uint8_t)(i + 1);
	}

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfTxRoom(&own) == 4);
	CHECK(Tx(own, out, 5) == 5);
	CHECK(Tx(own, out + 5, 30) == 30);		// the largest packet of the slot
	CHECK(Tx(own, out + 40, 1) == 1);
	CHECK(IpcIntrfTxRoom(&own) == 1);

	t_Core = CORE_PEER;
	IrqPump(CORE_PEER);
	CHECK(g_RxEvt[CORE_PEER] >= 1);

	// one read is one packet, whatever the buffer size
	memset(in, 0, sizeof(in));
	CHECK(Rx(peer, in, 64) == 5);
	CHECK(memcmp(in, out, 5) == 0);
	CHECK(Rx(peer, in, 64) == 30);
	CHECK(memcmp(in, out + 5, 30) == 0);
	CHECK(Rx(peer, in, 64) == 1);
	CHECK(in[0] == out[40]);
	CHECK(Rx(peer, in, 64) == 0);

	// the byte direction still merges
	CHECK(Tx(peer, out, 10) == 10);
	CHECK(Tx(peer, out + 10, 10) == 10);
	t_Core = CORE_OWNER;
	CHECK(Rx(own, in, 64) == 20);
	CHECK(memcmp(in, out, 20) == 0);

	// the peer was given no sizes: it took the modes from the device data
	CHECK(peer.RxBlk == 32U && peer.TxBlk == 1U);
	CHECK(own.TxBlk == 32U && own.RxBlk == 1U);
}

void TestPktFull(void)
{
	printf("packet stream full ring, oversize, small buffer\n");
	PortReset();
	FillShm(0x3C);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakePktCfg(true, 16, 3, 16, 3);
	IpcIntrfCfg_t pc = MakePktCfg(false, 16, 3, 16, 3);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	uint8_t out[32], in[32];
	for (int i = 0; i < 32; i++)
	{
		out[i] = (uint8_t)(0x40 + i);
	}

	// ring memory holds a whole number of slots, count what it took
	int slots = IpcIntrfTxRoom(&own);
	CHECK(slots >= 3);

	t_Core = CORE_OWNER;

	// a packet larger than the slot is refused, not cut
	CHECK(Tx(own, out, 15) == 0);
	CHECK(own.TxOversizeCnt == 1U);
	CHECK(IpcIntrfTxRoom(&own) == slots);

	for (int i = 0; i < slots; i++)
	{
		out[0] = (uint8_t)i;
		CHECK(Tx(own, out, 14) == 14);
	}
	CHECK(own.DevIntrf.bTxReady);
	CHECK(IpcIntrfTxRoom(&own) == 0);

	// a full ring takes nothing, whole or in part, and asks for a signal
	CHECK(Tx(own, out, 1) == 0);
	CHECK(!own.DevIntrf.bTxReady);
	CHECK(own.pHdr->Wait[0] == 1U);

	// a read buffer that is too small drops that packet and frees the slot
	t_Core = CORE_PEER;
	int sends = g_SendCnt[CORE_PEER];
	CHECK(Rx(peer, in, 8) == 0);
	CHECK(peer.RxDropCnt == 1U);
	CHECK(g_SendCnt[CORE_PEER] == sends + 1);

	t_Core = CORE_OWNER;
	g_TxReadyEvt[CORE_OWNER] = 0;
	IrqPump(CORE_OWNER);
	CHECK(g_TxReadyEvt[CORE_OWNER] == 1);
	CHECK(own.DevIntrf.bTxReady);
	CHECK(own.pHdr->Wait[0] == 0U);

	// the rest arrive whole and in order
	t_Core = CORE_PEER;
	for (int i = 1; i < slots; i++)
	{
		CHECK(Rx(peer, in, 32) == 14);
		CHECK(in[0] == (uint8_t)i);
	}
	CHECK(Rx(peer, in, 32) == 0);

	// room again: a one byte packet is a packet
	t_Core = CORE_OWNER;
	CHECK(Tx(own, out, 1) == 1);
	t_Core = CORE_PEER;
	CHECK(Rx(peer, in, 32) == 1);
}

struct PktStream {
	IpcIntrfDev_t *pTx;
	IpcIntrfDev_t *pRx;
	int TxCore;
	int RxCore;
	uint32_t MaxPayload;
	uint64_t Count;
	std::atomic<uint64_t> Sent;
	std::atomic<uint64_t> Rcvd;
	std::atomic<uint64_t> Short;
	std::atomic<int> Err;
	std::atomic<int> LostWake;
};

void FillPkt(uint8_t *pBuf, uint32_t Len, uint32_t Seq)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		pBuf[i] = (uint8_t)(Seq * 31U + i * 7U + Len);
	}
	if (Len >= 4U)
	{
		memcpy(pBuf, &Seq, 4);
	}
}

void PktTxThread(PktStream *pS, uint32_t Seed)
{
	t_Core = pS->TxCore;
	uint32_t rng = Seed;
	std::vector<uint8_t> buf(pS->MaxPayload);

	for (uint64_t seq = 0; seq < pS->Count; seq++)
	{
		uint32_t len = 4U + Rng(rng) % (pS->MaxPayload - 3U);

		FillPkt(buf.data(), len, (uint32_t)seq);

		for (;;)
		{
			int n = DeviceIntrfTxData(&pS->pTx->DevIntrf, buf.data(), (int)len);

			if (n == (int)len)
			{
				break;
			}
			if (n != 0)
			{
				pS->Err++;
				return;
			}

			pS->Short++;
			auto t0 = std::chrono::steady_clock::now();

			while (!pS->pTx->DevIntrf.bTxReady)
			{
				if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10))
				{
					pS->LostWake++;
					return;
				}
				std::this_thread::yield();
			}
		}

		pS->Sent = seq + 1U;
	}
}

void PktRxThread(PktStream *pS)
{
	t_Core = pS->RxCore;
	std::vector<uint8_t> buf(pS->MaxPayload), ref(pS->MaxPayload);
	auto last = std::chrono::steady_clock::now();

	for (uint64_t seq = 0; seq < pS->Count;)
	{
		int n = DeviceIntrfRxData(&pS->pRx->DevIntrf, buf.data(), (int)buf.size());

		if (n == 0)
		{
			if (std::chrono::steady_clock::now() - last > std::chrono::seconds(20))
			{
				pS->LostWake++;
				return;
			}
			std::this_thread::yield();
			continue;
		}
		last = std::chrono::steady_clock::now();

		if (n < 4 || n > (int)pS->MaxPayload)
		{
			pS->Err++;
			return;
		}

		// the packet must be one whole packet: header sequence and the fill
		// of its own length
		FillPkt(ref.data(), (uint32_t)n, (uint32_t)seq);
		if (memcmp(ref.data(), buf.data(), (size_t)n) != 0)
		{
			pS->Err++;
			return;
		}

		seq++;
		pS->Rcvd = seq;
	}
}

void TestPktStress(uint64_t Count)
{
	printf("packet stress, %llu packets each way, 3 slot rings\n", (unsigned long long)Count);
	PortReset();
	FillShm(0x9D);

	IpcIntrfDev_t own, peer;
	IpcIntrfCfg_t oc = MakePktCfg(true, 66, 3, 130, 3);
	IpcIntrfCfg_t pc = MakePktCfg(false, 130, 3, 66, 3);

	t_Core = CORE_OWNER;
	CHECK(IpcIntrfInit(&own, &oc));
	t_Core = CORE_PEER;
	CHECK(IpcIntrfInit(&peer, &pc));

	PktStream a2p, p2a;

	a2p.pTx = &own;
	a2p.pRx = &peer;
	a2p.TxCore = CORE_OWNER;
	a2p.RxCore = CORE_PEER;
	a2p.MaxPayload = 64U;
	a2p.Count = Count;
	a2p.Sent = a2p.Rcvd = a2p.Short = 0;
	a2p.Err = a2p.LostWake = 0;

	p2a.pTx = &peer;
	p2a.pRx = &own;
	p2a.TxCore = CORE_PEER;
	p2a.RxCore = CORE_OWNER;
	p2a.MaxPayload = 128U;
	p2a.Count = Count;
	p2a.Sent = p2a.Rcvd = p2a.Short = 0;
	p2a.Err = p2a.LostWake = 0;

	g_Run = true;

	std::thread irqO(IrqThread, CORE_OWNER);
	std::thread irqP(IrqThread, CORE_PEER);
	std::thread tx1(PktTxThread, &a2p, 0x2468ACE0U);
	std::thread rx1(PktRxThread, &a2p);
	std::thread tx2(PktTxThread, &p2a, 0x13579BDFU);
	std::thread rx2(PktRxThread, &p2a);

	tx1.join();
	rx1.join();
	tx2.join();
	rx2.join();
	g_Run = false;
	irqO.join();
	irqP.join();

	printf("  owner to peer: sent %llu rcvd %llu full %llu\n",
		   (unsigned long long)a2p.Sent.load(), (unsigned long long)a2p.Rcvd.load(),
		   (unsigned long long)a2p.Short.load());
	printf("  peer to owner: sent %llu rcvd %llu full %llu\n",
		   (unsigned long long)p2a.Sent.load(), (unsigned long long)p2a.Rcvd.load(),
		   (unsigned long long)p2a.Short.load());

	CHECK(a2p.Err == 0);
	CHECK(p2a.Err == 0);
	CHECK(a2p.LostWake == 0);
	CHECK(p2a.LostWake == 0);
	CHECK(a2p.Rcvd == Count);
	CHECK(p2a.Rcvd == Count);
	CHECK(a2p.Short > 0);
	CHECK(p2a.Short > 0);
}

// The C++ class on top of the C interface, with the framework start and stop
void TestClass(void)
{
	printf("class and framework path\n");
	PortReset();
	FillShm(0x11);

	IpcIntrf own, peer;
	IpcIntrfCfg_t oc = MakeCfg(true, RING_BYTES, RING_BYTES);
	IpcIntrfCfg_t pc = MakeCfg(false, RING_BYTES, RING_BYTES);

	t_Core = CORE_OWNER;
	CHECK(own.Init(oc));
	CHECK(own.Up());
	t_Core = CORE_PEER;
	CHECK(peer.Init(pc));
	CHECK(peer.Up());

	uint8_t out[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }, in[10];

	t_Core = CORE_OWNER;
	CHECK(own.Tx(0, out, 10) == 10);
	t_Core = CORE_PEER;
	CHECK(peer.Rx(0, in, 10) == 10);
	CHECK(memcmp(in, out, 10) == 0);
	CHECK(own.Rate() == 0);
	CHECK(own.Rate(1000) == 0);

	// the framework busy flag is released by the transfer
	CHECK(own.StartTx(0));
	CHECK(!own.StartTx(0));
	own.StopTx();
	CHECK(own.StartTx(0));
	own.StopTx();
}

int main(int argc, char **argv)
{
	uint64_t bytes = 4ULL * 1024ULL * 1024ULL;

	if (argc > 1)
	{
		bytes = strtoull(argv[1], nullptr, 0);
	}

	TestConfig();
	TestPeerFirst();
	TestRoundTrip();
	TestWrap();
	TestFull();
	TestEpoch();
	TestGeometry();
	TestClass();
	TestPktConfig();
	TestPktBoundaries();
	TestPktFull();
	TestStress(bytes);
	TestPktStress(bytes / 64U);

	printf("\n%d passed, %d failed\n", g_Pass, g_Fail);

	return g_Fail == 0 ? 0 : 1;
}
