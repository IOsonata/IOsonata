/**-------------------------------------------------------------------------
@file	bt_hci_h4_ipc_test.cpp

@brief	BtHciH4 over IpcIntrf host test

Two H4 framing layers, a host and a controller, each on one side of an
IpcIntrf pair in one shared region. The IPC port is replaced by a
synchronous one: raising a channel runs the receive work of the other side at
once. The race coverage of the rings is in tests/ipc.

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
#include <cstdint>
#include <cstring>
#include <vector>

#include "bluetooth/bt_hci_h4.h"
#include "bluetooth/bt_hcievt.h"
#include "ipc/ipc_intrf.h"
#include "syslog.h"
#include "bt_test_harness.h"

#include "ipc_sync_port.h"

namespace {

bttest::Context s_Ctx("bt_hci_h4_ipc_test");

#define CHECK(expr)		BT_CHECK(s_Ctx, expr)

#define RING_BYTES			1024U
#define SHMEM_SIZE			IPC_INTRF_MEMSIZE(RING_BYTES, 0, RING_BYTES, 0)
#define PKT_QUEUE			4U

struct RxPkt {
	uint8_t Type;
	std::vector<uint8_t> Data;
};

// One side of the link: the interface, its framing layer and what it received
struct Side {
	IpcIntrfDev_t Ipc;			// first member, the event handler finds the side from it
	BtHciH4Dev_t H4;
	alignas(8) uint8_t PktMem[BT_HCI_H4_PKTMEM_SIZE(PKT_QUEUE)];
	std::vector<RxPkt> Rx;
	int ReadyCnt;
	bool bAutoProcess;			// frame and dispatch as soon as bytes arrive
	bool bRespond;				// answer each command with a Command Complete
	uint8_t RespStatus;
};

alignas(8) uint8_t s_ShMem[SHMEM_SIZE];
Side s_Host;
Side s_Ctlr;
BtHciDevice_t s_Hci;

void RxHandler(BtHciH4Dev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	Side *side = (Side*)pDev->pCtx;

	side->Rx.push_back({Type, std::vector<uint8_t>(pPkt, pPkt + Len)});

	if (side->bRespond && Type == BT_HCI_H4_PKT_CMD && Len >= 3)
	{
		// Command Complete: event 0x0E, parameters are credit, opcode, status
		// and two bytes of return data
		uint8_t evt[2 + 6] = {BT_HCI_EVT_COMMAND_COMPLETE, 6, 1, pPkt[0], pPkt[1],
							  side->RespStatus, 0xA5, 0x5A};

		BtHciH4Send(pDev, BT_HCI_H4_PKT_EVT, evt, sizeof(evt));
	}
}

void CmdRspHandler(BtHciH4Dev_t * const, uint8_t, uint8_t *pPkt, uint16_t)
{
	BtHciProcessEvent(&s_Hci, (BtHciEvtPacket_t*)pPkt);
}

void RxReady(BtHciH4Dev_t * const pDev)
{
	Side *side = (Side*)pDev->pCtx;

	side->ReadyCnt++;
	if (side->bAutoProcess)
	{
		BtHciH4Process(pDev, 0);
	}
}

int TxRoom(BtHciH4Dev_t * const pDev)
{
	return IpcIntrfTxRoom(&((Side*)pDev->pCtx)->Ipc);
}

int IpcEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *, int)
{
	Side *side = (Side*)pDev->pDevData;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		BtHciH4RxNotify(&side->H4);
	}

	return 0;
}

bool InitSide(Side &S, bool bOwner, uint8_t *pMem, uint32_t MemSize, bool bCmdRsp)
{
	S.Rx.clear();
	S.ReadyCnt = 0;
	S.bAutoProcess = false;
	S.bRespond = false;
	S.RespStatus = 0;

	IpcIntrfCfg_t icfg = {};
	icfg.pMemBlk = s_ShMem;
	icfg.MemSize = sizeof(s_ShMem);
	icfg.TxNbBlk = RING_BYTES;
	icfg.RxNbBlk = RING_BYTES;
	icfg.bOwner = bOwner;
	icfg.TxChan = bOwner ? 0 : 1;
	icfg.RxChan = bOwner ? 1 : 0;
	icfg.IntPrio = 3;
	icfg.EvtCB = IpcEvt;

	if (IpcIntrfInit(&S.Ipc, &icfg) == false)
	{
		return false;
	}

	// An empty read ends the polling loop at once
	S.Ipc.DevIntrf.MaxRetry = 1;

	BtHciH4Cfg_t hcfg = {};
	hcfg.pPhyIntrf = &S.Ipc.DevIntrf;
	hcfg.pPktMem = pMem;
	hcfg.PktMemSize = MemSize;
	hcfg.RxHandler = RxHandler;
	hcfg.CmdRspHandler = bCmdRsp ? CmdRspHandler : nullptr;
	hcfg.RxReady = RxReady;
	hcfg.TxRoom = TxRoom;
	hcfg.TimeoutMs = 1;
	hcfg.pCtx = &S;

	return BtHciH4Init(&S.H4, &hcfg);
}

// Owner is the host, the peer is the controller
void Start()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));
	std::memset((void*)&s_Hci, 0, sizeof(s_Hci));
	s_Hci.CmdCredit = 1;

	CHECK(InitSide(s_Host, true, s_Host.PktMem, sizeof(s_Host.PktMem), true));
	CHECK(InitSide(s_Ctlr, false, s_Ctlr.PktMem, sizeof(s_Ctlr.PktMem), false));
	CHECK(IpcIntrfIsUp(&s_Ctlr.Ipc));
	CHECK(IpcIntrfIsUp(&s_Host.Ipc));
}

// Packet content that tells its position and type apart
std::vector<uint8_t> Pattern(uint8_t Type, uint32_t Len, uint32_t Seed)
{
	std::vector<uint8_t> v(Len);

	for (uint32_t i = 0; i < Len; i++)
	{
		v[i] = (uint8_t)(Seed * 31U + i * 7U + Type);
	}

	// The first bytes are the header, so the length field has to agree
	switch (Type)
	{
		case BT_HCI_H4_PKT_CMD:
			if (Len >= 3) { v[2] = (uint8_t)(Len - 3); }
			break;
		case BT_HCI_H4_PKT_EVT:
			// Not a Command Complete or Command Status, those do not reach the queue
			v[0] = 0x05;
			if (Len >= 2) { v[1] = (uint8_t)(Len - 2); }
			break;
		case BT_HCI_H4_PKT_ACL:
			if (Len >= 4) { v[2] = (uint8_t)((Len - 4) & 0xFF); v[3] = (uint8_t)((Len - 4) >> 8); }
			break;
		case BT_HCI_H4_PKT_ISO:
			if (Len >= 4) { v[2] = (uint8_t)((Len - 4) & 0xFF); v[3] = (uint8_t)(((Len - 4) >> 8) & 0x3F); }
			break;
		default:
			if (Len >= 3) { v[2] = (uint8_t)(Len - 3); }
			break;
	}

	return v;
}

uint32_t HdrLen(uint8_t Type)
{
	switch (Type)
	{
		case BT_HCI_H4_PKT_CMD: return 3;
		case BT_HCI_H4_PKT_ACL: return 4;
		case BT_HCI_H4_PKT_SCO: return 3;
		case BT_HCI_H4_PKT_EVT: return 2;
		default: return 4;
	}
}

void TestConfig()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));
	CHECK(InitSide(s_Host, true, s_Host.PktMem, sizeof(s_Host.PktMem), false));

	BtHciH4Cfg_t cfg = {};
	BtHciH4Dev_t h4;

	cfg.pPhyIntrf = &s_Host.Ipc.DevIntrf;
	cfg.pPktMem = s_Host.PktMem;
	cfg.PktMemSize = sizeof(s_Host.PktMem);
	cfg.TxRoom = TxRoom;
	CHECK(BtHciH4Init(&h4, &cfg));
	CHECK(h4.TimeoutMs == BT_HCI_H4_TIMEOUT_DEFAULT);

	BtHciH4Cfg_t bad = cfg;
	bad.pPhyIntrf = nullptr;
	CHECK(BtHciH4Init(&h4, &bad) == false);
	bad = cfg;
	bad.TxRoom = nullptr;
	CHECK(BtHciH4Init(&h4, &bad) == false);
	bad = cfg;
	bad.pPktMem = nullptr;
	CHECK(BtHciH4Init(&h4, &bad) == false);
	bad = cfg;
	bad.pPktMem = s_Host.PktMem + 1;
	CHECK(BtHciH4Init(&h4, &bad) == false);
	bad = cfg;
	bad.PktMemSize = 4;
	CHECK(BtHciH4Init(&h4, &bad) == false);
	CHECK(BtHciH4Init(nullptr, &cfg) == false);
	CHECK(BtHciH4Init(&h4, nullptr) == false);
}

void TestRoundTrip()
{
	Start();

	const uint8_t types[] = {BT_HCI_H4_PKT_CMD, BT_HCI_H4_PKT_ACL, BT_HCI_H4_PKT_SCO,
							 BT_HCI_H4_PKT_EVT, BT_HCI_H4_PKT_ISO};

	for (uint8_t t : types)
	{
		uint32_t lens[] = {HdrLen(t), HdrLen(t) + 1, 60, 200, HdrLen(t) + 255, (uint32_t)BT_HCI_H4_PKT_MAXLEN};

		for (uint32_t len : lens)
		{
			// Command, event and synchronous data carry a one byte length
			bool bShortLen = t != BT_HCI_H4_PKT_ACL && t != BT_HCI_H4_PKT_ISO;

			if (len > BT_HCI_H4_PKT_MAXLEN || (bShortLen && len > HdrLen(t) + 255))
			{
				continue;
			}

			s_Host.Rx.clear();
			s_Ctlr.Rx.clear();

			std::vector<uint8_t> pkt = Pattern(t, len, len);

			CHECK(BtHciH4Send(&s_Host.H4, t, pkt.data(), len));
			CHECK(s_Ctlr.ReadyCnt > 0);
			CHECK(BtHciH4Process(&s_Ctlr.H4, 0) == false);
			CHECK(s_Ctlr.Rx.size() == 1);
			if (s_Ctlr.Rx.size() == 1)
			{
				CHECK(s_Ctlr.Rx[0].Type == t);
				CHECK(s_Ctlr.Rx[0].Data == pkt);
			}

			// And back
			CHECK(BtHciH4Send(&s_Ctlr.H4, t, pkt.data(), len));
			CHECK(BtHciH4Process(&s_Host.H4, 0) == false);
			CHECK(s_Host.Rx.size() == 1);
			if (s_Host.Rx.size() == 1)
			{
				CHECK(s_Host.Rx[0].Type == t);
				CHECK(s_Host.Rx[0].Data == pkt);
			}
		}
	}

	CHECK(s_Host.H4.SyncErrCnt == 0 && s_Host.H4.DropCnt == 0 && s_Host.H4.TxFailCnt == 0);
	CHECK(s_Ctlr.H4.SyncErrCnt == 0 && s_Ctlr.H4.DropCnt == 0 && s_Ctlr.H4.TxFailCnt == 0);
}

// Many packets of mixed type and length through rings that wrap many times,
// the receiver draining a different number of packets each time
void TestStream()
{
	Start();

	uint32_t rnd = 12345;
	std::vector<RxPkt> sentToCtlr, sentToHost;
	uint32_t wraps = 0;

	for (int n = 0; n < 3000; n++)
	{
		rnd = rnd * 1664525U + 1013904223U;

		const uint8_t types[] = {BT_HCI_H4_PKT_ACL, BT_HCI_H4_PKT_EVT, BT_HCI_H4_PKT_ISO};
		uint8_t t = types[(rnd >> 24) % 3];
		uint32_t maxlen = t == BT_HCI_H4_PKT_EVT ? 2 + 255 : BT_HCI_H4_PKT_MAXLEN;
		uint32_t len = HdrLen(t) + ((rnd >> 8) % (maxlen - HdrLen(t) + 1));
		std::vector<uint8_t> pkt = Pattern(t, len, n);

		bool toCtlr = (rnd & 1) != 0;
		Side &from = toCtlr ? s_Host : s_Ctlr;
		Side &to = toCtlr ? s_Ctlr : s_Host;

		if (BtHciH4Send(&from.H4, t, pkt.data(), len) == false)
		{
			// Room ran out: the receiver has to drain first
			BtHciH4Process(&to.H4, 0);
			CHECK(BtHciH4Send(&from.H4, t, pkt.data(), len));
			wraps++;
		}
		(toCtlr ? sentToCtlr : sentToHost).push_back({t, pkt});

		if (((rnd >> 20) & 3) == 0)
		{
			BtHciH4Process(&to.H4, 0);
		}
	}

	BtHciH4Process(&s_Ctlr.H4, 0);
	BtHciH4Process(&s_Host.H4, 0);

	CHECK(s_Ctlr.Rx.size() == sentToCtlr.size());
	CHECK(s_Host.Rx.size() == sentToHost.size());

	bool bSame = s_Ctlr.Rx.size() == sentToCtlr.size() && s_Host.Rx.size() == sentToHost.size();
	for (size_t i = 0; bSame && i < sentToCtlr.size(); i++)
	{
		bSame = s_Ctlr.Rx[i].Type == sentToCtlr[i].Type && s_Ctlr.Rx[i].Data == sentToCtlr[i].Data;
	}
	for (size_t i = 0; bSame && i < sentToHost.size(); i++)
	{
		bSame = s_Host.Rx[i].Type == sentToHost[i].Type && s_Host.Rx[i].Data == sentToHost[i].Data;
	}
	CHECK(bSame);
	CHECK(wraps > 0);
	CHECK(s_Host.H4.SyncErrCnt == 0 && s_Ctlr.H4.SyncErrCnt == 0);
	CHECK(s_Host.H4.DropCnt == 0 && s_Ctlr.H4.DropCnt == 0);
}

// A packet that does not fit the ring is refused whole. Nothing of it is
// written, so the packets before and after it arrive intact.
void TestWholeOrNothing()
{
	Start();

	std::vector<RxPkt> sent;
	int ok = 0;

	for (int i = 0; i < 40; i++)
	{
		std::vector<uint8_t> pkt = Pattern(BT_HCI_H4_PKT_ACL, 120, i);

		if (BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, pkt.data(), 120) == false)
		{
			break;
		}
		sent.push_back({BT_HCI_H4_PKT_ACL, pkt});
		ok++;
	}

	// 121 bytes on the wire, 1024 bytes of ring
	CHECK(ok == 8);
	CHECK(s_Host.H4.TxFailCnt == 1);
	CHECK(IpcIntrfTxRoom(&s_Host.Ipc) < 121);

	// A shorter packet may still use the room left
	std::vector<uint8_t> small = Pattern(BT_HCI_H4_PKT_EVT, 10, 77);
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_EVT, small.data(), 10));
	sent.push_back({BT_HCI_H4_PKT_EVT, small});

	BtHciH4Process(&s_Ctlr.H4, 0);
	BtHciH4Process(&s_Ctlr.H4, 0);
	CHECK(s_Ctlr.Rx.size() == sent.size());

	bool bSame = s_Ctlr.Rx.size() == sent.size();
	for (size_t i = 0; bSame && i < sent.size(); i++)
	{
		bSame = s_Ctlr.Rx[i].Type == sent[i].Type && s_Ctlr.Rx[i].Data == sent[i].Data;
	}
	CHECK(bSame);
	CHECK(s_Ctlr.H4.SyncErrCnt == 0);

	// The ring is empty again and takes a full size packet
	std::vector<uint8_t> big = Pattern(BT_HCI_H4_PKT_ACL, BT_HCI_H4_PKT_MAXLEN, 5);
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, big.data(), BT_HCI_H4_PKT_MAXLEN));
	s_Ctlr.Rx.clear();
	BtHciH4Process(&s_Ctlr.H4, 0);
	CHECK(s_Ctlr.Rx.size() == 1 && s_Ctlr.Rx[0].Data == big);
}

void TestSendBadArgs()
{
	Start();

	uint8_t d[4] = {};

	CHECK(BtHciH4Send(nullptr, BT_HCI_H4_PKT_ACL, d, 4) == false);
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, nullptr, 4) == false);
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, d, 0) == false);
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, d, BT_HCI_H4_PKT_MAXLEN + 1) == false);
	CHECK(IpcIntrfTxRoom(&s_Host.Ipc) == (int)RING_BYTES);
}

// Bytes that are not a packet type are skipped, the packet after them arrives
void TestResync()
{
	Start();

	uint8_t junk[3] = {0x00, 0x77, 0xFF};
	CHECK(DeviceIntrfTx(&s_Ctlr.Ipc.DevIntrf, 0, junk, 3) == 3);

	std::vector<uint8_t> pkt = Pattern(BT_HCI_H4_PKT_EVT, 12, 3);
	CHECK(BtHciH4Send(&s_Ctlr.H4, BT_HCI_H4_PKT_EVT, pkt.data(), 12));

	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Host.H4.SyncErrCnt == 3);
	CHECK(s_Host.Rx.size() == 1 && s_Host.Rx[0].Data == pkt);
}

// A packet longer than a queue slot is dropped whole, the next one is framed
// from its own type byte
void TestOversize()
{
	Start();

	// ACL header with a data length that makes the packet longer than a slot
	uint32_t body = BT_HCI_H4_PKT_MAXLEN;
	std::vector<uint8_t> raw = {BT_HCI_H4_PKT_ACL, 0x01, 0x00, (uint8_t)(body & 0xFF), (uint8_t)(body >> 8)};
	raw.resize(raw.size() + body, 0xEE);

	size_t pos = 0;
	while (pos < raw.size())
	{
		int n = DeviceIntrfTx(&s_Ctlr.Ipc.DevIntrf, 0, raw.data() + pos, (int)(raw.size() - pos));
		CHECK(n > 0);
		if (n <= 0)
		{
			break;
		}
		pos += (size_t)n;
		BtHciH4Process(&s_Host.H4, 0);
	}

	std::vector<uint8_t> pkt = Pattern(BT_HCI_H4_PKT_EVT, 20, 9);
	CHECK(BtHciH4Send(&s_Ctlr.H4, BT_HCI_H4_PKT_EVT, pkt.data(), 20));
	BtHciH4Process(&s_Host.H4, 0);

	CHECK(s_Host.H4.DropCnt == 1);
	CHECK(s_Host.H4.SyncErrCnt == 0);
	CHECK(s_Host.Rx.size() == 1 && s_Host.Rx[0].Data == pkt);
}

void TestFlush()
{
	Start();

	std::vector<uint8_t> pkt = Pattern(BT_HCI_H4_PKT_ACL, 30, 1);

	for (int i = 0; i < 3; i++)
	{
		CHECK(BtHciH4Send(&s_Ctlr.H4, BT_HCI_H4_PKT_ACL, pkt.data(), 30));
	}
	BtHciH4Poll(&s_Host.H4);
	CHECK(CFifoUsed(s_Host.H4.hPktFifo) == 3);

	BtHciH4Flush(&s_Host.H4);
	CHECK(CFifoUsed(s_Host.H4.hPktFifo) == 0);

	// Bytes still in the ring are gone too
	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Host.Rx.empty());

	CHECK(BtHciH4Send(&s_Ctlr.H4, BT_HCI_H4_PKT_ACL, pkt.data(), 30));
	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Host.Rx.size() == 1 && s_Host.Rx[0].Data == pkt);
}

// The host sends commands and the controller answers from its receive event,
// all inside the send of the command
void TestCommand()
{
	Start();

	s_Ctlr.bAutoProcess = true;
	s_Ctlr.bRespond = true;

	for (int i = 0; i < 20; i++)
	{
		uint8_t param[2] = {(uint8_t)i, 0x55};
		uint8_t ret[2] = {};
		uint16_t op = (uint16_t)(0x2000 + i);

		s_Ctlr.RespStatus = i == 7 ? 0x0C : 0;
		s_Host.Rx.clear();

		uint8_t st = BtHciH4Command(&s_Host.H4, &s_Hci, op, param, sizeof(param), ret, sizeof(ret));

		CHECK(st == (i == 7 ? 0x0C : 0));
		CHECK(s_Hci.CmdCredit == 1);
		CHECK(s_Hci.CmdOpCode == 0);
		if (i != 7)
		{
			CHECK(ret[0] == 0xA5 && ret[1] == 0x5A);
		}
		// The response went to CmdRspHandler, not the queue
		CHECK(s_Host.Rx.empty());
	}

	CHECK(s_Ctlr.Rx.size() == 20);
	CHECK(s_Ctlr.Rx.size() == 20 && s_Ctlr.Rx[3].Type == BT_HCI_H4_PKT_CMD);
	CHECK(s_Ctlr.Rx.size() == 20 && s_Ctlr.Rx[3].Data.size() == 5 &&
		  s_Ctlr.Rx[3].Data[0] == 0x03 && s_Ctlr.Rx[3].Data[1] == 0x20 &&
		  s_Ctlr.Rx[3].Data[2] == 2 && s_Ctlr.Rx[3].Data[3] == 3);
}

// No controller answer: the command times out and clears its state
void TestCommandTimeout()
{
	Start();

	uint8_t st = BtHciH4Command(&s_Host.H4, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0);

	CHECK(st == BT_HCI_H4_ERR_TIMEOUT);
	CHECK(s_Hci.CmdOpCode == 0 && s_Hci.CmdCredit == 1);
	CHECK(s_Host.H4.bCmdActive == false);
}

// The peer has not attached yet: nothing can be sent, nothing is cut short
void TestLinkDown()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));

	// Only the peer, the owner has not formatted the region
	CHECK(InitSide(s_Ctlr, false, s_Ctlr.PktMem, sizeof(s_Ctlr.PktMem), false));
	CHECK(IpcIntrfIsUp(&s_Ctlr.Ipc) == false);

	uint8_t d[4] = {};

	CHECK(IpcIntrfTxRoom(&s_Ctlr.Ipc) == 0);
	CHECK(BtHciH4Send(&s_Ctlr.H4, BT_HCI_H4_PKT_ACL, d, 4) == false);
	CHECK(s_Ctlr.H4.TxFailCnt == 1);
	BtHciH4Poll(&s_Ctlr.H4);
	CHECK(s_Ctlr.Rx.empty());
}

}

// Trace of the HCI host, not used by the framing layer
SysLog_t *SysLogGet(void)
{
	static SysLog_t log = {};
	return &log;
}

int SysLogPrintf(SysLog_t * const, const char *, ...)
{
	return 0;
}

int main()
{
	s_Ctx.Run("config", TestConfig);
	s_Ctx.Run("round trip", TestRoundTrip);
	s_Ctx.Run("stream with wrap", TestStream);
	s_Ctx.Run("whole or nothing", TestWholeOrNothing);
	s_Ctx.Run("send bad arguments", TestSendBadArgs);
	s_Ctx.Run("resync", TestResync);
	s_Ctx.Run("oversize dropped", TestOversize);
	s_Ctx.Run("flush", TestFlush);
	s_Ctx.Run("command", TestCommand);
	s_Ctx.Run("command timeout", TestCommandTimeout);
	s_Ctx.Run("link down", TestLinkDown);

	return s_Ctx.Finish();
}
