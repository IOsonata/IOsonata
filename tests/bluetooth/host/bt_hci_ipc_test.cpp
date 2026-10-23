/**-------------------------------------------------------------------------
@file	bt_hci_ipc_test.cpp

@brief	BtHciIpc host test, one HCI packet per ring slot

A controller core that owns the shared block and a host core that attaches,
each with its BtHciIpc. The IPC port is replaced by a synchronous one:
raising a channel runs the receive work of the other side at once. The race
coverage of the rings is in tests/ipc.

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

#include "bluetooth/bt_hci_ipc.h"
#include "bluetooth/bt_hcievt.h"
#include "syslog.h"
#include "bt_test_harness.h"

#include "ipc_sync_port.h"

namespace {

bttest::Context s_Ctx("bt_hci_ipc_test");

#define CHECK(expr)		BT_CHECK(s_Ctx, expr)

#define NB_SLOT				6U
#define SHMEM_SIZE			BT_HCI_IPC_MEMSIZE(NB_SLOT)
#define PKT_QUEUE			3U

struct RxPkt {
	uint8_t Type;
	std::vector<uint8_t> Data;
};

struct Side {
	BtHciIpcDev_t Dev;			// first member
	alignas(8) uint8_t PktMem[BT_HCI_IPC_PKTMEM_SIZE(PKT_QUEUE)];
	std::vector<RxPkt> Rx;
	int ReadyCnt;
	bool bAutoProcess;
	bool bRespond;				// controller answers each command with a Command Complete
};

alignas(8) uint8_t s_ShMem[SHMEM_SIZE];
Side s_Ctlr;					// owner
Side s_Host;					// peer
BtHciDevice_t s_Hci;

void RxHandler(BtHciIpcDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	Side *side = (Side*)pDev->pCtx;

	side->Rx.push_back({Type, std::vector<uint8_t>(pPkt, pPkt + Len)});

	if (side->bRespond && Type == BT_HCI_IPC_PKT_CMD && Len >= 3)
	{
		uint8_t evt[2 + 6] = {BT_HCI_EVT_COMMAND_COMPLETE, 6, 1, pPkt[0], pPkt[1], 0, 0xA5, 0x5A};

		BtHciIpcSend(pDev, BT_HCI_IPC_PKT_EVT, evt, sizeof(evt));
	}
}

void CmdRspHandler(BtHciIpcDev_t * const, uint8_t, uint8_t *pPkt, uint16_t)
{
	BtHciProcessEvent(&s_Hci, (BtHciEvtPacket_t*)pPkt);
}

void RxReady(BtHciIpcDev_t * const pDev)
{
	Side *side = (Side*)pDev->pCtx;

	side->ReadyCnt++;
	if (side->bAutoProcess)
	{
		BtHciIpcProcess(pDev, 0);
	}
}

bool InitSide(Side &S, bool bOwner, bool bCmdRsp)
{
	S.Rx.clear();
	S.ReadyCnt = 0;
	S.bAutoProcess = false;
	S.bRespond = false;

	BtHciIpcCfg_t cfg = {};

	cfg.pMemBlk = s_ShMem;
	cfg.MemSize = sizeof(s_ShMem);
	cfg.bOwner = bOwner;
	cfg.NbPkt = NB_SLOT;
	cfg.TxChan = bOwner ? 0 : 1;
	cfg.RxChan = bOwner ? 1 : 0;
	cfg.IntPrio = 3;
	cfg.pPktMem = S.PktMem;
	cfg.PktMemSize = sizeof(S.PktMem);
	cfg.RxHandler = RxHandler;
	cfg.CmdRspHandler = bCmdRsp ? CmdRspHandler : nullptr;
	cfg.RxReady = RxReady;
	cfg.TimeoutMs = 1;
	cfg.pCtx = &S;

	return BtHciIpcInit(&S.Dev, &cfg);
}

void Start()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));
	std::memset((void*)&s_Hci, 0, sizeof(s_Hci));
	s_Hci.CmdCredit = 1;

	CHECK(InitSide(s_Ctlr, true, false));
	CHECK(InitSide(s_Host, false, true));
	CHECK(BtHciIpcIsUp(&s_Ctlr.Dev));
	CHECK(BtHciIpcIsUp(&s_Host.Dev));
}

uint32_t HdrLen(uint8_t Type)
{
	switch (Type)
	{
		case BT_HCI_IPC_PKT_CMD: return 3;
		case BT_HCI_IPC_PKT_ACL: return 4;
		case BT_HCI_IPC_PKT_SCO: return 3;
		case BT_HCI_IPC_PKT_EVT: return 2;
		default: return 4;
	}
}

// Packet content that tells its position and type apart, header length field agreeing
std::vector<uint8_t> Pattern(uint8_t Type, uint32_t Len, uint32_t Seed)
{
	std::vector<uint8_t> v(Len);

	for (uint32_t i = 0; i < Len; i++)
	{
		v[i] = (uint8_t)(Seed * 31U + i * 7U + Type);
	}

	switch (Type)
	{
		case BT_HCI_IPC_PKT_CMD:
			v[2] = (uint8_t)(Len - 3);
			break;
		case BT_HCI_IPC_PKT_EVT:
			v[0] = 0x05;	// not a Command Complete or Status, those bypass the queue
			v[1] = (uint8_t)(Len - 2);
			break;
		case BT_HCI_IPC_PKT_ACL:
			v[2] = (uint8_t)((Len - 4) & 0xFF);
			v[3] = (uint8_t)((Len - 4) >> 8);
			break;
		case BT_HCI_IPC_PKT_ISO:
			v[2] = (uint8_t)((Len - 4) & 0xFF);
			v[3] = (uint8_t)(((Len - 4) >> 8) & 0x3F);
			break;
		default:
			v[2] = (uint8_t)(Len - 3);
			break;
	}

	return v;
}

bool Same(const RxPkt &P, uint8_t Type, const std::vector<uint8_t> &D)
{
	return P.Type == Type && P.Data == D;
}

void TestConfig()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));

	BtHciIpcDev_t dev;
	BtHciIpcCfg_t cfg = {};

	cfg.pMemBlk = s_ShMem;
	cfg.MemSize = sizeof(s_ShMem);
	cfg.bOwner = true;
	cfg.NbPkt = NB_SLOT;
	cfg.TxChan = 0;
	cfg.RxChan = 1;
	cfg.pPktMem = s_Ctlr.PktMem;
	cfg.PktMemSize = sizeof(s_Ctlr.PktMem);

	CHECK(BtHciIpcInit(&dev, &cfg));
	CHECK(dev.H4.TimeoutMs == BT_HCI_IPC_TIMEOUT_DEFAULT);

	BtHciIpcCfg_t bad = cfg;
	bad.pMemBlk = nullptr;
	CHECK(BtHciIpcInit(&dev, &bad) == false);
	bad = cfg;
	bad.NbPkt = 0;
	CHECK(BtHciIpcInit(&dev, &bad) == false);
	bad = cfg;
	bad.MemSize = BT_HCI_IPC_MEMSIZE(NB_SLOT) - 4;
	CHECK(BtHciIpcInit(&dev, &bad) == false);
	bad = cfg;
	bad.pPktMem = nullptr;
	CHECK(BtHciIpcInit(&dev, &bad) == false);
	CHECK(BtHciIpcInit(nullptr, &cfg) == false);
	CHECK(BtHciIpcInit(&dev, nullptr) == false);
}

// Every packet type, every length from the header size up to the largest
void TestRoundTrip()
{
	Start();

	const uint8_t types[] = {BT_HCI_IPC_PKT_CMD, BT_HCI_IPC_PKT_ACL, BT_HCI_IPC_PKT_SCO,
							 BT_HCI_IPC_PKT_EVT, BT_HCI_IPC_PKT_ISO};

	for (uint8_t type : types)
	{
		uint32_t maxlen = BT_HCI_H4_PKT_MAXLEN;	// BtHciIpc queues through BtHciH4

		if (type == BT_HCI_IPC_PKT_CMD || type == BT_HCI_IPC_PKT_SCO || type == BT_HCI_IPC_PKT_EVT)
		{
			maxlen = HdrLen(type) + 255;	// one length byte
		}
		if (maxlen > BT_HCI_H4_PKT_MAXLEN)
		{
			maxlen = BT_HCI_H4_PKT_MAXLEN;
		}

		for (uint32_t len = HdrLen(type); len <= maxlen; len += (len < 20 ? 1 : 37))
		{
			// Host to controller for commands, either way for data
			Side &from = (type == BT_HCI_IPC_PKT_EVT) ? s_Ctlr : s_Host;
			Side &to = (type == BT_HCI_IPC_PKT_EVT) ? s_Host : s_Ctlr;
			std::vector<uint8_t> d = Pattern(type, len, len);

			to.Rx.clear();
			CHECK(BtHciIpcSend(&from.Dev, type, d.data(), len));
			BtHciIpcProcess(&to.Dev, 0);
			CHECK(to.Rx.size() == 1);
			if (to.Rx.size() == 1)
			{
				CHECK(Same(to.Rx[0], type, d));
			}
		}
	}

	CHECK(s_Ctlr.Dev.H4.SyncErrCnt == 0 && s_Host.Dev.H4.SyncErrCnt == 0);
}

// A burst the queue cannot hold: the rest waits in the ring, nothing is lost
void TestBurstOrder()
{
	Start();

	for (uint32_t round = 0; round < 10; round++)
	{
		s_Ctlr.Rx.clear();

		uint32_t sent = 0;
		for (uint32_t i = 0; i < NB_SLOT; i++)
		{
			std::vector<uint8_t> d = Pattern(BT_HCI_IPC_PKT_ACL, 20 + i * 40, round * 16 + i);

			CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d.data(), (uint32_t)d.size()));
			sent++;
		}

		// All slots are in use, one more is refused and nothing changed
		uint8_t extra[4] = {};
		CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, extra, 4) == false);

		for (int pass = 0; pass < 4 && s_Ctlr.Rx.size() < sent; pass++)
		{
			BtHciIpcProcess(&s_Ctlr.Dev, 0);
		}

		CHECK(s_Ctlr.Rx.size() == sent);
		for (uint32_t i = 0; i < s_Ctlr.Rx.size() && i < sent; i++)
		{
			CHECK(Same(s_Ctlr.Rx[i], BT_HCI_IPC_PKT_ACL,
					   Pattern(BT_HCI_IPC_PKT_ACL, 20 + i * 40, round * 16 + i)));
		}
	}
}

// Slots that are not a whole packet are dropped and the next good one is seen
void TestBadSlot()
{
	Start();

	DevIntrf_t *raw = &s_Host.Dev.Ipc.DevIntrf;
	std::vector<uint8_t> good = Pattern(BT_HCI_IPC_PKT_ACL, 30, 1);

	// Unknown type
	uint8_t bad1[5] = {0x77, 1, 2, 3, 4};
	// ACL whose header says 26 bytes follow but only 2 do
	uint8_t bad2[1 + 6] = {BT_HCI_IPC_PKT_ACL, 1, 0, 26, 0, 9, 9};
	// Only the type byte
	uint8_t bad3[1] = {BT_HCI_IPC_PKT_EVT};

	CHECK(DeviceIntrfTx(raw, 0, bad1, sizeof(bad1)) == (int)sizeof(bad1));
	CHECK(DeviceIntrfTx(raw, 0, bad2, sizeof(bad2)) == (int)sizeof(bad2));
	CHECK(DeviceIntrfTx(raw, 0, bad3, sizeof(bad3)) == (int)sizeof(bad3));
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, good.data(), (uint32_t)good.size()));

	s_Ctlr.Rx.clear();
	BtHciIpcProcess(&s_Ctlr.Dev, 0);
	CHECK(s_Ctlr.Rx.size() == 1);
	if (s_Ctlr.Rx.size() == 1)
	{
		CHECK(Same(s_Ctlr.Rx[0], BT_HCI_IPC_PKT_ACL, good));
	}
	CHECK(s_Ctlr.Dev.H4.SyncErrCnt == 3);
}

// The host core restarts: it attaches again and the link carries on, nothing
// from before the restart is read as part of a new packet
void TestPeerRestart()
{
	Start();

	std::vector<uint8_t> a = Pattern(BT_HCI_IPC_PKT_ACL, 40, 3);

	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, a.data(), (uint32_t)a.size()));

	// Host restarts: its state is lost, the block stays as it is
	CHECK(InitSide(s_Host, false, true));
	CHECK(BtHciIpcIsUp(&s_Host.Dev));

	s_Ctlr.Rx.clear();
	BtHciIpcProcess(&s_Ctlr.Dev, 0);
	CHECK(s_Ctlr.Rx.size() == 1);
	if (s_Ctlr.Rx.size() == 1)
	{
		CHECK(Same(s_Ctlr.Rx[0], BT_HCI_IPC_PKT_ACL, a));
	}

	std::vector<uint8_t> b = Pattern(BT_HCI_IPC_PKT_ACL, 60, 4);

	s_Ctlr.Rx.clear();
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, b.data(), (uint32_t)b.size()));
	BtHciIpcProcess(&s_Ctlr.Dev, 0);
	CHECK(s_Ctlr.Rx.size() == 1);
	if (s_Ctlr.Rx.size() == 1)
	{
		CHECK(Same(s_Ctlr.Rx[0], BT_HCI_IPC_PKT_ACL, b));
	}
}

void TestFlush()
{
	Start();

	std::vector<uint8_t> d = Pattern(BT_HCI_IPC_PKT_ACL, 30, 5);

	for (int i = 0; i < 4; i++)
	{
		CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d.data(), (uint32_t)d.size()));
	}
	BtHciIpcPoll(&s_Ctlr.Dev);
	BtHciIpcFlush(&s_Ctlr.Dev);
	s_Ctlr.Rx.clear();
	BtHciIpcProcess(&s_Ctlr.Dev, 0);
	CHECK(s_Ctlr.Rx.empty());

	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d.data(), (uint32_t)d.size()));
	BtHciIpcProcess(&s_Ctlr.Dev, 0);
	CHECK(s_Ctlr.Rx.size() == 1);
}

void TestSendBadArgs()
{
	Start();

	uint8_t d[8] = {};

	CHECK(BtHciIpcSend(nullptr, BT_HCI_IPC_PKT_ACL, d, 4) == false);
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, nullptr, 4) == false);
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d, 0) == false);
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d, BT_HCI_H4_PKT_MAXLEN + 1) == false);
}

// Host sends a command and waits, the controller answers from its handler
void TestCommand()
{
	Start();

	s_Ctlr.bRespond = true;
	s_Ctlr.bAutoProcess = true;
	s_Host.bAutoProcess = false;

	uint8_t ret[2] = {};
	uint8_t status = BtHciIpcCommand(&s_Host.Dev, &s_Hci, 0x0C03, nullptr, 0, ret, sizeof(ret));

	CHECK(status == 0);
	CHECK(ret[0] == 0xA5 && ret[1] == 0x5A);
	CHECK(s_Ctlr.Rx.size() == 1);
}

void TestCommandTimeout()
{
	Start();

	s_Ctlr.bRespond = false;
	s_Ctlr.bAutoProcess = true;

	uint8_t status = BtHciIpcCommand(&s_Host.Dev, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0);

	CHECK(status == BT_HCI_IPC_ERR_TIMEOUT);
}

// The host attaches before the controller has formatted the block
void TestLinkDown()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));

	CHECK(InitSide(s_Host, false, false));
	CHECK(BtHciIpcIsUp(&s_Host.Dev) == false);

	uint8_t d[4] = {};

	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d, 4) == false);
	CHECK(s_Host.Dev.H4.TxFailCnt == 1);
	BtHciIpcPoll(&s_Host.Dev);
	CHECK(s_Host.Rx.empty());

	CHECK(InitSide(s_Ctlr, true, false));
	CHECK(BtHciIpcIsUp(&s_Host.Dev));
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, d, 4));
}


// ---- Packet DevIntrf face, the controller side, against BtHciIpc as the host

BtHciIpcIntrfDev_t s_CtlrIntrf;
int s_IntrfRxEvt;
int s_IntrfTxEvt;

int IntrfEvt(DevIntrf_t * const, DEVINTRF_EVT EvtId, uint8_t *, int)
{
	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		s_IntrfRxEvt++;
	}
	else if (EvtId == DEVINTRF_EVT_TX_READY)
	{
		s_IntrfTxEvt++;
	}

	return 0;
}

void StartIntrf()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));
	std::memset((void*)&s_Hci, 0, sizeof(s_Hci));
	s_Hci.CmdCredit = 1;
	s_IntrfRxEvt = 0;
	s_IntrfTxEvt = 0;

	BtHciIpcIntrfCfg_t cfg = {};

	cfg.pMemBlk = s_ShMem;
	cfg.MemSize = sizeof(s_ShMem);
	cfg.bOwner = true;
	cfg.NbPkt = NB_SLOT;
	cfg.TxChan = 0;
	cfg.RxChan = 1;
	cfg.IntPrio = 3;
	cfg.EvtCB = IntrfEvt;

	CHECK(BtHciIpcIntrfInit(&s_CtlrIntrf, &cfg));
	CHECK(InitSide(s_Host, false, false));
	CHECK(BtHciIpcIntrfIsUp(&s_CtlrIntrf));
	CHECK(BtHciIpcIsUp(&s_Host.Dev));
}

void TestIntrfClass()
{
	// The class is the DeviceIntrf the controller takes
	static BtHciIpcIntrf intrf;
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));

	BtHciIpcIntrfCfg_t cfg = {};

	cfg.pMemBlk = s_ShMem;
	cfg.MemSize = sizeof(s_ShMem);
	cfg.bOwner = true;
	cfg.NbPkt = NB_SLOT;
	cfg.TxChan = 0;
	cfg.RxChan = 1;

	CHECK(intrf.Init(cfg));
	CHECK(intrf.Up());
	CHECK(intrf.Data() == (DevIntrf_t*)intrf);

	uint8_t d[4] = {0x40, 0x00, 0x00, 0x00};
	uint8_t buf[16];

	CHECK(intrf.Tx(BT_HCI_IPC_PKT_ACL, d, sizeof(d)) == (int)sizeof(d));
	CHECK(intrf.Rx(BT_HCI_IPC_PKT_ACL, buf, sizeof(buf)) == 0);
	intrf.Enable();
	intrf.Disable();
}

void TestIntrfConfig()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));

	BtHciIpcIntrfDev_t dev;
	BtHciIpcIntrfCfg_t cfg = {};

	cfg.pMemBlk = s_ShMem;
	cfg.MemSize = sizeof(s_ShMem);
	cfg.bOwner = true;
	cfg.NbPkt = NB_SLOT;
	cfg.TxChan = 0;
	cfg.RxChan = 1;

	CHECK(BtHciIpcIntrfInit(&dev, &cfg));
	CHECK(BtHciIpcIntrfGet(&dev) == &dev.DevIntrf);

	BtHciIpcIntrfCfg_t bad = cfg;
	bad.pMemBlk = nullptr;
	CHECK(BtHciIpcIntrfInit(&dev, &bad) == false);
	bad = cfg;
	bad.NbPkt = 0;
	CHECK(BtHciIpcIntrfInit(&dev, &bad) == false);
	bad = cfg;
	bad.MemSize = BT_HCI_IPC_MEMSIZE(NB_SLOT) - 4;
	CHECK(BtHciIpcIntrfInit(&dev, &bad) == false);
	CHECK(BtHciIpcIntrfInit(nullptr, &cfg) == false);
	CHECK(BtHciIpcIntrfInit(&dev, nullptr) == false);
}

// Host sends, the controller reads by type. The head packet is taken by its
// own type only, packets behind it wait.
void TestIntrfRx()
{
	StartIntrf();

	DevIntrf_t *c = BtHciIpcIntrfGet(&s_CtlrIntrf);
	std::vector<uint8_t> cmd = Pattern(BT_HCI_IPC_PKT_CMD, 12, 1);
	std::vector<uint8_t> acl = Pattern(BT_HCI_IPC_PKT_ACL, 80, 2);
	uint8_t buf[BT_HCI_IPC_PKT_MAXLEN];

	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_CMD, cmd.data(), (uint32_t)cmd.size()));
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, acl.data(), (uint32_t)acl.size()));
	CHECK(s_IntrfRxEvt >= 2);

	// ACL is behind the command: not offered until the command is taken
	CHECK(DeviceIntrfRx(c, BT_HCI_IPC_PKT_ACL, buf, sizeof(buf)) == 0);
	CHECK(DeviceIntrfRx(c, BT_HCI_IPC_PKT_EVT, buf, sizeof(buf)) == 0);

	int l = DeviceIntrfRx(c, BT_HCI_IPC_PKT_CMD, buf, sizeof(buf));
	CHECK(l == (int)cmd.size());
	CHECK(l == (int)cmd.size() && std::memcmp(buf, cmd.data(), cmd.size()) == 0);

	l = DeviceIntrfRx(c, BT_HCI_IPC_PKT_ACL, buf, sizeof(buf));
	CHECK(l == (int)acl.size());
	CHECK(l == (int)acl.size() && std::memcmp(buf, acl.data(), acl.size()) == 0);

	// Nothing left
	for (uint32_t t = BT_HCI_IPC_PKT_CMD; t <= BT_HCI_IPC_PKT_ISO; t++)
	{
		CHECK(DeviceIntrfRx(c, t, buf, sizeof(buf)) == 0);
	}
}

// Packets above what BtHciH4 queues, up to the slot: the packet face takes them
void TestIntrfLarge()
{
	StartIntrf();

	DevIntrf_t *c = BtHciIpcIntrfGet(&s_CtlrIntrf);
	DevIntrf_t *raw = &s_Host.Dev.Ipc.DevIntrf;
	uint8_t buf[BT_HCI_IPC_PKT_MAXLEN];

	// Host writes slots directly: type byte then the packet
	const uint32_t lens[] = {BT_HCI_H4_PKT_MAXLEN + 3, BT_HCI_IPC_PKT_MAXLEN};

	for (uint32_t len : lens)
	{
		std::vector<uint8_t> d = Pattern(BT_HCI_IPC_PKT_ISO, len, len);
		std::vector<uint8_t> slot(1 + len);

		slot[0] = BT_HCI_IPC_PKT_ISO;
		std::memcpy(&slot[1], d.data(), len);
		CHECK(DeviceIntrfTx(raw, 0, slot.data(), (int)slot.size()) == (int)slot.size());

		int l = DeviceIntrfRx(c, BT_HCI_IPC_PKT_ISO, buf, sizeof(buf));
		CHECK(l == (int)len);
		CHECK(l == (int)len && std::memcmp(buf, d.data(), len) == 0);

		// And back
		CHECK(DeviceIntrfTx(c, BT_HCI_IPC_PKT_ISO, d.data(), (int)len) == (int)len);
		std::vector<uint8_t> back(1 + BT_HCI_IPC_PKT_MAXLEN);
		l = DeviceIntrfRx(raw, 0, back.data(), (int)back.size());
		CHECK(l == (int)len + 1);
		CHECK(l == (int)len + 1 && back[0] == BT_HCI_IPC_PKT_ISO && std::memcmp(&back[1], d.data(), len) == 0);
	}

	// One byte more than a slot holds is refused
	std::vector<uint8_t> big(BT_HCI_IPC_PKT_MAXLEN + 1, 0);
	CHECK(DeviceIntrfTx(c, BT_HCI_IPC_PKT_ISO, big.data(), (int)big.size()) == 0);
}

void TestIntrfTx()
{
	StartIntrf();

	DevIntrf_t *c = BtHciIpcIntrfGet(&s_CtlrIntrf);

	for (uint32_t round = 0; round < 8; round++)
	{
		s_Host.Rx.clear();

		uint32_t sent = 0;
		for (uint32_t i = 0; i < NB_SLOT; i++)
		{
			uint8_t type = (i & 1) ? BT_HCI_IPC_PKT_ACL : BT_HCI_IPC_PKT_EVT;
			std::vector<uint8_t> d = Pattern(type, 10 + i * 30, round * 8 + i);

			CHECK(DeviceIntrfTx(c, type, d.data(), (int)d.size()) == (int)d.size());
			sent++;
		}

		// Ring full: refused with nothing written
		uint8_t extra[4] = {};
		CHECK(DeviceIntrfTx(c, BT_HCI_IPC_PKT_ACL, extra, 4) == 0);

		for (int pass = 0; pass < 4 && s_Host.Rx.size() < sent; pass++)
		{
			BtHciIpcProcess(&s_Host.Dev, 0);
		}

		CHECK(s_Host.Rx.size() == sent);
		for (uint32_t i = 0; i < s_Host.Rx.size() && i < sent; i++)
		{
			uint8_t type = (i & 1) ? BT_HCI_IPC_PKT_ACL : BT_HCI_IPC_PKT_EVT;

			CHECK(Same(s_Host.Rx[i], type, Pattern(type, 10 + i * 30, round * 8 + i)));
		}
	}

	// A type that is not HCI and an oversize packet are refused
	uint8_t d[4] = {};
	CHECK(DeviceIntrfTx(c, 0, d, 4) == 0);
	CHECK(DeviceIntrfTx(c, 6, d, 4) == 0);
	CHECK(DeviceIntrfTx(c, BT_HCI_IPC_PKT_ACL, d, 0) == 0);
}

void TestIntrfBadSlotAndSmallBuffer()
{
	StartIntrf();

	DevIntrf_t *c = BtHciIpcIntrfGet(&s_CtlrIntrf);
	DevIntrf_t *raw = &s_Host.Dev.Ipc.DevIntrf;
	uint8_t buf[BT_HCI_IPC_PKT_MAXLEN];

	uint8_t bad1[5] = {0x77, 1, 2, 3, 4};
	uint8_t bad2[1 + 6] = {BT_HCI_IPC_PKT_ACL, 1, 0, 26, 0, 9, 9};

	CHECK(DeviceIntrfTx(raw, 0, bad1, sizeof(bad1)) == (int)sizeof(bad1));
	CHECK(DeviceIntrfTx(raw, 0, bad2, sizeof(bad2)) == (int)sizeof(bad2));

	std::vector<uint8_t> acl = Pattern(BT_HCI_IPC_PKT_ACL, 60, 7);

	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, acl.data(), (uint32_t)acl.size()));

	// The two bad slots are skipped, the packet behind them is offered
	int l = DeviceIntrfRx(c, BT_HCI_IPC_PKT_ACL, buf, 10);
	CHECK(l == 0);
	CHECK(s_CtlrIntrf.SyncErrCnt == 2);
	CHECK(s_CtlrIntrf.RxDropCnt == 1);

	// The packet that did not fit is gone and the link carries on
	std::vector<uint8_t> next = Pattern(BT_HCI_IPC_PKT_ACL, 25, 9);
	CHECK(BtHciIpcSend(&s_Host.Dev, BT_HCI_IPC_PKT_ACL, next.data(), (uint32_t)next.size()));
	l = DeviceIntrfRx(c, BT_HCI_IPC_PKT_ACL, buf, sizeof(buf));
	CHECK(l == (int)next.size());
	CHECK(l == (int)next.size() && std::memcmp(buf, next.data(), next.size()) == 0);
}

}

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
	s_Ctx.Run("burst order", TestBurstOrder);
	s_Ctx.Run("bad slot", TestBadSlot);
	s_Ctx.Run("peer restart", TestPeerRestart);
	s_Ctx.Run("flush", TestFlush);
	s_Ctx.Run("send bad arguments", TestSendBadArgs);
	s_Ctx.Run("command", TestCommand);
	s_Ctx.Run("command timeout", TestCommandTimeout);
	s_Ctx.Run("link down", TestLinkDown);
	s_Ctx.Run("packet intrf class", TestIntrfClass);
	s_Ctx.Run("packet intrf config", TestIntrfConfig);
	s_Ctx.Run("packet intrf rx", TestIntrfRx);
	s_Ctx.Run("packet intrf large", TestIntrfLarge);
	s_Ctx.Run("packet intrf tx", TestIntrfTx);
	s_Ctx.Run("packet intrf bad slot, small buffer", TestIntrfBadSlotAndSmallBuffer);

	return s_Ctx.Finish();
}
