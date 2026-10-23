/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_h4_test.cpp

@brief	BtHciCtlrH4 bridge host test

A host on one end of an IpcIntrf pair, the bridge and a fake controller on the
other. Commands and ACL data go down, events and ACL data come up, through
BtHciH4 on both ends.

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

#include "bluetooth/bt_hci_ctlr_h4.h"
#include "bluetooth/bt_hcievt.h"
#include "ipc/ipc_intrf.h"
#include "syslog.h"
#include "bt_test_harness.h"

#include "ipc_sync_port.h"

namespace {

bttest::Context s_Ctx("bt_hci_ctlr_h4_test");

#define CHECK(expr)		BT_CHECK(s_Ctx, expr)

#define RING_BYTES			1024U
#define SHMEM_SIZE			IPC_INTRF_MEMSIZE(RING_BYTES, 0, RING_BYTES, 0)
#define PKT_QUEUE			4U

struct RxPkt {
	uint8_t Type;
	std::vector<uint8_t> Data;
};

struct Side {
	IpcIntrfDev_t Ipc;			// first member, the event handler finds the side from it
	BtHciH4Dev_t H4;
	alignas(8) uint8_t PktMem[BT_HCI_H4_PKTMEM_SIZE(PKT_QUEUE)];
	std::vector<RxPkt> Rx;
	bool bAutoProcess;
};

alignas(8) uint8_t s_ShMem[SHMEM_SIZE];
Side s_Host;
Side s_Ctlr;
BtHciDevice_t s_Hci;
BtHciCtlrDev_t s_Ctrl;
BtHciCtlrH4_t s_Bridge;

// Fake controller
std::vector<std::vector<uint8_t>> s_Cmds;
std::vector<std::vector<uint8_t>> s_Acl;
bool s_bCmdAccept;
bool s_bAclAccept;
uint8_t s_CmdStatus;
bool s_bUseCmdStatus;

void HostRx(BtHciH4Dev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	((Side*)pDev->pCtx)->Rx.push_back({Type, std::vector<uint8_t>(pPkt, pPkt + Len)});
}

void HostCmdRsp(BtHciH4Dev_t * const, uint8_t, uint8_t *pPkt, uint16_t)
{
	BtHciProcessEvent(&s_Hci, (BtHciEvtPacket_t*)pPkt);
}

void HostReady(BtHciH4Dev_t * const)
{
}

void CtlrReady(BtHciH4Dev_t * const pDev)
{
	if (s_Ctlr.bAutoProcess)
	{
		BtHciH4Process(pDev, 0);
	}
}

int TxRoom(BtHciH4Dev_t * const pDev)
{
	// The host side keeps its Side in pCtx, the controller side keeps the bridge
	if (pDev == &s_Ctlr.H4)
	{
		return IpcIntrfTxRoom(&s_Ctlr.Ipc);
	}

	return IpcIntrfTxRoom(&s_Host.Ipc);
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

// Controller answers a command with Command Complete, or Command Status when
// asked to, through the same callback the Nordic controller port uses
size_t FakeSendCommand(BtHciCtlrDev_t * const pDev, void * const pData, size_t Len)
{
	const uint8_t *p = (const uint8_t*)pData;

	s_Cmds.push_back(std::vector<uint8_t>(p, p + Len));
	if (s_bCmdAccept == false)
	{
		return 0;
	}

	if (s_bUseCmdStatus)
	{
		uint8_t evt[2 + 4] = {BT_HCI_EVT_COMMAND_STATUS, 4, s_CmdStatus, 1, p[0], p[1]};

		pDev->RxHandler(pDev, true, evt);
	}
	else
	{
		uint8_t evt[2 + 6] = {BT_HCI_EVT_COMMAND_COMPLETE, 6, 1, p[0], p[1], 0, 0xC3, 0x3C};

		pDev->RxHandler(pDev, true, evt);
	}

	return Len;
}

size_t FakeSend(BtHciCtlrDev_t * const, void * const pData, size_t Len)
{
	const uint8_t *p = (const uint8_t*)pData;

	s_Acl.push_back(std::vector<uint8_t>(p, p + Len));

	return s_bAclAccept ? Len : 0;
}

bool InitSide(Side &S, bool bOwner, bool bHost)
{
	S.Rx.clear();
	S.bAutoProcess = false;

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
	S.Ipc.DevIntrf.MaxRetry = 1;

	BtHciH4Cfg_t hcfg = {};
	hcfg.pPhyIntrf = &S.Ipc.DevIntrf;
	hcfg.pPktMem = S.PktMem;
	hcfg.PktMemSize = sizeof(S.PktMem);
	hcfg.TxRoom = TxRoom;
	hcfg.TimeoutMs = 1;
	hcfg.pCtx = &S;
	if (bHost)
	{
		hcfg.RxHandler = HostRx;
		hcfg.CmdRspHandler = HostCmdRsp;
		hcfg.RxReady = HostReady;
	}
	else
	{
		hcfg.RxHandler = BtHciCtlrH4FromHost;
		hcfg.RxReady = CtlrReady;
	}

	return BtHciH4Init(&S.H4, &hcfg);
}

void Start()
{
	std::memset(s_pIpcByRx, 0, sizeof(s_pIpcByRx));
	std::memset(s_ShMem, 0, sizeof(s_ShMem));
	std::memset((void*)&s_Hci, 0, sizeof(s_Hci));
	s_Hci.CmdCredit = 1;
	s_Cmds.clear();
	s_Acl.clear();
	s_bCmdAccept = true;
	s_bAclAccept = true;
	s_bUseCmdStatus = false;
	s_CmdStatus = 0;

	CHECK(InitSide(s_Host, true, true));
	CHECK(InitSide(s_Ctlr, false, false));
	CHECK(IpcIntrfIsUp(&s_Ctlr.Ipc));

	BtHciCtlrCfg_t ccfg = {};
	ccfg.RxHandler = BtHciCtlrH4FromCtlr;
	CHECK(BtHciCtlrInit(&s_Ctrl, &ccfg));
	s_Ctrl.SendCommand = FakeSendCommand;
	s_Ctrl.Send = FakeSend;

	CHECK(BtHciCtlrH4Init(&s_Bridge, &s_Ctrl, &s_Ctlr.H4));
	s_Ctlr.bAutoProcess = true;
}

void TestInit()
{
	Start();

	CHECK(s_Ctrl.pObj == &s_Bridge);
	CHECK(s_Ctlr.H4.pCtx == &s_Bridge);
	CHECK(BtHciCtlrH4Init(nullptr, &s_Ctrl, &s_Ctlr.H4) == false);
	CHECK(BtHciCtlrH4Init(&s_Bridge, nullptr, &s_Ctlr.H4) == false);
	CHECK(BtHciCtlrH4Init(&s_Bridge, &s_Ctrl, nullptr) == false);
}

void TestCommandRoundTrip()
{
	Start();

	uint8_t param[2] = {0x11, 0x22};
	uint8_t ret[2] = {};

	CHECK(BtHciH4Command(&s_Host.H4, &s_Hci, 0x2005, param, sizeof(param), ret, sizeof(ret)) == 0);
	CHECK(s_Cmds.size() == 1);
	if (s_Cmds.size() == 1)
	{
		const std::vector<uint8_t> want = {0x05, 0x20, 2, 0x11, 0x22};
		CHECK(s_Cmds[0] == want);
	}
	CHECK(ret[0] == 0xC3 && ret[1] == 0x3C);
	CHECK(s_Hci.CmdCredit == 1);
	CHECK(s_Bridge.CmdFailCnt == 0 && s_Bridge.UpFailCnt == 0);
}

void TestCommandStatus()
{
	Start();

	s_bUseCmdStatus = true;
	s_CmdStatus = 0x0C;
	CHECK(BtHciH4Command(&s_Host.H4, &s_Hci, 0x200D, nullptr, 0, nullptr, 0) == 0x0C);

	s_CmdStatus = 0;
	CHECK(BtHciH4Command(&s_Host.H4, &s_Hci, 0x200D, nullptr, 0, nullptr, 0) == 0);
	CHECK(s_Cmds.size() == 2);
}

// The controller refuses the command: it is counted and the host times out
void TestCommandRefused()
{
	Start();

	s_bCmdAccept = false;
	CHECK(BtHciH4Command(&s_Host.H4, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0) == BT_HCI_H4_ERR_TIMEOUT);
	CHECK(s_Bridge.CmdFailCnt == 1);
}

void TestAclBothWays()
{
	Start();

	// Down: ACL header with handle 0x0040, length 6
	std::vector<uint8_t> down = {0x40, 0x00, 6, 0, 1, 2, 3, 4, 5, 6};

	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, down.data(), (uint32_t)down.size()));
	CHECK(s_Acl.size() == 1 && s_Acl[0] == down);

	// A refused ACL packet is counted
	s_bAclAccept = false;
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ACL, down.data(), (uint32_t)down.size()));
	CHECK(s_Bridge.AclFailCnt == 1);

	// Up: the length is read from the header
	std::vector<uint8_t> up(4 + 200);
	for (size_t i = 0; i < up.size(); i++)
	{
		up[i] = (uint8_t)(i * 3);
	}
	up[0] = 0x41; up[1] = 0x00; up[2] = 200; up[3] = 0;

	BtHciCtlrH4FromCtlr(&s_Ctrl, false, up.data());
	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Host.Rx.size() == 1);
	if (s_Host.Rx.size() == 1)
	{
		CHECK(s_Host.Rx[0].Type == BT_HCI_H4_PKT_ACL && s_Host.Rx[0].Data == up);
	}
}

void TestEventsUp()
{
	Start();

	std::vector<std::vector<uint8_t>> sent;

	for (int i = 0; i < 60; i++)
	{
		uint8_t plen = (uint8_t)((i * 37) % 200);
		std::vector<uint8_t> evt(2 + plen);

		evt[0] = 0x05;
		evt[1] = plen;
		for (uint8_t k = 0; k < plen; k++)
		{
			evt[2 + k] = (uint8_t)(i + k);
		}
		sent.push_back(evt);

		BtHciCtlrH4FromCtlr(&s_Ctrl, true, evt.data());
		if ((i % 3) == 2)
		{
			BtHciH4Process(&s_Host.H4, 0);
		}
	}
	BtHciH4Process(&s_Host.H4, 0);

	CHECK(s_Host.Rx.size() == sent.size());
	bool bSame = s_Host.Rx.size() == sent.size();
	for (size_t i = 0; bSame && i < sent.size(); i++)
	{
		bSame = s_Host.Rx[i].Type == BT_HCI_H4_PKT_EVT && s_Host.Rx[i].Data == sent[i];
	}
	CHECK(bSame);
	CHECK(s_Bridge.UpFailCnt == 0 && s_Bridge.BadLenCnt == 0);
}

void TestUnsupportedAndBadLength()
{
	Start();

	uint8_t sco[4] = {0, 0, 1, 0};
	uint8_t iso[5] = {0, 0, 1, 0, 0};

	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_SCO, sco, sizeof(sco)));
	CHECK(BtHciH4Send(&s_Host.H4, BT_HCI_H4_PKT_ISO, iso, sizeof(iso)));
	CHECK(s_Bridge.UnsupportedCnt == 2);
	CHECK(s_Cmds.empty() && s_Acl.empty());

	// An ACL packet from the controller longer than a link packet is dropped
	std::vector<uint8_t> big(4 + 300, 0);

	big[2] = (uint8_t)(300 & 0xFF);
	big[3] = (uint8_t)(300 >> 8);
	BtHciCtlrH4FromCtlr(&s_Ctrl, false, big.data());
	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Bridge.BadLenCnt == 1);
	CHECK(s_Host.Rx.empty());
}

// The host does not drain: the link fills, later events are counted as lost
// and the ones before them arrive intact and in order
void TestLinkFull()
{
	Start();

	std::vector<std::vector<uint8_t>> sent;

	for (int i = 0; i < 20; i++)
	{
		std::vector<uint8_t> evt(2 + 200);

		evt[0] = 0x05;
		evt[1] = 200;
		for (size_t k = 2; k < evt.size(); k++)
		{
			evt[k] = (uint8_t)(i * 5 + k);
		}
		uint32_t before = s_Bridge.UpFailCnt;

		BtHciCtlrH4FromCtlr(&s_Ctrl, true, evt.data());
		if (s_Bridge.UpFailCnt == before)
		{
			sent.push_back(evt);
		}
	}

	CHECK(s_Bridge.UpFailCnt > 0);
	CHECK(sent.size() == 5);

	BtHciH4Process(&s_Host.H4, 0);
	BtHciH4Process(&s_Host.H4, 0);
	CHECK(s_Host.Rx.size() == sent.size());
	bool bSame = s_Host.Rx.size() == sent.size();
	for (size_t i = 0; bSame && i < sent.size(); i++)
	{
		bSame = s_Host.Rx[i].Data == sent[i];
	}
	CHECK(bSame);
	CHECK(s_Host.H4.SyncErrCnt == 0);
}

}

// Trace of the HCI host, not used by the bridge
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
	s_Ctx.Run("init", TestInit);
	s_Ctx.Run("command round trip", TestCommandRoundTrip);
	s_Ctx.Run("command status", TestCommandStatus);
	s_Ctx.Run("command refused", TestCommandRefused);
	s_Ctx.Run("acl both ways", TestAclBothWays);
	s_Ctx.Run("events up", TestEventsUp);
	s_Ctx.Run("unsupported and bad length", TestUnsupportedAndBadLength);
	s_Ctx.Run("link full", TestLinkFull);

	return s_Ctx.Finish();
}
