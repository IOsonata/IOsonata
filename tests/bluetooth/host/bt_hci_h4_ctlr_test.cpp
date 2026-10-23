/**-------------------------------------------------------------------------
@file	bt_hci_h4_ctlr_test.cpp

@brief	Host test of the controller port for a controller behind an H4 link

Links bt_hci_h4_ctlr.cpp with the UART transport (bt_hci_ctlr_uart.cpp,
BtHciUart, BtHciH4) over a scripted UART, the way bt_hci_uart_test does, and
drives it the way the host does: BtHciCtlrEnable, BtHciCtlrCommand, Send,
RxReady from the interrupt, the Bluetooth work queue and BtHciCtlrCheckStatus.
The port of bt_hci_ctlr_ipc.cpp is the same code over BtHciIpc, whose own
test covers the IPC side.

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

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "bluetooth/bt_hci_uart.h"
#include "bluetooth/bt_hci_h4_ctlr.h"
#include "crypto/crypto_uecc.h"
#include "syslog.h"
#include "bt_test_harness.h"

namespace {

bttest::Context s_Ctx("bt_hci_h4_ctlr_test");

#define CHECK(expr)		BT_CHECK(s_Ctx, expr)

// UART byte script, see bt_hci_uart_test
std::vector<uint8_t> s_Rx;
size_t s_RxPos;
std::vector<uint8_t> s_Tx;
size_t s_TxPktStart;
int s_UartInitCount;
UARTCfg_t s_UartCfg;

// Responses handed out in order, one per command sent
std::vector<std::vector<uint8_t>> s_CmdRsp;

void Script(const std::vector<uint8_t> &Bytes)
{
	s_Rx.insert(s_Rx.end(), Bytes.begin(), Bytes.end());
}

bool FakeStartRx(DevIntrf_t * const, uint32_t) { return true; }
void FakeStopRx(DevIntrf_t * const) {}
bool FakeStartTx(DevIntrf_t * const, uint32_t) { return true; }
void FakeStopTx(DevIntrf_t * const) {}
void FakeDisable(DevIntrf_t * const) {}

int FakeRxData(DevIntrf_t * const, uint8_t *pBuff, int BuffLen)
{
	size_t n = s_Rx.size() - s_RxPos;
	if (n > (size_t)BuffLen)
	{
		n = (size_t)BuffLen;
	}
	std::memcpy(pBuff, s_Rx.data() + s_RxPos, n);
	s_RxPos += n;

	return (int)n;
}

int FakeTxData(DevIntrf_t * const, const uint8_t *pData, int DataLen)
{
	s_TxPktStart = s_Tx.size();
	s_Tx.insert(s_Tx.end(), pData, pData + DataLen);

	if (s_Tx[s_TxPktStart] == BT_HCI_UART_PKT_CMD && !s_CmdRsp.empty())
	{
		Script(s_CmdRsp.front());
		s_CmdRsp.erase(s_CmdRsp.begin());
	}

	return DataLen;
}

// Command Complete of OpCode with a status and return parameters
std::vector<uint8_t> CmdComplete(uint16_t OpCode, uint8_t Status, std::vector<uint8_t> Ret = {})
{
	std::vector<uint8_t> v = {BT_HCI_UART_PKT_EVT, BT_HCI_EVT_COMMAND_COMPLETE,
							  (uint8_t)(4 + Ret.size()), 1,
							  (uint8_t)(OpCode & 0xFF), (uint8_t)(OpCode >> 8), Status};
	v.insert(v.end(), Ret.begin(), Ret.end());
	return v;
}

// What the host would see: the received packets of its RxHandler
struct RxPkt {
	bool bIsEvent;
	std::vector<uint8_t> Data;
};

std::vector<RxPkt> s_RxPkts;
BtHciDevice_t s_Hci;
BtHciCtlrDev_t s_Ctlr;

// Routes as the host does: an event through BtHciProcessEvent, which is where
// a Command Complete or Command Status finishes the waiting command; the rest
// is recorded here
void RxHandler(BtHciCtlrDev_t * const, bool bIsEvent, uint8_t *pPacket)
{
	if (bIsEvent)
	{
		BtHciProcessEvent(&s_Hci, (BtHciEvtPacket_t*)pPacket);
		if (pPacket[0] == BT_HCI_EVT_COMMAND_COMPLETE || pPacket[0] == BT_HCI_EVT_COMMAND_STATUS)
		{
			return;
		}
	}
	size_t len = bIsEvent ? 2U + pPacket[1] : 4U + (pPacket[2] | (pPacket[3] << 8));
	s_RxPkts.push_back({bIsEvent, std::vector<uint8_t>(pPacket, pPacket + len)});
}

// The Bluetooth work queue of the application, one slot, refusable
BtEvtQueHandler_t s_Queued;
bool s_bQueueRefuse;
int s_QueueCount;

void RunQueue()
{
	while (s_Queued != nullptr)
	{
		BtEvtQueHandler_t h = s_Queued;
		s_Queued = nullptr;
		h(0, nullptr);
	}
}

uint16_t Opcode(size_t PktStart)
{
	return (uint16_t)(s_Tx[PktStart + 1] | (s_Tx[PktStart + 2] << 8));
}

// Every command the port sent, in order
std::vector<uint16_t> SentOpcodes()
{
	std::vector<uint16_t> v;
	size_t i = 0;
	while (i < s_Tx.size())
	{
		if (s_Tx[i] == BT_HCI_UART_PKT_CMD)
		{
			v.push_back((uint16_t)(s_Tx[i + 1] | (s_Tx[i + 2] << 8)));
			i += 4U + s_Tx[i + 3];
		}
		else if (s_Tx[i] == BT_HCI_UART_PKT_ACL)
		{
			i += 5U + (s_Tx[i + 3] | (s_Tx[i + 4] << 8));
		}
		else
		{
			break;
		}
	}
	return v;
}

void Reset()
{
	s_Rx.clear();
	s_RxPos = 0;
	s_Tx.clear();
	s_TxPktStart = 0;
	s_UartInitCount = 0;
	std::memset(&s_UartCfg, 0, sizeof(s_UartCfg));
	s_CmdRsp.clear();
	s_RxPkts.clear();
	s_Queued = nullptr;
	s_bQueueRefuse = false;
	s_QueueCount = 0;
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	std::memset(&s_Ctlr, 0, sizeof(s_Ctlr));
	// The host installs the port as the command executor of its device
	s_Hci.Command = BtHciCtlrCommand;
}

bool Enable()
{
	BtHciCtlrCfg_t cfg = {};
	cfg.RxHandler = RxHandler;
	cfg.Role = BT_GAP_ROLE_PERIPHERAL;
	cfg.MaxDataLen = 251;
	cfg.RxPktCount = 2;
	cfg.TxPktCount = 3;

	return BtHciCtlrEnable(&s_Ctlr, &cfg);
}

static void TestStartResetsTheController()
{
	Reset();

	// Noise from the controller start ahead of the first response
	Script({0x00, 0xA5});
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));

	CHECK(Enable());
	CHECK(s_UartInitCount == 1);
	// The UART of the application, with the FIFOs of the port filled in
	CHECK(s_UartCfg.DevNo == 2 && s_UartCfg.Rate == 1000000);
	CHECK(s_UartCfg.pRxMem != nullptr && s_UartCfg.pTxMem != nullptr);
	CHECK(s_UartCfg.RxMemSize == CFIFO_MEMSIZE(2048));
	CHECK(s_UartCfg.bIntMode && s_UartCfg.bFifoBlocking);

	std::vector<uint16_t> sent = SentOpcodes();
	CHECK(sent.size() == 1 && sent[0] == BT_HCI_CMD_BASEBAND_RESET);
	CHECK(s_Hci.CmdCredit == 1);
	CHECK(s_Ctlr.Send != nullptr);
	CHECK(s_Ctlr.RxHandler == RxHandler);
}

static void TestCommandRoundTrip()
{
	Reset();
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));
	CHECK(Enable());

	// LE Read Buffer Size: length 251, 3 packets
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_CTLR_READ_BUFF_SIZE, 0, {0xFB, 0x00, 0x03}));
	uint8_t ret[3] = {0};
	CHECK(BtHciCtlrCommand(&s_Hci, BT_HCI_CMD_CTLR_READ_BUFF_SIZE, nullptr, 0, ret, sizeof(ret)) == 0);
	CHECK(ret[0] == 0xFB && ret[1] == 0 && ret[2] == 3);

	// A refused command answers its status
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_CTLR_RAND, BT_HCI_ERR_UNKNOWN_COMMAND));
	CHECK(BtHciCtlrCommand(&s_Hci, BT_HCI_CMD_CTLR_RAND, nullptr, 0, nullptr, 0) == BT_HCI_ERR_UNKNOWN_COMMAND);

	// Parameters go out as given
	uint8_t addr[6] = {1, 2, 3, 4, 5, 6};
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, 0));
	size_t before = s_Tx.size();
	CHECK(BtHciCtlrCommand(&s_Hci, BT_HCI_CMD_CTLR_SET_RANDOM_ADDR, addr, sizeof(addr), nullptr, 0) == 0);
	CHECK(s_Tx.size() == before + 4 + 6);
	CHECK(Opcode(before) == BT_HCI_CMD_CTLR_SET_RANDOM_ADDR);
	CHECK(s_Tx[before + 3] == 6 && std::memcmp(&s_Tx[before + 4], addr, 6) == 0);
}

static void TestSendIsAnAclPacket()
{
	Reset();
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));
	CHECK(Enable());

	// ACL header: handle 0x0040, length 3, then the payload
	uint8_t acl[7] = {0x40, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC};
	size_t before = s_Tx.size();
	CHECK(s_Ctlr.Send(&s_Ctlr, acl, sizeof(acl)) == sizeof(acl));
	CHECK(s_Tx.size() == before + 1 + sizeof(acl));
	CHECK(s_Tx[before] == BT_HCI_UART_PKT_ACL);
	CHECK(std::memcmp(&s_Tx[before + 1], acl, sizeof(acl)) == 0);
}

static void TestReceiveGoesThroughTheQueue()
{
	Reset();
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));
	CHECK(Enable());

	// An event and an ACL packet arrive, the interrupt says so
	Script({BT_HCI_UART_PKT_EVT, BT_HCI_EVT_DISCONN_COMPLETE, 4, 0, 0x40, 0x00, 0x13});
	Script({BT_HCI_UART_PKT_ACL, 0x40, 0x00, 0x02, 0x00, 0x11, 0x22});
	BtHciH4CtlrRxReady();
	BtHciH4CtlrRxReady();

	// Nothing is framed in the interrupt, one event is queued
	CHECK(s_RxPkts.empty());
	CHECK(s_QueueCount == 1 && s_Queued != nullptr);

	RunQueue();
	CHECK(s_RxPkts.size() == 2);
	CHECK(s_RxPkts[0].bIsEvent && s_RxPkts[0].Data[0] == BT_HCI_EVT_DISCONN_COMPLETE);
	CHECK(!s_RxPkts[1].bIsEvent && s_RxPkts[1].Data[4] == 0x11 && s_RxPkts[1].Data[5] == 0x22);
	CHECK(s_Queued == nullptr);
}

static void TestRefusedQueueIsRetriedFromCheckStatus()
{
	Reset();
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));
	CHECK(Enable());

	Script({BT_HCI_UART_PKT_EVT, BT_HCI_EVT_DISCONN_COMPLETE, 4, 0, 0x40, 0x00, 0x13});
	s_bQueueRefuse = true;
	BtHciH4CtlrRxReady();
	CHECK(s_Queued == nullptr && s_RxPkts.empty());

	// Still refused: nothing changes
	BtHciCtlrCheckStatus();
	CHECK(s_Queued == nullptr);

	// Room again: the status check queues the pending work once
	s_bQueueRefuse = false;
	BtHciCtlrCheckStatus();
	BtHciCtlrCheckStatus();
	CHECK(s_QueueCount == 1 && s_Queued != nullptr);
	RunQueue();
	CHECK(s_RxPkts.size() == 1 && s_RxPkts[0].bIsEvent);

	// Nothing pending, the status check queues nothing
	BtHciCtlrCheckStatus();
	CHECK(s_Queued == nullptr);
}

static void TestCommandResponseIsNotQueued()
{
	Reset();
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_BASEBAND_RESET, 0));
	CHECK(Enable());

	// An event is ahead of the response in the stream: the response is
	// handled by the command, the event stays for the queue
	Script({BT_HCI_UART_PKT_EVT, BT_HCI_EVT_DISCONN_COMPLETE, 4, 0, 0x40, 0x00, 0x13});
	s_CmdRsp.push_back(CmdComplete(BT_HCI_CMD_CTLR_RAND, 0, {1, 2, 3, 4, 5, 6, 7, 8}));
	uint8_t rnd[8] = {0};
	CHECK(BtHciCtlrCommand(&s_Hci, BT_HCI_CMD_CTLR_RAND, nullptr, 0, rnd, sizeof(rnd)) == 0);
	CHECK(rnd[0] == 1 && rnd[7] == 8);
	CHECK(s_RxPkts.empty());

	BtHciH4CtlrRxReady();
	RunQueue();
	CHECK(s_RxPkts.size() == 1 && s_RxPkts[0].bIsEvent);
}

static void TestSilentControllerStopsTheStart()
{
	Reset();

	// No response to any reset: the start fails after its retries, and the
	// host learns it from BtHciCtlrEnable rather than at the first command
	CHECK(Enable() == false);
	std::vector<uint16_t> sent = SentOpcodes();
	CHECK(sent.size() == BT_HCI_H4_CTLR_RESET_TRY);
	for (uint16_t op : sent)
	{
		CHECK(op == BT_HCI_CMD_BASEBAND_RESET);
	}
}

static void TestFeatureSupportIsANoOp()
{
	// The controller behind a link has every feature of its image; the
	// calls connection support and the modules make on every port exist and
	// do nothing
	BtHciCtlrPeripheralSupport();
	BtHciCtlrLinkSupport();
	BtHciCtlrCentralSupport();
	BtHciCtlrPeriodicAdvSupport();
	CHECK(true);
}

} // namespace

// The UART of the application
const UARTCfg_t g_BtHciUartCfg = {
	.DevNo = 2,
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_HW,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
	.bIrDAMode = false,
	.bIrDAInvert = false,
	.bIrDAFixPulse = false,
	.IrDAPulseDiv = 0,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART,
};

// UART init, replaces the target driver
bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	s_UartInitCount++;
	s_UartCfg = *pCfg;

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.StartRx = FakeStartRx;
	pDev->DevIntrf.StopRx = FakeStopRx;
	pDev->DevIntrf.RxData = FakeRxData;
	pDev->DevIntrf.StartTx = FakeStartTx;
	pDev->DevIntrf.StopTx = FakeStopTx;
	pDev->DevIntrf.TxData = FakeTxData;
	pDev->DevIntrf.Disable = FakeDisable;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);

	return true;
}

// What the host provides to its controller port
BtHciDevice_t *BtAppHciDevice(void)
{
	return &s_Hci;
}

bool BtEvtQue(uint32_t, void *, BtEvtQueHandler_t Handler)
{
	if (s_bQueueRefuse || s_Queued != nullptr)
	{
		return false;
	}
	s_Queued = Handler;
	s_QueueCount++;
	return true;
}

// Not exercised here
CryptoUecc *CryptoUeccCreate(void *, size_t, RngEngine *)
{
	return nullptr;
}

RngEngine *BtCryptoCtlrRng(void)
{
	return nullptr;
}

SysLog_t *SysLogGet(void)
{
	return nullptr;
}

int SysLogPrintf(SysLog_t * const, const char *, ...)
{
	return 0;
}

int main()
{
	TestStartResetsTheController();
	TestCommandRoundTrip();
	TestSendIsAnAclPacket();
	TestReceiveGoesThroughTheQueue();
	TestRefusedQueueIsRetriedFromCheckStatus();
	TestCommandResponseIsNotQueued();
	TestSilentControllerStopsTheStart();
	TestFeatureSupportIsANoOp();

	return s_Ctx.Finish();
}
