/**-------------------------------------------------------------------------
@file	bt_hci_uart_test.cpp

@brief	Host test of the HCI UART (H4) transport

The UART driver is replaced by a byte script: RxData hands out the scripted
bytes, at most RxChunk per call, and TxData records what is sent. A command
response can be appended to the script when a command goes out, which is
when a controller would answer. Command Complete and Command Status are
passed to the real BtHciProcessEvent, as the nRF91 port does.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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

#include "bluetooth/bt_hci_uart.h"
#include "bluetooth/bt_hcievt.h"
#include "syslog.h"
#include "bt_test_harness.h"

namespace {

bttest::Context s_Ctx("bt_hci_uart_test");

#define CHECK(expr)		BT_CHECK(s_Ctx, expr)

// UART byte script
std::vector<uint8_t> s_Rx;
size_t s_RxPos;
size_t s_RxChunk;
std::vector<uint8_t> s_Tx;
size_t s_TxPktStart;
int s_TxWrites;
UARTCfg_t s_UartCfg;
int s_UartInitCount;
bool s_UartInitResult;
int s_DisableCount;

// Response appended to the script when the next command goes out
std::vector<uint8_t> s_CmdRsp;
bool s_bCmdRspArmed;

void Reset()
{
	s_Rx.clear();
	s_RxPos = 0;
	s_RxChunk = 64;
	s_Tx.clear();
	s_TxPktStart = 0;
	s_TxWrites = 0;
	std::memset(&s_UartCfg, 0, sizeof(s_UartCfg));
	s_UartInitCount = 0;
	s_UartInitResult = true;
	s_DisableCount = 0;
	s_CmdRsp.clear();
	s_bCmdRspArmed = false;
}

void Script(std::initializer_list<uint8_t> Bytes)
{
	s_Rx.insert(s_Rx.end(), Bytes.begin(), Bytes.end());
}

void ScriptBytes(const std::vector<uint8_t> &Bytes)
{
	s_Rx.insert(s_Rx.end(), Bytes.begin(), Bytes.end());
}

void ArmCmdRsp(std::initializer_list<uint8_t> Bytes)
{
	s_CmdRsp.assign(Bytes.begin(), Bytes.end());
	s_bCmdRspArmed = true;
}

bool FakeStartRx(DevIntrf_t * const, uint32_t) { return true; }
void FakeStopRx(DevIntrf_t * const) {}
bool FakeStartTx(DevIntrf_t * const, uint32_t) { return true; }
void FakeStopTx(DevIntrf_t * const) {}
void FakeDisable(DevIntrf_t * const) { s_DisableCount++; }

int FakeRxData(DevIntrf_t * const, uint8_t *pBuff, int BuffLen)
{
	size_t n = s_Rx.size() - s_RxPos;
	if (n > s_RxChunk)
	{
		n = s_RxChunk;
	}
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
	// One write per packet, type byte first
	s_TxWrites++;
	s_TxPktStart = s_Tx.size();
	s_Tx.insert(s_Tx.end(), pData, pData + DataLen);

	size_t n = s_Tx.size() - s_TxPktStart;
	if (s_bCmdRspArmed && n >= 4 && s_Tx[s_TxPktStart] == BT_HCI_UART_PKT_CMD &&
		n == 4U + s_Tx[s_TxPktStart + 3])
	{
		s_bCmdRspArmed = false;
		ScriptBytes(s_CmdRsp);
	}

	return DataLen;
}

// Captured received packets
struct RxPkt {
	uint8_t Type;
	std::vector<uint8_t> Data;
};

std::vector<RxPkt> s_RxPkts;
int s_RxReadyCount;
BtHciDevice_t s_Hci;
BtHciUartDev_t s_Dev;

// Action run by the RX handler for the packet at this index, -1 for none
int s_CmdOnPkt;
uint8_t s_NestedStatus;
uint8_t s_NestedRet[4];
bool s_bNestedProcessResult;
bool s_bTryNestedProcess;

void RxHandler(BtHciUartDev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	s_RxPkts.push_back({Type, std::vector<uint8_t>(pPkt, pPkt + Len)});

	if (s_bTryNestedProcess)
	{
		s_bTryNestedProcess = false;
		s_bNestedProcessResult = BtHciUartProcess(pDev, 0);
	}

	if (s_CmdOnPkt >= 0 && (int)s_RxPkts.size() == s_CmdOnPkt + 1)
	{
		s_CmdOnPkt = -1;
		s_NestedStatus = BtHciUartCommand(pDev, &s_Hci, 0x2002, nullptr, 0,
										  s_NestedRet, sizeof(s_NestedRet));
	}
}

void CmdRspHandler(BtHciUartDev_t * const, uint8_t, uint8_t *pPkt, uint16_t)
{
	BtHciProcessEvent(&s_Hci, (BtHciEvtPacket_t*)pPkt);
}

void RxReady(BtHciUartDev_t * const)
{
	s_RxReadyCount++;
}

alignas(8) uint8_t s_PktMem[BT_HCI_UART_PKTMEM_SIZE(4)];
alignas(8) uint8_t s_SmallPktMem[BT_HCI_UART_PKTMEM_SIZE(2)];
alignas(8) uint8_t s_UartTxMem[CFIFO_MEMSIZE(512)];
alignas(8) uint8_t s_UartRxMem[CFIFO_MEMSIZE(64)];

const UARTCfg_t s_AppUartCfg = {
	.DevNo = 2,
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_HW,
	.bIntMode = false,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = false,
	.RxMemSize = sizeof(s_UartRxMem),
	.pRxMem = s_UartRxMem,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = true,
	.bIrDAMode = false,
	.bIrDAInvert = false,
	.bIrDAFixPulse = false,
	.IrDAPulseDiv = 0,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART,
};

BtHciUartCfg_t MakeCfg(uint8_t *pMem, uint32_t MemSize)
{
	BtHciUartCfg_t cfg = {};
	cfg.pUartCfg = &s_AppUartCfg;
	cfg.pPktMem = pMem;
	cfg.PktMemSize = MemSize;
	cfg.RxHandler = RxHandler;
	cfg.CmdRspHandler = CmdRspHandler;
	cfg.RxReady = RxReady;
	cfg.TimeoutMs = 1;
	return cfg;
}

void Start(uint8_t *pMem = s_PktMem, uint32_t MemSize = sizeof(s_PktMem))
{
	Reset();
	s_RxPkts.clear();
	s_RxReadyCount = 0;
	s_CmdOnPkt = -1;
	s_NestedStatus = 0xEE;
	std::memset(s_NestedRet, 0, sizeof(s_NestedRet));
	s_bNestedProcessResult = true;
	s_bTryNestedProcess = false;
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	s_Hci.CmdCredit = 1;

	BtHciUartCfg_t cfg = MakeCfg(pMem, MemSize);
	CHECK(BtHciUartInit(&s_Dev, &cfg));
}

// Command Complete of OpCode with a status and return parameters
std::vector<uint8_t> CmdComplete(uint16_t OpCode, uint8_t Status, std::vector<uint8_t> Ret = {},
								 uint8_t NbCmd = 1)
{
	std::vector<uint8_t> v = {BT_HCI_UART_PKT_EVT, BT_HCI_EVT_COMMAND_COMPLETE,
							  (uint8_t)(4 + Ret.size()), NbCmd,
							  (uint8_t)(OpCode & 0xFF), (uint8_t)(OpCode >> 8), Status};
	v.insert(v.end(), Ret.begin(), Ret.end());
	return v;
}

// Disconnection Complete, a plain queued event
std::vector<uint8_t> DiscEvt(uint8_t Hdl)
{
	return {BT_HCI_UART_PKT_EVT, 0x05, 4, 0x00, Hdl, 0x00, 0x13};
}

std::vector<uint8_t> AclPkt(uint8_t Hdl, std::vector<uint8_t> Data)
{
	std::vector<uint8_t> v = {BT_HCI_UART_PKT_ACL, Hdl, 0x20,
							  (uint8_t)(Data.size() & 0xFF), (uint8_t)(Data.size() >> 8)};
	v.insert(v.end(), Data.begin(), Data.end());
	return v;
}

} // namespace

// Trace of the HCI host, not used by the transport
SysLog_t *SysLogGet(void)
{
	static SysLog_t log = {};
	return &log;
}

int SysLogPrintf(SysLog_t * const, const char *, ...)
{
	return 0;
}

// UART init, replaces the target driver
bool UARTInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg)
{
	s_UartInitCount++;
	s_UartCfg = *pCfg;
	if (s_UartInitResult == false)
	{
		return false;
	}

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

static void TestInit()
{
	Reset();

	BtHciUartCfg_t cfg = MakeCfg(s_PktMem, sizeof(s_PktMem));
	CHECK(BtHciUartInit(nullptr, &cfg) == false);
	CHECK(BtHciUartInit(&s_Dev, nullptr) == false);

	cfg.pPktMem = s_PktMem + 1;
	CHECK(BtHciUartInit(&s_Dev, &cfg) == false);

	cfg = MakeCfg(s_PktMem, BT_HCI_UART_PKTMEM_SIZE(1) - 1);
	CHECK(BtHciUartInit(&s_Dev, &cfg) == false);
	CHECK(s_UartInitCount == 0);

	// UART configuration is copied with what the transport needs forced
	cfg = MakeCfg(s_PktMem, sizeof(s_PktMem));
	CHECK(BtHciUartInit(&s_Dev, &cfg));
	CHECK(s_UartInitCount == 1);
	CHECK(s_UartCfg.bIntMode);
	CHECK(s_UartCfg.bFifoBlocking);
	CHECK(s_UartCfg.EvtCallback != nullptr);
	CHECK(s_UartCfg.DevNo == 2 && s_UartCfg.Rate == 1000000);
	CHECK(s_Dev.TimeoutMs == 1);
	CHECK(s_Dev.Uart.DevIntrf.MaxRetry == 1);

	cfg.TimeoutMs = 0;
	CHECK(BtHciUartInit(&s_Dev, &cfg));
	CHECK(s_Dev.TimeoutMs == BT_HCI_UART_TIMEOUT_DEFAULT);

	// UART refused
	s_UartInitResult = false;
	CHECK(BtHciUartInit(&s_Dev, &cfg) == false);
	s_UartInitResult = true;

	// TX FIFO that cannot hold one packet
	alignas(8) static uint8_t smalltx[CFIFO_MEMSIZE(BT_HCI_UART_TXFIFO_MIN - 1)];
	UARTCfg_t ucfg = s_AppUartCfg;
	ucfg.pTxMem = smalltx;
	ucfg.TxMemSize = sizeof(smalltx);
	cfg.pUartCfg = &ucfg;
	CHECK(BtHciUartInit(&s_Dev, &cfg) == false);
	CHECK(s_DisableCount == 1);

	alignas(8) static uint8_t mintx[CFIFO_MEMSIZE(BT_HCI_UART_TXFIFO_MIN)];
	ucfg.pTxMem = mintx;
	ucfg.TxMemSize = sizeof(mintx);
	CHECK(BtHciUartInit(&s_Dev, &cfg));
}

static void TestRxReady()
{
	Start();

	UARTEvtHandler_t cb = s_UartCfg.EvtCallback;
	cb(&s_Dev.Uart, UART_EVT_RXDATA, nullptr, 1);
	cb(&s_Dev.Uart, UART_EVT_RXTIMEOUT, nullptr, 0);
	cb(&s_Dev.Uart, UART_EVT_TXREADY, nullptr, 10);
	cb(&s_Dev.Uart, UART_EVT_LINESTATE, nullptr, 0);
	CHECK(s_RxReadyCount == 2);
}

static void TestFramingByteByByte()
{
	Start();
	s_RxChunk = 1;

	ScriptBytes(DiscEvt(0x40));
	ScriptBytes(AclPkt(0x40, {1, 2, 3, 4, 5, 6, 7}));
	Script({BT_HCI_UART_PKT_ISO, 0x41, 0x00, 0x03, 0xC0, 0xA, 0xB, 0xC});	// top length bits are flags
	Script({BT_HCI_UART_PKT_SCO, 0x42, 0x00, 0x02, 0x55, 0x66});
	Script({BT_HCI_UART_PKT_EVT, 0x3E, 0x00});								// empty body

	CHECK(BtHciUartProcess(&s_Dev, 0) == false);
	CHECK(s_RxPkts.size() == 5);
	if (s_RxPkts.size() == 5)
	{
		CHECK(s_RxPkts[0].Type == BT_HCI_UART_PKT_EVT && s_RxPkts[0].Data.size() == 6);
		CHECK(s_RxPkts[0].Data[0] == 0x05 && s_RxPkts[0].Data[5] == 0x13);
		CHECK(s_RxPkts[1].Type == BT_HCI_UART_PKT_ACL && s_RxPkts[1].Data.size() == 11);
		CHECK(s_RxPkts[1].Data[4] == 1 && s_RxPkts[1].Data[10] == 7);
		CHECK(s_RxPkts[2].Type == BT_HCI_UART_PKT_ISO && s_RxPkts[2].Data.size() == 7);
		CHECK(s_RxPkts[3].Type == BT_HCI_UART_PKT_SCO && s_RxPkts[3].Data.size() == 5);
		CHECK(s_RxPkts[4].Type == BT_HCI_UART_PKT_EVT && s_RxPkts[4].Data.size() == 2);
	}
	CHECK(s_Dev.SyncErrCnt == 0 && s_Dev.DropCnt == 0);
}

static void TestResync()
{
	Start();

	Script({0x00, 0xFF, 0x7F});
	ScriptBytes(DiscEvt(1));
	Script({0x06});
	ScriptBytes(DiscEvt(2));

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_Dev.SyncErrCnt == 4);
	CHECK(s_RxPkts.size() == 2);
	if (s_RxPkts.size() == 2)
	{
		CHECK(s_RxPkts[0].Data[3] == 1 && s_RxPkts[1].Data[3] == 2);
	}
}

static void TestOversizeDropped()
{
	Start();
	s_RxChunk = 7;

	std::vector<uint8_t> big(BT_HCI_UART_PKT_MAXLEN, 0x02);	// header makes it 4 too long
	ScriptBytes(AclPkt(0x40, big));
	ScriptBytes(DiscEvt(3));

	// Largest packet that fits
	std::vector<uint8_t> fit(BT_HCI_UART_PKT_MAXLEN - 4, 0x5A);
	ScriptBytes(AclPkt(0x41, fit));

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_Dev.DropCnt == 1);
	CHECK(s_Dev.SyncErrCnt == 0);
	CHECK(s_RxPkts.size() == 2);
	if (s_RxPkts.size() == 2)
	{
		CHECK(s_RxPkts[0].Type == BT_HCI_UART_PKT_EVT && s_RxPkts[0].Data[3] == 3);
		CHECK(s_RxPkts[1].Data.size() == BT_HCI_UART_PKT_MAXLEN);
		CHECK(s_RxPkts[1].Data.back() == 0x5A);
	}
}

static void TestQueueFullLeavesBytesInUart()
{
	Start(s_SmallPktMem, sizeof(s_SmallPktMem));

	for (uint8_t i = 0; i < 5; i++)
	{
		ScriptBytes(DiscEvt(i));
	}

	// Two slots and the spare: three packets framed, the fourth one is not
	// started
	BtHciUartPoll(&s_Dev);
	CHECK(CFifoUsed(s_Dev.hPktFifo) == 2);
	CHECK(s_Dev.bSpareHeld);
	CHECK(s_RxPkts.empty());
	CHECK(s_RxPos < s_Rx.size());

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_RxPkts.size() == 5);
	for (size_t i = 0; i < s_RxPkts.size(); i++)
	{
		CHECK(s_RxPkts[i].Data[3] == i);
	}
	CHECK(s_RxPos == s_Rx.size());
}

static void TestSparePacketKeepsOrder()
{
	Start(s_SmallPktMem, sizeof(s_SmallPktMem));

	// Two packets fill the queue, a third one is half received into the spare
	// slot when the UART runs dry.
	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	std::vector<uint8_t> acl = AclPkt(0x40, {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6});
	s_Rx.insert(s_Rx.end(), acl.begin(), acl.begin() + 5);

	// One packet dispatched frees a queue slot while the spare is incomplete
	CHECK(BtHciUartProcess(&s_Dev, 1) == true);
	CHECK(s_RxPkts.size() == 1);

	// The rest of the spare packet and one more behind it
	s_Rx.insert(s_Rx.end(), acl.begin() + 5, acl.end());
	ScriptBytes(DiscEvt(3));

	CHECK(BtHciUartProcess(&s_Dev, 0) == false);
	CHECK(s_RxPkts.size() == 4);
	if (s_RxPkts.size() == 4)
	{
		CHECK(s_RxPkts[0].Data[3] == 1);
		CHECK(s_RxPkts[1].Data[3] == 2);
		CHECK(s_RxPkts[2].Type == BT_HCI_UART_PKT_ACL && s_RxPkts[2].Data.size() == 10);
		CHECK(s_RxPkts[2].Data[4] == 0xA1 && s_RxPkts[2].Data[9] == 0xA6);
		CHECK(s_RxPkts[3].Type == BT_HCI_UART_PKT_EVT && s_RxPkts[3].Data[3] == 3);
	}
	CHECK(s_Dev.bSpareHeld == false);
}

static void TestFlush()
{
	Start(s_SmallPktMem, sizeof(s_SmallPktMem));

	// Queue full, spare held, a packet half framed: all of it goes
	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	ScriptBytes(DiscEvt(3));
	Script({BT_HCI_UART_PKT_EVT, 0x05});
	BtHciUartPoll(&s_Dev);
	CHECK(CFifoUsed(s_Dev.hPktFifo) == 2 && s_Dev.bSpareHeld);

	BtHciUartFlush(&s_Dev);
	CHECK(CFifoUsed(s_Dev.hPktFifo) == 0);
	CHECK(s_Dev.bSpareHeld == false);
	CHECK(s_RxPos == s_Rx.size());

	// Framing starts again at the next byte
	ScriptBytes(DiscEvt(4));
	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_RxPkts.size() == 1);
	if (s_RxPkts.size() == 1)
	{
		CHECK(s_RxPkts[0].Data[3] == 4);
	}
}

static void TestSendWhileUartBusy()
{
	Start();

	// The interface is held by a read in another context: nothing is written
	atomic_flag_test_and_set(&s_Dev.Uart.DevIntrf.bBusy);
	uint8_t acl[6] = {0x40, 0x00, 0x02, 0x00, 0xAA, 0xBB};
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, sizeof(acl)) == false);
	CHECK(s_Tx.empty());
	CHECK(s_Dev.TxFailCnt == 1);
	atomic_flag_clear(&s_Dev.Uart.DevIntrf.bBusy);

	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, sizeof(acl)));
	CHECK(s_TxWrites == 1 && s_Tx.size() == 7);
}

static void TestMaxPkt()
{
	Start();

	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	ScriptBytes(DiscEvt(3));

	CHECK(BtHciUartProcess(&s_Dev, 2) == true);
	CHECK(s_RxPkts.size() == 2);
	CHECK(BtHciUartProcess(&s_Dev, 2) == false);
	CHECK(s_RxPkts.size() == 3);
}

static void TestNestedProcessIgnored()
{
	Start();

	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	s_bTryNestedProcess = true;

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_bNestedProcessResult == false);
	CHECK(s_RxPkts.size() == 2);
	if (s_RxPkts.size() == 2)
	{
		CHECK(s_RxPkts[0].Data[3] == 1 && s_RxPkts[1].Data[3] == 2);
	}
}

static void TestCommandWire()
{
	Start();

	ArmCmdRsp({});
	s_CmdRsp = CmdComplete(0x0C03, 0);
	uint8_t param[3] = {0x11, 0x22, 0x33};
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x2006, param, sizeof(param), nullptr, 0) == 0xFF);
	CHECK(s_Tx.size() == 7);
	if (s_Tx.size() == 7)
	{
		CHECK(s_Tx[0] == BT_HCI_UART_PKT_CMD && s_Tx[1] == 0x06 && s_Tx[2] == 0x20);
		CHECK(s_Tx[3] == 3 && s_Tx[4] == 0x11 && s_Tx[6] == 0x33);
	}
	// The response to another opcode did not complete it
	CHECK(s_Hci.CmdOpCode == 0);
	CHECK(s_Hci.pCmdRet == nullptr);
	CHECK(s_Hci.CmdCredit == 1);

	// ACL goes out with its type byte
	s_Tx.clear();
	uint8_t acl[6] = {0x40, 0x00, 0x02, 0x00, 0xAA, 0xBB};
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, sizeof(acl)));
	CHECK(s_Tx.size() == 7 && s_Tx[0] == BT_HCI_UART_PKT_ACL && s_Tx[6] == 0xBB);

	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, 0) == false);
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, nullptr, 6) == false);
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, BT_HCI_UART_PKT_MAXLEN + 1) == false);
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x2006, nullptr, 3, nullptr, 0) ==
		  BT_HCI_ERR_INVALID_COMMAND_PARAM);
}

static void TestCommandComplete()
{
	Start();

	// Events already waiting stay queued, the response is picked from behind them
	ScriptBytes(DiscEvt(7));
	ScriptBytes(AclPkt(0x40, {9, 9}));
	s_CmdRsp = CmdComplete(0x2002, 0, {0xFB, 0x00, 0x06}, 5);
	s_bCmdRspArmed = true;

	uint8_t ret[3] = {};
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x2002, nullptr, 0, ret, sizeof(ret)) == 0);
	CHECK(ret[0] == 0xFB && ret[1] == 0x00 && ret[2] == 0x06);
	CHECK(s_Hci.CmdCredit == 5);
	CHECK(s_Hci.CmdOpCode == 0);
	CHECK(s_RxPkts.empty());
	CHECK(CFifoUsed(s_Dev.hPktFifo) == 2);

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_RxPkts.size() == 2);
	if (s_RxPkts.size() == 2)
	{
		CHECK(s_RxPkts[0].Type == BT_HCI_UART_PKT_EVT && s_RxPkts[0].Data[3] == 7);
		CHECK(s_RxPkts[1].Type == BT_HCI_UART_PKT_ACL);
	}

	// Error status
	ArmCmdRsp({});
	s_CmdRsp = CmdComplete(0x2006, BT_HCI_ERR_INVALID_COMMAND_PARAM);
	uint8_t p = 0;
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x2006, &p, 1, nullptr, 0) == BT_HCI_ERR_INVALID_COMMAND_PARAM);
}

static void TestCommandStatus()
{
	Start();

	ArmCmdRsp({BT_HCI_UART_PKT_EVT, BT_HCI_EVT_COMMAND_STATUS, 4, 0x00, 1, 0x0D, 0x20});
	uint8_t param[25] = {};
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x200D, param, sizeof(param), nullptr, 0) == 0);

	ArmCmdRsp({BT_HCI_UART_PKT_EVT, BT_HCI_EVT_COMMAND_STATUS, 4, 0x0C, 1, 0x0D, 0x20});
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x200D, param, sizeof(param), nullptr, 0) == 0x0C);
	CHECK(s_RxPkts.empty());
}

static void TestCommandTimeout()
{
	Start();

	uint8_t ret[2] = {};
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x0C03, nullptr, 0, ret, sizeof(ret)) == BT_HCI_UART_ERR_TIMEOUT);
	CHECK(s_Hci.CmdOpCode == 0);
	CHECK(s_Hci.pCmdRet == nullptr);
	CHECK(s_Hci.CmdCredit == 1);
	CHECK(s_Dev.bCmdActive == false);

	// A late response matches nothing and only sets the credits
	ScriptBytes(CmdComplete(0x0C03, 0, {}, 2));
	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_Hci.CmdCredit == 2);
	CHECK(s_Hci.CmdDone == false);
	CHECK(s_RxPkts.empty());
}

static void TestCommandCredit()
{
	Start();

	// No credit and none returned: refused without sending
	s_Hci.CmdCredit = 0;
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0) == BT_HCI_UART_ERR_TIMEOUT);
	CHECK(s_Tx.empty());
	CHECK(s_Hci.CmdCredit == 1);

	// A Command Complete without opcode returns the credit, then the command goes
	s_Hci.CmdCredit = 0;
	ScriptBytes(CmdComplete(0x0000, 0, {}, 1));
	s_CmdRsp = CmdComplete(0x0C03, 0);
	s_bCmdRspArmed = true;
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0) == 0);
	CHECK(s_Tx.size() == 4);
}

static void TestCommandFromHandler()
{
	Start();

	// The handler of the second packet sends a command. Its response is
	// behind two more packets.
	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	s_CmdOnPkt = 1;
	s_CmdRsp = DiscEvt(3);
	std::vector<uint8_t> tail = AclPkt(0x40, {4});
	s_CmdRsp.insert(s_CmdRsp.end(), tail.begin(), tail.end());
	std::vector<uint8_t> cc = CmdComplete(0x2002, 0, {0x1B, 0x00, 0x03, 0x77});
	s_CmdRsp.insert(s_CmdRsp.end(), cc.begin(), cc.end());
	s_bCmdRspArmed = true;

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_NestedStatus == 0);
	CHECK(s_NestedRet[0] == 0x1B && s_NestedRet[2] == 0x03 && s_NestedRet[3] == 0x77);
	CHECK(s_RxPkts.size() == 4);
	if (s_RxPkts.size() == 4)
	{
		CHECK(s_RxPkts[0].Data[3] == 1);
		CHECK(s_RxPkts[1].Data[3] == 2);
		CHECK(s_RxPkts[2].Data[3] == 3);
		CHECK(s_RxPkts[3].Type == BT_HCI_UART_PKT_ACL && s_RxPkts[3].Data[4] == 4);
	}
}

static void TestCommandFromHandlerQueueFull()
{
	Start(s_SmallPktMem, sizeof(s_SmallPktMem));

	// The handler of the first packet sends a command while the second one
	// holds the other slot. The response is framed into the spare slot.
	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	s_CmdOnPkt = 0;
	s_CmdRsp = CmdComplete(0x2002, 0, {0x1B, 0x00, 0x03});
	s_bCmdRspArmed = true;

	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_NestedStatus == 0);
	CHECK(s_RxPkts.size() == 2);
}

static void TestCommandBehindFullQueueTimesOut()
{
	Start(s_SmallPktMem, sizeof(s_SmallPktMem));

	// Both slots and the spare taken ahead of the response: the response is
	// not reached, the command times out and nothing is lost.
	ScriptBytes(DiscEvt(1));
	ScriptBytes(DiscEvt(2));
	ScriptBytes(DiscEvt(3));
	ScriptBytes(CmdComplete(0x2002, 0, {0x1B, 0x00, 0x03}));

	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x2002, nullptr, 0, nullptr, 0) == BT_HCI_UART_ERR_TIMEOUT);
	BtHciUartProcess(&s_Dev, 0);
	CHECK(s_RxPkts.size() == 3);
}

static void TestTxNoRoom()
{
	Start();

	// Fill the TX FIFO so a 6 byte packet with its type byte does not fit
	int avail = CFifoAvail(s_Dev.Uart.hTxFifo);
	for (int i = 0; i < avail - 6; i++)
	{
		CFifoPut(s_Dev.Uart.hTxFifo);
	}
	uint8_t acl[6] = {};
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, sizeof(acl)) == false);
	CHECK(s_Dev.TxFailCnt == 1);
	CHECK(s_Tx.empty());
	CHECK(BtHciUartSend(&s_Dev, BT_HCI_UART_PKT_ACL, acl, 5));
	CHECK(s_Tx.size() == 6);

	// A command that cannot be sent clears its wait state
	CHECK(BtHciUartCommand(&s_Dev, &s_Hci, 0x0C03, nullptr, 0, nullptr, 0) == BT_HCI_UART_ERR_TIMEOUT);
	CHECK(s_Hci.CmdOpCode == 0 && s_Hci.CmdCredit == 1);
}

int main()
{
	s_Ctx.Run("init", TestInit);
	s_Ctx.Run("rx ready", TestRxReady);
	s_Ctx.Run("framing byte by byte", TestFramingByteByByte);
	s_Ctx.Run("resync", TestResync);
	s_Ctx.Run("oversize dropped", TestOversizeDropped);
	s_Ctx.Run("queue full leaves bytes in uart", TestQueueFullLeavesBytesInUart);
	s_Ctx.Run("spare packet keeps order", TestSparePacketKeepsOrder);
	s_Ctx.Run("flush", TestFlush);
	s_Ctx.Run("send while uart busy", TestSendWhileUartBusy);
	s_Ctx.Run("max packets per call", TestMaxPkt);
	s_Ctx.Run("nested process ignored", TestNestedProcessIgnored);
	s_Ctx.Run("command wire format", TestCommandWire);
	s_Ctx.Run("command complete", TestCommandComplete);
	s_Ctx.Run("command status", TestCommandStatus);
	s_Ctx.Run("command timeout", TestCommandTimeout);
	s_Ctx.Run("command credit", TestCommandCredit);
	s_Ctx.Run("command from handler", TestCommandFromHandler);
	s_Ctx.Run("command from handler queue full", TestCommandFromHandlerQueueFull);
	s_Ctx.Run("command behind full queue", TestCommandBehindFullQueueTimesOut);
	s_Ctx.Run("tx no room", TestTxNoRoom);

	return s_Ctx.Finish();
}
