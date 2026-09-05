/**-------------------------------------------------------------------------
@file	usb_intrf_test.cpp

@brief	Host regression tests for the UsbIntrf endpoint-pair data path.

The fake controller records the fixed staging buffers submitted by UsbIntrf.
Tests verify that OUT completion copies whole packets into the application
CFifo, reuses the same transfer buffer and applies USB backpressure when the
FIFO is full.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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
#include <string.h>

#include "usb/usb_intrf.h"

#define MPS			64U
#define BUFFER_SIZE	128U
#define SLOTS		4U
#define EP_NO		1U
#define PACKET_SLOTS	3U
#define PACKET_BLOCK_SIZE (sizeof(UsbPktHdr_t) + MPS)

static uint8_t *s_OutBuf;
static uint8_t *s_InBuf;
static uint16_t s_OutLen;
static uint16_t s_InLen;
static bool s_OutBusy;
static bool s_InBusy;
static int s_OutSubmitCnt;
static int s_InSubmitCnt;
static bool s_XferOk = true;
static bool s_HighSpeed;

extern "C" {
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return s_HighSpeed; }
void UsbCtrlrIntEnable(int) {}
void UsbCtrlrIntDisable(int) {}
void UsbCtrlrConnect(int) {}
void UsbCtrlrDisconnect(int) {}
void UsbCtrlrRemoteWakeup(int) {}
void UsbCtrlrSofEnable(int, bool) {}
void UsbCtrlrSetAddress(int, uint8_t) {}
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
void UsbCtrlrEpClose(int, uint8_t) {}
void UsbCtrlrEpCloseAll(int) {}
bool UsbCtrlrEpBusy(int, uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? s_InBusy : s_OutBusy;
}
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint8_t *pBuf, uint16_t Len)
{
	if (!s_XferOk) { return false; }
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		if (s_InBusy) { return false; }
		s_InBuf = pBuf;
		s_InLen = Len;
		s_InBusy = true;
		s_InSubmitCnt++;
	}
	else
	{
		if (s_OutBusy) { return false; }
		s_OutBuf = pBuf;
		s_OutLen = Len;
		s_OutBusy = true;
		s_OutSubmitCnt++;
	}
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(SLOTS, BUFFER_SIZE)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_TxPacketMem[
	CFIFO_TOTAL_MEMSIZE(PACKET_SLOTS, PACKET_BLOCK_SIZE)];
alignas(4) static uint8_t s_RxTransfer[BUFFER_SIZE];
alignas(4) static uint8_t s_TxTransfer[BUFFER_SIZE];
static UsbDevIntrf_t s_Intrf;

static void Deliver(const uint8_t *pData, uint16_t Len)
{
	CHECK(s_OutBusy);
	CHECK(s_OutBuf == s_RxTransfer);
	CHECK(Len <= s_OutLen);
	if (!s_OutBusy || s_OutBuf == nullptr) { return; }
	if (Len > 0U) { memcpy(s_OutBuf, pData, Len); }
	s_OutBusy = false;
	s_OutBuf = nullptr;
	UsbIntrfXferComplete(&s_Intrf, USB_ENDPADDR_DIROUT(EP_NO), Len,
						 USB_CTRLR_XFER_SUCCESS);
}

static bool Setup(void)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = EP_NO;
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = (int)sizeof(s_RxMem);
	cfg.pTxFifoMem = s_TxMem;
	cfg.TxFifoMemSize = (int)sizeof(s_TxMem);
	cfg.TxFifoBlkSize = 1U;
	cfg.BufferSize = BUFFER_SIZE;
	cfg.pRxBuffer = s_RxTransfer;
	cfg.pTxBuffer = s_TxTransfer;
	memset(static_cast<void *>(&s_Intrf), 0, sizeof(s_Intrf));
	s_OutBuf = nullptr;
	s_InBuf = nullptr;
	s_OutBusy = false;
	s_InBusy = false;
	s_OutSubmitCnt = 0;
	s_InSubmitCnt = 0;
	s_XferOk = true;
	s_HighSpeed = false;
	return UsbIntrfInit(&s_Intrf, &cfg) && UsbIntrfConfigure(&s_Intrf, MPS);
}

static bool SetupPacketMode(void)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = EP_NO;
	cfg.bBlocking = true;
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = (int)sizeof(s_RxMem);
	cfg.pTxFifoMem = s_TxPacketMem;
	cfg.TxFifoMemSize = (int)sizeof(s_TxPacketMem);
	cfg.TxFifoBlkSize = PACKET_BLOCK_SIZE;
	cfg.BufferSize = BUFFER_SIZE;
	cfg.pRxBuffer = s_RxTransfer;
	cfg.pTxBuffer = s_TxTransfer;
	memset(static_cast<void *>(&s_Intrf), 0, sizeof(s_Intrf));
	s_OutBuf = nullptr;
	s_InBuf = nullptr;
	s_OutBusy = false;
	s_InBusy = false;
	s_OutSubmitCnt = 0;
	s_InSubmitCnt = 0;
	s_XferOk = true;
	s_HighSpeed = false;
	return UsbIntrfInit(&s_Intrf, &cfg) && UsbIntrfConfigure(&s_Intrf, MPS);
}

static UsbPkt_t *PacketAt(uint8_t *pBlocks, unsigned Index)
{
	return reinterpret_cast<UsbPkt_t *>(
		pBlocks + Index * PACKET_BLOCK_SIZE);
}

static void TestGeometry(void)
{
	CHECK(Setup());
	CHECK(s_Intrf.EpNo == EP_NO);
	CHECK(CFifoAvail(s_Intrf.hRxFifo) == (int)SLOTS);
	CHECK(CFifoBlockSize(s_Intrf.hRxFifo) ==
		  USB_INTRF_PKT_BLKSIZE(BUFFER_SIZE));
	CHECK(s_OutSubmitCnt == 1);
	CHECK(s_OutLen == MPS);
}

static void TestStagingBuffer(void)
{
	CHECK(Setup());
	CHECK(s_OutBuf == s_RxTransfer);
	const uint8_t in[3] = { 1, 2, 3 };
	Deliver(in, sizeof(in));
	CHECK(s_OutBuf == s_RxTransfer);
	CHECK(s_OutSubmitCnt == 2);
	UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(CFifoPeek(s_Intrf.hRxFifo));
	CHECK(pPacket != nullptr);
	CHECK(pPacket != reinterpret_cast<UsbPkt_t *>(s_RxTransfer));
	CHECK(pPacket != nullptr && pPacket->Hdr.Length == sizeof(in));
	CHECK(pPacket != nullptr && memcmp(pPacket->Data, in, sizeof(in)) == 0);
	uint8_t out[sizeof(in)] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) ==
		  (int)sizeof(out));
	CHECK(s_OutSubmitCnt == 2);
}

static void TestWholePackets(void)
{
	CHECK(Setup());
	const uint8_t p1[4] = { 'a', 'b', 'c', 'd' };
	const uint8_t p2[3] = { 'e', 'f', 'g' };
	Deliver(p1, sizeof(p1));
	Deliver(p2, sizeof(p2));

	uint8_t out[8] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 2) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 2);
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 7);
	CHECK(memcmp(out, "abcdefg", 7) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
}

static void TestZlp(void)
{
	CHECK(Setup());
	Deliver(nullptr, 0);
	const uint8_t in[2] = { 'x', 'y' };
	Deliver(in, sizeof(in));
	uint8_t out[4] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 2);
	CHECK(memcmp(out, in, sizeof(in)) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
}

static void TestBackpressure(void)
{
	CHECK(Setup());
	uint8_t in[8] = {};
	for (uint32_t i = 0; i < SLOTS; i++)
	{
		in[0] = (uint8_t)i;
		Deliver(in, sizeof(in));
	}
	CHECK(CFifoAvail(s_Intrf.hRxFifo) == 0);
	CHECK(!s_OutBusy);
	CHECK(s_Intrf.RxDropCnt == 0U);

	uint8_t out[8] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
	CHECK(out[0] == 0U);
	CHECK(s_OutBusy);
	CHECK(s_OutBuf == s_RxTransfer);
}

// Foreground may observe available storage just before the OUT completion
// interrupt fills the final slot and leaves the endpoint unarmed. Releasing a
// packet must therefore retry OUT even when the FIFO was not seen full.
static void TestReleasedStorageRearmsOut(void)
{
	CHECK(Setup());
	const uint8_t in[4] = { 1, 2, 3, 4 };
	Deliver(in, sizeof(in));
	CHECK(s_OutBusy);

	// Model the completion interrupt winning after the foreground full check.
	s_OutBusy = false;
	s_OutBuf = nullptr;
	const int submitted = s_OutSubmitCnt;

	uint8_t out[sizeof(in)] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) ==
		  (int)sizeof(out));
	CHECK(memcmp(out, in, sizeof(out)) == 0);
	CHECK(s_OutBusy);
	CHECK(s_OutSubmitCnt == submitted + 1);
}

static void TestWrap(void)
{
	CHECK(Setup());
	uint8_t in[16];
	uint8_t out[16];
	for (int i = 0; i < 1000; i++)
	{
		memset(in, (uint8_t)i, sizeof(in));
		Deliver(in, sizeof(in));
		CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 16);
		if (memcmp(out, in, sizeof(in)) != 0) { CHECK(false); break; }
	}
}

static void TestFailedAndWrongEndpoint(void)
{
	CHECK(Setup());
	const int used = CFifoUsed(s_Intrf.hRxFifo);
	UsbIntrfXferComplete(&s_Intrf, USB_ENDPADDR_DIROUT(2U), 0,
						 USB_CTRLR_XFER_FAILED);
	CHECK(s_Intrf.RxDropCnt == 0U);
	s_OutBusy = false;
	s_OutBuf = nullptr;
	UsbIntrfXferComplete(&s_Intrf, USB_ENDPADDR_DIROUT(EP_NO), 0,
						 USB_CTRLR_XFER_FAILED);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == used);
	CHECK(s_Intrf.RxDropCnt == 1U);
	CHECK(s_OutBusy);
}

// Rate reports the negotiated link rate and refuses to be set. An
// unconfigured endpoint has no link, so it reports zero.
static void TestRate(void)
{
	CHECK(Setup());
	CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == USB_LINK_RATE_FULL);

	// Any request comes back as the rate already running, the same answer
	// the getter gives. 0xFFFFFFFF is the largest rate the parameter can
	// express, so nothing can ask for more than this.
	CHECK(DeviceIntrfSetRate(&s_Intrf.DevIntrf, 0xFFFFFFFFU) ==
		  USB_LINK_RATE_FULL);
	CHECK(DeviceIntrfSetRate(&s_Intrf.DevIntrf, 1U) == USB_LINK_RATE_FULL);

	s_HighSpeed = true;
	CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == USB_LINK_RATE_HIGH);
	CHECK(DeviceIntrfSetRate(&s_Intrf.DevIntrf, 0U) == USB_LINK_RATE_HIGH);

	UsbIntrfUnconfigure(&s_Intrf);
	CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == 0U);
	CHECK(DeviceIntrfSetRate(&s_Intrf.DevIntrf, USB_LINK_RATE_HIGH) == 0U);
}

static void TestUnconfigure(void)
{
	CHECK(Setup());
	UsbIntrfUnconfigure(&s_Intrf);
	CHECK(s_Intrf.Mps == 0U);
	CHECK(s_Intrf.hRxFifo != nullptr);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
	CHECK(s_Intrf.EpNo == EP_NO);
	s_OutBusy = false;
	CHECK(UsbIntrfConfigure(&s_Intrf, MPS));
	CHECK(s_OutBusy);
}

// EnCnt gates every arm, and UsbIntrfDisable does nothing on its own, so
// DeviceIntrfEnable is the only thing that restarts receive after a disable.
// A packet completing while disabled is discarded rather than stored.
static void TestDisableEnable(void)
{
	CHECK(Setup());
	const uint8_t in[4] = { 1, 2, 3, 4 };

	DeviceIntrfDisable(&s_Intrf.DevIntrf);
	Deliver(in, sizeof(in));
	CHECK(!s_OutBusy);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);

	const int submitted = s_OutSubmitCnt;
	DeviceIntrfEnable(&s_Intrf.DevIntrf);
	CHECK(s_OutBusy);
	CHECK(s_OutSubmitCnt == submitted + 1);
	CHECK(s_OutBuf == s_RxTransfer);
	CHECK(s_OutLen == MPS);

	const uint8_t after[2] = { 'p', 'q' };
	Deliver(after, sizeof(after));
	uint8_t out[8] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 2);
	CHECK(memcmp(out, after, sizeof(after)) == 0);
}

static void CompleteIn(uint16_t Len)
{
	CHECK(s_InBusy);
	s_InBusy = false;
	UsbIntrfXferComplete(&s_Intrf, USB_ENDPADDR_DIRIN(EP_NO), Len,
						 USB_CTRLR_XFER_SUCCESS);
}

static void TestTxChaining(void)
{
	CHECK(Setup());
	uint8_t data[MPS * 2U + 3U];
	for (uint32_t i = 0; i < sizeof(data); i++)
	{
		data[i] = (uint8_t)i;
	}

	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, data, sizeof(data)) ==
		  (int)sizeof(data));
	CHECK(s_InBusy && s_InBuf == s_TxTransfer && s_InLen == MPS);
	CHECK(memcmp(s_InBuf, data, MPS) == 0);
	CompleteIn(MPS);
	CHECK(s_InBusy && s_InLen == MPS);
	CHECK(memcmp(s_InBuf, data + MPS, MPS) == 0);
	CompleteIn(MPS);
	CHECK(s_InBusy && s_InLen == 3U);
	CHECK(memcmp(s_InBuf, data + MPS * 2U, 3U) == 0);

	// An empty FIFO is the one thing that releases the flag.
	CompleteIn(3U);
	CHECK(!s_InBusy);
	CHECK(s_InSubmitCnt == 3);

	const uint8_t tail[3] = { 1, 2, 3 };
	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, tail, sizeof(tail)) == 3);
	CHECK(s_InBusy && s_InLen == sizeof(tail));
	CompleteIn(sizeof(tail));
	CHECK(!s_InBusy);
}

// While the flag is held the producer only appends, so what arrives during a
// transfer is still there for the completion to pick up.
static void TestTxAccumulatesDuringTransfer(void)
{
	CHECK(Setup());
	uint8_t byte = 0xA5;

	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, &byte, 1) == 1);
	CHECK(s_InBusy && s_InLen == 1U);
	CHECK(s_InSubmitCnt == 1);

	// Everything written now is queued, not sent.
	for (int i = 0; i < 40; i++)
	{
		CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, &byte, 1) == 1);
	}
	CHECK(s_InSubmitCnt == 1);
	CHECK(CFifoUsed(s_Intrf.hTxFifo) == 40);

	// The completion takes the whole accumulation in one packet.
	CompleteIn(1U);
	CHECK(s_InBusy && s_InLen == 40U);
	CHECK(s_InSubmitCnt == 2);
	CompleteIn(40U);
	CHECK(!s_InBusy);
}

static void TestTxPacketMode(void)
{
	CHECK(SetupPacketMode());
	CHECK(UsbIntrfRequestToSend(&s_Intrf, PACKET_BLOCK_SIZE));
	CHECK(UsbIntrfRequestToSend(&s_Intrf, PACKET_BLOCK_SIZE * 2U));
	CHECK(!UsbIntrfRequestToSend(&s_Intrf, PACKET_BLOCK_SIZE - 1U));

	alignas(4) uint8_t packets[PACKET_BLOCK_SIZE * 2U] = {};
	UsbPkt_t *p1 = PacketAt(packets, 0);
	UsbPkt_t *p2 = PacketAt(packets, 1);
	p1->Hdr.Length = MPS;
	p2->Hdr.Length = 3U;
	memset(p1->Data, 0x5A, MPS);
	p2->Data[0] = 1U;
	p2->Data[1] = 2U;
	p2->Data[2] = 3U;

	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, packets, sizeof(packets)) ==
		  (int)sizeof(packets));
	CHECK(s_InBusy && s_InLen == MPS);
	CHECK(memcmp(s_InBuf, p1->Data, MPS) == 0);
	CHECK(CFifoUsed(s_Intrf.hTxFifo) == 1);
	CompleteIn(MPS);
	CHECK(s_InBusy && s_InLen == 3U);
	CHECK(memcmp(s_InBuf, p2->Data, 3U) == 0);
	CompleteIn(3U);
	CHECK(!s_InBusy);

	CHECK(CFifoUsed(s_Intrf.hTxFifo) == 0);
}

static void TestTxPacketZlp(void)
{
	CHECK(SetupPacketMode());

	alignas(4) uint8_t packets[PACKET_BLOCK_SIZE * 3U] = {};
	UsbPkt_t *p1 = PacketAt(packets, 0);
	UsbPkt_t *zlp = PacketAt(packets, 1);
	UsbPkt_t *p2 = PacketAt(packets, 2);
	p1->Hdr.Length = 2U;
	p1->Data[0] = 0x11U;
	p1->Data[1] = 0x12U;
	zlp->Hdr.Length = 0U;
	p2->Hdr.Length = 1U;
	p2->Data[0] = 0x21U;

	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, packets, sizeof(packets)) ==
		  (int)sizeof(packets));
	CHECK(s_InBusy && s_InLen == 2U);
	CHECK(memcmp(s_InBuf, p1->Data, 2U) == 0);
	CompleteIn(2U);
	CHECK(s_InBusy && s_InLen == 0U);
	CompleteIn(0U);
	CHECK(s_InBusy && s_InLen == 1U);
	CHECK(s_InBuf[0] == p2->Data[0]);
	CompleteIn(1U);
	CHECK(!s_InBusy);
}

static void TestTxPacketFull(void)
{
	CHECK(SetupPacketMode());

	alignas(4) uint8_t first[PACKET_BLOCK_SIZE] = {};
	UsbPkt_t *pFirst = PacketAt(first, 0);
	pFirst->Hdr.Length = 1U;
	pFirst->Data[0] = 0x10U;
	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, first, sizeof(first)) ==
		  (int)sizeof(first));
	CHECK(s_InBusy && s_InLen == 1U && s_InBuf[0] == 0x10U);

	alignas(4) uint8_t queued[PACKET_BLOCK_SIZE * PACKET_SLOTS] = {};
	for (unsigned i = 0; i < PACKET_SLOTS; i++)
	{
		UsbPkt_t *pkt = PacketAt(queued, i);
		pkt->Hdr.Length = 1U;
		pkt->Data[0] = (uint8_t)(0x20U + i);
	}
	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, queued, sizeof(queued)) ==
		  (int)sizeof(queued));
	CHECK(CFifoUsed(s_Intrf.hTxFifo) == (int)PACKET_SLOTS);
	CHECK(!UsbIntrfRequestToSend(&s_Intrf, PACKET_BLOCK_SIZE));
	CHECK(!UsbIntrfRequestToSend(&s_Intrf, PACKET_BLOCK_SIZE - 1U));
	CHECK(s_InSubmitCnt == 1);
	CHECK(s_InBuf[0] == 0x10U);
	CHECK(CFifoUsed(s_Intrf.hTxFifo) == (int)PACKET_SLOTS);

	alignas(4) uint8_t extra[PACKET_BLOCK_SIZE] = {};
	PacketAt(extra, 0)->Hdr.Length = 1U;
	CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, extra, sizeof(extra)) == 0);
	CHECK(CFifoUsed(s_Intrf.hTxFifo) == (int)PACKET_SLOTS);

	for (unsigned i = 0; i < PACKET_SLOTS; i++)
	{
		CompleteIn(1U);
		CHECK(s_InBusy && s_InLen == 1U);
		CHECK(s_InBuf[0] == (uint8_t)(0x20U + i));
	}
	CompleteIn(1U);
	CHECK(!s_InBusy);
}

struct Case { const char *Name; void (*Fn)(void); };

int main(void)
{
	static const Case cases[] = {
		{ "geometry", TestGeometry },
		{ "staging buffer", TestStagingBuffer },
		{ "whole packets", TestWholePackets },
		{ "zero length packet", TestZlp },
		{ "backpressure", TestBackpressure },
		{ "released storage rearms", TestReleasedStorageRearmsOut },
		{ "ring wrap", TestWrap },
		{ "failed and wrong ep", TestFailedAndWrongEndpoint },
		{ "rate", TestRate },
		{ "unconfigure", TestUnconfigure },
		{ "disable then enable", TestDisableEnable },
		{ "tx chaining", TestTxChaining },
		{ "tx accumulates", TestTxAccumulatesDuringTransfer },
		{ "tx packet boundaries", TestTxPacketMode },
		{ "tx packet ZLP", TestTxPacketZlp },
		{ "tx packet full", TestTxPacketFull },
	};

	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		const int before = s_Fail;
		cases[i].Fn();
		printf("%-24s %s\n", cases[i].Name,
			   s_Fail == before ? "pass" : "FAIL");
	}

	printf("%s\n", s_Fail == 0 ? "all pass" : "FAILURES");
	return s_Fail == 0 ? 0 : 1;
}
