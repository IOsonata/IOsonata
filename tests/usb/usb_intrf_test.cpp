/**-------------------------------------------------------------------------
@file	usb_intrf_test.cpp

@brief	Host regression tests for the UsbIntrf receive packet ring.

Links the production usb_intrf.cpp against a fake controller. The fake records
the buffer it was armed with, so the tests can check that the controller is
given the ring slot itself and that a slot is published only once its transfer
has completed.

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
#include <vector>

#include "usb/usb_intrf.h"

#define MPS			64U
#define SLOTS		4U

static uint8_t *s_XferBuf;
static uint16_t s_XferLen;
static int s_XferCnt;
static bool s_XferOk = true;

extern "C" {
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return false; }
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
bool UsbCtrlrEpBusy(int, uint8_t) { return false; }
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

bool UsbCtrlrEpXfer(int, uint8_t, uint8_t *pBuf, uint16_t Len)
{
	if (!s_XferOk) { return false; }
	s_XferBuf = pBuf;
	s_XferLen = Len;
	s_XferCnt++;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(SLOTS, MPS)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(256)];
static UsbDevIntrf_t s_Intrf;

// Simulate the controller writing one OUT packet and completing it.
static void Deliver(const uint8_t *pData, uint16_t Len)
{
	CHECK(s_XferBuf != nullptr);
	if (s_XferBuf == nullptr) { return; }
	if (Len > 0U) { memcpy(s_XferBuf, pData, Len); }
	s_XferBuf = nullptr;
	UsbIntrfRxXferComplete(&s_Intrf, Len, USB_CTRLR_XFER_SUCCESS);
}

static bool Setup(void)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = (int)sizeof(s_RxMem);
	cfg.pTxFifoMem = s_TxMem;
	cfg.TxFifoMemSize = (int)sizeof(s_TxMem);
	cfg.TxFifoBlkSize = 1U;
	memset(&s_Intrf, 0, sizeof(s_Intrf));
	s_XferBuf = nullptr;
	s_XferCnt = 0;
	s_XferOk = true;
	if (!UsbIntrfInit(&s_Intrf, &cfg)) { return false; }
	if (!UsbIntrfConfigRx(&s_Intrf, 0x01U, MPS)) { return false; }
	UsbIntrfRxEnable(&s_Intrf, true);
	return true;
}

// The ring must carve exactly SLOTS slots out of the supplied memory.
static void TestGeometry(void)
{
	CHECK(Setup());
	CHECK(CFifoAvail(s_Intrf.hRxFifo) == (int)SLOTS);
	CHECK(CFifoBlockSize(s_Intrf.hRxFifo) == sizeof(UsbPktHdr_t) + MPS);
	CHECK(s_XferCnt == 1);			// armed on enable
	CHECK(s_XferLen == MPS);
}

// The controller must be handed the slot itself, never a staging buffer.
static void TestZeroCopyTarget(void)
{
	CHECK(Setup());
	uint8_t *pSlot0 = s_RxMem + sizeof(CFifo_t) + sizeof(UsbPktHdr_t);
	CHECK(s_XferBuf == pSlot0);

	const uint8_t a[3] = { 1, 2, 3 };
	Deliver(a, 3);
	// Next arm points at the second slot, the ring advanced.
	CHECK(s_XferBuf == s_RxMem + sizeof(CFifo_t) + CFifoBlockSize(s_Intrf.hRxFifo) + sizeof(UsbPktHdr_t));
	// The delivered bytes are still where the controller put them.
	CHECK(memcmp(pSlot0, a, 3) == 0);
}

// A byte-stream read spanning several packets, plus a partial read.
static void TestStreamAcrossPackets(void)
{
	CHECK(Setup());
	uint8_t p1[4] = { 'a', 'b', 'c', 'd' };
	uint8_t p2[3] = { 'e', 'f', 'g' };
	Deliver(p1, 4);
	Deliver(p2, 3);
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 7);

	uint8_t out[8] = {};
	int n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 2);
	CHECK(n == 2);
	CHECK(out[0] == 'a' && out[1] == 'b');
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 5);

	memset(out, 0, sizeof(out));
	n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 8);
	CHECK(n == 5);
	CHECK(memcmp(out, "cdefg", 5) == 0);
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 0);

	n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 8);
	CHECK(n == 0);
}

// A zero length packet is a packet. It occupies a slot and is released
// without consuming any of the reader's buffer.
static void TestZlp(void)
{
	CHECK(Setup());
	const int used0 = CFifoUsed(s_Intrf.hRxFifo);
	Deliver(nullptr, 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == used0 + 1);
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 0);

	uint8_t p[2] = { 'x', 'y' };
	Deliver(p, 2);

	uint8_t out[4] = {};
	int n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 4);
	CHECK(n == 2);
	CHECK(out[0] == 'x' && out[1] == 'y');
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == used0);		// both slots released
}

// A full ring must leave the endpoint unarmed, not drop a packet. Reading
// one packet must rearm it.
static void TestBackpressure(void)
{
	CHECK(Setup());
	uint8_t p[8] = { 0 };
	for (uint32_t i = 0; i < SLOTS; i++)
	{
		p[0] = (uint8_t)i;
		Deliver(p, 8);
	}
	CHECK(CFifoAvail(s_Intrf.hRxFifo) == 0);
	CHECK(s_XferBuf == nullptr);				// unarmed, host backpressured
	CHECK(s_Intrf.RxActive == false);
	CHECK(s_Intrf.RxDropCnt == 0U);

	uint8_t out[8] = {};
	int n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 8);
	CHECK(n == 8);
	CHECK(out[0] == 0);
	CHECK(s_XferBuf != nullptr);				// rearmed after release
	CHECK(s_XferLen == MPS);

	// Remaining packets keep their order.
	for (uint32_t i = 1; i < SLOTS; i++)
	{
		memset(out, 0xFF, sizeof(out));
		n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 8);
		CHECK(n == 8);
		CHECK(out[0] == (uint8_t)i);
	}
}

// The ring index is monotonic, so it must survive wrapping many times.
static void TestWrap(void)
{
	CHECK(Setup());
	uint8_t in[16];
	uint8_t out[16];
	for (int i = 0; i < 1000; i++)
	{
		memset(in, (uint8_t)i, sizeof(in));
		Deliver(in, 16);
		memset(out, 0, sizeof(out));
		int n = DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 16);
		CHECK(n == 16);
		if (memcmp(out, in, 16) != 0) { CHECK(false); break; }
	}
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 0);
}

// A failed transfer counts a drop and does not publish a slot.
static void TestXferFailed(void)
{
	CHECK(Setup());
	const int used0 = CFifoUsed(s_Intrf.hRxFifo);
	s_XferBuf = nullptr;
	UsbIntrfRxXferComplete(&s_Intrf, 0, USB_CTRLR_XFER_FAILED);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == used0);
	CHECK(s_Intrf.RxDropCnt == 1U);
	CHECK(s_XferBuf != nullptr);				// rearmed
}

// Reset must drop endpoint and ring state.
static void TestReset(void)
{
	CHECK(Setup());
	uint8_t p[4] = { 1, 2, 3, 4 };
	Deliver(p, 4);
	UsbIntrfResetRx(&s_Intrf);
	CHECK(s_Intrf.hRxFifo == nullptr);
	CHECK(s_Intrf.RxEpAddr == 0U);
	CHECK(UsbIntrfRxUsed(&s_Intrf) == 0);

	uint8_t out[4];
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, 4) == 0);
}

// RX memory too small for one slot must be refused.
static void TestTooSmall(void)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = 8;
	cfg.pTxFifoMem = s_TxMem;
	cfg.TxFifoMemSize = (int)sizeof(s_TxMem);
	cfg.TxFifoBlkSize = 1U;
	memset(&s_Intrf, 0, sizeof(s_Intrf));
	CHECK(UsbIntrfInit(&s_Intrf, &cfg));
	CHECK(UsbIntrfConfigRx(&s_Intrf, 0x01U, MPS) == false);
}

struct Case { const char *Name; void (*Fn)(void); };

int main(void)
{
	static const Case cases[] = {
		{ "geometry", TestGeometry },
		{ "zero copy target", TestZeroCopyTarget },
		{ "stream across packets", TestStreamAcrossPackets },
		{ "zero length packet", TestZlp },
		{ "backpressure", TestBackpressure },
		{ "ring wrap", TestWrap },
		{ "transfer failed", TestXferFailed },
		{ "reset", TestReset },
		{ "memory too small", TestTooSmall },
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
