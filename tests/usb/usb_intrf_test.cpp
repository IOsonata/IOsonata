/**-------------------------------------------------------------------------
@file	usb_intrf_test.cpp

@brief	Host regression tests for the UsbIntrf endpoint-pair data path.

The fake controller models a packet resident in endpoint hardware before
the controller starts OUT DMA. The OUT DMA destination is the buffer
submitted at DRDY through EpReceive. Byte and packet modes submit the
reserved RX FIFO block. IN transfers use the explicit DMA source supplied
to EpSend.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "usb/usb_intrf.h"

#define MPS                 64U
#define BUFFER_SIZE         128U
#define SLOTS               4U
#define EP_NO               1U
#define PACKET_SLOTS        3U
#define PACKET_BLOCK_SIZE   (sizeof(UsbPktHdr_t) + MPS)
#define MAX_PACKET_BLOCK_SIZE (sizeof(UsbPktHdr_t) + BUFFER_SIZE)

static uint8_t *s_OutRegBuf;
static uint8_t *s_InRegBuf;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBlocking;

static bool s_HwOutReady;
static uint16_t s_HwOutLen;
static uint8_t s_HwOut[BUFFER_SIZE];
static bool s_OutDma;
static uint16_t s_OutDmaLen;
static uint8_t *s_OutDmaBuf;

static bool s_InBusy;
static uint16_t s_InLen;
static int s_OutSubmitCnt;
static int s_InSubmitCnt;
static int s_TxTimeoutCnt;
static bool s_XferOk = true;
static bool s_HighSpeed;

static void ReceiveDma(void)
{
    if (s_HwOutReady && !s_OutDma && s_OutRegBuf != nullptr)
    {
        s_OutDma = true;
        s_OutDmaBuf = s_OutRegBuf;
        s_OutDmaLen = MPS;
        s_OutSubmitCnt++;
    }
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

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
void UsbCtrlrEpClose(int, uint8_t, bool) {}
void UsbCtrlrEpCloseAll(int) {}
void UsbCtrlrEpStall(int, uint8_t, bool) {}
void UsbCtrlrEpClearStall(int, uint8_t, bool) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

void UsbCtrlrEpBind(int, uint8_t, bool bIn, bool Blocking,
                        UsbCtrlrEpHandler_t Handler, void *pContext)
{
    if (bIn)
    {
        s_InRegBuf = nullptr;
        CHECK(s_InHandler == nullptr);
        s_InHandler = Handler;
        s_InContext = pContext;
    }
    else
    {
        s_OutRegBuf = nullptr;
        CHECK(s_OutHandler == nullptr);
        s_OutHandler = Handler;
        s_OutContext = pContext;
        s_OutBlocking = Blocking;
    }
}

bool UsbCtrlrEpReceive(int, uint8_t, uint8_t *pBuffer, uint16_t Capacity)
{
	if (!s_HwOutReady || s_OutDma || pBuffer == nullptr || Capacity == 0U) return false;
	s_OutRegBuf = pBuffer;
	ReceiveDma();
	return s_OutDma;
}

bool UsbCtrlrEpSend(int, uint8_t EpNum, uint8_t *pBuffer, uint16_t Len)
{
	if ((EpNum & 0x80U) != 0U) return false;
	if (!s_XferOk || s_InBusy) return false;
	s_InRegBuf = pBuffer;
	s_InBusy = true;
	s_InLen = Len;
	s_InSubmitCnt++;
	return true;
}

}

alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(SLOTS, BUFFER_SIZE)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_TxPacketMem[
    CFIFO_TOTAL_MEMSIZE(PACKET_SLOTS, PACKET_BLOCK_SIZE)];
alignas(4) static uint8_t s_TxMaxPacketMem[
    CFIFO_TOTAL_MEMSIZE(PACKET_SLOTS, MAX_PACKET_BLOCK_SIZE)];
alignas(4) static uint8_t s_TxTransfer[BUFFER_SIZE];
static UsbDevIntrf_t s_Intrf;

static void ResetFake(void)
{
    s_OutRegBuf = nullptr;
    s_InRegBuf = nullptr;
    s_OutHandler = nullptr;
    s_InHandler = nullptr;
    s_OutContext = nullptr;
    s_InContext = nullptr;
    s_OutBlocking = true;
    s_HwOutReady = false;
    s_HwOutLen = 0U;
    memset(s_HwOut, 0, sizeof(s_HwOut));
    s_OutDma = false;
    s_OutDmaLen = 0U;
    s_OutDmaBuf = nullptr;
    s_InBusy = false;
    s_InLen = 0U;
    s_OutSubmitCnt = 0;
    s_InSubmitCnt = 0;
    s_XferOk = true;
    s_HighSpeed = false;
}

static bool SetupWithTx(uint8_t *pTxMem, int TxMemSize, uint16_t TxBlkSize,
                        bool bBlocking = true)
{
    UsbIntrfCfg_t cfg = {};
    cfg.DevNo = 0;
    cfg.EpNo = EP_NO;
    cfg.bBlocking = bBlocking;
    cfg.pRxFifoMem = s_RxMem;
    cfg.RxFifoMemSize = (int)sizeof(s_RxMem);
    cfg.pTxFifoMem = pTxMem;
    cfg.TxFifoMemSize = TxMemSize;
    cfg.TxFifoBlkSize = TxBlkSize;
    cfg.BufferSize = BUFFER_SIZE;
    cfg.pTxBuffer = s_TxTransfer;
    memset(static_cast<void *>(&s_Intrf), 0, sizeof(s_Intrf));
    ResetFake();
    return UsbIntrfInit(&s_Intrf, &cfg) && UsbIntrfConfigure(&s_Intrf, MPS);
}

static bool Setup(void)
{
    return SetupWithTx(s_TxMem, (int)sizeof(s_TxMem), 1U);
}

static bool SetupPacketMode(void)
{
    return SetupWithTx(s_TxPacketMem, (int)sizeof(s_TxPacketMem),
                       PACKET_BLOCK_SIZE);
}

static bool SetupMaxPacketMode(void)
{
    return SetupWithTx(s_TxMaxPacketMem, (int)sizeof(s_TxMaxPacketMem),
                       MAX_PACKET_BLOCK_SIZE);
}

static bool Drdy(const uint8_t *pData, uint16_t Len)
{
    CHECK(!s_HwOutReady);
    CHECK(Len <= sizeof(s_HwOut));
    if (s_HwOutReady || Len > sizeof(s_HwOut))
        return false;

    if (Len > 0U && pData != nullptr)
        memcpy(s_HwOut, pData, Len);
    s_HwOutLen = Len;
    s_HwOutReady = true;

    if (!s_OutDma)
    {
        s_OutHandler(USB_CTRLR_EVT_DRDY,
                     Len, s_OutContext);
    }
    ReceiveDma();
    return s_OutDma;
}

// The OUT DMA destination is a block of the RX FIFO memory.
static bool InRxFifo(const uint8_t *p)
{
    return p != nullptr && p > s_RxMem && p < s_RxMem + sizeof(s_RxMem);
}

// Data of the block the next completion publishes.
static uint8_t *NextRxSlot(void)
{
    UsbPkt_t *pSlot = reinterpret_cast<UsbPkt_t *>(CFifoResv(s_Intrf.hRxFifo));
    return pSlot != nullptr ? pSlot->Data : nullptr;
}

static void CompleteOut(UsbCtrlrEvtType_t Event = USB_CTRLR_EVT_XFER_CMPL)
{
    CHECK(s_HwOutReady);
    CHECK(s_OutDma);
    CHECK(InRxFifo(s_OutDmaBuf));
    if (!s_HwOutReady || !s_OutDma)
        return;

    const uint16_t len = s_HwOutLen < s_OutDmaLen ? s_HwOutLen : s_OutDmaLen;
    if (Event == USB_CTRLR_EVT_XFER_CMPL && len > 0U)
        memcpy(s_OutDmaBuf, s_HwOut, len);

    s_HwOutReady = false;
    s_OutDma = false;
    s_OutRegBuf = nullptr;
    s_OutHandler(Event,
                 len, s_OutContext);
}

static void Deliver(const uint8_t *pData, uint16_t Len)
{
    CHECK(Drdy(pData, Len));
    if (s_OutDma)
        CompleteOut();
}

static void CompleteIn(uint16_t Len,
                       UsbCtrlrEvtType_t Event = USB_CTRLR_EVT_XFER_CMPL)
{
    CHECK(s_InBusy);
    if (!s_InBusy)
        return;
    s_InBusy = false;
    s_InHandler(Event,
                Len, s_InContext);
}

static UsbPkt_t *PacketAt(uint8_t *pBlocks, unsigned Index)
{
    return reinterpret_cast<UsbPkt_t *>(pBlocks + Index * PACKET_BLOCK_SIZE);
}

static void TestGeometry(void)
{
    CHECK(Setup());
    CHECK(s_Intrf.EpNo == EP_NO);
    CHECK(CFifoAvail(s_Intrf.hRxFifo) == (int)SLOTS);
    CHECK(CFifoBlockSize(s_Intrf.hRxFifo) == USB_INTRF_PKT_BLKSIZE(BUFFER_SIZE));
    // Init binds callbacks; the first DRDY will submit the first RX block.
    CHECK(s_OutRegBuf == nullptr);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
    CHECK(s_InRegBuf == nullptr);
    CHECK(s_OutSubmitCnt == 0);
    CHECK(!s_OutDma);
}

static void TestInPlaceReceive(void)
{
    CHECK(Setup());
    uint8_t *pArmed = NextRxSlot();
    const uint8_t in[3] = { 1, 2, 3 };
    Deliver(in, sizeof(in));
    CHECK(s_OutSubmitCnt == 1);
    CHECK(!s_OutDma);
    // The published packet is the block the DMA wrote. Another DRDY is
    // required before the next block is submitted.
    UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(CFifoPeek(s_Intrf.hRxFifo));
    CHECK(pPacket != nullptr && pPacket->Data == pArmed);
    CHECK(s_OutRegBuf == nullptr && NextRxSlot() != pArmed);
    CHECK(pPacket != nullptr && pPacket->Hdr.Length == sizeof(in));
    CHECK(pPacket != nullptr && memcmp(pPacket->Data, in, sizeof(in)) == 0);
    uint8_t out[sizeof(in)] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == (int)sizeof(out));
    CHECK(s_OutSubmitCnt == 1);
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
}

static void TestZlp(void)
{
    CHECK(Setup());
    uint8_t full[MPS];
    for (uint32_t i = 0; i < sizeof(full); i++) full[i] = (uint8_t)i;
    Deliver(full, sizeof(full));
    Deliver(nullptr, 0U);
    const uint8_t in[2] = { 'x', 'y' };
    Deliver(in, sizeof(in));
    uint8_t out[MPS + sizeof(in)] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == (int)sizeof(out));
    CHECK(memcmp(out, full, sizeof(full)) == 0);
    CHECK(memcmp(out + sizeof(full), in, sizeof(in)) == 0);
}

static void TestZlpReleasesPendingReceive(void)
{
	CHECK(Setup());
	for (unsigned i = 0; i < SLOTS; i++) Deliver(nullptr, 0U);
	const uint8_t pending = 0xA7;
	CHECK(!Drdy(&pending, 1U) && s_Intrf.RxPending);
	uint8_t out = 0;
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, &out, 1) == 0);
	CHECK(s_OutDma && !s_Intrf.RxPending);
	CHECK(s_OutSubmitCnt == (int)SLOTS + 1);
	CompleteOut();
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, &out, 1) == 1);
	CHECK(out == pending);
}

static void TestZeroCapacityZlp(void)
{
	CHECK(Setup());
	Deliver(nullptr, 0U);
	const uint8_t data = 0xA7U;
	Deliver(&data, 1U);
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, nullptr, 0) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 1);
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, nullptr, 0) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 1);
	uint8_t out = 0U;
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, &out, 1) == 1);
	CHECK(out == data);

	CHECK(Setup());
	for (unsigned i = 0; i < SLOTS; ++i) Deliver(nullptr, 0U);
	CHECK(!Drdy(&data, 1U) && s_Intrf.RxPending);
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, nullptr, 0) == 0);
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)SLOTS - 1);
	CHECK(s_OutDma && !s_Intrf.RxPending);
	CompleteOut();
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, &out, 1) == 1);
	CHECK(out == data);
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
    // The FIFO is full, but no further packet has requested a destination.
    CHECK(CFifoAvail(s_Intrf.hRxFifo) == 0);
    CHECK(!s_Intrf.RxPending && s_OutRegBuf == nullptr);

    uint8_t pending[8] = { 0xA5 };
    CHECK(!Drdy(pending, sizeof(pending)));
    CHECK(s_HwOutReady);
    CHECK(!s_OutDma);
    CHECK(s_Intrf.RxPending);

    uint8_t out[8] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
    CHECK(out[0] == 0U);
    // Freeing space submits the held packet to the controller immediately.
    CHECK(s_OutDma);
    CHECK(!s_Intrf.RxPending && InRxFifo(s_OutRegBuf));
    CompleteOut();
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)SLOTS);
    for (uint32_t i = 1; i < SLOTS; i++)
    {
        CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
        CHECK(out[0] == (uint8_t)i);
    }
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
    CHECK(out[0] == 0xA5U);
    CHECK(s_Intrf.hRxFifo->DropCnt == 0U);
}

// Non-blocking: the next packet arriving at a full FIFO takes the
// oldest one's block when it arrives, the same packet a non-blocking put
// would have replaced. Every block holds a packet until then.
static void TestNonBlockingDropOldest(void)
{
    CHECK(SetupWithTx(s_TxMem, (int)sizeof(s_TxMem), 1U, false));
    uint8_t in[8] = {};
    for (uint32_t i = 0; i < SLOTS; i++)
    {
        in[0] = (uint8_t)i;
        Deliver(in, sizeof(in));
    }
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)SLOTS);
    CHECK(!s_Intrf.RxPending && s_OutRegBuf == nullptr);
    CHECK(s_Intrf.hRxFifo->DropCnt == 0U);

    in[0] = 0x5AU;
    Deliver(in, sizeof(in));
    CHECK(s_Intrf.hRxFifo->DropCnt == 1U);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)SLOTS);

    uint8_t out[8] = {};
    for (uint32_t i = 1; i < SLOTS; i++)
    {
        CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
        CHECK(out[0] == (uint8_t)i);
    }
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
    CHECK(out[0] == 0x5AU);
}

// Configure drains the RX FIFO from the read side only, so an OUT DMA
// already running into the reserved block completes into the block that is
// published.
static void TestReconfigureDuringDma(void)
{
    CHECK(Setup());
    uint8_t in[8] = { 1 };
    Deliver(in, sizeof(in));
    in[0] = 2;
    CHECK(Drdy(in, sizeof(in)));
    uint8_t *pArmed = s_OutDmaBuf;
    CHECK(UsbIntrfConfigure(&s_Intrf, MPS));
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
    CHECK(s_OutRegBuf == pArmed);
    CompleteOut();
    UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(CFifoPeek(s_Intrf.hRxFifo));
    CHECK(pPacket != nullptr && pPacket->Data == pArmed);
    uint8_t out[8] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
    CHECK(out[0] == 2U);
}

// Reset shares cleanup with configuration but preserves the controller binding
// and the reserved DMA block, including when called inside a serialized RX.
static void TestResetDuringDma(void)
{
	CHECK(Setup());
	const uint8_t in[] = { 7, 8, 9 };
	Deliver(in, sizeof(in));
	CHECK(Drdy(in, sizeof(in)));
	uint8_t *pArmed = s_OutDmaBuf;
	const auto enabled = atomic_load(&s_Intrf.DevIntrf.EnCnt);
	CHECK(DeviceIntrfStartRx(&s_Intrf.DevIntrf, 0));
	DeviceIntrfReset(&s_Intrf.DevIntrf);
	CHECK(s_Intrf.Mps == 0U);
	CHECK(atomic_load(&s_Intrf.DevIntrf.EnCnt) == enabled);
	CHECK(atomic_flag_test_and_set(&s_Intrf.DevIntrf.bBusy));
	CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
	CHECK(s_OutRegBuf == pArmed);
	DeviceIntrfStopRx(&s_Intrf.DevIntrf);
	CHECK(UsbIntrfConfigure(&s_Intrf, MPS));
	CompleteOut();
	uint8_t out[sizeof(in)] = {};
	CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == (int)sizeof(out));
	CHECK(memcmp(out, in, sizeof(in)) == 0);
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
    // Endpoint routing is the controller's job now: completions arrive only
    // through the registered handler, so only the failure event remains
    // observable at this layer.
    const int used = CFifoUsed(s_Intrf.hRxFifo);
    s_OutHandler(USB_CTRLR_EVT_XFER_FAILED, 0, s_OutContext);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == used);
    CHECK(s_Intrf.RxDropCnt == 1U);
    CHECK(s_OutSubmitCnt == 0);
}

static void TestRate(void)
{
    CHECK(Setup());
    CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == USB_LINK_RATE_FULL);
    CHECK(DeviceIntrfSetRate(&s_Intrf.DevIntrf, 1U) == USB_LINK_RATE_FULL);
    s_HighSpeed = true;
    CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == USB_LINK_RATE_HIGH);
    UsbIntrfUnconfigure(&s_Intrf);
    CHECK(DeviceIntrfGetRate(&s_Intrf.DevIntrf) == 0U);
}

static void TestUnconfigure(void)
{
    CHECK(Setup());
    UsbIntrfUnconfigure(&s_Intrf);
    CHECK(s_Intrf.Mps == 0U);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == 0);
    CHECK(UsbIntrfConfigure(&s_Intrf, MPS));
    CHECK(s_OutSubmitCnt == 0);
}

static void TestDisableDoesNotGate(void)
{
    CHECK(Setup());
    DeviceIntrfDisable(&s_Intrf.DevIntrf);
    const uint8_t in[4] = { 1, 2, 3, 4 };
    Deliver(in, sizeof(in));
    DeviceIntrfEnable(&s_Intrf.DevIntrf);
    uint8_t out[4] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 4);
    CHECK(memcmp(out, in, sizeof(in)) == 0);
}

static void TestTxChaining(void)
{
    CHECK(Setup());
    uint8_t data[MPS * 2U + 3U];
    for (uint32_t i = 0; i < sizeof(data); i++) data[i] = (uint8_t)i;

    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, data, sizeof(data)) == (int)sizeof(data));
    CHECK(s_InBusy && s_InLen == MPS);
    CHECK(memcmp(s_InRegBuf, data, MPS) == 0);
    CompleteIn(MPS);
    CHECK(s_InBusy && s_InLen == MPS);
    CHECK(memcmp(s_InRegBuf, data + MPS, MPS) == 0);
    CompleteIn(MPS);
    CHECK(s_InBusy && s_InLen == 3U);
    CHECK(memcmp(s_InRegBuf, data + MPS * 2U, 3U) == 0);
    CompleteIn(3U);
    CHECK(!s_InBusy);
    CHECK(s_InSubmitCnt == 3);
}

static void TestTxRejectRetriesHead(void)
{
    CHECK(Setup());
    uint8_t first[MPS];
    for (unsigned i = 0; i < sizeof(first); ++i)
        first[i] = (uint8_t)(0x40U + i);
    const uint8_t next = 0xE7U;

    // The first controller submission is rejected before it owns the source.
    // The FIFO head must stay queued and the Tx ownership token must return.
    s_XferOk = false;
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, first, sizeof(first)) ==
          (int)sizeof(first));
    CHECK(!s_InBusy && s_InSubmitCnt == 0);
    CHECK(CFifoUsed(s_Intrf.hTxFifo) == (int)sizeof(first));
    CHECK(atomic_load_explicit(&s_Intrf.DevIntrf.bTxReady,
          memory_order_acquire));

    // A later Tx call adds newer data but must retry the old head first.
    s_XferOk = true;
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, &next, 1) == 1);
    CHECK(s_InBusy && s_InSubmitCnt == 1 && s_InLen == MPS);
    CHECK(memcmp(s_InRegBuf, first, MPS) == 0);

    CompleteIn(MPS);
    CHECK(s_InBusy && s_InSubmitCnt == 2 && s_InLen == 1U);
    CHECK(s_InRegBuf[0] == next);
    CompleteIn(1U);
    CHECK(!s_InBusy);
}

static void TestTxAccumulatesDuringTransfer(void)
{
    CHECK(Setup());
    uint8_t byte = 0xA5;
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, &byte, 1) == 1);
    CHECK(s_InSubmitCnt == 1);
    for (int i = 0; i < 40; i++)
        CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, &byte, 1) == 1);
    CHECK(s_InSubmitCnt == 1);
    CHECK(CFifoUsed(s_Intrf.hTxFifo) == 41);
    CompleteIn(1U);
    CHECK(s_InBusy && s_InLen == 40U && s_InSubmitCnt == 2);
    CompleteIn(40U);
    CHECK(!s_InBusy);
}

static void TestTxPacketMode(void)
{
    CHECK(SetupPacketMode());
    CHECK(CFifoAvail(s_Intrf.hTxFifo) > 0);
    alignas(4) uint8_t packets[PACKET_BLOCK_SIZE * 2U] = {};
    UsbPkt_t *p1 = PacketAt(packets, 0);
    UsbPkt_t *p2 = PacketAt(packets, 1);
    p1->Hdr.Length = MPS;
    p2->Hdr.Length = 3U;
    memset(p1->Data, 0x5A, MPS);
    p2->Data[0] = 1U; p2->Data[1] = 2U; p2->Data[2] = 3U;

    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, packets, sizeof(packets)) ==
          (int)sizeof(packets));
    CHECK(s_InBusy && s_InLen == MPS);
    CHECK(memcmp(s_InRegBuf, p1->Data, MPS) == 0);
    CompleteIn(MPS);
    CHECK(s_InBusy && s_InLen == 3U);
    CHECK(memcmp(s_InRegBuf, p2->Data, 3U) == 0);
    CompleteIn(3U);
    CHECK(!s_InBusy);
}

static void TestTxPacketZlp(void)
{
    CHECK(SetupPacketMode());
    alignas(4) uint8_t packets[PACKET_BLOCK_SIZE * 3U] = {};
    UsbPkt_t *p1 = PacketAt(packets, 0);
    UsbPkt_t *zlp = PacketAt(packets, 1);
    UsbPkt_t *p2 = PacketAt(packets, 2);
    p1->Hdr.Length = MPS;
    memset(p1->Data, 0x11, MPS);
    zlp->Hdr.Length = 0U;
    p2->Hdr.Length = 1U;
    p2->Data[0] = 0x21U;

    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, packets, sizeof(packets)) ==
          (int)sizeof(packets));
    CompleteIn(MPS);
    CHECK(s_InBusy && s_InLen == 0U);
    CompleteIn(0U);
    CHECK(s_InBusy && s_InLen == 1U && s_InRegBuf[0] == 0x21U);
    CompleteIn(1U);
    CHECK(!s_InBusy);
}

static void TestTxPacketFull(void)
{
    CHECK(SetupPacketMode());
    alignas(4) uint8_t first[PACKET_BLOCK_SIZE] = {};
    PacketAt(first, 0)->Hdr.Length = 1U;
    PacketAt(first, 0)->Data[0] = 0x10U;
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, first, sizeof(first)) == (int)sizeof(first));

    alignas(4) uint8_t queued[PACKET_BLOCK_SIZE * PACKET_SLOTS] = {};
    for (unsigned i = 0; i < PACKET_SLOTS; i++)
    {
        UsbPkt_t *pkt = PacketAt(queued, i);
        pkt->Hdr.Length = 1U;
        pkt->Data[0] = (uint8_t)(0x20U + i);
    }
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, queued, sizeof(queued)) ==
          (int)(sizeof(queued) - PACKET_BLOCK_SIZE));
    CHECK(CFifoUsed(s_Intrf.hTxFifo) == (int)PACKET_SLOTS);
    CHECK(CFifoAvail(s_Intrf.hTxFifo) == 0);

    for (unsigned i = 0; i < PACKET_SLOTS - 1U; i++)
    {
        CompleteIn(1U);
        CHECK(s_InBusy && s_InLen == 1U);
        CHECK(s_InRegBuf[0] == (uint8_t)(0x20U + i));
    }
    CompleteIn(1U);
    CHECK(!s_InBusy);
}

static void TestTxPacketMaximumSlot(void)
{
    CHECK(SetupMaxPacketMode());
    alignas(4) uint8_t block[MAX_PACKET_BLOCK_SIZE] = {};
    UsbPkt_t *packet = reinterpret_cast<UsbPkt_t *>(block);
    packet->Hdr.Length = MPS;
    memset(packet->Data, 0xA6, MPS);
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, block, sizeof(block)) == (int)sizeof(block));
    CHECK(s_InBusy && s_InLen == MPS);
    CompleteIn(MPS);
    packet->Hdr.Length = MPS + 1U;
    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, block, sizeof(block)) == 0);
}

static void TestTxFailureEvent(void)
{
    // A failed transfer releases its FIFO data, reports a timeout, and does
    // not chain the next packet or report a successful completion.
    for (unsigned packetMode = 0; packetMode < 2; packetMode++)
    {
        CHECK(packetMode ? SetupPacketMode() : Setup());
        s_TxTimeoutCnt = 0;
        s_Intrf.DevIntrf.EvtCB = [](DevIntrf_t *, DEVINTRF_EVT event,
                                  uint8_t *, int length) -> int {
            CHECK(event == DEVINTRF_EVT_TX_TIMEOUT);
            CHECK(length == (int)MPS);
            s_TxTimeoutCnt++;
            return length;
        };
        alignas(4) uint8_t data[PACKET_BLOCK_SIZE * 2U] = {};
        int length = MPS * 2U;
        if (packetMode)
        {
            PacketAt(data, 0)->Hdr.Length = MPS;
            PacketAt(data, 1)->Hdr.Length = MPS;
            length = sizeof(data);
        }
        CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, data, length) == length);
        CompleteIn(MPS, USB_CTRLR_EVT_XFER_FAILED);
        CHECK(s_TxTimeoutCnt == 1);
        CHECK(CFifoUsed(s_Intrf.hTxFifo) == (packetMode ? 1 : (int)MPS));
        CHECK(!s_InBusy && s_InSubmitCnt == 1);
    }
}

struct Case { const char *Name; void (*Fn)(void); };

int main(void)
{
    static const Case cases[] = {
        { "geometry", TestGeometry },
        { "in-place receive", TestInPlaceReceive },
        { "whole packets", TestWholePackets },
        { "zero length packet", TestZlp },
        { "ZLP pending receive", TestZlpReleasesPendingReceive },
		{ "zero-capacity ZLP", TestZeroCapacityZlp },
        { "backpressure", TestBackpressure },
        { "non-blocking drop", TestNonBlockingDropOldest },
        { "reconfigure during DMA", TestReconfigureDuringDma },
        { "reset during DMA", TestResetDuringDma },
        { "ring wrap", TestWrap },
        { "failed and wrong ep", TestFailedAndWrongEndpoint },
        { "rate", TestRate },
        { "unconfigure", TestUnconfigure },
        { "disable does not gate", TestDisableDoesNotGate },
        { "tx chaining", TestTxChaining },
        { "tx rejected retry", TestTxRejectRetriesHead },
        { "tx accumulates", TestTxAccumulatesDuringTransfer },
        { "tx packet boundaries", TestTxPacketMode },
        { "tx packet ZLP", TestTxPacketZlp },
        { "tx packet full", TestTxPacketFull },
        { "tx packet maximum slot", TestTxPacketMaximumSlot },
        { "tx failure event", TestTxFailureEvent },
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

// The process event of the USB core is not part of this test.
void UsbProcessQue(int) {}

