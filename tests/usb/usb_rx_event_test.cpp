#include <stdio.h>
#include <string.h>

#include "usb/usb_intrf.h"

#define MPS 64U
#define BUFFER_SIZE 128U
#define RX_SLOTS 4U
#define EP_NO 1U

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_HwOutReady;
static bool s_OutBlocking;
static bool s_OutDma;
static bool s_InDma;
static uint16_t s_HwOutLength;
static uint16_t s_InLength;
static uint8_t s_HwOut[BUFFER_SIZE];
static int s_OutSubmit;
static int s_InSubmit;
static int s_Fail;

#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; \
} } while (0)

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
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer, bool Blocking,
                        UsbCtrlrEpHandler_t Handler, void *pContext)
{
    if (USB_ENDPADDR_IS_IN(EpAddr))
    {
        s_InBuffer = pBuffer;
        s_InHandler = Handler;
        s_InContext = pContext;
    }
    else
    {
        s_OutBuffer = pBuffer;
        s_OutHandler = Handler;
        s_OutContext = pContext;
        s_OutBlocking = Blocking;
    }
    return pBuffer != nullptr && Handler != nullptr;
}

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
    if (USB_ENDPADDR_IS_IN(EpAddr))
    {
        if (s_InDma) return false;
        s_InDma = true;
        s_InLength = Length;
        s_InSubmit++;
        return true;
    }

    // OUT DMA must only be submitted after DRDY.
    if (!s_HwOutReady || s_OutDma) return false;
    s_OutDma = true;
    s_OutSubmit++;
    return true;
}

bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
}

alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(RX_SLOTS, BUFFER_SIZE)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_RxDma[BUFFER_SIZE];
alignas(4) static uint8_t s_TxDma[BUFFER_SIZE];
static UsbDevIntrf_t s_Intrf;

static bool Setup(bool Blocking)
{
    memset(&s_Intrf, 0, sizeof(s_Intrf));
    memset(s_HwOut, 0, sizeof(s_HwOut));
    s_OutBuffer = nullptr;
    s_InBuffer = nullptr;
    s_OutHandler = nullptr;
    s_InHandler = nullptr;
    s_OutContext = nullptr;
    s_InContext = nullptr;
    s_HwOutReady = false;
    s_OutBlocking = true;
    s_OutDma = false;
    s_InDma = false;
    s_HwOutLength = 0;
    s_InLength = 0;
    s_OutSubmit = 0;
    s_InSubmit = 0;

    UsbIntrfCfg_t cfg = {};
    cfg.DevNo = 0;
    cfg.EpNo = EP_NO;
    cfg.bBlocking = Blocking;
    cfg.RxFifoMemSize = sizeof(s_RxMem);
    cfg.pRxFifoMem = s_RxMem;
    cfg.TxFifoMemSize = sizeof(s_TxMem);
    cfg.pTxFifoMem = s_TxMem;
    cfg.TxFifoBlkSize = 1U;
    cfg.BufferSize = BUFFER_SIZE;
    cfg.pRxBuffer = s_RxDma;
    cfg.pTxBuffer = s_TxDma;

    return UsbIntrfInit(&s_Intrf, &cfg) &&
           UsbIntrfConfigure(&s_Intrf, MPS);
}

static bool Drdy(const uint8_t *pData, uint16_t Length)
{
    CHECK(!s_HwOutReady);
    if (s_HwOutReady || Length > sizeof(s_HwOut)) return false;
    if (Length > 0) memcpy(s_HwOut, pData, Length);
    s_HwOutLength = Length;
    s_HwOutReady = true;
    if (s_OutBlocking)
    {
        s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_DRDY,
                     Length, USB_CTRLR_XFER_SUCCESS, s_OutContext);
    }
    else
    {
        s_OutDma = true;
        s_OutSubmit++;
    }
    return s_OutDma;
}

static void CompleteOut(void)
{
    CHECK(s_HwOutReady && s_OutDma);
    CHECK(s_OutBuffer == s_RxDma);
    if (!s_HwOutReady || !s_OutDma) return;
    if (s_HwOutLength > 0)
        memcpy(s_OutBuffer, s_HwOut, s_HwOutLength);
    uint16_t len = s_HwOutLength;
    s_HwOutReady = false;
    s_OutDma = false;
    s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
                 len, USB_CTRLR_XFER_SUCCESS, s_OutContext);
}

static void Deliver(const uint8_t *pData, uint16_t Length)
{
    CHECK(Drdy(pData, Length));
    if (s_OutDma) CompleteOut();
}

static void TestNoPreArm(void)
{
    CHECK(Setup(true));
    CHECK(s_OutBuffer == s_RxDma);
    CHECK(s_InBuffer == s_TxDma);
    CHECK(s_OutSubmit == 0);
    CHECK(!s_OutDma);

    const uint8_t p[] = {1,2,3};
    Deliver(p, sizeof(p));
    CHECK(s_OutSubmit == 1);
    CHECK(!s_OutDma);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == 1);

    uint8_t out[3] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 3);
    CHECK(memcmp(out, p, sizeof(p)) == 0);
    // Draining data without a pending DRDY never submits another RX DMA.
    CHECK(s_OutSubmit == 1);
}

static void TestBlocking(void)
{
    CHECK(Setup(true));
    uint8_t p[8] = {};
    for (unsigned i = 0; i < RX_SLOTS; i++)
    {
        p[0] = (uint8_t)i;
        Deliver(p, sizeof(p));
    }
    CHECK(CFifoAvail(s_Intrf.hRxFifo) == 0);
    CHECK(s_OutSubmit == (int)RX_SLOTS);

    uint8_t pending[8] = {0xA5};
    CHECK(!Drdy(pending, sizeof(pending)));
    CHECK(s_HwOutReady);
    CHECK(!s_OutDma);
    CHECK(s_Intrf.RxPending);
    CHECK(s_OutSubmit == (int)RX_SLOTS);

    uint8_t out[8] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 8);
    CHECK(out[0] == 0);
    CHECK(s_OutDma);
    CHECK(!s_Intrf.RxPending);
    CHECK(s_OutSubmit == (int)RX_SLOTS + 1);

    CompleteOut();
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)RX_SLOTS);
}

static void TestNonBlocking(void)
{
    CHECK(Setup(false));
    CHECK(!s_Intrf.hRxFifo->bBlocking);
    uint8_t p[8] = {};
    for (unsigned i = 0; i < RX_SLOTS; i++)
    {
        p[0] = (uint8_t)i;
        Deliver(p, sizeof(p));
    }
    CHECK(CFifoAvail(s_Intrf.hRxFifo) == 0);
    CHECK(s_Intrf.hRxFifo->DropCnt == 0U);

    uint8_t newest[8] = {0xA6};
    CHECK(Drdy(newest, sizeof(newest)));
    CHECK(s_OutDma);
    CHECK(s_OutSubmit == (int)RX_SLOTS + 1);
    CompleteOut();
    CHECK(s_Intrf.hRxFifo->DropCnt == 1U);
    CHECK(CFifoUsed(s_Intrf.hRxFifo) == (int)RX_SLOTS);

    uint8_t out[RX_SLOTS * 8] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) ==
          (int)sizeof(out));
    CHECK(out[0] == 1U);
    CHECK(out[8] == 2U);
    CHECK(out[16] == 3U);
    CHECK(out[24] == 0xA6U);
}

static void TestDisableDoesNotGateController(void)
{
    CHECK(Setup(true));
    DeviceIntrfDisable(&s_Intrf.DevIntrf);

    const uint8_t p[] = {9,8,7};
    CHECK(Drdy(p, sizeof(p)));
    CHECK(!s_Intrf.RxPending);
    CHECK(s_OutDma);
    CompleteOut();

    DeviceIntrfEnable(&s_Intrf.DevIntrf);
    uint8_t out[3] = {};
    CHECK(DeviceIntrfRxData(&s_Intrf.DevIntrf, out, sizeof(out)) == 3);
    CHECK(memcmp(out, p, sizeof(p)) == 0);
}

static void TestTxStillChains(void)
{
    CHECK(Setup(true));
    uint8_t data[MPS + 3U];
    for (unsigned i = 0; i < sizeof(data); i++) data[i] = (uint8_t)i;

    CHECK(DeviceIntrfTxData(&s_Intrf.DevIntrf, data, sizeof(data)) ==
          (int)sizeof(data));
    CHECK(s_InDma && s_InLength == MPS && s_InSubmit == 1);
    CHECK(memcmp(s_InBuffer, data, MPS) == 0);

    s_InDma = false;
    s_InHandler(USB_ENDPADDR_DIRIN(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
                MPS, USB_CTRLR_XFER_SUCCESS, s_InContext);
    CHECK(s_InDma && s_InLength == 3U && s_InSubmit == 2);
    CHECK(memcmp(s_InBuffer, data + MPS, 3U) == 0);

    s_InDma = false;
    s_InHandler(USB_ENDPADDR_DIRIN(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
                3U, USB_CTRLR_XFER_SUCCESS, s_InContext);
    CHECK(!s_InDma);
}

int main(void)
{
    TestNoPreArm();
    TestBlocking();
    TestNonBlocking();
    TestDisableDoesNotGateController();
    TestTxStillChains();

    printf("%s\n", s_Fail == 0 ? "usb_rx_event_test: PASS" :
                                 "usb_rx_event_test: FAIL");
    return s_Fail == 0 ? 0 : 1;
}