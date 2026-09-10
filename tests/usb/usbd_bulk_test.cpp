/**-------------------------------------------------------------------------
@file	usbd_bulk_test.cpp

@brief	Host tests for the public vendor bulk USB class.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "usb/usbd_bulk.h"

#define EP_NO      1U
#define ITF_NO     0
#define RX_SLOTS   4U
#define TX_SLOTS   4U

static UsbCfg_t s_UsbCfg;
static UsbdClassCfg_t s_ClassCfg;
static bool s_ClassRegistered;
static uint8_t s_ReservedFirst;
static uint8_t s_ReservedCount;
static uint16_t s_ReservedIn;
static uint16_t s_ReservedOut;
static UsbEndPointDesc_t s_OpenDesc[2];
static int s_OpenCount;
static int s_CloseCount;

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBlocking;
static bool s_HwOutReady;
static bool s_OutDma;
static uint16_t s_HwOutLength;
static uint8_t s_HwOut[USBD_BULK_FS_MPS];
static bool s_InBusy;
static uint16_t s_InLength;
static int s_OutSubmitCount;
static int s_SendCount;
static void *s_RequestContext;
static int s_RequestCount;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
    return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbdClassRegister(int DevNo, const UsbdClassCfg_t *pCfg)
{
    if (DevNo != 0 || pCfg == nullptr)
        return false;

    if (pCfg->InterfaceCount != 0U && s_ReservedCount != 0U)
    {
        const uint16_t firstA = pCfg->FirstInterface;
        const uint16_t lastA = firstA + pCfg->InterfaceCount;
        const uint16_t firstB = s_ReservedFirst;
        const uint16_t lastB = firstB + s_ReservedCount;
        if (firstA < lastB && firstB < lastA)
            return false;
    }

    if ((pCfg->EpInMask & s_ReservedIn) != 0U ||
        (pCfg->EpOutMask & s_ReservedOut) != 0U)
        return false;

    s_ClassCfg = *pCfg;
    s_ClassRegistered = true;
    return true;
}

bool UsbCtrlrHighSpeed(int) { return false; }

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
    if (pDesc == nullptr || s_OpenCount >= 2)
        return false;
    s_OpenDesc[s_OpenCount++] = *pDesc;
    return true;
}

void UsbCtrlrEpClose(int, uint8_t) { s_CloseCount++; }

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
        if (s_InBusy)
            return false;
        s_InBusy = true;
        s_InLength = Length;
        s_SendCount++;
        return true;
    }

    if (!s_HwOutReady || s_OutDma)
        return false;
    s_OutDma = true;
    s_OutSubmitCount++;
    return true;
}

bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[USBD_BULK_RXMEM_SIZE(RX_SLOTS)];
alignas(4) static uint8_t s_TxByteMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_TxPacketMem[USBD_BULK_TXMEM_SIZE(TX_SLOTS)];

static bool VendorRequest(const UsbSetupData_t *, UsbCtrlStage_t,
                          uint8_t **, uint16_t *, void *pContext)
{
    s_RequestContext = pContext;
    s_RequestCount++;
    return true;
}

static UsbdBulkCfg_t MakeCfg(UsbdBulkMode_t Mode)
{
    UsbdBulkCfg_t cfg = {};
    cfg.bBlocking = true;
    cfg.RxFifoMemSize = sizeof(s_RxMem);
    cfg.pRxFifoMem = s_RxMem;
    cfg.TxFifoMemSize = Mode == USBD_BULK_MODE_PACKET ?
        sizeof(s_TxPacketMem) : sizeof(s_TxByteMem);
    cfg.pTxFifoMem = Mode == USBD_BULK_MODE_PACKET ? s_TxPacketMem : s_TxByteMem;
    cfg.DevNo = 0;
    cfg.SubClass = 0x12U;
    cfg.Protocol = 0x34U;
    cfg.InterfaceString = 5U;
    cfg.Mode = Mode;
    cfg.RequestHandler = VendorRequest;
    cfg.pRequestContext = &s_RequestContext;
    return cfg;
}

static void ResetFake(void)
{
    memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
    memset(&s_ClassCfg, 0, sizeof(s_ClassCfg));
    memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
    memset(s_HwOut, 0, sizeof(s_HwOut));
    s_ClassRegistered = false;
    s_ReservedFirst = 0U;
    s_ReservedCount = 0U;
    s_ReservedIn = 0U;
    s_ReservedOut = 0U;
    s_OpenCount = 0;
    s_CloseCount = 0;
    s_OutBuffer = nullptr;
    s_InBuffer = nullptr;
    s_OutHandler = nullptr;
    s_InHandler = nullptr;
    s_OutContext = nullptr;
    s_InContext = nullptr;
    s_OutBlocking = true;
    s_HwOutReady = false;
    s_OutDma = false;
    s_HwOutLength = 0U;
    s_InBusy = false;
    s_InLength = 0U;
    s_OutSubmitCount = 0;
    s_SendCount = 0;
    s_RequestContext = nullptr;
    s_RequestCount = 0;
    s_UsbCfg.DevNo = 0;
}

static void DeliverOut(const uint8_t *pData, uint16_t Length)
{
    CHECK(!s_HwOutReady);
    CHECK(s_OutHandler != nullptr);
    CHECK(s_OutBlocking);
    if (Length > 0U)
        memcpy(s_HwOut, pData, Length);
    s_HwOutLength = Length;
    s_HwOutReady = true;

    s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_DRDY,
                 Length, USB_CTRLR_XFER_SUCCESS, s_OutContext);
    CHECK(s_OutDma);
    if (!s_OutDma)
        return;

    if (Length > 0U)
        memcpy(s_OutBuffer, s_HwOut, Length);
    s_HwOutReady = false;
    s_OutDma = false;
    s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
                 s_HwOutLength, USB_CTRLR_XFER_SUCCESS, s_OutContext);
}

static void CompleteIn(void)
{
    CHECK(s_InBusy);
    if (!s_InBusy)
        return;
    const uint16_t length = s_InLength;
    s_InBusy = false;
    s_InHandler(USB_ENDPADDR_DIRIN(EP_NO), USB_CTRLR_EVT_XFER_CMPL,
                length, USB_CTRLR_XFER_SUCCESS, s_InContext);
}

static void TestDescriptor(void)
{
    ResetFake();
    UsbdBulk bulk;
    UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_BYTE);
    UsbdBulkDesc_t desc = {};
    cfg.pDesc = &desc;

    CHECK(bulk.Init(cfg));
    CHECK(sizeof(desc) == sizeof(UsbIntrfDesc_t) + 2U * sizeof(UsbEndPointDesc_t));
    CHECK(desc.Interface.bInterfaceNumber == ITF_NO);
    CHECK(desc.Interface.bNumEndpoints == 2U);
    CHECK(desc.Interface.bInterfaceClass == USB_INTRFCLASS_VENDOR);
    CHECK(desc.Interface.bInterfaceSubClass == cfg.SubClass);
    CHECK(desc.Interface.bInterfaceProtocol == cfg.Protocol);
    CHECK(desc.Out.bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
    CHECK(desc.In.bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
    CHECK(desc.Out.bmAttributes == USB_ENDPATT_TRANS_BULK);
    CHECK(desc.Out.wMaxPacketSize == USBD_BULK_FS_MPS);
}

static void TestAutoPlacement(void)
{
    ResetFake();
    s_ReservedFirst = 0U;
    s_ReservedCount = 1U;
    s_ReservedIn = (uint16_t)(1U << 1);
    s_ReservedOut = (uint16_t)(1U << 1);

    UsbdBulk bulk;
    const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_BYTE);
    CHECK(bulk.Init(cfg));
    CHECK(s_ClassRegistered);
    CHECK(s_ClassCfg.FirstInterface == 1U);
    CHECK(s_ClassCfg.EpInMask == (1U << 2));
    CHECK(s_ClassCfg.EpOutMask == (1U << 2));
}

static void TestByteMode(void)
{
    ResetFake();
    UsbdBulk bulk;
    const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_BYTE);

    CHECK(bulk.Init(cfg));
    CHECK(s_ClassRegistered);
    CHECK(s_OutBuffer != nullptr && s_InBuffer != nullptr);
    CHECK(s_OutBuffer != s_InBuffer);
    CHECK(s_OutSubmitCount == 0);

    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(s_OpenCount == 2);
    CHECK(s_OpenDesc[0].bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
    CHECK(s_OpenDesc[1].bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
    CHECK(s_OpenDesc[0].wMaxPacketSize == USBD_BULK_FS_MPS);
    CHECK(s_OutSubmitCount == 0);

    const uint8_t rx[] = { 1U, 2U, 3U, 4U };
    DeliverOut(rx, sizeof(rx));
    CHECK(s_OutSubmitCount == 1);
    uint8_t received[sizeof(rx)] = {};
    CHECK(bulk.RxData(received, sizeof(received)) == (int)sizeof(received));
    CHECK(memcmp(received, rx, sizeof(rx)) == 0);

    const uint8_t tx[] = { 5U, 6U, 7U };
    CHECK(bulk.TxData(tx, sizeof(tx)) == (int)sizeof(tx));
    CHECK(s_InBusy && s_InLength == sizeof(tx));
    CHECK(memcmp(s_InBuffer, tx, sizeof(tx)) == 0);
    CompleteIn();
    CHECK(!s_InBusy);

    UsbSetupData_t setup = {};
    setup.bmRequestType = USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE;
    setup.wIndex = ITF_NO;
    uint16_t length = 0U;
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_SETUP, nullptr, &length,
                                   s_ClassCfg.pContext));
    CHECK(s_RequestCount == 1);
    CHECK(s_RequestContext == &s_RequestContext);

    CHECK(s_ClassCfg.ConfigHandler(0U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfGetRate(bulk.Data()) == 0U);
}

static void TestPacketMode(void)
{
    ResetFake();
    UsbdBulk bulk;
    const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_PACKET);
    CHECK(bulk.Init(cfg));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));

    alignas(4) uint8_t block[USBD_BULK_PKT_BLKSIZE] = {};
    UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(block);
    pPacket->Hdr.Length = 5U;
    memcpy(pPacket->Data, "bulk!", 5U);

    CHECK(bulk.TxData(block, sizeof(block)) == (int)sizeof(block));
    CHECK(s_InBusy && s_InLength == 5U);
    CHECK(memcmp(s_InBuffer, "bulk!", 5U) == 0);
}

int main(void)
{
    TestDescriptor();
    TestAutoPlacement();
    TestByteMode();
    TestPacketMode();

    if (s_Fail != 0)
    {
        printf("%d failed\n", s_Fail);
        return 1;
    }
    printf("all pass\n");
    return 0;
}
