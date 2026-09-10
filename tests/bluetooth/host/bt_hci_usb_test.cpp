/**-------------------------------------------------------------------------
@file	bt_hci_usb_test.cpp

@brief	Host tests for Bluetooth HCI USB over event-driven UsbIntrf/UsbIsoIntrf.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "bluetooth/bt_hci_usb.h"

#define RX_SLOTS 8U
#define TX_SLOTS 20U

typedef struct {
    uint8_t EpAddr;
    uint8_t *pBuffer;
    bool Blocking;
    UsbCtrlrEpHandler_t Handler;
    void *pContext;
} RegisteredEp_t;

typedef struct {
    uint8_t EpAddr;
    uint16_t Length;
    uint8_t Data[BT_HCI_USB_ACL_MAX_MPS];
} SentPacket_t;

static UsbCfg_t s_UsbCfg;
static UsbdClassCfg_t s_ClassCfg;
static bool s_FuncRegistered;
static uint8_t s_ReservedFirst;
static uint8_t s_ReservedCount;
static uint16_t s_ReservedIn;
static uint16_t s_ReservedOut;
static UsbEndPointDesc_t s_OpenDesc[32];
static int s_OpenCount;
static int s_OpenFailAt;
static int s_CloseCount;
static RegisteredEp_t s_Registered[5];
static int s_RegisteredCount;
static bool s_InBusy[16];
static bool s_OutDma[16];
static bool s_HwOutReady[16];
static uint16_t s_HwOutLength[16];
static uint8_t s_HwOut[16][BT_HCI_USB_ACL_MAX_MPS];
static SentPacket_t s_Sent[64];
static int s_SendCount;
static int s_OutSubmitCount;
static int s_RxEventCount;
static int s_TxEventCount;
static int s_LastEventLength;
static DEVINTRF_EVT s_LastEvent;
static DevIntrf_t *s_LastEventDev;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
    return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbdClassRegister(int DevNo, const UsbdClassCfg_t *pCfg)
{
    if (DevNo != 0 || pCfg == nullptr ||
        (((pCfg->EpInMask | pCfg->EpOutMask) & 1U) != 0U))
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
    s_FuncRegistered = true;
    return true;
}

bool UsbCtrlrHighSpeed(int) { return false; }

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
    if (pDesc == nullptr || s_OpenCount >= 32)
        return false;
    if (s_OpenCount == s_OpenFailAt)
    {
        s_OpenFailAt = -1;
        return false;
    }
    s_OpenDesc[s_OpenCount++] = *pDesc;
    return true;
}

void UsbCtrlrEpClose(int, uint8_t EpAddr)
{
    s_CloseCount++;
    const uint8_t epNo = USB_ENDPADDR_NUM(EpAddr);
    if (epNo < 16U)
    {
        if (USB_ENDPADDR_IS_IN(EpAddr)) s_InBusy[epNo] = false;
        else s_OutDma[epNo] = false;
    }
}
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer, bool Blocking,
                        UsbCtrlrEpHandler_t Handler, void *pContext)
{
    if (pBuffer == nullptr || Handler == nullptr || s_RegisteredCount >= 5)
        return false;
    s_Registered[s_RegisteredCount++] = {
        EpAddr, pBuffer, Blocking, Handler, pContext
    };
    return true;
}

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
    const uint8_t epNo = USB_ENDPADDR_NUM(EpAddr);
    if (epNo >= 16U)
        return false;

    RegisteredEp_t *pReg = nullptr;
    for (int i = 0; i < s_RegisteredCount; i++)
    {
        if (s_Registered[i].EpAddr == EpAddr)
        {
            pReg = &s_Registered[i];
            break;
        }
    }
    if (pReg == nullptr)
        return false;

    if (USB_ENDPADDR_IS_IN(EpAddr))
    {
        if (s_InBusy[epNo] || s_SendCount >= 64 || Length > sizeof(s_Sent[0].Data))
            return false;
        SentPacket_t *pSent = &s_Sent[s_SendCount++];
        pSent->EpAddr = EpAddr;
        pSent->Length = Length;
        if (Length > 0U) memcpy(pSent->Data, pReg->pBuffer, Length);
        s_InBusy[epNo] = true;
        return true;
    }

    if (!s_HwOutReady[epNo] || s_OutDma[epNo])
        return false;
    s_OutDma[epNo] = true;
    s_OutSubmitCount++;
    return true;
}

bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[BT_HCI_USB_ACL_RXMEM_SIZE(RX_SLOTS)];
alignas(4) static uint8_t s_TxMem[BT_HCI_USB_ACL_TXMEM_SIZE(TX_SLOTS)];

static int HciEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Evt,
                    uint8_t *, int Length)
{
    s_LastEvent = Evt;
    s_LastEventDev = pDev;
    s_LastEventLength = Length;
    if (Evt == DEVINTRF_EVT_RX_DATA) s_RxEventCount++;
    if (Evt == DEVINTRF_EVT_TX_READY || Evt == DEVINTRF_EVT_TX_FIFO_EMPTY)
        s_TxEventCount++;
    return Length;
}

static BtHciUsbCfg_t MakeCfg(bool Sco = false, bool Serial = false)
{
    BtHciUsbCfg_t cfg = {};
    cfg.bBlocking = true;
    cfg.bSco = Sco;
    cfg.bBulkSerialization = Serial;
    cfg.RxFifoMemSize = sizeof(s_RxMem);
    cfg.pRxFifoMem = s_RxMem;
    cfg.TxFifoMemSize = sizeof(s_TxMem);
    cfg.pTxFifoMem = s_TxMem;
    cfg.DevNo = 0;
    cfg.InterfaceString = 4U;
    cfg.EvtCB = HciEvent;
    return cfg;
}

static RegisteredEp_t *FindRegistered(uint8_t EpAddr)
{
    for (int i = 0; i < s_RegisteredCount; i++)
        if (s_Registered[i].EpAddr == EpAddr) return &s_Registered[i];
    return nullptr;
}

static void CompleteIn(uint8_t EpNo,
                       UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
    RegisteredEp_t *pReg = FindRegistered(USB_ENDPADDR_DIRIN(EpNo));
    CHECK(pReg != nullptr);
    CHECK(EpNo < 16U && s_InBusy[EpNo]);
    if (pReg == nullptr || EpNo >= 16U || !s_InBusy[EpNo]) return;

    uint16_t length = 0U;
    for (int i = s_SendCount - 1; i >= 0; i--)
    {
        if (s_Sent[i].EpAddr == USB_ENDPADDR_DIRIN(EpNo))
        {
            length = s_Sent[i].Length;
            break;
        }
    }
    s_InBusy[EpNo] = false;
    pReg->Handler(USB_ENDPADDR_DIRIN(EpNo), USB_CTRLR_EVT_XFER_CMPL,
                  length, Result, pReg->pContext);
}

static void ReceiveOut(uint8_t EpNo, const uint8_t *pData, uint16_t Length)
{
    RegisteredEp_t *pReg = FindRegistered(USB_ENDPADDR_DIROUT(EpNo));
    CHECK(pReg != nullptr);
    CHECK(EpNo < 16U);
    CHECK(!s_HwOutReady[EpNo]);
    if (pReg == nullptr || EpNo >= 16U || Length > sizeof(s_HwOut[EpNo])) return;

    if (Length > 0U) memcpy(s_HwOut[EpNo], pData, Length);
    s_HwOutLength[EpNo] = Length;
    s_HwOutReady[EpNo] = true;

    if (pReg->Blocking)
    {
        pReg->Handler(USB_ENDPADDR_DIROUT(EpNo), USB_CTRLR_EVT_DRDY,
                      Length, USB_CTRLR_XFER_SUCCESS, pReg->pContext);
        CHECK(s_OutDma[EpNo]);
        if (!s_OutDma[EpNo]) return;
    }
    else
    {
        // Non-blocking endpoint data goes directly to controller DMA.
        s_OutDma[EpNo] = true;
    }

    if (Length > 0U) memcpy(pReg->pBuffer, s_HwOut[EpNo], Length);
    s_HwOutReady[EpNo] = false;
    s_OutDma[EpNo] = false;
    pReg->Handler(USB_ENDPADDR_DIROUT(EpNo), USB_CTRLR_EVT_XFER_CMPL,
                  Length, USB_CTRLR_XFER_SUCCESS, pReg->pContext);
}

static void ResetFake(void)
{
    memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
    memset(&s_ClassCfg, 0, sizeof(s_ClassCfg));
    memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
    memset(s_Registered, 0, sizeof(s_Registered));
    memset(s_InBusy, 0, sizeof(s_InBusy));
    memset(s_OutDma, 0, sizeof(s_OutDma));
    memset(s_HwOutReady, 0, sizeof(s_HwOutReady));
    memset(s_HwOutLength, 0, sizeof(s_HwOutLength));
    memset(s_HwOut, 0, sizeof(s_HwOut));
    memset(s_Sent, 0, sizeof(s_Sent));
    s_FuncRegistered = false;
    s_ReservedFirst = 0U;
    s_ReservedCount = 0U;
    s_ReservedIn = 0U;
    s_ReservedOut = 0U;
    s_OpenCount = 0;
    s_OpenFailAt = -1;
    s_CloseCount = 0;
    s_RegisteredCount = 0;
    s_SendCount = 0;
    s_OutSubmitCount = 0;
    s_RxEventCount = 0;
    s_TxEventCount = 0;
    s_LastEventLength = 0;
    s_LastEvent = DEVINTRF_EVT_RX_TIMEOUT;
    s_LastEventDev = nullptr;
    s_UsbCfg.DevNo = 0;
}

static void QueueRxPacket(BtHciUsb &hci, const uint8_t *pData, uint16_t Length)
{
    UsbDevIntrf_t *pIntrf =
        static_cast<UsbDevIntrf_t *>(hci.Data()->pDevData);
    CHECK(pIntrf != nullptr);
    CHECK(pIntrf != nullptr && Length <= pIntrf->Mps);
    if (pIntrf == nullptr || Length > pIntrf->Mps) return;

    UsbPkt_t *pPacket =
        reinterpret_cast<UsbPkt_t *>(CFifoPut(pIntrf->hRxFifo));
    CHECK(pPacket != nullptr);
    if (pPacket == nullptr) return;

    pPacket->Hdr.Length = Length;
    pPacket->Hdr.Reserved = 0U;
    if (Length > 0U)
    {
        CHECK(pData != nullptr);
        if (pData != nullptr) memcpy(pPacket->Data, pData, Length);
    }
}

static void TestDescriptors(void)
{
    ResetFake();
    BtHciUsb hci;
    BtHciUsbDesc_t desc = {};
    BtHciUsbCfg_t cfg = MakeCfg();
    cfg.pDesc = &desc;
    CHECK(hci.Init(cfg));
    CHECK(desc.Association.bFirstInterface == 0U);
    CHECK(desc.Association.bInterfaceCount == 2U);
    CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
    CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
    CHECK(desc.AclIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
    CHECK(desc.Sync.bInterfaceNumber == 1U);

    ResetFake();
    BtHciUsb sco;
    BtHciUsbScoDesc_t scoDesc = {};
    cfg = MakeCfg(true);
    cfg.pScoDesc = &scoDesc;
    CHECK(sco.Init(cfg));
    static const uint16_t mps[BT_HCI_USB_SCO_ALT_COUNT] = {9U,17U,25U,33U,49U,63U};
    for (uint8_t i = 0U; i < BT_HCI_USB_SCO_ALT_COUNT; i++)
    {
        CHECK(scoDesc.Alt[i].Interface.bAlternateSetting == i + 1U);
        CHECK(scoDesc.Alt[i].Out.bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
        CHECK(scoDesc.Alt[i].In.bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
        CHECK(scoDesc.Alt[i].Out.wMaxPacketSize == mps[i]);
    }

    ResetFake();
    BtHciUsb serial;
    BtHciUsbSerialDesc_t serialDesc = {};
    cfg = MakeCfg(false, true);
    cfg.pSerialDesc = &serialDesc;
    CHECK(serial.Init(cfg));
    CHECK(serialDesc.Serialized.Interface.bAlternateSetting == 1U);
    CHECK(serialDesc.Serialized.Interface.bNumEndpoints == 2U);
}

static void TestDescriptorSelection(void)
{
    ResetFake();
    BtHciUsb wrongLegacy;
    BtHciUsbDesc_t legacy = {};
    BtHciUsbCfg_t cfg = MakeCfg(true);
    cfg.pDesc = &legacy;
    CHECK(!wrongLegacy.Init(cfg));

    ResetFake();
    BtHciUsb wrongSco;
    BtHciUsbScoDesc_t sco = {};
    cfg = MakeCfg(true, true);
    cfg.pScoDesc = &sco;
    CHECK(!wrongSco.Init(cfg));

    ResetFake();
    BtHciUsb multiple;
    BtHciUsbFullDesc_t full = {};
    cfg = MakeCfg(true, true);
    cfg.pScoDesc = &sco;
    cfg.pFullDesc = &full;
    CHECK(!multiple.Init(cfg));

    ResetFake();
    BtHciUsb validFull;
    memset(&full, 0, sizeof(full));
    cfg = MakeCfg(true, true);
    cfg.pFullDesc = &full;
    CHECK(validFull.Init(cfg));
    CHECK(full.Base.Serialized.Interface.bAlternateSetting == 1U);
    CHECK(full.Alt[0].Out.bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
    CHECK(full.Alt[0].In.bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
}

static void TestAutoPlacement(void)
{
    ResetFake();
    s_ReservedFirst = 0U;
    s_ReservedCount = 2U;
    s_ReservedIn = (uint16_t)(1U << 1);
    s_ReservedOut = (uint16_t)(1U << 1);
    BtHciUsb hci;
    BtHciUsbDesc_t desc = {};
    BtHciUsbCfg_t cfg = MakeCfg();
    cfg.pDesc = &desc;
    CHECK(hci.Init(cfg));
    CHECK(s_ClassCfg.FirstInterface == 2U);
    CHECK(s_ClassCfg.EpInMask == ((1U << 2) | (1U << 3)));
    CHECK(s_ClassCfg.EpOutMask == (1U << 3));
    CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
    CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(3U));
}

static void TestScoAutoPlacement(void)
{
    ResetFake();
    s_ReservedIn = (uint16_t)(1U << 8);
    s_ReservedOut = (uint16_t)(1U << 8);
    BtHciUsb hci;
    BtHciUsbScoDesc_t desc = {};
    BtHciUsbCfg_t cfg = MakeCfg(true);
    cfg.pScoDesc = &desc;
    CHECK(hci.Init(cfg));
    CHECK(s_ClassCfg.EpInMask == ((1U << 1) | (1U << 2) | (1U << 9)));
    CHECK(s_ClassCfg.EpOutMask == ((1U << 2) | (1U << 9)));
    CHECK(desc.Alt[0].Out.bEndpointAddress == USB_ENDPADDR_DIROUT(9U));
    CHECK(desc.Alt[0].In.bEndpointAddress == USB_ENDPADDR_DIRIN(9U));

    ResetFake();
    s_ReservedIn = (uint16_t)((1U << 8) | (1U << 9));
    s_ReservedOut = s_ReservedIn;
    BtHciUsb blocked;
    CHECK(!blocked.Init(MakeCfg(true)));
}

static void TestConfigurationAndAcl(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg()));
    CHECK(s_RegisteredCount == 3);
    CHECK(FindRegistered(USB_ENDPADDR_DIROUT(2U))->Blocking);
    CHECK(s_ClassCfg.ConfigHandler(BT_HCI_USB_CONFIG_VALUE, s_ClassCfg.pContext));
    CHECK(s_OpenCount == 3);
    CHECK(s_OpenDesc[0].bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
    CHECK(s_OpenDesc[1].bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
    CHECK(s_OpenDesc[2].bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
    CHECK(s_OutSubmitCount == 0);
    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 0U, s_ClassCfg.pContext));

    uint8_t acl[8] = { 0x01,0x00,0x04,0x00, 0x11,0x22,0x33,0x44 };
    CHECK(DeviceIntrfStartRx(hci.Data(), BT_HCI_USB_PACKET_ACL));
    ReceiveOut(2U, acl, sizeof(acl));
    CHECK(s_OutSubmitCount == 1);
    CHECK(s_RxEventCount == 1);
    uint8_t out[sizeof(acl)] = {};
    CHECK(hci.Data()->RxData(hci.Data(), out, sizeof(out)) == (int)sizeof(acl));
    DeviceIntrfStopRx(hci.Data());
    CHECK(memcmp(out, acl, sizeof(acl)) == 0);

    CHECK(DeviceIntrfStartTx(hci.Data(), BT_HCI_USB_PACKET_ACL));
    CHECK(hci.RequestToSend(sizeof(acl)));
    CHECK(hci.Data()->TxData(hci.Data(), acl, sizeof(acl)) == (int)sizeof(acl));
    DeviceIntrfStopTx(hci.Data());
    CHECK(s_SendCount == 1);
    CHECK(s_Sent[0].EpAddr == USB_ENDPADDR_DIRIN(2U));
    CHECK(s_Sent[0].Length == sizeof(acl));
    CHECK(memcmp(s_Sent[0].Data, acl, sizeof(acl)) == 0);
    CompleteIn(2U);

    CHECK(s_ClassCfg.ConfigHandler(0U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfGetRate(hci.Data()) == 0U);
}

static void TestBulkZlpPreservesNextPacket(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg()));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfStartRx(hci.Data(), BT_HCI_USB_PACKET_ACL));

    const uint8_t acl[] = { 0x01U,0x00U,0x04U,0x00U, 0x11U,0x22U,0x33U,0x44U };
    QueueRxPacket(hci, nullptr, 0U);
    QueueRxPacket(hci, acl, sizeof(acl));
    CHECK(hci.Data()->EvtCB(hci.Data(), DEVINTRF_EVT_RX_DATA, nullptr, 0) ==
          (int)sizeof(acl));
    CHECK(s_RxEventCount == 1);

    uint8_t out[sizeof(acl)] = {};
    CHECK(hci.Data()->RxData(hci.Data(), out, sizeof(out)) == (int)sizeof(acl));
    CHECK(memcmp(out, acl, sizeof(acl)) == 0);
    DeviceIntrfStopRx(hci.Data());
}

static void TestCommandAndEvent(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg()));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));

    const uint8_t cmd[] = { 0x03U, 0x0CU, 0x00U };
    UsbSetupData_t setup = {};
    setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
    setup.bRequest = 0U;
    setup.wIndex = 0U;
    setup.wLength = BT_HCI_USB_COMMAND_HEADER_SIZE - 1U;
    uint8_t *pData = nullptr;
    uint16_t length = 0U;
    CHECK(!s_ClassCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
                                    s_ClassCfg.pContext));

    setup.wLength = sizeof(cmd);
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
                                   s_ClassCfg.pContext));
    memcpy(pData, cmd, sizeof(cmd));
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_DATA, &pData, &length,
                                   s_ClassCfg.pContext));
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_COMPLETE, &pData, &length,
                                   s_ClassCfg.pContext));
    CHECK(s_RxEventCount == 1);
    uint8_t rx[8] = {};
    CHECK(DeviceIntrfRx(hci.Data(), BT_HCI_USB_PACKET_COMMAND, rx, sizeof(rx)) == 3);
    CHECK(memcmp(rx, cmd, sizeof(cmd)) == 0);

    setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_DEVICE;
    setup.bRequest = 0xE0U;
    setup.wValue = 0x1234U;
    setup.wIndex = 0x5678U;
    setup.wLength = sizeof(cmd);
    pData = nullptr;
    length = 0U;
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
                                   s_ClassCfg.pContext));
    memcpy(pData, cmd, sizeof(cmd));
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_DATA, &pData, &length,
                                   s_ClassCfg.pContext));
    CHECK(s_ClassCfg.RequestHandler(&setup, USB_CTRL_COMPLETE, &pData, &length,
                                   s_ClassCfg.pContext));
    CHECK(s_RxEventCount == 2);
    memset(rx, 0, sizeof(rx));
    CHECK(DeviceIntrfRx(hci.Data(), BT_HCI_USB_PACKET_COMMAND, rx, sizeof(rx)) == 3);
    CHECK(memcmp(rx, cmd, sizeof(cmd)) == 0);

    const uint8_t evt[] = { 0x0EU, 0x01U, 0x00U };
    CHECK(DeviceIntrfStartTx(hci.Data(), BT_HCI_USB_PACKET_EVENT));
    CHECK(hci.RequestToSend(sizeof(evt)));
    CHECK(hci.Data()->TxData(hci.Data(), evt, sizeof(evt)) == (int)sizeof(evt));
    DeviceIntrfStopTx(hci.Data());
    CHECK(s_SendCount == 1);
    if (s_SendCount > 0)
        CHECK(s_Sent[s_SendCount - 1].EpAddr == USB_ENDPADDR_DIRIN(1U));
    CompleteIn(1U);
    CHECK(s_LastEvent == DEVINTRF_EVT_TX_READY);
}

static void TestBulkSerialization(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg(false, true)));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(s_ClassCfg.SetInterfaceHandler(0U, 1U, s_ClassCfg.pContext));
    BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(hci);
    CHECK(pHci->HciAlt == 1U && pHci->BulkSerialization);

    const int sameAltOpen = s_OpenCount;
    const int sameAltClose = s_CloseCount;
    CHECK(s_ClassCfg.SetInterfaceHandler(0U, 1U, s_ClassCfg.pContext));
    CHECK(s_OpenCount == sameAltOpen);
    CHECK(s_CloseCount == sameAltClose);
    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 0U, s_ClassCfg.pContext));
    CHECK(s_OpenCount == sameAltOpen);
    CHECK(s_CloseCount == sameAltClose);

    const uint8_t cmd[] = { 0x03U, 0x0CU, 0x00U };
    const uint8_t wireCmd[] = {
        BT_HCI_USB_PACKET_COMMAND, 0x03U, 0x0CU, 0x00U
    };
    CHECK(DeviceIntrfStartRx(hci.Data(), BT_HCI_USB_PACKET_COMMAND));
    ReceiveOut(2U, wireCmd, sizeof(wireCmd));
    CHECK(s_RxEventCount == 1);
    uint8_t rx[sizeof(cmd)] = {};
    CHECK(hci.Data()->RxData(hci.Data(), rx, sizeof(rx)) == (int)sizeof(cmd));
    DeviceIntrfStopRx(hci.Data());
    CHECK(memcmp(rx, cmd, sizeof(cmd)) == 0);

    const uint8_t evt[] = { 0x0EU, 0x01U, 0x00U };
    CHECK(DeviceIntrfStartTx(hci.Data(), BT_HCI_USB_PACKET_EVENT));
    CHECK(hci.RequestToSend(sizeof(evt)));
    CHECK(hci.Data()->TxData(hci.Data(), evt, sizeof(evt)) == (int)sizeof(evt));
    DeviceIntrfStopTx(hci.Data());
    CHECK(s_SendCount == 1);
    if (s_SendCount > 0)
    {
        CHECK(s_Sent[0].EpAddr == USB_ENDPADDR_DIRIN(2U));
        CHECK(s_Sent[0].Length == sizeof(evt) + 1U);
        CHECK(s_Sent[0].Data[0] == BT_HCI_USB_PACKET_EVENT);
        CHECK(memcmp(&s_Sent[0].Data[1], evt, sizeof(evt)) == 0);
    }
    CompleteIn(2U);

    s_OpenFailAt = s_OpenCount + 2;
    CHECK(!s_ClassCfg.SetInterfaceHandler(0U, 0U, s_ClassCfg.pContext));
    CHECK(pHci->HciAlt == 1U);
    CHECK(pHci->BulkSerialization);
    CHECK(DeviceIntrfGetRate(hci.Data()) != 0U);
}

static void TestScoTransport(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg(true)));
    CHECK(s_RegisteredCount == 5);
    RegisteredEp_t *scoOut = FindRegistered(USB_ENDPADDR_DIROUT(8U));
    CHECK(scoOut != nullptr && !scoOut->Blocking);
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 1U, s_ClassCfg.pContext));

    // Base endpoints 0..2, then ISO opens IN before OUT.
    CHECK(s_OpenCount == 5);
    CHECK(s_OpenDesc[3].bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
    CHECK(s_OpenDesc[4].bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
    CHECK(s_OpenDesc[3].bmAttributes == USB_ENDPATT_TRANS_ISO);
    CHECK(s_OpenDesc[3].wMaxPacketSize == 9U);

    uint8_t packet[20] = {};
    packet[0] = 0x01U;
    packet[1] = 0x00U;
    packet[2] = 17U;
    for (unsigned i = 3U; i < sizeof(packet); i++) packet[i] = (uint8_t)(0x20U + i);

    const int beforeOutSubmit = s_OutSubmitCount;
    CHECK(DeviceIntrfStartRx(hci.Data(), BT_HCI_USB_PACKET_SCO));
    ReceiveOut(8U, packet, 9U);
    ReceiveOut(8U, &packet[9], 9U);
    ReceiveOut(8U, &packet[18], 2U);
    CHECK(s_OutSubmitCount == beforeOutSubmit); // non-blocking: no DRDY round trip
    CHECK(s_RxEventCount == 1);
    uint8_t received[sizeof(packet)] = {};
    CHECK(hci.Data()->RxData(hci.Data(), received, sizeof(received)) ==
          (int)sizeof(packet));
    DeviceIntrfStopRx(hci.Data());
    CHECK(memcmp(received, packet, sizeof(packet)) == 0);

    CHECK(DeviceIntrfStartTx(hci.Data(), BT_HCI_USB_PACKET_SCO));
    CHECK(hci.RequestToSend(sizeof(packet)));
    CHECK(hci.Data()->TxData(hci.Data(), packet, sizeof(packet)) ==
          (int)sizeof(packet));
    DeviceIntrfStopTx(hci.Data());
    CHECK(s_SendCount > 0);
    if (s_SendCount > 0)
    {
        CHECK(s_Sent[s_SendCount - 1].EpAddr == USB_ENDPADDR_DIRIN(8U));
        CHECK(s_Sent[s_SendCount - 1].Length == 9U);
    }
    CompleteIn(8U);
    CHECK(s_SendCount > 0 && s_Sent[s_SendCount - 1].Length == 9U);
    CompleteIn(8U);
    CHECK(s_SendCount > 0 && s_Sent[s_SendCount - 1].Length == 2U);
    CompleteIn(8U);
    CHECK(s_LastEvent == DEVINTRF_EVT_TX_READY);

    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 0U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfTx(hci.Data(), BT_HCI_USB_PACKET_SCO,
                        packet, sizeof(packet)) == 0);
}

static void TestScoBackpressure(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg(true)));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 1U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfStartRx(hci.Data(), BT_HCI_USB_PACKET_SCO));

    const uint8_t first[] = { 0x01U,0x00U,0x02U,0x11U,0x22U };
    const uint8_t second[] = { 0x02U,0x00U,0x02U,0x33U,0x44U };
    ReceiveOut(8U, first, sizeof(first));
    ReceiveOut(8U, second, sizeof(second));
    CHECK(s_RxEventCount == 1);
    CHECK(s_LastEvent == DEVINTRF_EVT_RX_FIFO_FULL);
    CHECK(s_LastEventLength == (int)sizeof(second));

    uint8_t out[sizeof(first)] = {};
    CHECK(hci.Data()->RxData(hci.Data(), out, sizeof(out)) == (int)sizeof(first));
    CHECK(memcmp(out, first, sizeof(first)) == 0);

    ReceiveOut(8U, second, sizeof(second));
    CHECK(s_RxEventCount == 2);
    memset(out, 0, sizeof(out));
    CHECK(hci.Data()->RxData(hci.Data(), out, sizeof(out)) == (int)sizeof(second));
    CHECK(memcmp(out, second, sizeof(second)) == 0);
    DeviceIntrfStopRx(hci.Data());
}

static void TestScoAlternateLifecycle(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg(true)));
    CHECK(s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    BtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(hci);
    static const uint16_t mps[BT_HCI_USB_SCO_ALT_COUNT] = {9U,17U,25U,33U,49U,63U};
    for (uint8_t alt = 1U; alt <= BT_HCI_USB_SCO_ALT_COUNT; alt++)
    {
        const int before = s_OpenCount;
        CHECK(s_ClassCfg.SetInterfaceHandler(1U, alt, s_ClassCfg.pContext));
        CHECK(s_OpenCount == before + 2);
        CHECK(s_OpenDesc[before].bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
        CHECK(s_OpenDesc[before + 1].bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
        CHECK(s_OpenDesc[before].wMaxPacketSize == mps[alt - 1U]);

        const int sameAltOpen = s_OpenCount;
        const int sameAltClose = s_CloseCount;
        CHECK(s_ClassCfg.SetInterfaceHandler(1U, alt, s_ClassCfg.pContext));
        CHECK(s_OpenCount == sameAltOpen);
        CHECK(s_CloseCount == sameAltClose);
    }
    CHECK(!s_ClassCfg.SetInterfaceHandler(1U, 7U, s_ClassCfg.pContext));

    s_OpenFailAt = s_OpenCount;
    CHECK(!s_ClassCfg.SetInterfaceHandler(1U, 5U, s_ClassCfg.pContext));
    CHECK(pHci->ScoAlt == 6U);
    CHECK(pHci->ScoIso.Opened);
    CHECK(pHci->ScoIso.Mps == mps[5]);

    CHECK(s_ClassCfg.SetInterfaceHandler(1U, 0U, s_ClassCfg.pContext));
    CHECK(pHci->ScoAlt == 0U);
}

static void TestConfigurationFailure(void)
{
    ResetFake();
    BtHciUsb hci;
    CHECK(hci.Init(MakeCfg()));
    s_OpenFailAt = 1;
    CHECK(!s_ClassCfg.ConfigHandler(1U, s_ClassCfg.pContext));
    CHECK(DeviceIntrfGetRate(hci.Data()) == 0U);
}

int main(void)
{
    TestDescriptors();
    TestDescriptorSelection();
    TestAutoPlacement();
    TestScoAutoPlacement();
    TestConfigurationAndAcl();
    TestBulkZlpPreservesNextPacket();
    TestCommandAndEvent();
    TestBulkSerialization();
    TestScoTransport();
    TestScoBackpressure();
    TestScoAlternateLifecycle();
    TestConfigurationFailure();

    if (s_Fail != 0)
    {
        printf("%d failed\n", s_Fail);
        return 1;
    }
    printf("all pass\n");
    return 0;
}
