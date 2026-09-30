/**-------------------------------------------------------------------------
@file	usbd_hid_test.cpp

@brief	Host tests for the generic USB HID device class.
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "usb/usbd_hid.h"

#define EP_NO	1U
#define ITF_NO	0

static UsbCfg_t s_UsbCfg;
static UsbDeviceClass *s_ClassObject;
static bool s_Registered;
static uint8_t s_ReservedFirst;
static uint8_t s_ReservedCount;
static uint16_t s_ReservedIn;
static uint16_t s_ReservedOut;
static const uint8_t *s_FsDescriptor;
static uint16_t s_FsDescriptorLength;
static uint8_t s_FsDescriptorStorage[sizeof(UsbdHidDesc_t)];
static UsbEndPointDesc_t s_Open[2];
static int s_OpenCount;
static int s_CloseCount;
static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBlocking;
static bool s_InBusy;
static uint16_t s_InLength;
static int s_OutXferCount;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbCtrlrHighSpeed(int) { return false; }

// Opens are counted without limit, as the controller allows a pair to be
// closed and opened again; the first two descriptors are kept for checks.
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr)
	{
		return false;
	}
	if (s_OpenCount < 2)
	{
		s_Open[s_OpenCount] = *pDesc;
	}
	s_OpenCount++;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t, bool) { s_CloseCount++; }

void UsbCtrlrEpBind(int, uint8_t, bool bIn, bool Blocking,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (bIn)
	{
		s_InBuffer = nullptr;
		s_InHandler = Handler;
		s_InContext = pContext;
	}
	else
	{
		s_OutBuffer = nullptr;
		s_OutHandler = Handler;
		s_OutContext = pContext;
		s_OutBlocking = Blocking;
	}
}

bool UsbCtrlrEpReceive(int, uint8_t, uint8_t *pBuffer, uint16_t Capacity)
{
	if (s_OutBuffer != nullptr || pBuffer == nullptr || Capacity == 0U) return false;
	s_OutBuffer = pBuffer;
	return true;
}

bool UsbCtrlrEpSend(int, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if ((EpNum & 0x80U) != 0U) return false;
	if (s_InBusy) return false;
	s_InBuffer = pBuffer;
	s_InBusy = true;
	s_InLength = Length;
	return true;
}

}

bool UsbClassRegister(int DevNo, UsbDeviceClass *pClass,
					  uint8_t FirstInterface, uint8_t InterfaceCount,
					  uint16_t EpInMask, uint16_t EpOutMask)
{
	if (DevNo != 0 || pClass == nullptr || s_ClassObject != nullptr)
	{
		return false;
	}
	if (InterfaceCount != 0U && s_ReservedCount != 0U)
	{
		const unsigned lastA = FirstInterface + InterfaceCount;
		const unsigned lastB = s_ReservedFirst + s_ReservedCount;
		if (FirstInterface < lastB && s_ReservedFirst < lastA)
		{
			return false;
		}
	}
	if ((EpInMask & s_ReservedIn) != 0U ||
		(EpOutMask & s_ReservedOut) != 0U)
	{
		return false;
	}
	s_ClassObject = pClass;
	s_ReservedFirst = FirstInterface;
	s_ReservedCount = InterfaceCount;
	s_ReservedIn = EpInMask;
	s_ReservedOut = EpOutMask;
	s_Registered = true;
	return true;
}

bool UsbDescRegister(int DevNo, UsbDeviceClass *pClass,
								   const void *pDescriptor,
								   uint16_t DescriptorLength,
								   UsbDescBuild_t Build)
{
	if (DevNo != 0 || pClass != s_ClassObject || pDescriptor == nullptr ||
		DescriptorLength != sizeof(s_FsDescriptorStorage) || Build == nullptr)
	{
		return false;
	}
	memcpy(s_FsDescriptorStorage, pDescriptor, DescriptorLength);
	Build(pClass, s_FsDescriptorStorage, USB_SPEED_FULL);
	s_FsDescriptor = s_FsDescriptorStorage;
	s_FsDescriptorLength = DescriptorLength;
	return true;
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static const uint8_t s_ReportDesc[] = {
	0x06U, 0x00U, 0xFFU,		// Usage Page (Vendor)
	0x09U, 0x01U,				// Usage 1
	0xA1U, 0x01U,				// Collection (Application)
	0x75U, 0x08U, 0x95U, 0x08U,
	0x09U, 0x01U, 0x81U, 0x02U,
	0x09U, 0x01U, 0x91U, 0x02U,
	0xC0U,
};

static int s_RxCount;
static int s_TxCount;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;
static uint8_t s_LastRx[USBD_HID_FS_MPS];
static int s_ReportStageCount;
static UsbCtrlStage_t s_ReportStage[4];
static uint8_t s_ControlReport[8];

static int HidEvent(DevIntrf_t *pDev, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	CHECK(UsbdHidGetDevHandle(pDev)->pContext == &s_Fail);
	const UsbCtrlrXferResult_t result =
		(event == DEVINTRF_EVT_RX_TIMEOUT || event == DEVINTRF_EVT_TX_TIMEOUT) ?
		USB_CTRLR_XFER_FAILED : USB_CTRLR_XFER_SUCCESS;
	if (event == DEVINTRF_EVT_RX_DATA || event == DEVINTRF_EVT_RX_TIMEOUT)
	{
		CHECK(result == USB_CTRLR_XFER_SUCCESS);
		s_RxCount++;
		s_LastRxLength = Length;
		if (Length != 0U)
		{
			memcpy(s_LastRx, pData, Length);
		}
	}
	else if (event == DEVINTRF_EVT_TX_FIFO_EMPTY || event == DEVINTRF_EVT_TX_TIMEOUT)
	{
		CHECK(result == USB_CTRLR_XFER_SUCCESS);
		s_TxCount++;
		s_LastTxLength = Length;
	}
	return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
}

static bool ReportRequest(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
						  uint8_t **ppData, uint16_t *pLength)
{
	if (s_ReportStageCount < 4)
	{
		s_ReportStage[s_ReportStageCount++] = Stage;
	}
	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr || pLength == nullptr)
		{
			return false;
		}
		*ppData = s_ControlReport;
		*pLength = pSetup->bRequest == USB_HID_REQ_GET_REPORT ?
			3U : sizeof(s_ControlReport);
	}
	return true;
}

class TestHid final : public UsbdHid {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		if (pSetup != nullptr &&
			(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) ==
				USB_REQTYPE_CLASS &&
			(pSetup->bmRequestType & USB_REQTYPE_MASK_RECIPIENT) ==
				USB_REQTYPE_INTERFACE &&
			(pSetup->wIndex & 0xFF00U) == 0U &&
			(uint8_t)pSetup->wIndex == FirstInterface() &&
			(pSetup->bRequest == USB_HID_REQ_GET_REPORT ||
			 pSetup->bRequest == USB_HID_REQ_SET_REPORT))
		{
			return ReportRequest(pSetup, Stage, ppData, pLength);
		}
		return UsbdHid::Control(pSetup, Stage, ppData, pLength);
	}
};

static UsbdHidCfg_t MakeCfg(void)
{
	UsbdHidCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.pReportDesc = s_ReportDesc;
	cfg.ReportDescLength = sizeof(s_ReportDesc);
	cfg.SubClass = USB_HID_SUBCLASS_BOOT;
	cfg.Protocol = USB_HID_PROT_KEYBOARD;
	cfg.CountryCode = 33U;
	cfg.InterfaceString = 4U;
	cfg.EvtCB = HidEvent;
	cfg.pContext = &s_Fail;
	return cfg;
}

static void ResetFake(void)
{
	memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
	memset(s_Open, 0, sizeof(s_Open));
	memset(s_LastRx, 0, sizeof(s_LastRx));
	memset(s_ReportStage, 0, sizeof(s_ReportStage));
	memset(s_ControlReport, 0, sizeof(s_ControlReport));
	s_UsbCfg.DevNo = 0;
	s_ClassObject = nullptr;
	s_Registered = false;
	s_ReservedFirst = 0U;
	s_ReservedCount = 0U;
	s_ReservedIn = 0U;
	s_ReservedOut = 0U;
	s_FsDescriptor = nullptr;
	s_FsDescriptorLength = 0U;
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_OutBlocking = false;
	s_InBusy = false;
	s_InLength = 0U;
	s_OutXferCount = 0;
	s_RxCount = 0;
	s_TxCount = 0;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_ReportStageCount = 0;
}

static void CompleteIn(void)
{
	CHECK(s_InBusy);
	const uint16_t length = s_InLength;
	s_InBusy = false;
	s_InHandler(USB_CTRLR_EVT_XFER_CMPL,
		length, s_InContext);
}

static void Receive(const uint8_t *pData, uint16_t Length)
{
	s_OutHandler(USB_CTRLR_EVT_DRDY, Length, s_OutContext);
	CHECK(s_OutBuffer != nullptr);
	s_OutXferCount++;
	if (Length != 0U)
	{
		memcpy(s_OutBuffer, pData, Length);
	}
	s_OutBuffer = nullptr;
	s_OutHandler(USB_CTRLR_EVT_XFER_CMPL, Length, s_OutContext);
}

static bool Control(UsbdHid &Hid, const UsbSetupData_t *pSetup,
					UsbCtrlStage_t Stage,
					uint8_t **ppData, uint16_t *pLength)
{
	return Hid.Control(pSetup, Stage, ppData, pLength);
}

static void TestDescriptorAndPlacement(void)
{
	ResetFake();
	s_ReservedFirst = 0U;
	s_ReservedCount = 1U;
	s_ReservedIn = (uint16_t)(1U << 1);
	s_ReservedOut = (uint16_t)(1U << 1);

	TestHid hid;
	UsbdHidCfg_t cfg = MakeCfg();
	CHECK(hid.Init(cfg));
	UsbIntrf *pTransport = &hid;
	DeviceIntrf *pDevice = pTransport;
	CHECK(pTransport->Data() == &static_cast<UsbdHidDev_t *>(hid)->pIntIntrf->pData->DevIntrf);
	CHECK(static_cast<DevIntrf_t *>(*pDevice) == hid.Data());
	CHECK(s_FsDescriptorLength == sizeof(UsbdHidDesc_t));
	const UsbdHidDesc_t &desc =
		*reinterpret_cast<const UsbdHidDesc_t *>(s_FsDescriptor);
	CHECK(s_Registered);
	CHECK(s_ClassObject == &hid);
	CHECK(s_ReservedFirst == 1U);
	CHECK(s_ReservedIn == (1U << 2));
	CHECK(s_ReservedOut == (1U << 2));
	CHECK(desc.Interface.bInterfaceNumber == 1U);
	CHECK(desc.Interface.bNumEndpoints == 2U);
	CHECK(desc.Interface.bInterfaceClass == USB_INTRFCLASS_HID);
	CHECK(desc.Interface.bInterfaceSubClass == USB_HID_SUBCLASS_BOOT);
	CHECK(desc.Interface.bInterfaceProtocol == USB_HID_PROT_KEYBOARD);
	CHECK(desc.Hid.bDescriptorType == USB_DESCTYPE_HID);
	CHECK(desc.Hid.bcdHID == USBD_HID_BCD_VERSION);
	CHECK(desc.Hid.bCountryCode == 33U);
	CHECK(desc.Hid.RepDesc[0].bDescriptorType == USB_DESCTYPE_HID_REPORT);
	CHECK(desc.Hid.RepDesc[0].wDescriptorLength == sizeof(s_ReportDesc));
	CHECK(desc.Out.bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
	CHECK(desc.In.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(desc.Out.bmAttributes == USB_ENDPATT_TRANS_INT);
}

static void TestDataAndLifecycle(void)
{
	ResetFake();
	TestHid hid;
	const UsbdHidCfg_t cfg = MakeCfg();
	CHECK(hid.Init(cfg));
	CHECK(s_OutBlocking);
	CHECK(hid.SelectConfig(USBD_HID_CONFIG_VALUE));
	CHECK(s_OpenCount == 2);
	CHECK(s_Open[0].bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
	CHECK(s_Open[1].bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
	CHECK(s_Open[0].bmAttributes == USB_ENDPATT_TRANS_INT);

	const uint8_t tx[] = { 1U, 2U, 3U };
	CHECK(hid.Tx(0, tx, sizeof(tx)) == (int)sizeof(tx));
	CHECK(s_InBusy && s_InLength == sizeof(tx));
	CHECK(memcmp(s_InBuffer, tx, sizeof(tx)) == 0);
	CHECK(hid.Tx(0, tx, sizeof(tx)) == 0);
	CompleteIn();
	CHECK(s_TxCount == 1 && s_LastTxLength == sizeof(tx));

	DevIntrf_t *pDev = hid.Data();
	CHECK(DeviceIntrfStartTx(pDev, 0));
	CHECK(DeviceIntrfTxData(pDev, nullptr, 0) == 0);
	CHECK(!atomic_load(&pDev->bTxReady));
	DeviceIntrfStopTx(pDev);
	CompleteIn();
	CHECK(s_TxCount == 2 && s_LastTxLength == 0U);

	const uint8_t rx[] = { 4U, 5U };
	Receive(rx, sizeof(rx));
	CHECK(s_OutXferCount == 1);
	CHECK(s_RxCount == 1 && s_LastRxLength == sizeof(rx));
	CHECK(memcmp(s_LastRx, rx, sizeof(rx)) == 0);
	Receive(nullptr, 0U);
	CHECK(s_RxCount == 2 && s_LastRxLength == 0U);

	hid.Disable();
	CHECK(hid.Tx(0, tx, sizeof(tx)) == 0);
	CHECK(static_cast<UsbdHidDev_t *>(hid)->pIntIntrf->Mps != 0U);
	CHECK(static_cast<UsbdHidDev_t *>(hid)->pIntIntrf->pData->Mps == 0U);
	hid.Enable();
	CHECK(hid.Tx(0, tx, sizeof(tx)) == (int)sizeof(tx));
	CompleteIn();

	CHECK(hid.SelectConfig(0U));
	CHECK(hid.Tx(0, tx, sizeof(tx)) == 0);
}

static void TestClassRequestDirectionAndLength(void)
{
	const uint8_t requests[] = {USB_HID_REQ_GET_IDLE, USB_HID_REQ_GET_PROTOCOL,
		USB_HID_REQ_SET_IDLE, USB_HID_REQ_SET_PROTOCOL};
	const UsbCtrlStage_t stages[] = {USB_CTRL_SETUP, USB_CTRL_DATA,
		USB_CTRL_COMPLETE, USB_CTRL_ABORT};
	for (unsigned request = 0U; request < 4U; request++)
	{
		for (unsigned direction = 0U; direction < 2U; direction++)
		{
			for (unsigned count = 0U; count < 3U; count++)
			{
				ResetFake();
				TestHid hid;
				CHECK(hid.Init(MakeCfg()));
				UsbSetupData_t setup = {};
				setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE |
					(direction ? USB_REQTYPE_DIRHOST : USB_REQTYPE_DIRDEV);
				setup.bRequest = requests[request];
				setup.wIndex = ITF_NO;
				setup.wLength = count;
				const bool get = request < 2U;
				const bool valid = (direction != 0U) == get && count == (get ? 1U : 0U);
				for (UsbCtrlStage_t stage : stages)
				{
					uint8_t *data = nullptr;
					uint16_t length = 0U;
					CHECK(Control(hid, &setup, stage, &data, &length) == valid);
				}
			}
		}
	}
}

static void TestRequestRecipientAndType(void)
{
	ResetFake();
	UsbdHid hid;
	CHECK(hid.Init(MakeCfg()));
	for (unsigned requestType = 0U; requestType <= 255U; requestType++)
	{
		UsbSetupData_t setup = {};
		setup.bmRequestType = requestType;
		setup.wIndex = ITF_NO;
		setup.wLength = 1U;
		uint8_t *data = nullptr;
		uint16_t length = 0U;
		setup.bRequest = USB_HID_REQ_GET_IDLE;
		CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length) ==
			(requestType == (USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
			 USB_REQTYPE_INTERFACE)));
		setup.bRequest = USB_REQ_GET_DESCRIPTOR;
		setup.wValue = USB_DESCTYPE_HID_REPORT << 8;
		CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length) ==
			(requestType == (USB_REQTYPE_DIRHOST | USB_REQTYPE_STANDARD |
			 USB_REQTYPE_INTERFACE)));
	}
}

static void TestControlRequests(void)
{
	ResetFake();
	TestHid hid;
	const UsbdHidCfg_t cfg = MakeCfg();
	CHECK(hid.Init(cfg));
	CHECK(hid.SelectConfig(USBD_HID_CONFIG_VALUE));

	UsbSetupData_t setup = {};
	uint8_t *pData = nullptr;
	uint16_t length = 0U;
	setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_STANDARD |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_REQ_GET_DESCRIPTOR;
	setup.wIndex = ITF_NO;
	setup.wLength = 255U;
	setup.wValue = (uint16_t)(USB_DESCTYPE_HID_REPORT << 8);
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(pData == s_ReportDesc && length == sizeof(s_ReportDesc));
	setup.wValue = (uint16_t)(USB_DESCTYPE_HID << 8);
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(length == sizeof(UsbHidDesc_t));

	setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_SET_IDLE;
	setup.wValue = 7U << 8;
	setup.wLength = 0U;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(Control(hid, &setup, USB_CTRL_COMPLETE, &pData, &length));
	setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_GET_IDLE;
	setup.wValue = 0U;
	setup.wLength = 1U;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(length == 1U && pData[0] == 7U);

	setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_SET_PROTOCOL;
	setup.wValue = USBD_HID_PROTOCOL_BOOT;
	setup.wLength = 0U;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(Control(hid, &setup, USB_CTRL_COMPLETE, &pData, &length));
	setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_GET_PROTOCOL;
	setup.wValue = 0U;
	setup.wLength = 1U;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(length == 1U && pData[0] == USBD_HID_PROTOCOL_BOOT);

	setup.bRequest = USB_HID_REQ_GET_REPORT;
	setup.wValue = USB_HID_REPTYPE_INPUT | 1U;
	setup.wLength = 3U;
	s_ReportStageCount = 0;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(pData == s_ControlReport && length == 3U);
	CHECK(Control(hid, &setup, USB_CTRL_COMPLETE, &pData, &length));
	CHECK(s_ReportStageCount == 2);
	CHECK(s_ReportStage[0] == USB_CTRL_SETUP);
	CHECK(s_ReportStage[1] == USB_CTRL_COMPLETE);

	setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_SET_REPORT;
	setup.wValue = USB_HID_REPTYPE_OUTPUT | 2U;
	setup.wLength = 4U;
	s_ReportStageCount = 0;
	CHECK(Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));
	CHECK(length == sizeof(s_ControlReport));
	CHECK(Control(hid, &setup, USB_CTRL_DATA, &pData, &setup.wLength));
	CHECK(Control(hid, &setup, USB_CTRL_COMPLETE, &pData, &setup.wLength));
	CHECK(s_ReportStageCount == 3);

	setup.wIndex = 1U;
	CHECK(!Control(hid, &setup, USB_CTRL_SETUP, &pData, &length));

	hid.Reset();
	CHECK(hid.Tx(0, s_ControlReport, 1) == 0);
}

static void TestValidation(void)
{
	ResetFake();
	TestHid hid;
	UsbdHidCfg_t cfg = MakeCfg();
	cfg.pReportDesc = nullptr;
	CHECK(!hid.Init(cfg));
	cfg = MakeCfg();
	cfg.ReportDescLength = 0U;
	CHECK(!hid.Init(cfg));
	cfg = MakeCfg();
	cfg.FsMps = USB_INT_INTRF_FS_MPS + 1U;
	CHECK(!hid.Init(cfg));
	cfg = MakeCfg();
	cfg.SubClass = USB_HID_SUBCLASS_NONE;
	cfg.Protocol = USB_HID_PROT_KEYBOARD;
	CHECK(!hid.Init(cfg));
}

static void TestSubclassProtocolValidation(void)
{
	// Only non-boot/none and boot/keyboard or boot/mouse are supported.
	for (unsigned subclass = 0; subclass <= 255; subclass++)
	for (unsigned protocol = 0; protocol <= 255; protocol++)
	{
		ResetFake();
		TestHid hid;
		auto cfg = MakeCfg();
		cfg.SubClass = subclass;
		cfg.Protocol = protocol;
		const bool valid = (subclass == 0 && protocol == 0) ||
			(subclass == 1 && (protocol == 1 || protocol == 2));
		CHECK(hid.Init(cfg) == valid);
	}
}

static void TestPendingControlLifetime(void)
{
	ResetFake();
	UsbdHid hid;
	CHECK(hid.Init(MakeCfg()));
	UsbdHidDev_t *state = hid;
	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
		USB_REQTYPE_INTERFACE;
	setup.bRequest = USB_HID_REQ_SET_IDLE;
	setup.wValue = 7U << 8;
	uint8_t *data = nullptr;
	uint16_t length = 0U;
	CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length));
	CHECK(state->Idle == 0U && state->PendingIdle == 7U);
	CHECK(hid.Control(&setup, USB_CTRL_DATA, &data, &length));
	CHECK(state->Idle == 0U && state->PendingRequest == USB_HID_REQ_SET_IDLE);
	CHECK(hid.Control(&setup, USB_CTRL_ABORT, &data, &length));
	CHECK(state->Idle == 0U && state->PendingRequest == 0U);
	CHECK(hid.Control(&setup, USB_CTRL_COMPLETE, &data, &length));
	CHECK(state->Idle == 0U);

	setup.bRequest = USB_HID_REQ_SET_PROTOCOL;
	setup.wValue = USBD_HID_PROTOCOL_BOOT;
	CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length));
	setup.bRequest = USB_HID_REQ_SET_IDLE;
	CHECK(hid.Control(&setup, USB_CTRL_COMPLETE, &data, &length));
	CHECK(state->PendingRequest == USB_HID_REQ_SET_PROTOCOL);
	CHECK(state->Idle == 0U && state->ActiveProtocol == USBD_HID_PROTOCOL_REPORT);
	setup.bRequest = USB_HID_REQ_SET_PROTOCOL;
	CHECK(hid.Control(&setup, USB_CTRL_COMPLETE, &data, &length));
	CHECK(state->ActiveProtocol == USBD_HID_PROTOCOL_BOOT && state->PendingRequest == 0U);
}

static void TestProtocolValues(void)
{
	for (unsigned boot = 0; boot < 2; boot++)
	{
		ResetFake();
		UsbdHid hid;
		auto cfg = MakeCfg();
		cfg.SubClass = boot ? USB_HID_SUBCLASS_BOOT : USB_HID_SUBCLASS_NONE;
		cfg.Protocol = boot ? USB_HID_PROT_KEYBOARD : USB_HID_PROT_NONE;
		CHECK(hid.Init(cfg));
		UsbSetupData_t setup = {};
		uint8_t *data = nullptr;
		uint16_t length = 0U;
		for (unsigned value = 0; value <= 65535; value++)
		{
			setup.wValue = value;
			setup.bmRequestType = USB_REQTYPE_DIRHOST | USB_REQTYPE_CLASS |
				USB_REQTYPE_INTERFACE;
			setup.bRequest = USB_HID_REQ_GET_PROTOCOL;
			setup.wLength = 1U;
			CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length) ==
				(boot != 0 && value == 0));
			setup.bmRequestType = USB_REQTYPE_DIRDEV | USB_REQTYPE_CLASS |
				USB_REQTYPE_INTERFACE;
			setup.bRequest = USB_HID_REQ_SET_PROTOCOL;
			setup.wLength = 0U;
			CHECK(hid.Control(&setup, USB_CTRL_SETUP, &data, &length) ==
				(boot != 0 && value <= USBD_HID_PROTOCOL_REPORT));
		}
	}
}

int main(void)
{
	TestProtocolValues();
	TestSubclassProtocolValidation();
	TestPendingControlLifetime();
	TestDescriptorAndPlacement();
	TestDataAndLifecycle();
	TestClassRequestDirectionAndLength();
	TestRequestRecipientAndType();
	TestControlRequests();
	TestValidation();
	printf("%s\n", s_Fail == 0 ? "usbd_hid_test: PASS" :
		"usbd_hid_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}


