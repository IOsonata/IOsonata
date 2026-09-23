/**-------------------------------------------------------------------------
@example	usb_combo_stress.cpp

@brief	USB composite stress test: dual CDC + HID + interrupt + ISO loopback.

Runs all USB data paths on one controller at the same time. Two CDC functions
retain the UsbDualCdcStress loopback/PRBS behavior. HID, raw interrupt and
isochronous functions each provide a bidirectional loopback. Interface and
endpoint numbers are assigned by the common allocator; ISO reserves the
controller-supported ISO endpoint.

Host runner: Python/usb_combo_stress.py

@author	Hoang Nguyen Hoan
@date	Sep. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>

#include "cfifo.h"
#include "prbs.h"
#include "usb/usb.h"
#include "usb/usb_int.h"
#include "usb/usb_iso.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_epalloc.h"
#include "usb/usbd_hid.h"

#define USB_DEVNO			0

#define CDC_BUFFER_SIZE		USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)
#define CDC_RXFIFO_PKTCNT	4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, CDC_BUFFER_SIZE)
#define LOOPBACK_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)
#define PRBS_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

#define COMBO_STR_INTERFACE	4U
#define HID_REPORT_SIZE			64U

#define INT_ALT_COUNT			3U
#define INT_MPS					64U

#define ISO_ALT_COUNT			6U
#define ISO_REQ_GET_DIAG		0x5AU

alignas(4) static uint8_t s_LoopbackRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_LoopbackTxFifoMem[LOOPBACK_TXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsTxFifoMem[PRBS_TXFIFO_MEMSIZE];

UsbdCdc g_LoopbackCdc;
UsbdCdc g_PrbsCdc;

static atomic_bool s_LoopbackSessionStart = true;

static int LoopbackEvtHandler(DevIntrf_t * const, DEVINTRF_EVT EvtId,
							  uint8_t *, int Len)
{
	if (EvtId == DEVINTRF_EVT_STATECHG && Len)
	{
		static const char msg[] = "\r\nIOsonata USB Combo Stress\r\n";
		atomic_store(&s_LoopbackSessionStart, true);
		g_LoopbackCdc.Tx(0, reinterpret_cast<const uint8_t *>(msg),
			(int)sizeof(msg) - 1);
	}
	return 0;
}

static const UsbdCdcCfg_t s_LoopbackCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_LoopbackRxFifoMem,
	.TxFifoMemSize = LOOPBACK_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_LoopbackTxFifoMem,
	.EvtCB = LoopbackEvtHandler,
};

static const UsbdCdcCfg_t s_PrbsCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_PrbsRxFifoMem,
	.TxFifoMemSize = PRBS_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_PrbsTxFifoMem,
	.EvtCB = nullptr,
};

/* HID --------------------------------------------------------------------- */

static const uint8_t s_HidReportDesc[] = {
	0x06U, 0x00U, 0xFFU,
	0x09U, 0x01U,
	0xA1U, 0x01U,
	0x75U, 0x08U,
	0x95U, HID_REPORT_SIZE,
	0x09U, 0x01U,
	0x81U, 0x02U,
	0x95U, HID_REPORT_SIZE,
	0x09U, 0x01U,
	0x91U, 0x02U,
	0xC0U,
};

static uint8_t s_HidPending[HID_REPORT_SIZE];
static uint16_t s_HidPendingLength;
static uint8_t s_HidControlReport[HID_REPORT_SIZE];

static bool HidReportRequest(const UsbSetupData_t *pSetup,
	UsbCtrlStage_t Stage, uint8_t **ppData, uint16_t *pLength);

class HidLoopback final : public UsbdHid {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		if (pSetup != nullptr &&
			(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) == USB_REQTYPE_CLASS &&
			(pSetup->bRequest == USB_HID_REQ_GET_REPORT ||
			 pSetup->bRequest == USB_HID_REQ_SET_REPORT))
		{
			return HidReportRequest(pSetup, Stage, ppData, pLength);
		}
		return UsbdHid::Control(pSetup, Stage, ppData, pLength);
	}
};

static HidLoopback g_Hid;

static void HidRx(UsbdHidDev_t *, const uint8_t *pData, uint16_t Length,
	UsbCtrlrXferResult_t Result, void *)
{
	if (Result != USB_CTRLR_XFER_SUCCESS || Length > sizeof(s_HidPending))
	{
		return;
	}
	if (!g_Hid.SendReport(pData, Length))
	{
		memcpy(s_HidPending, pData, Length);
		s_HidPendingLength = Length;
	}
}

static void HidTx(UsbdHidDev_t *, uint16_t, UsbCtrlrXferResult_t Result,
	void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS && s_HidPendingLength != 0U)
	{
		const uint16_t length = s_HidPendingLength;
		s_HidPendingLength = 0U;
		if (!g_Hid.SendReport(s_HidPending, length))
		{
			s_HidPendingLength = length;
		}
	}
}

static bool HidReportRequest(const UsbSetupData_t *pSetup,
	UsbCtrlStage_t Stage, uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup == nullptr || pLength == nullptr)
	{
		return false;
	}
	if (Stage == USB_CTRL_ABORT)
	{
		return true;
	}
	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr || pSetup->wLength > sizeof(s_HidControlReport))
		{
			return false;
		}
		*ppData = s_HidControlReport;
		*pLength = sizeof(s_HidControlReport);
		return true;
	}
	if (Stage == USB_CTRL_COMPLETE &&
		pSetup->bRequest == USB_HID_REQ_SET_REPORT)
	{
		return g_Hid.SendReport(s_HidControlReport, *pLength);
	}
	return true;
}

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_HidReportDesc,
	.ReportDescLength = sizeof(s_HidReportDesc),
	.BcdHid = 0U,
	.FsMps = HID_REPORT_SIZE,
	.HsMps = HID_REPORT_SIZE,
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_NONE,
	.Protocol = USB_HID_PROT_NONE,
	.CountryCode = 0U,
	.InterfaceString = COMBO_STR_INTERFACE,
	.RxHandler = HidRx,
	.TxHandler = HidTx,
	.pContext = nullptr,
};

/* Raw interrupt ----------------------------------------------------------- */

static constexpr uint8_t s_IntIntervals[INT_ALT_COUNT] = { 1U, 4U, 16U };

#pragma pack(push, 1)
typedef struct __Combo_Int_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} ComboIntAltDesc_t;

typedef struct __Combo_Int_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboIntAltDesc_t Alt[INT_ALT_COUNT];
} ComboIntFunctionDesc_t;
#pragma pack(pop)

static UsbIntIntrf_t s_Int;
static UsbDevIntrf_t s_IntData;
static bool s_IntConfigured;
static uint8_t s_IntAlt;
static uint8_t s_IntInterfaceNo;
static uint8_t s_IntEpNo;

static void IntRxPacket(UsbIntIntrf_t *, const uint8_t *pData,
	uint16_t Length, UsbCtrlrXferResult_t Result, void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS)
	{
		(void)UsbIntIntrfSendPacket(&s_Int, pData, Length);
	}
}

static bool IntSelectConfig(uint8_t Configuration)
{
	UsbIntIntrfClose(&s_Int);
	s_IntConfigured = false;
	s_IntAlt = 0U;
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != 1U)
	{
		return false;
	}
	s_IntConfigured = true;
	return true;
}

static bool IntSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_IntConfigured || InterfaceNo != s_IntInterfaceNo ||
		Alt > INT_ALT_COUNT)
	{
		return false;
	}

	UsbIntIntrfClose(&s_Int);
	s_IntAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}
	if (!UsbIntIntrfOpen(&s_Int, INT_MPS, s_IntIntervals[Alt - 1U]))
	{
		return false;
	}
	s_IntAlt = Alt;
	return true;
}

static void IntReset(void)
{
	s_IntConfigured = false;
	s_IntAlt = 0U;
	UsbIntIntrfReset(&s_Int);
}

static void IntProcess(void)
{
	if (!s_IntConfigured || s_IntAlt == 0U)
	{
		return;
	}
	const bool suspended = UsbSuspended(USB_DEVNO);
	if (suspended && !s_Int.Suspended)
	{
		UsbIntIntrfSuspend(&s_Int);
	}
	else if (!suspended && s_Int.Suspended)
	{
		(void)UsbIntIntrfResume(&s_Int);
	}
}

static constexpr ComboIntFunctionDesc_t IntFunctionDescTemplate(void)
{
	ComboIntFunctionDesc_t desc = {};
	desc.Alt0.bLength = sizeof(desc.Alt0);
	desc.Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	desc.Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	desc.Alt0.iInterface = COMBO_STR_INTERFACE;

	for (unsigned i = 0U; i < INT_ALT_COUNT; i++)
	{
		ComboIntAltDesc_t &alt = desc.Alt[i];
		alt.Interface = desc.Alt0;
		alt.Interface.bAlternateSetting = (uint8_t)(i + 1U);
		alt.Interface.bNumEndpoints = 2U;
		alt.Out.bLength = sizeof(alt.Out);
		alt.Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		alt.Out.bmAttributes = USB_ENDPATT_TRANS_INT;
		alt.Out.wMaxPacketSize = INT_MPS;
		alt.Out.bInterval = s_IntIntervals[i];
		alt.In = alt.Out;
	}
	return desc;
}

static constexpr ComboIntFunctionDesc_t s_IntFunctionDesc =
	IntFunctionDescTemplate();

static void IntPatchFunctionDesc(const UsbDeviceClass *, uint8_t *pData,
	UsbSpeed_t)
{
	ComboIntFunctionDesc_t *pDesc =
		reinterpret_cast<ComboIntFunctionDesc_t *>(pData);
	pDesc->Alt0.bInterfaceNumber = s_IntInterfaceNo;
	for (unsigned i = 0U; i < INT_ALT_COUNT; i++)
	{
		ComboIntAltDesc_t &alt = pDesc->Alt[i];
		alt.Interface.bInterfaceNumber = s_IntInterfaceNo;
		alt.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_IntEpNo);
		alt.In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_IntEpNo);
	}
}

class IntLoopbackClass final : public UsbDeviceClass {
public:
	bool SelectConfig(uint8_t ConfigValue) override {
		return IntSelectConfig(ConfigValue);
	}
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override {
		return IntSelectInterface(InterfaceNo, Option);
	}
	void Reset(void) override { IntReset(); }
	void Process(void) override { IntProcess(); }
};

static IntLoopbackClass s_IntClass;

static bool IntInit(void)
{
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;
	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IntClass, &alloc))
	{
		return false;
	}

	s_IntInterfaceNo = alloc.FirstInterface;
	s_IntEpNo = alloc.Bidirectional[0];

	UsbIntIntrfCfg_t cfg = {};
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_IntEpNo;
	cfg.RxHandler = IntRxPacket;
	if (!UsbIntIntrfInit(&s_Int, &s_IntData, &cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IntClass,
		&s_IntFunctionDesc, sizeof(s_IntFunctionDesc), IntPatchFunctionDesc);
}

/* ISO --------------------------------------------------------------------- */

static constexpr uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

#pragma pack(push, 1)
typedef struct __Combo_Iso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} ComboIsoAltDesc_t;

typedef struct __Combo_Iso_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	ComboIsoAltDesc_t Alt[ISO_ALT_COUNT];
} ComboIsoFunctionDesc_t;
#pragma pack(pop)

static UsbIsoIntrf_t s_Iso;
static UsbDevIntrf_t s_IsoData;
static bool s_IsoConfigured;
static uint8_t s_IsoAlt;
static uint8_t s_IsoInterfaceNo;
static uint8_t s_IsoEpNo;
static uint32_t s_IsoLoopbackDropCnt;

#pragma pack(push, 1)
typedef struct __Combo_Iso_Diag {
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
} ComboIsoDiag_t;
#pragma pack(pop)

static ComboIsoDiag_t s_IsoDiag;

static uint8_t IsoFirstEndpoint(uint16_t Mask)
{
	for (uint8_t ep = 1U; ep < 16U; ep++)
	{
		if ((Mask & (uint16_t)(1U << ep)) != 0U)
		{
			return ep;
		}
	}
	return 0U;
}

static void IsoRxFrame(UsbIsoIntrf_t *, const uint8_t *pData,
	uint16_t Length, UsbCtrlrXferResult_t Result, void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS &&
		!UsbIsoIntrfSendFrame(&s_Iso, pData, Length))
	{
		s_IsoLoopbackDropCnt++;
	}
}

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup == nullptr ||
		pSetup->bmRequestType !=
			(USB_REQTYPE_DIRHOST | USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE) ||
		pSetup->bRequest != ISO_REQ_GET_DIAG ||
		pSetup->wValue != 0U ||
		pSetup->wIndex != s_IsoInterfaceNo ||
		pSetup->wLength != sizeof(s_IsoDiag))
	{
		return false;
	}
	if (Stage != USB_CTRL_SETUP)
	{
		return true;
	}
	if (ppData == nullptr || pLength == nullptr)
	{
		return false;
	}

	s_IsoDiag.RxMissCnt = s_Iso.RxMissCnt;
	s_IsoDiag.TxMissCnt = s_Iso.TxMissCnt;
	s_IsoDiag.LoopbackDropCnt = s_IsoLoopbackDropCnt;
	s_IsoDiag.RxEmptyCnt = s_Iso.RxEmptyCnt;
	s_IsoDiag.TxEmptyCnt = s_Iso.TxEmptyCnt;
	*ppData = reinterpret_cast<uint8_t *>(&s_IsoDiag);
	*pLength = sizeof(s_IsoDiag);
	return true;
}

static bool IsoSelectConfig(uint8_t Configuration)
{
	UsbIsoIntrfClose(&s_Iso);
	s_IsoConfigured = false;
	s_IsoAlt = 0U;
	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != 1U)
	{
		return false;
	}
	s_IsoConfigured = true;
	return true;
}

static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_IsoConfigured || InterfaceNo != s_IsoInterfaceNo ||
		Alt > ISO_ALT_COUNT)
	{
		return false;
	}

	UsbIsoIntrfClose(&s_Iso);
	s_IsoAlt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	s_IsoLoopbackDropCnt = 0U;
	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
	if (!UsbIsoIntrfOpen(&s_Iso, s_IsoMps[Alt - 1U], interval))
	{
		return false;
	}
	s_IsoAlt = Alt;
	return true;
}

static void IsoReset(void)
{
	s_IsoConfigured = false;
	s_IsoAlt = 0U;
	UsbIsoIntrfReset(&s_Iso);
}

static void IsoProcess(void)
{
	if (!s_IsoConfigured || s_IsoAlt == 0U)
	{
		return;
	}
	const bool suspended = UsbSuspended(USB_DEVNO);
	if (suspended && !s_Iso.Suspended)
	{
		UsbIsoIntrfSuspend(&s_Iso);
	}
	else if (!suspended && s_Iso.Suspended)
	{
		(void)UsbIsoIntrfResume(&s_Iso);
	}
}

static constexpr UsbIntrfDesc_t s_IsoAlt0Desc = {
	.bLength = sizeof(UsbIntrfDesc_t),
	.bDescriptorType = USB_DESCTYPE_INTERFACE,
	.bInterfaceNumber = 0U,
	.bAlternateSetting = 0U,
	.bNumEndpoints = 0U,
	.bInterfaceClass = USB_INTRFCLASS_VENDOR,
	.bInterfaceSubClass = 0U,
	.bInterfaceProtocol = 0U,
	.iInterface = COMBO_STR_INTERFACE,
};

static constexpr ComboIsoAltDesc_t s_IsoAltDesc = {
	.Interface = {
		.bLength = sizeof(UsbIntrfDesc_t),
		.bDescriptorType = USB_DESCTYPE_INTERFACE,
		.bInterfaceNumber = 0U,
		.bAlternateSetting = 0U,
		.bNumEndpoints = 2U,
		.bInterfaceClass = USB_INTRFCLASS_VENDOR,
		.bInterfaceSubClass = 0U,
		.bInterfaceProtocol = 0U,
		.iInterface = COMBO_STR_INTERFACE,
	},
	.Out = {
		.bLength = sizeof(UsbEndPointDesc_t),
		.bDescriptorType = USB_DESCTYPE_ENDPOINT,
		.bEndpointAddress = 0U,
		.bmAttributes = USB_ENDPATT_TRANS_ISO,
		.wMaxPacketSize = 0U,
		.bInterval = 0U,
	},
	.In = {
		.bLength = sizeof(UsbEndPointDesc_t),
		.bDescriptorType = USB_DESCTYPE_ENDPOINT,
		.bEndpointAddress = 0U,
		.bmAttributes = USB_ENDPATT_TRANS_ISO,
		.wMaxPacketSize = 0U,
		.bInterval = 0U,
	},
};

static void IsoBuildFunctionDesc(const UsbDeviceClass *, uint8_t *pData,
	UsbSpeed_t Speed)
{
	ComboIsoFunctionDesc_t *pDesc =
		reinterpret_cast<ComboIsoFunctionDesc_t *>(pData);
	pDesc->Alt0 = s_IsoAlt0Desc;
	pDesc->Alt0.bInterfaceNumber = s_IsoInterfaceNo;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (unsigned i = 0U; i < ISO_ALT_COUNT; i++)
	{
		ComboIsoAltDesc_t &alt = pDesc->Alt[i];
		alt = s_IsoAltDesc;
		alt.Interface.bInterfaceNumber = s_IsoInterfaceNo;
		alt.Interface.bAlternateSetting = (uint8_t)(i + 1U);
		alt.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_IsoEpNo);
		alt.Out.wMaxPacketSize = s_IsoMps[i];
		alt.Out.bInterval = interval;
		alt.In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_IsoEpNo);
		alt.In.wMaxPacketSize = s_IsoMps[i];
		alt.In.bInterval = interval;
	}
}

class IsoLoopbackClass final : public UsbDeviceClass {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		return IsoControl(pSetup, Stage, ppData, pLength);
	}
	bool SelectConfig(uint8_t ConfigValue) override {
		return IsoSelectConfig(ConfigValue);
	}
	bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override {
		return IsoSelectInterface(InterfaceNo, Option);
	}
	void Reset(void) override { IsoReset(); }
	void Process(void) override { IsoProcess(); }
};

static IsoLoopbackClass s_IsoClass;

static bool IsoInit(void)
{
	if (!USB_ISO_SUPPORTED(USB_DEVNO))
	{
		return false;
	}

	const uint16_t isoMask = (uint16_t)(
		USB_ISO_EPIN_MASK(USB_DEVNO) & USB_ISO_EPOUT_MASK(USB_DEVNO));
	const uint8_t epNo = IsoFirstEndpoint(isoMask);
	if (epNo == 0U)
	{
		return false;
	}

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.FixedInMask = (uint16_t)(1U << epNo);
	req.FixedOutMask = (uint16_t)(1U << epNo);
	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IsoClass, &alloc))
	{
		return false;
	}

	s_IsoInterfaceNo = alloc.FirstInterface;
	s_IsoEpNo = epNo;

	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = USB_DEVNO;
	cfg.EpNo = s_IsoEpNo;
	cfg.RxHandler = IsoRxFrame;
	if (!UsbIsoIntrfInit(&s_Iso, &s_IsoData, &cfg))
	{
		return false;
	}

	return UsbDescRegister(USB_DEVNO, &s_IsoClass,
		nullptr, sizeof(ComboIsoFunctionDesc_t), IsoBuildFunctionDesc);
}

/* Device ------------------------------------------------------------------ */

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0008,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB Combo Stress",
	.pSerial = nullptr,
	.pFuncName = "USB Combo Stress",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = true,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

int main()
{
	uint8_t loopbackBuffer[CDC_BUFFER_SIZE];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0;
	int loopbackPending = 0;
	int loopbackOffset = 0;

	if (!UsbInit(&s_UsbCfg) ||
		!g_LoopbackCdc.Init(s_LoopbackCfg) ||
		!g_PrbsCdc.Init(s_PrbsCfg) ||
		!g_Hid.Init(s_HidCfg) ||
		!IntInit() ||
		!IsoInit())
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	bool hidSuspended = false;

	while (1)
	{
		UsbProcess(USB_DEVNO);

		const bool suspended = UsbSuspended(USB_DEVNO);
		if (suspended != hidSuspended)
		{
			hidSuspended = suspended;
			if (suspended)
			{
				g_Hid.Suspend();
			}
			else
			{
				(void)g_Hid.Resume();
			}
		}

		if (loopbackPending > 0)
		{
			const int length = g_LoopbackCdc.Tx(
				0, &loopbackBuffer[loopbackOffset], loopbackPending);
			if (length > 0)
			{
				loopbackOffset += length;
				loopbackPending -= length;
			}
		}
		else
		{
			const int length = g_LoopbackCdc.Rx(
				0, loopbackBuffer, sizeof(loopbackBuffer));
			if (length > 0)
			{
				if (atomic_exchange(&s_LoopbackSessionStart, false))
				{
					loopbackExpected = Prbs8(0xff);
				}
				for (int i = 0; i < length; i++)
				{
					if (loopbackBuffer[i] != loopbackExpected)
					{
						loopbackRxErrorNotify++;
					}
					loopbackExpected = Prbs8(loopbackBuffer[i]);
				}
				loopbackPending = length;
				loopbackOffset = 0;
			}
		}

		const uint8_t prbsByte = loopbackRxErrorNotify > 0U ? 0U : prbs;
		if (g_PrbsCdc.IsPortOpen() && g_PrbsCdc.Tx(0, &prbsByte, 1) > 0)
		{
			if (loopbackRxErrorNotify > 0U)
			{
				loopbackRxErrorNotify--;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
	}

	return 0;
}
