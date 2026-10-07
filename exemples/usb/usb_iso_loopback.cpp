/**-------------------------------------------------------------------------
@example	usb_iso_loopback.cpp

@brief	USB isochronous loopback for UsbIsoIntrf hardware validation.

This example deliberately contains no Bluetooth logic. It exposes one vendor
specific interface with alternate setting 0 disabled and alternate settings
1 through 6 using one controller-supported bidirectional isochronous endpoint.
The function selects its interface and endpoint internally from the USB core
allocator and controller ISO capability masks; the application does not assign
USB topology.

The host test is Python/usb_iso_loopback.py. ISO OUT DMA lands directly in the
RX FIFO slot reserved by UsbIntrf. Its DeviceIntrf event callback pulls each
completed frame with UsbIsoIntrf::Rx and queues the echo with UsbIsoIntrf::Tx.
Each direction uses the two-frame FIFO held by the UsbIsoIntrf object.

A vendor/interface IN request (bRequest 0x5A) returns loopback-only diagnostic
counters. It is intentionally outside UsbIsoIntrf so the reusable ISO layer does
not acquire test or class semantics.

@author	Hoang Nguyen Hoan
@date	Sep. 9, 2026

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
#include <stdint.h>
#include <string.h>

#include "app_evt_handler.h"
#include "usb/usb.h"
#include "usb/usb_iso.h"
#include "usb/usbd_epalloc.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define USB_DEVNO			0
#define ISO_CONFIG_VALUE	1U
#define ISO_ALT_COUNT		6U
#define ISO_MAX_MPS		63U
#define ISO_REQ_GET_DIAG	0x5AU

#define ISO_DIAG_FLAG_OPENED		(1U << 0)
#define ISO_DIAG_FLAG_SUSPENDED	(1U << 1)
#define ISO_DIAG_FLAG_TX_READY		(1U << 2)

#define ISO_STR_INTERFACE		4U

//#define ISO_TEST_RX_CLAMP		9

#pragma pack(push, 1)
typedef struct __Iso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} IsoAltDesc_t;

typedef struct __Iso_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	IsoAltDesc_t Alt[ISO_ALT_COUNT];
} IsoFunctionDesc_t;

typedef struct __Iso_Diag {
	uint32_t Reserved;
	uint32_t RxCnt;
	uint32_t TxSubmitCnt;
	uint32_t TxDoneCnt;
	uint32_t TxFailCnt;
	uint32_t LoopbackDropCnt;
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t LastRxLength;
	uint16_t LastTxLength;
	uint16_t Mps;
	uint8_t Alt;
	uint8_t Flags;
} IsoDiag_t;
#pragma pack(pop)

static_assert(sizeof(IsoDiag_t) == 48U, "ISO diagnostic wire format changed");

typedef struct __Iso_Function_State {
	bool Configured;
	uint8_t Alt;
	uint8_t InterfaceNo;
	uint8_t EpNo;
	uint32_t RxCnt;
	uint32_t TxSubmitCnt;
	uint32_t TxDoneCnt;
	uint32_t TxFailCnt;
	uint32_t LoopbackDropCnt;
	uint16_t LastRxLength;
	uint16_t LastTxLength;
} IsoFunctionState_t;

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength);
static bool IsoSelectConfig(uint8_t Configuration);
static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt);
static void IsoReset(void);
static void IsoProcess(void);

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

static constexpr uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

static constexpr UsbIntrfDesc_t s_IsoAlt0Desc = {
	.bLength = sizeof(UsbIntrfDesc_t),
	.bDescriptorType = USB_DESCTYPE_INTERFACE,
	.bInterfaceNumber = 0U,
	.bAlternateSetting = 0U,
	.bNumEndpoints = 0U,
	.bInterfaceClass = USB_INTRFCLASS_VENDOR,
	.bInterfaceSubClass = 0U,
	.bInterfaceProtocol = 0U,
	.iInterface = ISO_STR_INTERFACE,
};

static constexpr IsoAltDesc_t s_IsoAltDesc = {
	.Interface = {
		.bLength = sizeof(UsbIntrfDesc_t),
		.bDescriptorType = USB_DESCTYPE_INTERFACE,
		.bInterfaceNumber = 0U,
		.bAlternateSetting = 0U,
		.bNumEndpoints = 2U,
		.bInterfaceClass = USB_INTRFCLASS_VENDOR,
		.bInterfaceSubClass = 0U,
		.bInterfaceProtocol = 0U,
		.iInterface = ISO_STR_INTERFACE,
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

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0003,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB ISO Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB ISO Loopback",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
#endif
	.DeviceClass = USB_DEVCLASS_NONE,
	.DeviceSubClass = 0U,
	.DeviceProtocol = 0U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static UsbIsoIntrf s_Iso;

static IsoFunctionState_t s_Fn;
static IsoDiag_t s_DiagReply;
static IsoLoopbackClass s_IsoClass;

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

static void IsoClearDiag(void)
{
	s_Fn.RxCnt = 0U;
	s_Fn.TxSubmitCnt = 0U;
	s_Fn.TxDoneCnt = 0U;
	s_Fn.TxFailCnt = 0U;
	s_Fn.LoopbackDropCnt = 0U;
	s_Fn.LastRxLength = 0U;
	s_Fn.LastTxLength = 0U;

	UsbIsoIntrf_t *pIso = s_Iso;
	pIso->RxMissCnt = 0U;
	pIso->TxMissCnt = 0U;
	pIso->RxEmptyCnt = 0U;
	pIso->TxEmptyCnt = 0U;
}

static void IsoBuildDiag(void)
{
	const UsbIsoIntrf_t *pIso = s_Iso;

	memset(&s_DiagReply, 0, sizeof(s_DiagReply));
	s_DiagReply.RxCnt = s_Fn.RxCnt;
	s_DiagReply.TxSubmitCnt = s_Fn.TxSubmitCnt;
	s_DiagReply.TxDoneCnt = s_Fn.TxDoneCnt;
	s_DiagReply.TxFailCnt = s_Fn.TxFailCnt;
	s_DiagReply.LoopbackDropCnt = s_Fn.LoopbackDropCnt;
	s_DiagReply.RxMissCnt = pIso->RxMissCnt;
	s_DiagReply.TxMissCnt = pIso->TxMissCnt;
	s_DiagReply.RxEmptyCnt = pIso->RxEmptyCnt;
	s_DiagReply.TxEmptyCnt = pIso->TxEmptyCnt;
	s_DiagReply.LastRxLength = s_Fn.LastRxLength;
	s_DiagReply.LastTxLength = s_Fn.LastTxLength;
	s_DiagReply.Mps = pIso->Mps;
	s_DiagReply.Alt = s_Fn.Alt;

	if (pIso->Opened)
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_OPENED;
	}
	if (pIso->Suspended)
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_SUSPENDED;
	}
	if (s_Iso.TxReady())
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_TX_READY;
	}
}

// Use the same FIFO-backed DeviceIntrf event flow as the combo example.
static int IsoEvent(DevIntrf_t * const, DEVINTRF_EVT Event,
	uint8_t *, int Length)
{
	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
		{
			uint8_t frame[ISO_MAX_MPS];
			int total = 0;
			int len;
			while ((len = s_Iso.Rx(0, frame, sizeof(frame))) > 0)
			{
				s_Fn.RxCnt++;
				s_Fn.LastRxLength = (uint16_t)len;
				if (s_Iso.Tx(0, frame, len) == len)
				{
					s_Fn.TxSubmitCnt++;
				}
				else
				{
					s_Fn.LoopbackDropCnt++;
				}
				total += len;
			}
			return total;
		}

		case DEVINTRF_EVT_TX_READY:
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			s_Fn.LastTxLength = (uint16_t)Length;
			s_Fn.TxDoneCnt++;
			break;

		case DEVINTRF_EVT_TX_TIMEOUT:
			s_Fn.LastTxLength = (uint16_t)Length;
			s_Fn.TxFailCnt++;
			break;

		default:
			break;
	}
	return 0;
}

static bool IsoControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					   uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup == nullptr)
	{
		return false;
	}
	if (Stage != USB_CTRL_SETUP)
	{
		return true;
	}
	if (ppData == nullptr || pLength == nullptr ||
		pSetup->bmRequestType !=
			(USB_REQTYPE_DIRHOST | USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE) ||
		pSetup->bRequest != ISO_REQ_GET_DIAG || pSetup->wValue != 0U ||
		pSetup->wIndex != s_Fn.InterfaceNo || pSetup->wLength != sizeof(IsoDiag_t))
	{
		return false;
	}

	IsoBuildDiag();
	*ppData = reinterpret_cast<uint8_t *>(&s_DiagReply);
	*pLength = sizeof(s_DiagReply);
	return true;
}

static bool IsoSelectConfig(uint8_t Configuration)
{
	s_Iso.Close();
	s_Fn.Configured = false;
	s_Fn.Alt = 0U;

	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != ISO_CONFIG_VALUE)
	{
		return false;
	}

	s_Fn.Configured = true;
	return true;
}

static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_Fn.Configured || InterfaceNo != s_Fn.InterfaceNo || Alt > ISO_ALT_COUNT)
	{
		return false;
	}

	s_Iso.Close();
	s_Fn.Alt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	IsoClearDiag();
	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
#ifdef ISO_TEST_RX_CLAMP
	// Hardware test hook: open the endpoint smaller than the descriptor
	// advertises so a full size host frame is oversized at the controller.
	// A frame longer than the opened MPS must be dropped for that frame
	// only; reception resumes on the next fitting frame. Host side test:
	// Python/usb_iso_oversize_test.py. Never define for normal builds.
	const uint16_t mps = s_IsoMps[Alt - 1U] > (ISO_TEST_RX_CLAMP) ?
		(uint16_t)(ISO_TEST_RX_CLAMP) : s_IsoMps[Alt - 1U];
#else
	const uint16_t mps = s_IsoMps[Alt - 1U];
#endif
	if (!s_Iso.Open(mps, interval))
	{
		return false;
	}

	s_Fn.Alt = Alt;
	return true;
}

static void IsoReset(void)
{
	s_Fn.Configured = false;
	s_Fn.Alt = 0U;
	s_Iso.Reset();
	IsoClearDiag();
}

static void IsoProcess(void)
{
	if (!s_Fn.Configured || s_Fn.Alt == 0U)
	{
		return;
	}

	const UsbIsoIntrf_t *pIso = s_Iso;
	const bool suspended = UsbSuspended(USB_DEVNO);
	if (suspended && !pIso->Suspended)
	{
		s_Iso.Suspend();
	}
	else if (!suspended && pIso->Suspended)
	{
		(void)s_Iso.Resume();
	}
}

static void IsoPatchFunctionDesc(const UsbDeviceClass *, uint8_t *pData,
								 UsbSpeed_t Speed)
{
	IsoFunctionDesc_t *pDesc =
		reinterpret_cast<IsoFunctionDesc_t *>(pData);
	pDesc->Alt0 = s_IsoAlt0Desc;
	pDesc->Alt0.bInterfaceNumber = s_Fn.InterfaceNo;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (unsigned i = 0U; i < ISO_ALT_COUNT; i++)
	{
		IsoAltDesc_t &alt = pDesc->Alt[i];
		alt = s_IsoAltDesc;
		alt.Interface.bInterfaceNumber = s_Fn.InterfaceNo;
		alt.Interface.bAlternateSetting = (uint8_t)(i + 1U);
		alt.Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_Fn.EpNo);
		alt.Out.wMaxPacketSize = s_IsoMps[i];
		alt.Out.bInterval = interval;
		alt.In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_Fn.EpNo);
		alt.In.wMaxPacketSize = s_IsoMps[i];
		alt.In.bInterval = interval;
	}
}

static bool IsoRegisterFunction(void)
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

	const uint16_t epBit = (uint16_t)(1U << epNo);

	// The ISO endpoint is controller constrained. Reserve one supported
	// bidirectional endpoint while the allocator chooses the interface number.
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.FixedInMask = epBit;
	req.FixedOutMask = epBit;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_IsoClass, &alloc))
	{
		return false;
	}

	s_Fn.InterfaceNo = alloc.FirstInterface;
	s_Fn.EpNo = epNo;
	return UsbDescRegister(USB_DEVNO, &s_IsoClass,
		nullptr, sizeof(IsoFunctionDesc_t), IsoPatchFunctionDesc);
}

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !IsoRegisterFunction())
	{
		return -1;
	}

	// No FIFO memory given: the UsbIsoIntrf object uses its own.
	UsbIsoIntrfCfg_t isoCfg = {};
	isoCfg.DevNo = USB_DEVNO;
	isoCfg.EpNo = s_Fn.EpNo;
	isoCfg.EvtCB = IsoEvent;
	if (!s_Iso.Init(isoCfg))
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	while (1)
	{
		AppEvtHandlerExec();
	}

	return 0;
}

