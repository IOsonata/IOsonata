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
buffer registered by UsbIntrf. UsbIsoIntrf publishes the completed frame to the
callback, and the callback echoes it through the single ISO TX slot. There is
no CFifo and no obsolete function-level endpoint completion callback.

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

#include "usb/usb.h"
#include "usb/usb_iso.h"
#include "usb/usbd_epalloc.h"

#define USB_DEVNO			0
#define ISO_CONFIG_VALUE	1U
#define ISO_ALT_COUNT		6U
#define ISO_REQ_GET_DIAG	0x5AU

#define ISO_DIAG_FLAG_OPENED		(1U << 0)
#define ISO_DIAG_FLAG_SUSPENDED	(1U << 1)
#define ISO_DIAG_FLAG_TX_READY		(1U << 2)

#define ISO_STR_INTERFACE		4U

static const uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

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

static UsbIsoIntrf_t s_Iso;
static bool s_Configured;
static uint8_t s_Alt;
static uint8_t s_InterfaceNo;
static uint8_t s_EpNo;
static uint32_t s_RxCnt;
static uint32_t s_TxSubmitCnt;
static uint32_t s_TxDoneCnt;
static uint32_t s_TxFailCnt;
static uint32_t s_LoopbackDropCnt;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;

static IsoFunctionDesc_t s_FsFunctionDesc;
static IsoFunctionDesc_t s_HsFunctionDesc;
static IsoDiag_t s_DiagReply;

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
	s_RxCnt = 0U;
	s_TxSubmitCnt = 0U;
	s_TxDoneCnt = 0U;
	s_TxFailCnt = 0U;
	s_LoopbackDropCnt = 0U;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_Iso.RxMissCnt = 0U;
	s_Iso.TxMissCnt = 0U;
	s_Iso.RxEmptyCnt = 0U;
	s_Iso.TxEmptyCnt = 0U;
}

static void IsoBuildDiag(void)
{
	memset(&s_DiagReply, 0, sizeof(s_DiagReply));
	s_DiagReply.RxCnt = s_RxCnt;
	s_DiagReply.TxSubmitCnt = s_TxSubmitCnt;
	s_DiagReply.TxDoneCnt = s_TxDoneCnt;
	s_DiagReply.TxFailCnt = s_TxFailCnt;
	s_DiagReply.LoopbackDropCnt = s_LoopbackDropCnt;
	s_DiagReply.RxMissCnt = s_Iso.RxMissCnt;
	s_DiagReply.TxMissCnt = s_Iso.TxMissCnt;
	s_DiagReply.RxEmptyCnt = s_Iso.RxEmptyCnt;
	s_DiagReply.TxEmptyCnt = s_Iso.TxEmptyCnt;
	s_DiagReply.LastRxLength = s_LastRxLength;
	s_DiagReply.LastTxLength = s_LastTxLength;
	s_DiagReply.Mps = s_Iso.Mps;
	s_DiagReply.Alt = s_Alt;

	if (s_Iso.Opened)
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_OPENED;
	}
	if (s_Iso.Suspended)
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_SUSPENDED;
	}
	if (UsbIsoIntrfTxReady(&s_Iso))
	{
		s_DiagReply.Flags |= ISO_DIAG_FLAG_TX_READY;
	}
}

static void IsoRxFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
					   uint16_t Length, UsbCtrlrXferResult_t Result,
					   void *pContext)
{
	(void)pIntrf;
	(void)pContext;

	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		return;
	}

	s_RxCnt++;
	s_LastRxLength = Length;

	// The callback owns the current RX frame only for this call. SendFrame
	// copies it into the independent ISO TX slot before returning.
	if (UsbIsoIntrfSendFrame(&s_Iso, pData, Length))
	{
		s_TxSubmitCnt++;
	}
	else
	{
		// ISO is deadline driven. A busy TX slot means this service
		// opportunity is missed; never queue stale data for a later frame.
		s_LoopbackDropCnt++;
	}
}

static void IsoTxFrame(UsbIsoIntrf_t *, uint16_t Length,
					   UsbCtrlrXferResult_t Result, void *)
{
	s_LastTxLength = Length;
	if (Result == USB_CTRLR_XFER_SUCCESS)
	{
		s_TxDoneCnt++;
	}
	else
	{
		s_TxFailCnt++;
	}
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
		pSetup->wIndex != s_InterfaceNo || pSetup->wLength != sizeof(IsoDiag_t))
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
	UsbIsoIntrfClose(&s_Iso);
	s_Configured = false;
	s_Alt = 0U;

	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != ISO_CONFIG_VALUE)
	{
		return false;
	}

	s_Configured = true;
	return true;
}

static bool IsoSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
{
	if (!s_Configured || InterfaceNo != s_InterfaceNo || Alt > ISO_ALT_COUNT)
	{
		return false;
	}

	UsbIsoIntrfClose(&s_Iso);
	s_Alt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	IsoClearDiag();
	const uint8_t interval = UsbCtrlrHighSpeed(USB_DEVNO) ? 4U : 1U;
	if (!UsbIsoIntrfOpen(&s_Iso, s_IsoMps[Alt - 1U], interval))
	{
		return false;
	}

	s_Alt = Alt;
	return true;
}

static void IsoReset(void)
{
	s_Configured = false;
	s_Alt = 0U;
	UsbIsoIntrfReset(&s_Iso);
	IsoClearDiag();
}

static void IsoProcess(void)
{
	if (!s_Configured || s_Alt == 0U)
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

static bool IsoBuildFunctionDesc(IsoFunctionDesc_t *pDesc, UsbSpeed_t Speed)
{
	if (pDesc == nullptr || s_EpNo == 0U)
	{
		return false;
	}

	memset(pDesc, 0, sizeof(*pDesc));
	pDesc->Alt0.bLength = sizeof(pDesc->Alt0);
	pDesc->Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	pDesc->Alt0.bInterfaceNumber = s_InterfaceNo;
	pDesc->Alt0.bAlternateSetting = 0U;
	pDesc->Alt0.bNumEndpoints = 0U;
	pDesc->Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	pDesc->Alt0.iInterface = ISO_STR_INTERFACE;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (uint8_t i = 0U; i < ISO_ALT_COUNT; i++)
	{
		IsoAltDesc_t *pAlt = &pDesc->Alt[i];
		pAlt->Interface = pDesc->Alt0;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;
		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_EpNo);
		pAlt->Out.bmAttributes = USB_ENDPATT_TRANS_ISO;
		pAlt->Out.wMaxPacketSize = s_IsoMps[i];
		pAlt->Out.bInterval = interval;
		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_EpNo);
	}
	return true;
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
	static IsoLoopbackClass s_Class;

	// The ISO endpoint is controller constrained. Reserve one supported
	// bidirectional endpoint while the allocator chooses the interface number.
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.FixedInMask = epBit;
	req.FixedOutMask = epBit;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_Class, &alloc))
	{
		return false;
	}

	s_InterfaceNo = alloc.FirstInterface;
	s_EpNo = epNo;
	if (!IsoBuildFunctionDesc(&s_FsFunctionDesc, USB_SPEED_FULL))
	{
		return false;
	}
	const void *pHsDesc = nullptr;
	uint16_t hsDescLength = 0U;
	if (USB_HIGHSPEED_CAPABLE(USB_DEVNO))
	{
		if (!IsoBuildFunctionDesc(&s_HsFunctionDesc, USB_SPEED_HIGH))
		{
			return false;
		}
		pHsDesc = &s_HsFunctionDesc;
		hsDescLength = sizeof(s_HsFunctionDesc);
	}
	return UsbDescriptorRegister(USB_DEVNO, &s_Class,
		&s_FsFunctionDesc, sizeof(s_FsFunctionDesc), pHsDesc, hsDescLength);
}

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
	.DeviceClass = USB_DEVCLASS_NONE,
	.DeviceSubClass = 0U,
	.DeviceProtocol = 0U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

int main()
{
	if (!UsbInit(&s_UsbCfg) || !IsoRegisterFunction())
	{
		return -1;
	}

	UsbIsoIntrfCfg_t isoCfg = {};
	isoCfg.DevNo = USB_DEVNO;
	isoCfg.EpNo = s_EpNo;
	isoCfg.RxHandler = IsoRxFrame;
	isoCfg.TxHandler = IsoTxFrame;
	if (!UsbIsoIntrfInit(&s_Iso, &isoCfg))
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	while (1)
	{
		UsbProcess(USB_DEVNO);
	}

	return 0;
}
