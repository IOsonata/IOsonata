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

#define ISO_STR_MANUFACTURER	1U
#define ISO_STR_PRODUCT		2U
#define ISO_STR_SERIAL			3U
#define ISO_STR_INTERFACE		4U
#define ISO_STR_MAXLEN			40U

static const uint16_t s_IsoMps[ISO_ALT_COUNT] = {
	9U, 17U, 25U, 33U, 49U, 63U,
};

#pragma pack(push, 1)
typedef struct __Iso_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} IsoAltDesc_t;

typedef struct __Iso_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbIntrfDesc_t Alt0;
	IsoAltDesc_t Alt[ISO_ALT_COUNT];
} IsoConfigDesc_t;

typedef struct __Iso_Diag {
	uint32_t SofCnt;
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
static uint32_t s_SofCnt;
static uint32_t s_RxCnt;
static uint32_t s_TxSubmitCnt;
static uint32_t s_TxDoneCnt;
static uint32_t s_TxFailCnt;
static uint32_t s_LoopbackDropCnt;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static IsoConfigDesc_t s_ConfigDesc;
static IsoDiag_t s_DiagReply;
static uint8_t s_StringDesc[2U + (ISO_STR_MAXLEN * 2U)];

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
	s_SofCnt = 0U;
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
	s_DiagReply.SofCnt = s_SofCnt;
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

static bool IsoRequest(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					   uint8_t **ppData, uint16_t *pLength, void *)
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

static bool IsoConfig(uint8_t Configuration, void *)
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

static bool IsoSetInterface(uint8_t InterfaceNo, uint8_t Alt, void *)
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

static void IsoReset(void *)
{
	s_Configured = false;
	s_Alt = 0U;
	UsbIsoIntrfReset(&s_Iso);
	IsoClearDiag();
}

static void IsoSof(uint16_t, void *)
{
	if (s_Configured && s_Alt != 0U)
	{
		s_SofCnt++;
	}
}

static void IsoProcess(void *)
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

	UsbdClassCfg_t coreCfg = {};
	coreCfg.RequestHandler = IsoRequest;
	coreCfg.ConfigHandler = IsoConfig;
	coreCfg.SetInterfaceHandler = IsoSetInterface;
	coreCfg.ResetHandler = IsoReset;
	coreCfg.SofHandler = IsoSof;
	coreCfg.ProcessHandler = IsoProcess;

	// The ISO endpoint is controller constrained. Reserve one supported
	// bidirectional endpoint while the allocator chooses the interface number.
	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.FixedInMask = epBit;
	req.FixedOutMask = epBit;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &coreCfg, &alloc))
	{
		return false;
	}

	s_InterfaceNo = alloc.FirstInterface;
	s_EpNo = epNo;
	return true;
}

static uint8_t IsoMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}
	uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *IsoDeviceDescriptor(uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}

	memset(&s_DeviceDesc, 0, sizeof(s_DeviceDesc));
	s_DeviceDesc.bLength = sizeof(s_DeviceDesc);
	s_DeviceDesc.bDescriptorType = USB_DESCTYPE_DEVICE;
	s_DeviceDesc.bcdUSB = 0x0200U;
	s_DeviceDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_DeviceDesc.bMaxPacketSize = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_DeviceDesc.idVendor = pCfg->Vid;
	s_DeviceDesc.idProduct = pCfg->Pid;
	s_DeviceDesc.bcdDevice = pCfg->DevVer;
	s_DeviceDesc.iManufacturer = pCfg->pManufacturer != nullptr ?
		ISO_STR_MANUFACTURER : 0U;
	s_DeviceDesc.iProduct = pCfg->pProduct != nullptr ? ISO_STR_PRODUCT : 0U;
	s_DeviceDesc.iSerialNumber = UsbGetSerial(USB_DEVNO) != nullptr ?
		ISO_STR_SERIAL : 0U;
	s_DeviceDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

static const uint8_t *IsoQualifierDescriptor(uint16_t *pLength)
{
	if (pLength == nullptr || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
	{
		return nullptr;
	}

	memset(&s_QualifierDesc, 0, sizeof(s_QualifierDesc));
	s_QualifierDesc.bLength = sizeof(s_QualifierDesc);
	s_QualifierDesc.bDescriptorType = USB_DESCTYPE_DEVICE_QUALIFIER;
	s_QualifierDesc.bcdUSB = 0x0200U;
	s_QualifierDesc.bDeviceClass = USB_DEVCLASS_NONE;
	s_QualifierDesc.bMaxPacketSize0 = USB_PKT_MAXLEN(USB_DEVNO, CONTROL);
	s_QualifierDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_QualifierDesc);
	return reinterpret_cast<const uint8_t *>(&s_QualifierDesc);
}

static const uint8_t *IsoConfigurationDescriptor(UsbSpeed_t Speed,
											 bool OtherSpeed,
											 uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr || s_EpNo == 0U)
	{
		return nullptr;
	}

	memset(&s_ConfigDesc, 0, sizeof(s_ConfigDesc));
	s_ConfigDesc.Config.bLength = sizeof(s_ConfigDesc.Config);
	s_ConfigDesc.Config.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	s_ConfigDesc.Config.wTotalLength = sizeof(s_ConfigDesc);
	s_ConfigDesc.Config.bNumInterfaces = 1U;
	s_ConfigDesc.Config.bConfigurationValue = ISO_CONFIG_VALUE;
	s_ConfigDesc.Config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		s_ConfigDesc.Config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	s_ConfigDesc.Config.bMaxPower = IsoMaxPower(pCfg);

	s_ConfigDesc.Alt0.bLength = sizeof(s_ConfigDesc.Alt0);
	s_ConfigDesc.Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	s_ConfigDesc.Alt0.bInterfaceNumber = s_InterfaceNo;
	s_ConfigDesc.Alt0.bAlternateSetting = 0U;
	s_ConfigDesc.Alt0.bNumEndpoints = 0U;
	s_ConfigDesc.Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	s_ConfigDesc.Alt0.iInterface = ISO_STR_INTERFACE;

	const uint8_t interval = Speed == USB_SPEED_HIGH ? 4U : 1U;
	for (uint8_t i = 0U; i < ISO_ALT_COUNT; i++)
	{
		IsoAltDesc_t *pAlt = &s_ConfigDesc.Alt[i];
		pAlt->Interface = s_ConfigDesc.Alt0;
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

	*pLength = sizeof(s_ConfigDesc);
	return reinterpret_cast<const uint8_t *>(&s_ConfigDesc);
}

static const uint8_t *IsoStringDescriptor(uint8_t Index, uint16_t LangId,
										  uint16_t *pLength)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}
	if (Index == 0U)
	{
		s_StringDesc[0] = 4U;
		s_StringDesc[1] = USB_DESCTYPE_STRING;
		s_StringDesc[2] = 0x09U;
		s_StringDesc[3] = 0x04U;
		*pLength = 4U;
		return s_StringDesc;
	}
	if (Index > ISO_STR_INTERFACE ||
		(LangId != 0U && LangId != 0x0409U))
	{
		return nullptr;
	}

	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr)
	{
		return nullptr;
	}

	const char *pStr = nullptr;
	switch (Index)
	{
		case ISO_STR_MANUFACTURER:
			pStr = pCfg->pManufacturer;
			break;
		case ISO_STR_PRODUCT:
			pStr = pCfg->pProduct;
			break;
		case ISO_STR_SERIAL:
			pStr = UsbGetSerial(USB_DEVNO);
			break;
		case ISO_STR_INTERFACE:
			pStr = pCfg->pFuncName;
			break;
		default:
			return nullptr;
	}
	if (pStr == nullptr)
	{
		return nullptr;
	}

	size_t length = strlen(pStr);
	if (length > ISO_STR_MAXLEN)
	{
		length = ISO_STR_MAXLEN;
	}
	s_StringDesc[0] = (uint8_t)(2U + (length * 2U));
	s_StringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0U; i < length; i++)
	{
		s_StringDesc[2U + (i * 2U)] = (uint8_t)pStr[i];
		s_StringDesc[3U + (i * 2U)] = 0U;
	}
	*pLength = s_StringDesc[0];
	return s_StringDesc;
}

static const uint8_t *IsoDescHandler(uint8_t DescType, uint8_t DescIndex,
									 uint16_t LangId, UsbSpeed_t Speed,
									 uint16_t *pLength, void *)
{
	if (pLength == nullptr)
	{
		return nullptr;
	}
	*pLength = 0U;

	switch (DescType)
	{
		case USB_DESCTYPE_DEVICE:
			return DescIndex == 0U ? IsoDeviceDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ?
				IsoConfigurationDescriptor(Speed, false, pLength) : nullptr;
		case USB_DESCTYPE_STRING:
			return IsoStringDescriptor(DescIndex, LangId, pLength);
		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? IsoQualifierDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_OSC:
			if (DescIndex != 0U || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
			{
				return nullptr;
			}
			return IsoConfigurationDescriptor(
				Speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH,
				true, pLength);
		default:
			return nullptr;
	}
}

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0003,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB ISO Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB ISO Loopback",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
	.DescHandler = IsoDescHandler,
	.pDescContext = nullptr,
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
