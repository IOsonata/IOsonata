/**-------------------------------------------------------------------------
@example	usb_int_loopback.cpp

@brief	USB interrupt loopback for UsbIntIntrf hardware validation.

This example exposes one vendor-specific interface with alternate setting 0
disabled and alternate settings 1 through 3 selecting different interrupt
endpoint intervals. The USB core allocator chooses the interface and endpoint
pair. Every completed OUT packet is echoed through the independent DIRECT TX
slot; no CFifo or HID behavior is involved.

The host test is Python/usb_int_loopback.py. A vendor/interface IN request
(bRequest 0x5B) returns loopback diagnostics. The request is deliberately part
of this test application, not UsbIntIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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
#include "usb/usb_int.h"
#include "usb/usbd_epalloc.h"

#define USB_DEVNO			0
#define INT_CONFIG_VALUE	1U
#define INT_ALT_COUNT		3U
#define INT_MPS				64U
#define INT_REQ_GET_DIAG	0x5BU

#define INT_DIAG_FLAG_OPENED		(1U << 0)
#define INT_DIAG_FLAG_SUSPENDED		(1U << 1)
#define INT_DIAG_FLAG_TX_READY		(1U << 2)

#define INT_STR_MANUFACTURER	1U
#define INT_STR_PRODUCT		2U
#define INT_STR_SERIAL		3U
#define INT_STR_INTERFACE	4U
#define INT_STR_MAXLEN		40U

static const uint8_t s_IntIntervals[INT_ALT_COUNT] = { 1U, 4U, 16U };

#pragma pack(push, 1)
typedef struct __Int_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} IntAltDesc_t;

typedef struct __Int_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbIntrfDesc_t Alt0;
	IntAltDesc_t Alt[INT_ALT_COUNT];
} IntConfigDesc_t;

typedef struct __Int_Diag {
	uint32_t RxCnt;
	uint32_t TxSubmitCnt;
	uint32_t TxDoneCnt;
	uint32_t TxFailCnt;
	uint32_t LoopbackDropCnt;
	uint32_t CoreRxDropCnt;
	uint32_t RxErrorCnt;
	uint32_t TxErrorCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t LastRxLength;
	uint16_t LastTxLength;
	uint16_t Mps;
	uint8_t Interval;
	uint8_t Alt;
	uint8_t Flags;
	uint8_t Reserved;
} IntDiag_t;
#pragma pack(pop)

static_assert(sizeof(IntDiag_t) == 50U,
	"interrupt diagnostic wire format changed");

static UsbIntIntrf_t s_Int;
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

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static IntConfigDesc_t s_ConfigDesc;
static IntDiag_t s_DiagReply;
static uint8_t s_StringDesc[2U + (INT_STR_MAXLEN * 2U)];

static void IntClearDiag(void)
{
	s_RxCnt = 0U;
	s_TxSubmitCnt = 0U;
	s_TxDoneCnt = 0U;
	s_TxFailCnt = 0U;
	s_LoopbackDropCnt = 0U;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_Int.IntrfData.RxDropCnt = 0U;
	s_Int.RxErrorCnt = 0U;
	s_Int.TxErrorCnt = 0U;
	s_Int.RxEmptyCnt = 0U;
	s_Int.TxEmptyCnt = 0U;
}

static void IntBuildDiag(void)
{
	memset(&s_DiagReply, 0, sizeof(s_DiagReply));
	s_DiagReply.RxCnt = s_RxCnt;
	s_DiagReply.TxSubmitCnt = s_TxSubmitCnt;
	s_DiagReply.TxDoneCnt = s_TxDoneCnt;
	s_DiagReply.TxFailCnt = s_TxFailCnt;
	s_DiagReply.LoopbackDropCnt = s_LoopbackDropCnt;
	s_DiagReply.CoreRxDropCnt = s_Int.IntrfData.RxDropCnt;
	s_DiagReply.RxErrorCnt = s_Int.RxErrorCnt;
	s_DiagReply.TxErrorCnt = s_Int.TxErrorCnt;
	s_DiagReply.RxEmptyCnt = s_Int.RxEmptyCnt;
	s_DiagReply.TxEmptyCnt = s_Int.TxEmptyCnt;
	s_DiagReply.LastRxLength = s_LastRxLength;
	s_DiagReply.LastTxLength = s_LastTxLength;
	s_DiagReply.Mps = s_Int.Mps;
	s_DiagReply.Interval = s_Int.Interval;
	s_DiagReply.Alt = s_Alt;

	if (s_Int.Opened)
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_OPENED;
	}
	if (s_Int.Suspended)
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_SUSPENDED;
	}
	if (UsbIntIntrfTxReady(&s_Int))
	{
		s_DiagReply.Flags |= INT_DIAG_FLAG_TX_READY;
	}
}

static void IntRxPacket(UsbIntIntrf_t *, const uint8_t *pData,
						uint16_t Length, UsbCtrlrXferResult_t Result, void *)
{
	if (Result != USB_CTRLR_XFER_SUCCESS)
	{
		return;
	}

	s_RxCnt++;
	s_LastRxLength = Length;
	if (UsbIntIntrfSendPacket(&s_Int, pData, Length))
	{
		s_TxSubmitCnt++;
	}
	else
	{
		s_LoopbackDropCnt++;
	}
}

static void IntTxPacket(UsbIntIntrf_t *, uint16_t Length,
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

static bool IntRequest(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
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
		pSetup->bRequest != INT_REQ_GET_DIAG || pSetup->wValue != 0U ||
		pSetup->wIndex != s_InterfaceNo || pSetup->wLength != sizeof(IntDiag_t))
	{
		return false;
	}

	IntBuildDiag();
	*ppData = reinterpret_cast<uint8_t *>(&s_DiagReply);
	*pLength = sizeof(s_DiagReply);
	return true;
}

static bool IntConfig(uint8_t Configuration, void *)
{
	UsbIntIntrfClose(&s_Int);
	s_Configured = false;
	s_Alt = 0U;

	if (Configuration == 0U)
	{
		return true;
	}
	if (Configuration != INT_CONFIG_VALUE)
	{
		return false;
	}

	s_Configured = true;
	return true;
}

static bool IntSetInterface(uint8_t InterfaceNo, uint8_t Alt, void *)
{
	if (!s_Configured || InterfaceNo != s_InterfaceNo || Alt > INT_ALT_COUNT)
	{
		return false;
	}

	UsbIntIntrfClose(&s_Int);
	s_Alt = 0U;
	if (Alt == 0U)
	{
		return true;
	}

	IntClearDiag();
	if (!UsbIntIntrfOpen(&s_Int, INT_MPS, s_IntIntervals[Alt - 1U]))
	{
		return false;
	}

	s_Alt = Alt;
	return true;
}

static void IntReset(void *)
{
	s_Configured = false;
	s_Alt = 0U;
	UsbIntIntrfReset(&s_Int);
	IntClearDiag();
}

static void IntProcess(void *)
{
	if (!s_Configured || s_Alt == 0U)
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

static bool IntRegisterFunction(void)
{
	UsbdClassCfg_t coreCfg = {};
	coreCfg.RequestHandler = IntRequest;
	coreCfg.ConfigHandler = IntConfig;
	coreCfg.SetInterfaceHandler = IntSetInterface;
	coreCfg.ResetHandler = IntReset;
	coreCfg.ProcessHandler = IntProcess;

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &coreCfg, &alloc))
	{
		return false;
	}

	s_InterfaceNo = alloc.FirstInterface;
	s_EpNo = alloc.Bidirectional[0];
	return s_EpNo != 0U;
}

static uint8_t IntMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}
	uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *IntDeviceDescriptor(uint16_t *pLength)
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
		INT_STR_MANUFACTURER : 0U;
	s_DeviceDesc.iProduct = pCfg->pProduct != nullptr ? INT_STR_PRODUCT : 0U;
	s_DeviceDesc.iSerialNumber = UsbGetSerial(USB_DEVNO) != nullptr ?
		INT_STR_SERIAL : 0U;
	s_DeviceDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

static const uint8_t *IntQualifierDescriptor(uint16_t *pLength)
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

static const uint8_t *IntConfigurationDescriptor(UsbSpeed_t,
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
	s_ConfigDesc.Config.bConfigurationValue = INT_CONFIG_VALUE;
	s_ConfigDesc.Config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		s_ConfigDesc.Config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	s_ConfigDesc.Config.bMaxPower = IntMaxPower(pCfg);

	s_ConfigDesc.Alt0.bLength = sizeof(s_ConfigDesc.Alt0);
	s_ConfigDesc.Alt0.bDescriptorType = USB_DESCTYPE_INTERFACE;
	s_ConfigDesc.Alt0.bInterfaceNumber = s_InterfaceNo;
	s_ConfigDesc.Alt0.bAlternateSetting = 0U;
	s_ConfigDesc.Alt0.bNumEndpoints = 0U;
	s_ConfigDesc.Alt0.bInterfaceClass = USB_INTRFCLASS_VENDOR;
	s_ConfigDesc.Alt0.iInterface = INT_STR_INTERFACE;

	for (uint8_t i = 0U; i < INT_ALT_COUNT; i++)
	{
		IntAltDesc_t *pAlt = &s_ConfigDesc.Alt[i];
		pAlt->Interface = s_ConfigDesc.Alt0;
		pAlt->Interface.bAlternateSetting = (uint8_t)(i + 1U);
		pAlt->Interface.bNumEndpoints = 2U;

		pAlt->Out.bLength = sizeof(pAlt->Out);
		pAlt->Out.bDescriptorType = USB_DESCTYPE_ENDPOINT;
		pAlt->Out.bEndpointAddress = USB_ENDPADDR_DIROUT(s_EpNo);
		pAlt->Out.bmAttributes = USB_ENDPATT_TRANS_INT;
		pAlt->Out.wMaxPacketSize = INT_MPS;
		pAlt->Out.bInterval = s_IntIntervals[i];

		pAlt->In = pAlt->Out;
		pAlt->In.bEndpointAddress = USB_ENDPADDR_DIRIN(s_EpNo);
	}

	*pLength = sizeof(s_ConfigDesc);
	return reinterpret_cast<const uint8_t *>(&s_ConfigDesc);
}

static const uint8_t *IntStringDescriptor(uint8_t Index, uint16_t LangId,
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
	if (Index > INT_STR_INTERFACE ||
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
		case INT_STR_MANUFACTURER:
			pStr = pCfg->pManufacturer;
			break;
		case INT_STR_PRODUCT:
			pStr = pCfg->pProduct;
			break;
		case INT_STR_SERIAL:
			pStr = UsbGetSerial(USB_DEVNO);
			break;
		case INT_STR_INTERFACE:
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
	if (length > INT_STR_MAXLEN)
	{
		length = INT_STR_MAXLEN;
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

static const uint8_t *IntDescHandler(uint8_t DescType, uint8_t DescIndex,
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
			return DescIndex == 0U ? IntDeviceDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ?
				IntConfigurationDescriptor(Speed, false, pLength) : nullptr;
		case USB_DESCTYPE_STRING:
			return IntStringDescriptor(DescIndex, LangId, pLength);
		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? IntQualifierDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_OSC:
			if (DescIndex != 0U || !USB_HIGHSPEED_CAPABLE(USB_DEVNO))
			{
				return nullptr;
			}
			return IntConfigurationDescriptor(
				Speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH,
				true, pLength);
		default:
			return nullptr;
	}
}

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0004,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB Interrupt Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB Interrupt Loopback",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
	.DescHandler = IntDescHandler,
	.pDescContext = nullptr,
};

int main()
{
	if (!UsbInit(&s_UsbCfg) || !IntRegisterFunction())
	{
		return -1;
	}

	UsbIntIntrfCfg_t intCfg = {};
	intCfg.DevNo = USB_DEVNO;
	intCfg.EpNo = s_EpNo;
	intCfg.RxHandler = IntRxPacket;
	intCfg.TxHandler = IntTxPacket;
	if (!UsbIntIntrfInit(&s_Int, &intCfg))
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
