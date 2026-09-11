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

#define INT_STR_INTERFACE	4U

static const uint8_t s_IntIntervals[INT_ALT_COUNT] = { 1U, 4U, 16U };

#pragma pack(push, 1)
typedef struct __Int_Alt_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} IntAltDesc_t;

typedef struct __Int_Function_Descriptor {
	UsbIntrfDesc_t Alt0;
	IntAltDesc_t Alt[INT_ALT_COUNT];
} IntFunctionDesc_t;

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

static IntFunctionDesc_t s_FsFunctionDesc;
static IntFunctionDesc_t s_HsFunctionDesc;
static IntDiag_t s_DiagReply;

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

static bool IntControl(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
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

static bool IntSelectConfig(uint8_t Configuration)
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

static bool IntSelectInterface(uint8_t InterfaceNo, uint8_t Alt)
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

static void IntReset(void)
{
	s_Configured = false;
	s_Alt = 0U;
	UsbIntIntrfReset(&s_Int);
	IntClearDiag();
}

static void IntProcess(void)
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

static bool IntBuildFunctionDesc(IntFunctionDesc_t *pDesc)
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
	pDesc->Alt0.iInterface = INT_STR_INTERFACE;

	for (uint8_t i = 0U; i < INT_ALT_COUNT; i++)
	{
		IntAltDesc_t *pAlt = &pDesc->Alt[i];
		pAlt->Interface = pDesc->Alt0;
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
	return true;
}

static bool IntRegisterFunction(void)
{
	class IntLoopbackClass final : public UsbDeviceClass {
	public:
		bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
					 uint8_t **ppData, uint16_t *pLength) override {
			return IntControl(pSetup, Stage, ppData, pLength);
		}
		bool SelectConfig(uint8_t ConfigValue) override {
			return IntSelectConfig(ConfigValue);
		}
		bool SelectInterface(uint8_t InterfaceNo, uint8_t Option) override {
			return IntSelectInterface(InterfaceNo, Option);
		}
		void Reset(void) override { IntReset(); }
		void Process(void) override { IntProcess(); }
	};
	static IntLoopbackClass s_Class;

	UsbdEpAllocReq_t req = {};
	req.InterfaceCount = 1U;
	req.BidirectionalCount = 1U;

	UsbdEpAllocRes_t alloc = {};
	if (!UsbdEpAlloc(USB_DEVNO, &req, &s_Class, &alloc))
	{
		return false;
	}

	s_InterfaceNo = alloc.FirstInterface;
	s_EpNo = alloc.Bidirectional[0];
	if (!IntBuildFunctionDesc(&s_FsFunctionDesc))
	{
		return false;
	}
	const void *pHsDesc = nullptr;
	uint16_t hsDescLength = 0U;
	if (USB_HIGHSPEED_CAPABLE(USB_DEVNO))
	{
		if (!IntBuildFunctionDesc(&s_HsFunctionDesc))
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
	.Pid = 0x0004,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB Interrupt Loopback",
	.pSerial = nullptr,
	.pFuncName = "USB Interrupt Loopback",
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
