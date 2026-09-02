/**-------------------------------------------------------------------------
@file	usbd_core_test.cpp

@brief	Host regression tests for the native IOsonata USB device core.

Links the production usbd_core.cpp against a fake controller so Chapter 9,
EP0 sequencing and USB function dispatch can be exercised without hardware.

@author	Hoang Nguyen Hoan
@date	Sep. 2, 2026

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
#include <stdio.h>
#include <string.h>

#include "usb/usbd_core.h"

#define EP0_OUT		0x00U
#define EP0_IN		0x80U
#define EP1_OUT		0x01U
#define EP1_IN		0x81U
#define EP2_IN		0x82U

#define STD_DEV_OUT	0x00U
#define STD_IF_OUT		0x01U
#define STD_EP_OUT		0x02U
#define STD_DEV_IN		0x80U
#define STD_IF_IN		0x81U
#define STD_EP_IN		0x82U
#define CLASS_IF_OUT	0x21U
#define CLASS_EP_OUT	0x22U

#define CLASS_NO_DATA	0x40U
#define CLASS_OUT_DATA	0x41U
#define XFER_LOG_CNT	32
#define STAGE_LOG_CNT	16

#define CHECK(_Expr) \
	do { \
		if (!(_Expr)) { \
			printf("FAIL %s:%d: %s\n", __func__, __LINE__, #_Expr); \
			return false; \
		} \
	} while (0)

typedef struct {
	uint8_t EpAddr;
	uint8_t *pBuffer;
	uint16_t Length;
	uint8_t Data[4];
} XferLog_t;

typedef struct {
	UsbdCtrlrEvtHandler_t Handler;
	void *pContext;
	XferLog_t Xfer[XFER_LOG_CNT];
	int XferCnt;
	int IntEnableCnt;
	int IntDisableCnt;
	int ConnectCnt;
	int DisconnectCnt;
	int RemoteWakeCnt;
	int SofEnableCnt;
	int SofDisableCnt;
	int SetAddressCnt;
	int CloseAllCnt;
	int StallCnt;
	int ClearStallCnt;
	uint8_t LastAddress;
	uint8_t LastStallEp;
	uint8_t LastClearStallEp;
} CtrlrState_t;

typedef struct {
	UsbdCoreCtrlStage_t Stage[STAGE_LOG_CNT];
	uint16_t StageLen[STAGE_LOG_CNT];
	int StageCnt;
	int ConfigCnt;
	uint8_t LastConfig;
	int SetIfCnt;
	uint8_t LastAlt;
	int XferCnt;
	uint8_t LastXferEp;
	uint16_t LastXferLen;
	UsbdCtrlrXferResult_t LastXferResult;
	int ResetCnt;
	int SofCnt;
	uint16_t LastFrame;
	uint8_t CtrlBuffer[16];
} FuncState_t;

typedef struct {
	uint8_t Config[64];
	uint16_t ConfigLen;
	uint8_t String64[64];
} DescState_t;

static CtrlrState_t s_Ctrlr;
static FuncState_t s_Func;
static DescState_t s_Desc;

static const uint8_t s_DeviceDesc[] = {
	18, USB_DESCTYPE_DEVICE, 0x00, 0x02, 0, 0, 0, 64,
	0x34, 0x12, 0x78, 0x56, 0x00, 0x01, 1, 2, 3, 1
};

// One interface: alternate 0 owns EP1 OUT/IN, alternate 1 owns EP2 IN.
static const uint8_t s_ConfigDesc[] = {
	9, USB_DESCTYPE_CONFIGURATION, 48, 0, 1, 1, 0,
	(uint8_t)(USB_CONFATT_RESERVED | USB_CONFATT_SELF_POWERED |
		USB_CONFATT_REMOTE_WAKEUP), 50,
	9, USB_DESCTYPE_INTERFACE, 0, 0, 2, USB_INTRFCLASS_VENDOR, 0, 0, 0,
	7, USB_DESCTYPE_ENDPOINT, EP1_OUT, 2, 64, 0, 0,
	7, USB_DESCTYPE_ENDPOINT, EP1_IN, 2, 64, 0, 0,
	9, USB_DESCTYPE_INTERFACE, 0, 1, 1, USB_INTRFCLASS_VENDOR, 0, 0, 0,
	7, USB_DESCTYPE_ENDPOINT, EP2_IN, 2, 64, 0, 0
};

static const XferLog_t *LastXfer(void)
{
	return s_Ctrlr.XferCnt > 0 ? &s_Ctrlr.Xfer[s_Ctrlr.XferCnt - 1] : nullptr;
}

static void ClearCtrlrLog(void)
{
	UsbdCtrlrEvtHandler_t handler = s_Ctrlr.Handler;
	void *pContext = s_Ctrlr.pContext;
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.Handler = handler;
	s_Ctrlr.pContext = pContext;
}

static void Setup(uint8_t Type, uint8_t Request, uint16_t Value,
				  uint16_t Index, uint16_t Length)
{
	UsbdCtrlrEvt_t evt = {};
	evt.Type = USBD_CTRLR_EVT_SETUP;
	evt.Setup.bmRequestType = Type;
	evt.Setup.bRequest = Request;
	evt.Setup.wValue = Value;
	evt.Setup.wIndex = Index;
	evt.Setup.wLength = Length;
	s_Ctrlr.Handler(&evt, s_Ctrlr.pContext);
}

static void Complete(uint8_t EpAddr, uint16_t Length,
					 UsbdCtrlrXferResult_t Result = USBD_CTRLR_XFER_SUCCESS)
{
	UsbdCtrlrEvt_t evt = {};
	evt.Type = USBD_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = Result;
	s_Ctrlr.Handler(&evt, s_Ctrlr.pContext);
}

static void Event(UsbdCtrlrEvtType_t Type)
{
	UsbdCtrlrEvt_t evt = {};
	evt.Type = Type;
	s_Ctrlr.Handler(&evt, s_Ctrlr.pContext);
}

static const uint8_t *Descriptor(uint8_t Type, uint8_t Index, uint16_t,
								 UsbdSpeed_t, uint16_t *pLength, void *)
{
	if (Type == USB_DESCTYPE_DEVICE && Index == 0)
	{
		*pLength = sizeof(s_DeviceDesc);
		return s_DeviceDesc;
	}
	if (Type == USB_DESCTYPE_CONFIGURATION && Index == 0)
	{
		*pLength = s_Desc.ConfigLen;
		return s_Desc.Config;
	}
	if (Type == USB_DESCTYPE_STRING && Index == 1)
	{
		*pLength = sizeof(s_Desc.String64);
		return s_Desc.String64;
	}
	*pLength = 0;
	return nullptr;
}

static bool Request(const UsbSetupData_t *pSetup, UsbdCoreCtrlStage_t Stage,
					uint8_t **ppData, uint16_t *pLength, void *)
{
	if (s_Func.StageCnt < STAGE_LOG_CNT)
	{
		s_Func.Stage[s_Func.StageCnt] = Stage;
		s_Func.StageLen[s_Func.StageCnt] = pLength != nullptr ? *pLength : 0;
		s_Func.StageCnt++;
	}
	if (Stage != USBD_CORE_CTRL_SETUP)
	{
		return true;
	}
	if (pSetup->bRequest == CLASS_NO_DATA && pSetup->wLength == 0)
	{
		*ppData = nullptr;
		*pLength = 0;
		return true;
	}
	if (pSetup->bRequest == CLASS_OUT_DATA &&
		pSetup->wLength <= sizeof(s_Func.CtrlBuffer))
	{
		*ppData = s_Func.CtrlBuffer;
		*pLength = sizeof(s_Func.CtrlBuffer);
		return true;
	}
	return false;
}

static bool Configure(uint8_t Value, void *)
{
	s_Func.ConfigCnt++;
	s_Func.LastConfig = Value;
	return true;
}

static bool SetInterface(uint8_t, uint8_t Alternate, void *)
{
	s_Func.SetIfCnt++;
	s_Func.LastAlt = Alternate;
	return true;
}

static void FunctionXfer(uint8_t EpAddr, uint16_t Length,
						 UsbdCtrlrXferResult_t Result, void *)
{
	s_Func.XferCnt++;
	s_Func.LastXferEp = EpAddr;
	s_Func.LastXferLen = Length;
	s_Func.LastXferResult = Result;
}

static void FunctionReset(void *)
{
	s_Func.ResetCnt++;
}

static void FunctionSof(uint16_t FrameNo, void *)
{
	s_Func.SofCnt++;
	s_Func.LastFrame = FrameNo;
}

static bool Fixture(bool WithSetInterface = true)
{
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	memset(&s_Func, 0, sizeof(s_Func));
	memset(&s_Desc, 0, sizeof(s_Desc));
	memcpy(s_Desc.Config, s_ConfigDesc, sizeof(s_ConfigDesc));
	s_Desc.ConfigLen = sizeof(s_Desc.Config);
	s_Desc.String64[0] = sizeof(s_Desc.String64);
	s_Desc.String64[1] = USB_DESCTYPE_STRING;

	UsbdCoreCfg_t core = {};
	core.DescHandler = Descriptor;
	core.Speed = USBD_SPEED_FULL;
	core.Ep0Mps = 64;
	if (!UsbdCoreInit(&core))
	{
		return false;
	}

	UsbdCoreFuncCfg_t func = {};
	func.FirstInterface = 0;
	func.InterfaceCount = 1;
	func.EpInMask = (1U << 1) | (1U << 2);
	func.EpOutMask = (1U << 1);
	func.RequestHandler = Request;
	func.ConfigHandler = Configure;
	func.SetInterfaceHandler = WithSetInterface ? SetInterface : nullptr;
	func.XferHandler = FunctionXfer;
	func.ResetHandler = FunctionReset;
	func.SofHandler = FunctionSof;
	if (!UsbdCoreRegisterFunction(&func))
	{
		return false;
	}
	UsbdCoreStart();
	return s_Ctrlr.Handler != nullptr && s_Ctrlr.IntEnableCnt == 1 &&
		s_Ctrlr.ConnectCnt == 1;
}

static bool SetAddress(uint8_t Address)
{
	int base = s_Ctrlr.XferCnt;
	Setup(STD_DEV_OUT, USB_REQ_SET_ADDRESS, Address, 0, 0);
	if (s_Ctrlr.XferCnt != base + 1 || LastXfer()->EpAddr != EP0_IN ||
		LastXfer()->Length != 0 || UsbdCoreAddress() == Address)
	{
		return false;
	}
	Complete(EP0_IN, 0);
	return UsbdCoreAddress() == Address;
}

static bool SetConfig(uint8_t Value)
{
	int base = s_Ctrlr.XferCnt;
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, Value, 0, 0);
	if (s_Ctrlr.XferCnt != base + 1 || LastXfer()->EpAddr != EP0_IN ||
		LastXfer()->Length != 0 || UsbdCoreConfiguration() != Value)
	{
		return false;
	}
	Complete(EP0_IN, 0);
	return true;
}

static bool TestDescriptors(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_DEVICE << 8), 0, 8);
	CHECK(s_Ctrlr.XferCnt == 1 && LastXfer()->EpAddr == EP0_IN);
	CHECK(LastXfer()->Length == 8 && LastXfer()->Data[0] == 18);
	Complete(EP0_IN, 8);
	CHECK(LastXfer()->EpAddr == EP0_OUT && LastXfer()->Length == 0);
	Complete(EP0_OUT, 0);

	int base = s_Ctrlr.XferCnt;
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_CONFIGURATION << 8), 0, 255);
	CHECK(s_Ctrlr.XferCnt == base + 1);
	CHECK(LastXfer()->Length == sizeof(s_ConfigDesc));

	int stalls = s_Ctrlr.StallCnt;
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_DEVICE << 8), 1, 18);
	CHECK(s_Ctrlr.StallCnt == stalls + 1);
	return true;
}

static bool TestDescriptorValidation(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	s_Desc.ConfigLen = sizeof(s_ConfigDesc) - 1;
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_CONFIGURATION << 8), 0, 255);
	CHECK(s_Ctrlr.StallCnt == 1 && s_Ctrlr.XferCnt == 0);

	memcpy(s_Desc.Config, s_ConfigDesc, sizeof(s_ConfigDesc));
	s_Desc.ConfigLen = sizeof(s_Desc.Config);
	s_Desc.Config[9] = 1;
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_CONFIGURATION << 8), 0, 255);
	CHECK(s_Ctrlr.StallCnt == 2 && s_Ctrlr.XferCnt == 0);

	memcpy(s_Desc.Config, s_ConfigDesc, sizeof(s_ConfigDesc));
	s_Desc.Config[2] = 10;
	s_Desc.Config[3] = 0;
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_CONFIGURATION << 8), 0, 255);
	CHECK(s_Ctrlr.StallCnt == 3 && s_Ctrlr.XferCnt == 0);
	return true;
}

static bool TestControlZlp(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)((USB_DESCTYPE_STRING << 8) | 1), 0x0409, 65);
	CHECK(s_Ctrlr.XferCnt == 1 && LastXfer()->Length == 64);
	Complete(EP0_IN, 64);
	CHECK(s_Ctrlr.XferCnt == 2 && LastXfer()->EpAddr == EP0_IN &&
		LastXfer()->Length == 0);
	Complete(EP0_IN, 0);
	CHECK(s_Ctrlr.XferCnt == 3 && LastXfer()->EpAddr == EP0_OUT &&
		LastXfer()->Length == 0);
	Complete(EP0_OUT, 0);
	return true;
}

static bool TestAddress(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	CHECK(SetAddress(7));
	CHECK(s_Ctrlr.SetAddressCnt == 1 && s_Ctrlr.LastAddress == 7);

	Setup(STD_DEV_OUT, USB_REQ_SET_ADDRESS, 9, 0, 0);
	CHECK(UsbdCoreAddress() == 7);
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_DEVICE << 8), 0, 8);
	Complete(EP0_IN, 8);
	Complete(EP0_OUT, 0);
	CHECK(UsbdCoreAddress() == 7);

	Setup(STD_DEV_OUT, USB_REQ_SET_ADDRESS, 11, 0, 0);
	Event(USBD_CTRLR_EVT_RESET);
	CHECK(UsbdCoreAddress() == 0);
	int calls = s_Ctrlr.SetAddressCnt;
	int stalls = s_Ctrlr.StallCnt;
	Setup(STD_DEV_OUT, USB_REQ_SET_ADDRESS, 128, 0, 0);
	CHECK(s_Ctrlr.SetAddressCnt == calls && s_Ctrlr.StallCnt == stalls + 1);
	return true;
}

static bool TestConfiguration(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, 1, 0, 0);
	CHECK(s_Ctrlr.StallCnt == 1 && UsbdCoreConfiguration() == 0);
	CHECK(SetAddress(5));
	CHECK(SetConfig(1));
	CHECK(UsbdCoreConfigured() && s_Func.LastConfig == 1);
	CHECK(s_Ctrlr.SofEnableCnt == 1);

	Setup(STD_DEV_IN, USB_REQ_GET_CONFIGURATION, 0, 0, 1);
	CHECK(LastXfer()->Length == 1 && LastXfer()->Data[0] == 1);
	Complete(EP0_IN, 1);
	Complete(EP0_OUT, 0);

	CHECK(SetConfig(0));
	CHECK(!UsbdCoreConfigured() && s_Func.LastConfig == 0);
	int stalls = s_Ctrlr.StallCnt;
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, 2, 0, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1 && UsbdCoreConfiguration() == 0);
	return true;
}

static bool TestInterfaceAndHalt(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	CHECK(SetAddress(3) && SetConfig(1));

	Setup(STD_IF_IN, USB_REQ_GET_INTERFACE, 0, 0, 1);
	CHECK(LastXfer()->Length == 1 && LastXfer()->Data[0] == 0);
	Complete(EP0_IN, 1);
	Complete(EP0_OUT, 0);

	int stalls = s_Ctrlr.StallCnt;
	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP2_IN, 2);
	CHECK(s_Ctrlr.StallCnt == stalls + 1);

	Setup(STD_EP_OUT, USB_REQ_SET_FEATURE,
		  USB_FEATSEL_ENDPOINT_HALT, EP1_IN, 0);
	CHECK(s_Ctrlr.LastStallEp == EP1_IN);
	Complete(EP0_IN, 0);
	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP1_IN, 2);
	CHECK((LastXfer()->Data[0] & USB_ENDPSTATUS_HALT) != 0);
	Complete(EP0_IN, 2);
	Complete(EP0_OUT, 0);

	Setup(STD_IF_OUT, USB_REQ_SET_INTERFACE, 1, 0, 0);
	CHECK(s_Func.SetIfCnt == 1 && s_Func.LastAlt == 1);
	CHECK(UsbdCoreAlternate(0) == 1);
	Complete(EP0_IN, 0);

	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP2_IN, 2);
	CHECK(LastXfer()->Length == 2 && LastXfer()->Data[0] == 0);
	Complete(EP0_IN, 2);
	Complete(EP0_OUT, 0);

	stalls = s_Ctrlr.StallCnt;
	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP1_IN, 2);
	CHECK(s_Ctrlr.StallCnt == stalls + 1);

	Setup(STD_IF_OUT, USB_REQ_SET_INTERFACE, 0, 0, 0);
	CHECK(s_Func.SetIfCnt == 2 && s_Func.LastAlt == 0);
	CHECK(UsbdCoreAlternate(0) == 0);
	Complete(EP0_IN, 0);

	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP1_IN, 2);
	CHECK(LastXfer()->Length == 2 && LastXfer()->Data[0] == 0);
	Complete(EP0_IN, 2);
	Complete(EP0_OUT, 0);

	Setup(STD_EP_OUT, USB_REQ_SET_FEATURE,
		  USB_FEATSEL_ENDPOINT_HALT, EP1_IN, 0);
	CHECK(s_Ctrlr.LastStallEp == EP1_IN);
	Complete(EP0_IN, 0);
	Setup(STD_IF_OUT, USB_REQ_SET_INTERFACE, 0, 0, 0);
	CHECK(s_Func.SetIfCnt == 3 && UsbdCoreAlternate(0) == 0);
	Complete(EP0_IN, 0);

	Setup(STD_EP_IN, USB_REQ_GET_STATUS, 0, EP1_IN, 2);
	CHECK(LastXfer()->Length == 2 && LastXfer()->Data[0] == 0);
	Complete(EP0_IN, 2);
	Complete(EP0_OUT, 0);
	return true;
}

static bool TestInterfaceRequiresHandler(void)
{
	CHECK(Fixture(false));
	ClearCtrlrLog();
	CHECK(SetAddress(6) && SetConfig(1));

	const int stalls = s_Ctrlr.StallCnt;
	Setup(STD_IF_OUT, USB_REQ_SET_INTERFACE, 0, 0, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1);
	CHECK(s_Func.SetIfCnt == 0 && UsbdCoreAlternate(0) == 0);
	return true;
}

static bool TestFunctionControl(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	CHECK(SetAddress(2));

	s_Func.StageCnt = 0;
	int stalls = s_Ctrlr.StallCnt;
	Setup(CLASS_IF_OUT, CLASS_NO_DATA, 0, 0, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1 && s_Func.StageCnt == 0);

	stalls = s_Ctrlr.StallCnt;
	Setup(CLASS_EP_OUT, CLASS_NO_DATA, 0, EP1_OUT, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1 && s_Func.StageCnt == 0);

	CHECK(SetConfig(1));
	s_Func.StageCnt = 0;

	Setup(CLASS_IF_OUT, CLASS_NO_DATA, 0, 0, 0);
	CHECK(s_Func.StageCnt == 1 && s_Func.Stage[0] == USBD_CORE_CTRL_SETUP);
	Complete(EP0_IN, 0);
	CHECK(s_Func.StageCnt == 2 &&
		s_Func.Stage[1] == USBD_CORE_CTRL_COMPLETE);

	s_Func.StageCnt = 0;
	Setup(CLASS_EP_OUT, CLASS_NO_DATA, 0, EP1_OUT, 0);
	CHECK(s_Func.StageCnt == 1 && s_Func.Stage[0] == USBD_CORE_CTRL_SETUP);
	Complete(EP0_IN, 0);
	CHECK(s_Func.StageCnt == 2 &&
		s_Func.Stage[1] == USBD_CORE_CTRL_COMPLETE);

	s_Func.StageCnt = 0;
	stalls = s_Ctrlr.StallCnt;
	Setup(CLASS_EP_OUT, CLASS_NO_DATA, 0, EP2_IN, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1 && s_Func.StageCnt == 0);

	// Keep the function registered for interface 0, but remove interface 0
	// from the active descriptor. Dispatch must follow the descriptor.
	s_Desc.Config[11] = 1;
	s_Desc.Config[34] = 1;
	s_Func.StageCnt = 0;
	stalls = s_Ctrlr.StallCnt;
	Setup(CLASS_IF_OUT, CLASS_NO_DATA, 0, 0, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1 && s_Func.StageCnt == 0);
	s_Desc.Config[11] = 0;
	s_Desc.Config[34] = 0;

	s_Func.StageCnt = 0;
	Setup(CLASS_IF_OUT, CLASS_OUT_DATA, 0, 0, 4);
	CHECK(LastXfer()->EpAddr == EP0_OUT && LastXfer()->Length == 4);
	Complete(EP0_OUT, 4);
	CHECK(s_Func.StageCnt == 2 && s_Func.Stage[1] == USBD_CORE_CTRL_DATA &&
		s_Func.StageLen[1] == 4);
	Complete(EP0_IN, 0);
	CHECK(s_Func.StageCnt == 3 &&
		s_Func.Stage[2] == USBD_CORE_CTRL_COMPLETE && s_Func.StageLen[2] == 4);

	s_Func.StageCnt = 0;
	Setup(CLASS_IF_OUT, CLASS_NO_DATA, 0, 0, 0);
	Setup(STD_DEV_IN, USB_REQ_GET_DESCRIPTOR,
		  (uint16_t)(USB_DESCTYPE_DEVICE << 8), 0, 8);
	CHECK(s_Func.StageCnt == 2 && s_Func.Stage[1] == USBD_CORE_CTRL_ABORT);
	Complete(EP0_IN, 8);
	Complete(EP0_OUT, 0);

	s_Func.StageCnt = 0;
	Setup(CLASS_IF_OUT, CLASS_NO_DATA, 0, 0, 0);
	UsbdCoreStop();
	CHECK(s_Func.StageCnt == 2 && s_Func.Stage[1] == USBD_CORE_CTRL_ABORT);
	CHECK(s_Ctrlr.DisconnectCnt == 1 && s_Ctrlr.IntDisableCnt == 1);
	return true;
}

static bool TestResetSuspendAndDispatch(void)
{
	CHECK(Fixture());
	ClearCtrlrLog();
	CHECK(SetAddress(4) && SetConfig(1));
	Setup(STD_DEV_OUT, USB_REQ_SET_FEATURE,
		  USB_FEATSEL_DEVICE_REMOTE_WAKEUP, 0, 0);
	CHECK(UsbdCoreRemoteWakeupEnabled());
	Complete(EP0_IN, 0);

	Event(USBD_CTRLR_EVT_SUSPEND);
	CHECK(UsbdCoreSuspended() && UsbdCoreRemoteWakeup());
	CHECK(s_Ctrlr.RemoteWakeCnt == 1);
	Event(USBD_CTRLR_EVT_RESUME);
	CHECK(!UsbdCoreSuspended() && !UsbdCoreRemoteWakeup());

	UsbdCtrlrEvt_t sof = {};
	sof.Type = USBD_CTRLR_EVT_SOF;
	sof.FrameNo = 1234;
	s_Ctrlr.Handler(&sof, s_Ctrlr.pContext);
	CHECK(s_Func.SofCnt == 1 && s_Func.LastFrame == 1234);
	Complete(EP1_IN, 37);
	CHECK(s_Func.XferCnt == 1 && s_Func.LastXferEp == EP1_IN &&
		s_Func.LastXferLen == 37);
	Complete(EP1_OUT, 12, USBD_CTRLR_XFER_CANCELLED);
	CHECK(s_Func.XferCnt == 2 &&
		s_Func.LastXferResult == USBD_CTRLR_XFER_CANCELLED);

	Event(USBD_CTRLR_EVT_RESET);
	CHECK(UsbdCoreAddress() == 0 && UsbdCoreConfiguration() == 0);
	CHECK(!UsbdCoreRemoteWakeupEnabled() && UsbdCoreAlternate(0) == 0);
	CHECK(s_Func.ResetCnt == 1);
	return true;
}

extern "C" bool UsbdCtrlrInit(UsbdCtrlrEvtHandler_t Handler, void *pContext)
{
	s_Ctrlr.Handler = Handler;
	s_Ctrlr.pContext = pContext;
	return Handler != nullptr;
}
extern "C" bool UsbdCtrlrStart(void) { return true; }
extern "C" void UsbdCtrlrStop(void) {}
extern "C" bool UsbdCtrlrHighSpeed(void) { return false; }
extern "C" void UsbdCtrlrIntEnable(void) { s_Ctrlr.IntEnableCnt++; }
extern "C" void UsbdCtrlrIntDisable(void) { s_Ctrlr.IntDisableCnt++; }
extern "C" void UsbdCtrlrConnect(void) { s_Ctrlr.ConnectCnt++; }
extern "C" void UsbdCtrlrDisconnect(void) { s_Ctrlr.DisconnectCnt++; }
extern "C" void UsbdCtrlrRemoteWakeup(void) { s_Ctrlr.RemoteWakeCnt++; }
extern "C" void UsbdCtrlrSofEnable(bool Enable)
{
	Enable ? s_Ctrlr.SofEnableCnt++ : s_Ctrlr.SofDisableCnt++;
}
extern "C" void UsbdCtrlrSetAddress(uint8_t Address)
{
	s_Ctrlr.SetAddressCnt++;
	s_Ctrlr.LastAddress = Address;
}
extern "C" bool UsbdCtrlrEpOpen(const UsbEndPointDesc_t *) { return true; }
extern "C" void UsbdCtrlrEpClose(uint8_t) {}
extern "C" void UsbdCtrlrEpCloseAll(void) { s_Ctrlr.CloseAllCnt++; }
extern "C" bool UsbdCtrlrEpXfer(uint8_t EpAddr, uint8_t *pBuffer,
								 uint16_t Length)
{
	if (s_Ctrlr.XferCnt >= XFER_LOG_CNT)
	{
		return false;
	}
	XferLog_t *p = &s_Ctrlr.Xfer[s_Ctrlr.XferCnt++];
	memset(p, 0, sizeof(*p));
	p->EpAddr = EpAddr;
	p->pBuffer = pBuffer;
	p->Length = Length;
	if (pBuffer != nullptr && Length != 0)
	{
		uint16_t n = Length < sizeof(p->Data) ? Length : sizeof(p->Data);
		memcpy(p->Data, pBuffer, n);
	}
	return true;
}
extern "C" bool UsbdCtrlrEpBusy(uint8_t) { return false; }
extern "C" void UsbdCtrlrEpStall(uint8_t EpAddr)
{
	s_Ctrlr.StallCnt++;
	s_Ctrlr.LastStallEp = EpAddr;
}
extern "C" void UsbdCtrlrEpClearStall(uint8_t EpAddr)
{
	s_Ctrlr.ClearStallCnt++;
	s_Ctrlr.LastClearStallEp = EpAddr;
}

typedef bool (*TestHandler_t)(void);
typedef struct { const char *pName; TestHandler_t Handler; } TestCase_t;

int main(void)
{
	static const TestCase_t tests[] = {
		{ "descriptors", TestDescriptors },
		{ "descriptor validation", TestDescriptorValidation },
		{ "control terminating ZLP", TestControlZlp },
		{ "SET_ADDRESS", TestAddress },
		{ "configuration", TestConfiguration },
		{ "alternate interface and halt", TestInterfaceAndHalt },
		{ "SET_INTERFACE requires handler", TestInterfaceRequiresHandler },
		{ "function control lifecycle", TestFunctionControl },
		{ "reset suspend and dispatch", TestResetSuspendAndDispatch },
	};

	for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
	{
		printf("RUN  %s\n", tests[i].pName);
		if (!tests[i].Handler())
		{
			printf("Result: FAIL (%u/%zu passed)\n", i,
				   sizeof(tests) / sizeof(tests[0]));
			return 1;
		}
		printf("PASS %s\n", tests[i].pName);
	}
	printf("Result: PASS (%zu/%zu)\n", sizeof(tests) / sizeof(tests[0]),
		   sizeof(tests) / sizeof(tests[0]));
	return 0;
}
