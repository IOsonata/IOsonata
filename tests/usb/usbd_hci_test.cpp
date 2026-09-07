/**-------------------------------------------------------------------------
@file	usbd_hci_test.cpp

@brief	Host tests for the Bluetooth HCI USB function.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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

#include "usb/usbd_hci.h"

static UsbCfg_t s_UsbCfg;
static UsbFuncCfg_t s_FuncCfg;
static bool s_FuncRegistered;
static uint8_t s_ReservedFirst;
static uint8_t s_ReservedCount;
static uint16_t s_ReservedIn;
static uint16_t s_ReservedOut;
static UsbEndPointDesc_t s_OpenDesc[3];
static int s_OpenCount;
static int s_CloseCount;
static int s_RequestCount;
static void *s_RequestContext;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbRegisterFunc(int DevNo, const UsbFuncCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr)
	{
		return false;
	}

	if (pCfg->InterfaceCount != 0U && s_ReservedCount != 0U)
	{
		const uint16_t firstA = pCfg->FirstInterface;
		const uint16_t lastA = firstA + pCfg->InterfaceCount;
		const uint16_t firstB = s_ReservedFirst;
		const uint16_t lastB = firstB + s_ReservedCount;

		if (firstA < lastB && firstB < lastA)
		{
			return false;
		}
	}

	if ((pCfg->EpInMask & s_ReservedIn) != 0U ||
		(pCfg->EpOutMask & s_ReservedOut) != 0U)
	{
		return false;
	}

	s_FuncCfg = *pCfg;
	s_FuncRegistered = true;
	return true;
}

bool UsbCtrlrHighSpeed(int) { return false; }

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || s_OpenCount >= 3)
	{
		return false;
	}
	s_OpenDesc[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t) { s_CloseCount++; }
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static bool HciRequest(const UsbSetupData_t *, UsbCtrlStage_t,
					   uint8_t **, uint16_t *, void *pContext)
{
	s_RequestContext = pContext;
	s_RequestCount++;
	return true;
}

static UsbdHciCfg_t MakeCfg(void)
{
	UsbdHciCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.InterfaceString = 4U;
	cfg.RequestHandler = HciRequest;
	cfg.pRequestContext = &s_RequestContext;
	return cfg;
}

static void ResetFake(void)
{
	memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
	memset(&s_FuncCfg, 0, sizeof(s_FuncCfg));
	memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
	s_FuncRegistered = false;
	s_ReservedFirst = 0U;
	s_ReservedCount = 0U;
	s_ReservedIn = 0U;
	s_ReservedOut = 0U;
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_RequestCount = 0;
	s_RequestContext = nullptr;
	s_UsbCfg.DevNo = 0;
}

static void TestDescriptor(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	UsbdHciDesc_t desc = {};

	CHECK(hci.Init(cfg));
	CHECK(hci.MakeDesc(&desc, USB_SPEED_FULL));
	CHECK(sizeof(desc) == sizeof(UsbInrtfAssDesc_t) +
		2U * sizeof(UsbIntrfDesc_t) + 3U * sizeof(UsbEndPointDesc_t));
	CHECK(desc.Association.bDescriptorType == USB_DESCTYPE_IA);
	CHECK(desc.Association.bFirstInterface == 0U);
	CHECK(desc.Association.bInterfaceCount == 2U);
	CHECK(desc.Association.bFunctionClass == USB_INTRFCLASS_WIRELESS);
	CHECK(desc.Association.bFunctionSubClass == USBD_HCI_SUBCLASS_RF);
	CHECK(desc.Association.bFunctionProtocol == USBD_HCI_PROTOCOL_BT);
	CHECK(desc.Hci.bInterfaceNumber == 0U);
	CHECK(desc.Hci.bNumEndpoints == 3U);
	CHECK(desc.Hci.bInterfaceClass == USB_INTRFCLASS_WIRELESS);
	CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
	CHECK(desc.EventIn.bmAttributes == USB_ENDPATT_TRANS_INT);
	CHECK(desc.EventIn.wMaxPacketSize == USBD_HCI_EVENT_FS_MPS);
	CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
	CHECK(desc.AclIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(desc.AclOut.bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(desc.AclOut.wMaxPacketSize == USBD_HCI_ACL_FS_MPS);
	CHECK(desc.Sync.bInterfaceNumber == 1U);
	CHECK(desc.Sync.bNumEndpoints == 0U);
}

static void TestAutoPlacement(void)
{
	ResetFake();
	s_ReservedFirst = 0U;
	s_ReservedCount = 2U;
	s_ReservedIn = (uint16_t)(1U << 1);
	s_ReservedOut = (uint16_t)(1U << 1);

	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	UsbdHciDesc_t desc = {};

	CHECK(hci.Init(cfg));
	CHECK(s_FuncRegistered);
	CHECK(s_FuncCfg.FirstInterface == 2U);
	CHECK(s_FuncCfg.InterfaceCount == 2U);
	CHECK(s_FuncCfg.EpInMask == ((1U << 2) | (1U << 3)));
	CHECK(s_FuncCfg.EpOutMask == (1U << 3));
	CHECK(hci.MakeDesc(&desc, USB_SPEED_FULL));
	CHECK(desc.Association.bFirstInterface == 2U);
	CHECK(desc.Hci.bInterfaceNumber == 2U);
	CHECK(desc.Sync.bInterfaceNumber == 3U);
	CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(3U));
	CHECK(desc.AclIn.bEndpointAddress == USB_ENDPADDR_DIRIN(3U));
}

static void TestConfiguration(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();

	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler != nullptr);
	CHECK(s_FuncCfg.ConfigHandler(USBD_HCI_CONFIG_VALUE, s_FuncCfg.pContext));
	CHECK(s_OpenCount == 3);
	CHECK(s_OpenDesc[0].bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
	CHECK(s_OpenDesc[0].bmAttributes == USB_ENDPATT_TRANS_INT);
	CHECK(s_OpenDesc[1].bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
	CHECK(s_OpenDesc[1].bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(s_OpenDesc[2].bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(s_OpenDesc[2].bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(s_FuncCfg.ConfigHandler(0U, s_FuncCfg.pContext));
}

static void TestControlRouting(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();

	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.RequestHandler != nullptr);
	CHECK(s_FuncCfg.SetInterfaceHandler != nullptr);

	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
	setup.wIndex = 0U;
	uint16_t length = 0U;
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, nullptr, &length,
								   s_FuncCfg.pContext));
	CHECK(s_RequestCount == 1);
	CHECK(s_RequestContext == &s_RequestContext);

	setup.wIndex = 1U;
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, nullptr, &length,
									s_FuncCfg.pContext));
	CHECK(s_RequestCount == 1);

	setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_DEVICE;
	setup.wIndex = 0U;
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, nullptr, &length,
								   s_FuncCfg.pContext));
	CHECK(s_RequestCount == 2);

	CHECK(s_FuncCfg.SetInterfaceHandler(0U, 0U, s_FuncCfg.pContext));
	CHECK(s_FuncCfg.SetInterfaceHandler(1U, 0U, s_FuncCfg.pContext));
	CHECK(!s_FuncCfg.SetInterfaceHandler(0U, 1U, s_FuncCfg.pContext));
	CHECK(!s_FuncCfg.SetInterfaceHandler(2U, 0U, s_FuncCfg.pContext));
}

int main(void)
{
	TestDescriptor();
	TestAutoPlacement();
	TestConfiguration();
	TestControlRouting();

	if (s_Fail != 0)
	{
		printf("%d failed\n", s_Fail);
		return 1;
	}

	printf("all pass\n");
	return 0;
}
