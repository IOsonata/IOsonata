/**-------------------------------------------------------------------------
@example	usb_hid_loopback.cpp

@brief	USB HID loopback example

This example creates a vendor-page HID function with one Interrupt OUT/IN
endpoint pair. UsbdHid allocates the interface and endpoint numbers and fills
the HID descriptor fragment. The application supplies the report descriptor
and returns each output report as an input report.

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
#include "usb/usbd_hid.h"

#define USB_DEVNO			0
#define HID_STR_INTERFACE	4U
#define HID_REPORT_SIZE		64U

static const uint8_t s_ReportDesc[] = {
	0x06U, 0x00U, 0xFFU,		// Usage Page (Vendor 0xFF00)
	0x09U, 0x01U,				// Usage 1
	0xA1U, 0x01U,				// Collection (Application)
	0x75U, 0x08U,				// Report Size 8
	0x95U, HID_REPORT_SIZE,		// Report Count 64
	0x09U, 0x01U,
	0x81U, 0x02U,				// Input (Data, Variable, Absolute)
	0x95U, HID_REPORT_SIZE,
	0x09U, 0x01U,
	0x91U, 0x02U,				// Output (Data, Variable, Absolute)
	0xC0U,
};

static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength);

class HidLoopback final : public UsbdHid {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		if (pSetup != nullptr &&
			(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) ==
				USB_REQTYPE_CLASS &&
			(pSetup->bRequest == USB_HID_REQ_GET_REPORT ||
			 pSetup->bRequest == USB_HID_REQ_SET_REPORT))
		{
			return HidReportRequest(pSetup, Stage, ppData, pLength);
		}
		return UsbdHid::Control(pSetup, Stage, ppData, pLength);
	}
};

static HidLoopback g_Hid;
static uint8_t s_Pending[HID_REPORT_SIZE];
static uint16_t s_PendingLength;
static uint8_t s_ControlReport[HID_REPORT_SIZE];

static void HidRx(UsbdHidDev_t *, const uint8_t *pData, uint16_t Length,
				  UsbCtrlrXferResult_t Result, void *)
{
	if (Result != USB_CTRLR_XFER_SUCCESS || Length > sizeof(s_Pending))
	{
		return;
	}
	if (!g_Hid.SendReport(pData, Length))
	{
		memcpy(s_Pending, pData, Length);
		s_PendingLength = Length;
	}
}

static void HidTx(UsbdHidDev_t *, uint16_t, UsbCtrlrXferResult_t Result,
				  void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS && s_PendingLength != 0U)
	{
		const uint16_t length = s_PendingLength;
		s_PendingLength = 0U;
		if (!g_Hid.SendReport(s_Pending, length))
		{
			s_PendingLength = length;
		}
	}
}

static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength)
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
		if (ppData == nullptr || pSetup->wLength > sizeof(s_ControlReport))
		{
			return false;
		}
		*ppData = s_ControlReport;
		*pLength = sizeof(s_ControlReport);
		return true;
	}
	if (Stage == USB_CTRL_COMPLETE &&
		pSetup->bRequest == USB_HID_REQ_SET_REPORT)
	{
		return g_Hid.SendReport(s_ControlReport, *pLength);
	}
	return true;
}

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.BcdHid = 0U,
	.FsMps = HID_REPORT_SIZE,
	.HsMps = HID_REPORT_SIZE,
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_NONE,
	.Protocol = USB_HID_PROT_NONE,
	.CountryCode = 0U,
	.InterfaceString = HID_STR_INTERFACE,
	.RxHandler = HidRx,
	.TxHandler = HidTx,
	.pContext = nullptr,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB HID Loopback",
	.pSerial = nullptr,
	.pFuncName = "HID Loopback",
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
	if (!UsbInit(&s_UsbCfg) || !g_Hid.Init(s_HidCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);
	bool suspended = false;
	while (1)
	{
		UsbProcess(USB_DEVNO);
		const bool nowSuspended = UsbSuspended(USB_DEVNO);
		if (nowSuspended != suspended)
		{
			suspended = nowSuspended;
			if (suspended) g_Hid.Suspend();
			else (void)g_Hid.Resume();
		}
	}
	return 0;
}
