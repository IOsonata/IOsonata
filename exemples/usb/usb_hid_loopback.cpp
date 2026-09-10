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
#define HID_STR_MANUFACTURER	1U
#define HID_STR_PRODUCT		2U
#define HID_STR_SERIAL		3U
#define HID_STR_INTERFACE	4U
#define HID_STR_MAXLEN		32U
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

static UsbdHid g_Hid;
static uint8_t s_Pending[HID_REPORT_SIZE];
static uint16_t s_PendingLength;
static uint8_t s_ControlReport[HID_REPORT_SIZE];

#pragma pack(push, 1)
typedef struct __Hid_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbdHidDesc_t Hid;
} HidConfigDesc_t;
#pragma pack(pop)

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static HidConfigDesc_t s_ConfigDesc;
static uint8_t s_StringDesc[2U + (HID_STR_MAXLEN * 2U)];

static const uint8_t *HidDescHandler(uint8_t DescType, uint8_t DescIndex,
								 uint16_t LangId, UsbSpeed_t Speed,
								 uint16_t *pLength, void *pContext);

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
							 uint16_t *pLength, void *)
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
	.pDesc = &s_ConfigDesc.Hid,
	.ReportHandler = HidReportRequest,
	.pReportContext = nullptr,
	.RxHandler = HidRx,
	.TxHandler = HidTx,
	.pContext = nullptr,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB HID Loopback",
	.pSerial = nullptr,
	.pFuncName = "HID Loopback",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
	.DescHandler = HidDescHandler,
	.pDescContext = nullptr,
};

static uint8_t HidMaxPower(const UsbCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->bSelfPowered)
	{
		return 0U;
	}
	const uint32_t units = ((uint32_t)pCfg->MaxPower + 1U) / 2U;
	return units > 255U ? 255U : (uint8_t)units;
}

static const uint8_t *HidDeviceDescriptor(uint16_t *pLength)
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
	s_DeviceDesc.iManufacturer = HID_STR_MANUFACTURER;
	s_DeviceDesc.iProduct = HID_STR_PRODUCT;
	s_DeviceDesc.iSerialNumber = UsbGetSerial(USB_DEVNO) != nullptr ?
		HID_STR_SERIAL : 0U;
	s_DeviceDesc.bNumConfigurations = 1U;
	*pLength = sizeof(s_DeviceDesc);
	return reinterpret_cast<const uint8_t *>(&s_DeviceDesc);
}

static const uint8_t *HidConfigDescriptor(bool OtherSpeed, uint16_t *pLength)
{
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	if (pCfg == nullptr || pLength == nullptr)
	{
		return nullptr;
	}
	memset(&s_ConfigDesc.Config, 0, sizeof(s_ConfigDesc.Config));
	s_ConfigDesc.Config.bLength = sizeof(s_ConfigDesc.Config);
	s_ConfigDesc.Config.bDescriptorType = OtherSpeed ?
		USB_DESCTYPE_OSC : USB_DESCTYPE_CONFIGURATION;
	s_ConfigDesc.Config.wTotalLength = sizeof(s_ConfigDesc);
	s_ConfigDesc.Config.bNumInterfaces = 1U;
	s_ConfigDesc.Config.bConfigurationValue = USBD_HID_CONFIG_VALUE;
	s_ConfigDesc.Config.bmAttributes = USB_CONFATT_RESERVED;
	if (pCfg->bSelfPowered)
	{
		s_ConfigDesc.Config.bmAttributes |= USB_CONFATT_SELF_POWERED;
	}
	s_ConfigDesc.Config.bMaxPower = HidMaxPower(pCfg);
	*pLength = sizeof(s_ConfigDesc);
	return reinterpret_cast<const uint8_t *>(&s_ConfigDesc);
}

static const uint8_t *HidQualifierDescriptor(uint16_t *pLength)
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

static const uint8_t *HidStringDescriptor(uint8_t Index, uint16_t LangId,
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
	if (Index > HID_STR_INTERFACE ||
		(LangId != 0U && LangId != 0x0409U))
	{
		return nullptr;
	}
	const UsbCfg_t *pCfg = UsbGetCfg(USB_DEVNO);
	const char *pStr = nullptr;
	if (pCfg != nullptr)
	{
		if (Index == HID_STR_MANUFACTURER) pStr = pCfg->pManufacturer;
		else if (Index == HID_STR_PRODUCT) pStr = pCfg->pProduct;
		else if (Index == HID_STR_SERIAL) pStr = UsbGetSerial(USB_DEVNO);
		else if (Index == HID_STR_INTERFACE) pStr = pCfg->pFuncName;
	}
	if (pStr == nullptr)
	{
		return nullptr;
	}
	size_t length = strlen(pStr);
	if (length > HID_STR_MAXLEN) length = HID_STR_MAXLEN;
	s_StringDesc[0] = (uint8_t)(2U + length * 2U);
	s_StringDesc[1] = USB_DESCTYPE_STRING;
	for (size_t i = 0U; i < length; i++)
	{
		s_StringDesc[2U + i * 2U] = (uint8_t)pStr[i];
		s_StringDesc[3U + i * 2U] = 0U;
	}
	*pLength = s_StringDesc[0];
	return s_StringDesc;
}

static const uint8_t *HidDescHandler(uint8_t DescType, uint8_t DescIndex,
								 uint16_t LangId, UsbSpeed_t Speed,
								 uint16_t *pLength, void *)
{
	(void)Speed;
	if (pLength == nullptr)
	{
		return nullptr;
	}
	*pLength = 0U;
	switch (DescType)
	{
		case USB_DESCTYPE_DEVICE:
			return DescIndex == 0U ? HidDeviceDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_CONFIGURATION:
			return DescIndex == 0U ? HidConfigDescriptor(false, pLength) : nullptr;
		case USB_DESCTYPE_STRING:
			return HidStringDescriptor(DescIndex, LangId, pLength);
		case USB_DESCTYPE_DEVICE_QUALIFIER:
			return DescIndex == 0U ? HidQualifierDescriptor(pLength) : nullptr;
		case USB_DESCTYPE_OSC:
			return DescIndex == 0U && USB_HIGHSPEED_CAPABLE(USB_DEVNO) ?
				HidConfigDescriptor(true, pLength) : nullptr;
		default:
			return nullptr;
	}
}

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
