/**-------------------------------------------------------------------------
@example	usb_hid_keyboard.cpp

@brief	USB HID boot keyboard demo

Button 1 sends an A key press while held. Button 2 sends Caps Lock so the
keyboard LED output report returned by the host can drive LED 1.

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

#include "idelay.h"
#include "iopinctrl.h"
#include "usb/usb.h"
#include "usb/usbd_hid.h"

#include "board.h"

#define USB_DEVNO			0
#define HID_STR_MANUFACTURER	1U
#define HID_STR_PRODUCT		2U
#define HID_STR_SERIAL		3U
#define HID_STR_INTERFACE	4U
#define HID_STR_MAXLEN		32U
#define HID_KEY_A			0x04U
#define HID_KEY_CAPS_LOCK	0x39U
#define HID_LED_CAPS_LOCK	(1U << 1)
#define HID_DEBOUNCE_COUNT	5U

#pragma pack(push, 1)
typedef struct __Hid_Keyboard_Report {
	uint8_t Modifiers;
	uint8_t Reserved;
	uint8_t Key[6];
} HidKeyboardReport_t;

typedef struct __Hid_Keyboard_Config_Descriptor {
	UsbCfgDesc_t Config;
	UsbdHidDesc_t Hid;
} HidKeyboardConfigDesc_t;
#pragma pack(pop)

static_assert(sizeof(HidKeyboardReport_t) == 8U,
	"Boot keyboard reports must be eight bytes");

static const uint8_t s_ReportDesc[] = {
	0x05U, 0x01U,			// Usage Page (Generic Desktop)
	0x09U, 0x06U,			// Usage (Keyboard)
	0xA1U, 0x01U,			// Collection (Application)
	0x05U, 0x07U,			// Usage Page (Keyboard/Keypad)
	0x19U, 0xE0U,			// Usage Minimum (Left Control)
	0x29U, 0xE7U,			// Usage Maximum (Right GUI)
	0x15U, 0x00U,			// Logical Minimum (0)
	0x25U, 0x01U,			// Logical Maximum (1)
	0x75U, 0x01U,			// Report Size (1)
	0x95U, 0x08U,			// Report Count (8)
	0x81U, 0x02U,			// Input (Data, Variable, Absolute)
	0x95U, 0x01U,
	0x75U, 0x08U,
	0x81U, 0x01U,			// Input (Constant)
	0x05U, 0x08U,			// Usage Page (LEDs)
	0x19U, 0x01U,			// Usage Minimum (Num Lock)
	0x29U, 0x05U,			// Usage Maximum (Kana)
	0x95U, 0x05U,
	0x75U, 0x01U,
	0x91U, 0x02U,			// Output (Data, Variable, Absolute)
	0x95U, 0x01U,
	0x75U, 0x03U,
	0x91U, 0x01U,			// Output (Constant)
	0x05U, 0x07U,			// Usage Page (Keyboard/Keypad)
	0x19U, 0x00U,			// Usage Minimum (Reserved)
	0x29U, 0x65U,			// Usage Maximum (Keyboard Application)
	0x15U, 0x00U,
	0x25U, 0x65U,
	0x75U, 0x08U,
	0x95U, 0x06U,
	0x81U, 0x00U,			// Input (Data, Array, Absolute)
	0xC0U,
};

static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength);

class HidKeyboard final : public UsbdHid {
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

static HidKeyboard g_Hid;
static HidKeyboardReport_t s_Report;
static uint8_t s_LedReport;
static bool s_ReportPending;

static UsbDevDesc_t s_DeviceDesc;
static UsbDevQualDesc_t s_QualifierDesc;
static HidKeyboardConfigDesc_t s_ConfigDesc;
static uint8_t s_StringDesc[2U + (HID_STR_MAXLEN * 2U)];

const uint8_t *UsbGetDescriptor(int DevNo, uint8_t DescType,
								 uint8_t DescIndex, uint16_t LangId,
								 UsbSpeed_t Speed, uint16_t *pLength);

static bool ButtonDebounce(bool Pressed, bool &Candidate, bool &Stable,
						   uint8_t &Count)
{
	if (Pressed != Candidate)
	{
		Candidate = Pressed;
		Count = 0U;
		return false;
	}
	if (Count < HID_DEBOUNCE_COUNT)
	{
		Count++;
		if (Count == HID_DEBOUNCE_COUNT && Candidate != Stable)
		{
			Stable = Candidate;
			return true;
		}
	}
	return false;
}

static void KeyboardReportUpdate(bool APressed, bool CapsPressed)
{
	memset(&s_Report, 0, sizeof(s_Report));
	uint8_t index = 0U;
	if (APressed)
	{
		s_Report.Key[index++] = HID_KEY_A;
	}
	if (CapsPressed)
	{
		s_Report.Key[index] = HID_KEY_CAPS_LOCK;
	}
	s_ReportPending = true;
}

static void KeyboardLedApply(uint8_t Report)
{
	if ((Report & HID_LED_CAPS_LOCK) != 0U)
	{
		IOPinClear(HID_LED_PORT, HID_LED_PIN);
	}
	else
	{
		IOPinSet(HID_LED_PORT, HID_LED_PIN);
	}
}

static void HidRx(UsbdHidDev_t *, const uint8_t *pData, uint16_t Length,
				  UsbCtrlrXferResult_t Result, void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS && pData != nullptr && Length == 1U)
	{
		s_LedReport = pData[0];
		KeyboardLedApply(s_LedReport);
	}
}

static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength)
{
	if (pSetup == nullptr || pLength == nullptr ||
		(pSetup->wValue & USB_HID_REPID_MASK) != 0U)
	{
		return false;
	}
	if (Stage == USB_CTRL_ABORT)
	{
		return true;
	}
	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr)
		{
			return false;
		}
		const uint16_t type = pSetup->wValue & USB_HID_REPTYPE_MASK;
		if (pSetup->bRequest == USB_HID_REQ_GET_REPORT &&
			type == USB_HID_REPTYPE_INPUT)
		{
			*ppData = reinterpret_cast<uint8_t *>(&s_Report);
			*pLength = sizeof(s_Report);
			return true;
		}
		if (type == USB_HID_REPTYPE_OUTPUT && pSetup->wLength == 1U)
		{
			*ppData = &s_LedReport;
			*pLength = 1U;
			return true;
		}
		return false;
	}
	if (Stage == USB_CTRL_COMPLETE &&
		pSetup->bRequest == USB_HID_REQ_SET_REPORT)
	{
		KeyboardLedApply(s_LedReport);
	}
	return true;
}

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.BcdHid = 0U,
	.FsMps = sizeof(HidKeyboardReport_t),
	.HsMps = sizeof(HidKeyboardReport_t),
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_BOOT,
	.Protocol = USB_HID_PROT_KEYBOARD,
	.CountryCode = 0U,
	.InterfaceString = HID_STR_INTERFACE,
	.pDesc = &s_ConfigDesc.Hid,
	.RxHandler = HidRx,
	.TxHandler = nullptr,
	.pContext = nullptr,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0006,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata HID Keyboard",
	.pSerial = nullptr,
	.pFuncName = "HID Keyboard",
	.NbCdc = 0,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
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

const uint8_t *UsbGetDescriptor(int DevNo, uint8_t DescType,
								 uint8_t DescIndex, uint16_t LangId,
								 UsbSpeed_t Speed, uint16_t *pLength)
{
	(void)DevNo;
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
	IOPinConfig(HID_BUTTON_PORT, HID_BUTTON_PIN, HID_BUTTON_PINOP,
		IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinConfig(HID_CAPS_BUTTON_PORT, HID_CAPS_BUTTON_PIN,
		HID_CAPS_BUTTON_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP,
		IOPINTYPE_NORMAL);
	IOPinConfig(HID_LED_PORT, HID_LED_PIN, HID_LED_PINOP,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	KeyboardLedApply(0U);

	if (!UsbInit(&s_UsbCfg) || !g_Hid.Init(s_HidCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	bool suspended = false;
	bool aStable = IOPinRead(HID_BUTTON_PORT, HID_BUTTON_PIN) == 0;
	bool aCandidate = aStable;
	uint8_t aDebounce = 0U;
	bool capsStable = IOPinRead(HID_CAPS_BUTTON_PORT,
		HID_CAPS_BUTTON_PIN) == 0;
	bool capsCandidate = capsStable;
	uint8_t capsDebounce = 0U;

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

		const bool aPressed = IOPinRead(HID_BUTTON_PORT,
			HID_BUTTON_PIN) == 0;
		const bool capsPressed = IOPinRead(HID_CAPS_BUTTON_PORT,
			HID_CAPS_BUTTON_PIN) == 0;
		const bool aChanged = ButtonDebounce(aPressed, aCandidate, aStable,
			aDebounce);
		const bool capsChanged = ButtonDebounce(capsPressed, capsCandidate,
			capsStable, capsDebounce);
		if (aChanged || capsChanged)
		{
			KeyboardReportUpdate(aStable, capsStable);
		}

		if (!suspended && s_ReportPending &&
			g_Hid.SendReport(reinterpret_cast<const uint8_t *>(&s_Report),
				sizeof(s_Report)))
		{
			s_ReportPending = false;
		}
		msDelay(1U);
	}
	return 0;
}
