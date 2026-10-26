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
#include "app_evt_handler.h"
#include "usb/usb.h"
#include "usb/usbd_hid.h"

#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

#define USB_DEVNO			0
#define HID_STR_INTERFACE	4U
#define HID_KEY_A			0x04U
#define HID_KEY_CAPS_LOCK	0x39U
#define HID_LED_CAPS_LOCK	(1U << 1)
#define HID_DEBOUNCE_COUNT	5U

static const uint8_t s_ReportDesc[] = USB_HID_KEYBOARD_REPORT_DESC;

static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *pContext);
static bool HidSetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *pContext);

static UsbdHid s_Hid;
static UsbHidKeyboardReport_t s_Report = {};
static uint8_t s_LedReport;
static bool s_ReportPending;

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length);

alignas(4) static uint8_t s_HidRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.BcdHid = 0U,
	.FsMps = sizeof(UsbHidKeyboardReport_t),
	.HsMps = sizeof(UsbHidKeyboardReport_t),
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_BOOT,
	.Protocol = USB_HID_PROT_KEYBOARD,
	.CountryCode = 0U,
	.InterfaceString = HID_STR_INTERFACE,
	.EvtCB = HidEvent,
	.pContext = nullptr,
	.pRxBuffer = s_HidRxBuffer,
	.pTxBuffer = s_HidTxBuffer,
	.GetReportCB = HidGetReport,
	.SetReportCB = HidSetReport,
};

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

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

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	const UsbCtrlrXferResult_t result =
		(event == DEVINTRF_EVT_RX_TIMEOUT || event == DEVINTRF_EVT_TX_TIMEOUT) ?
		USB_CTRLR_XFER_FAILED : USB_CTRLR_XFER_SUCCESS;
	if (event == DEVINTRF_EVT_RX_DATA || event == DEVINTRF_EVT_RX_TIMEOUT)
	{
		if (result == USB_CTRLR_XFER_SUCCESS && pData != nullptr && Length == 1U)
		{
			s_LedReport = pData[0];
			KeyboardLedApply(s_LedReport);
		}
	}
	return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
}


static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *)
{
	if (ReportId != 0U || BufferSize < sizeof(s_Report))
	{
		return -1;
	}
	if (Type == USB_HID_REPTYPE_INPUT)
	{
		memcpy(pBuffer, &s_Report, sizeof(s_Report));
		return sizeof(s_Report);
	}
	if (Type == USB_HID_REPTYPE_OUTPUT)
	{
		pBuffer[0] = s_LedReport;
		return 1;
	}
	return -1;
}

static bool HidSetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *)
{
	if (Type != USB_HID_REPTYPE_OUTPUT || ReportId != 0U || Length != 1U)
	{
		return false;
	}
	s_LedReport = pBuffer[0];
	KeyboardLedApply(s_LedReport);
	return true;
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

	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Hid.Init(s_HidCfg))
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
		AppEvtHandlerExec();
		const bool nowSuspended = UsbSuspended(USB_DEVNO);
		suspended = nowSuspended;

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
			s_Hid.Tx(0, reinterpret_cast<const uint8_t *>(&s_Report),
				sizeof(s_Report)) == (int)sizeof(s_Report))
		{
			s_ReportPending = false;
		}
		msDelay(1U);
	}
	return 0;
}

