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

#include "app_evt_handler.h"
#include "usb/usb.h"
#if __has_include("board.h")
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"
#endif
#include "usb/usbd_hid.h"
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

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

static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *pContext);
static bool HidSetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *pContext);

static UsbdHid s_Hid;
static uint8_t s_Pending[HID_REPORT_SIZE];
static uint16_t s_PendingLength;
static uint8_t s_ControlReport[HID_REPORT_SIZE];

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length);

alignas(4) static uint8_t s_HidRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.FsMps = HID_REPORT_SIZE,
	.HsMps = HID_REPORT_SIZE,
	.InterfaceString = HID_STR_INTERFACE,
	.EvtCB = HidEvent,
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
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB HID Loopback",
	.pFuncName = "HID Loopback",
	.IntPrio = 6,
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#endif
	.MaxPower = 100,
};

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT event,
	uint8_t *pData, int Length)
{
	const UsbCtrlrXferResult_t result =
		(event == DEVINTRF_EVT_RX_TIMEOUT || event == DEVINTRF_EVT_TX_TIMEOUT) ?
		USB_CTRLR_XFER_FAILED : USB_CTRLR_XFER_SUCCESS;
	if (event == DEVINTRF_EVT_RX_DATA || event == DEVINTRF_EVT_RX_TIMEOUT)
	{
		if (result != USB_CTRLR_XFER_SUCCESS || Length > sizeof(s_Pending))
		{
			return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
		}
		if (s_Hid.Tx(0, pData, Length) != (int)Length)
		{
			memcpy(s_Pending, pData, Length);
			s_PendingLength = Length;
		}
	}
	else if (event == DEVINTRF_EVT_TX_FIFO_EMPTY || event == DEVINTRF_EVT_TX_TIMEOUT)
	{
		if (result == USB_CTRLR_XFER_SUCCESS && s_PendingLength != 0U)
		{
			const uint16_t length = s_PendingLength;
			s_PendingLength = 0U;
			if (s_Hid.Tx(0, s_Pending, length) != (int)length)
			{
				s_PendingLength = length;
			}
		}
	}
	return result == USB_CTRLR_XFER_SUCCESS ? Length : 0;
}

static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *)
{
	if (Type != USB_HID_REPTYPE_INPUT || ReportId != 0U ||
		BufferSize < sizeof(s_ControlReport))
	{
		return -1;
	}
	memcpy(pBuffer, s_ControlReport, sizeof(s_ControlReport));
	return sizeof(s_ControlReport);
}

static bool HidSetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *)
{
	if (Type != USB_HID_REPTYPE_OUTPUT || ReportId != 0U ||
		Length > sizeof(s_ControlReport))
	{
		return false;
	}
	memcpy(s_ControlReport, pBuffer, Length);
	HidEvent(nullptr, DEVINTRF_EVT_RX_DATA, s_ControlReport, Length);
	return true;
}

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Hid.Init(s_HidCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);
	AppRun();
	return 0;
}

