/**-------------------------------------------------------------------------
@file	composite_firmware.cpp

@brief	LPC full-speed CDC/HID/bulk composite firmware link fixture.

Uses the real generic classes, event queue and native controller. Small
caller-owned buffers keep the fixture suitable for the LPC11U35 main RAM.
This is a build fixture, not a hardware-validated example.

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
#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_hid.h"
#include "usb/usbd_bulk.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Pins[] = USB_PINS;
static UsbdCdc s_Cdc;
static UsbdHid s_Hid;
static UsbdBulk s_Bulk;
alignas(4) static uint8_t s_BulkRx[USBD_BULK_RXMEM_SIZE(2)];
alignas(4) static uint8_t s_BulkTx[CFIFO_MEMSIZE(128)];
static const uint8_t s_ReportDesc[] = {
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01,
	0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02,
	0x09, 0x01, 0x91, 0x02, 0xC0
};
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
alignas(4) static uint8_t s_CdcRx[USB_INTRF_RXMEM_SIZE(2, 64)];
alignas(4) static uint8_t s_CdcTx[CFIFO_MEMSIZE(128)];
alignas(4) static uint8_t s_HidRx[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTx[USB_INT_INTRF_PKT_BLKSIZE];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = 0,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "LPC CDC/HID/bulk link fixture",
	.IntPrio = 6,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]),
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2,
	.DeviceProtocol = 1,
	.MaxPower = 100,
};
static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = 0,
	.bBlocking = true,
	.RxFifoMemSize = sizeof(s_CdcRx),
	.pRxFifoMem = s_CdcRx,
	.TxFifoMemSize = sizeof(s_CdcTx),
	.pTxFifoMem = s_CdcTx,
};
static const UsbdBulkCfg_t s_BulkCfg = {
	.DevNo = 0,
	.bBlocking = true,
	.RxFifoMemSize = sizeof(s_BulkRx),
	.pRxFifoMem = s_BulkRx,
	.TxFifoMemSize = sizeof(s_BulkTx),
	.pTxFifoMem = s_BulkTx,
};
static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = 0,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.pRxBuffer = s_HidRx,
	.pTxBuffer = s_HidTx,
};

// Preserve unsent bytes when a transport is busy. Each class has its own
// pending packet; servicing one class must not prevent another from making progress.
static void Echo(DeviceIntrf &Intrf, uint8_t *pBuffer, int &Pending)
{
	if (Pending == 0)
	{
		const int received = Intrf.Rx(0, pBuffer, 64);
		if (received > 0)
			Pending = received;
	}
	if (Pending > 0)
	{
		const int sent = Intrf.Tx(0, pBuffer, Pending);
		if (sent > 0)
		{
			Pending -= sent;
			for (int i = 0; i < Pending; ++i)
				pBuffer[i] = pBuffer[i + sent];
		}
	}
}

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg) || !s_Cdc.Init(s_CdcCfg) ||
		!s_Hid.Init(s_HidCfg) || !s_Bulk.Init(s_BulkCfg))
	{
		return -1;
	}
	(void)UsbEnable(0);
	uint8_t cdcPacket[64];
	uint8_t hidPacket[64];
	uint8_t bulkPacket[64];
	int cdcPending = 0;
	int hidPending = 0;
	int bulkPending = 0;
	while (1)
	{
		if (!AppEvtHandlerExec())
			UsbCheckStatus();
		Echo(s_Cdc, cdcPacket, cdcPending);
		Echo(s_Hid, hidPacket, hidPending);
		Echo(s_Bulk, bulkPacket, bulkPending);
	}
}
