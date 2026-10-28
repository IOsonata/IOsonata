/**-------------------------------------------------------------------------
@example	usb_custom_bulk_loopback.cpp

@brief	USB custom bulk loopback example

This example creates one custom USB interface with a bulk OUT endpoint and a
bulk IN endpoint. The application does not choose interface or endpoint
numbers. UsbdBulk allocates the USB topology, builds its descriptor fragments
and registers them with the generic USB layer when it is initialized.

The custom interface uses USB class 0xFF. Subclass and protocol are left at
zero so an application can define its own protocol on top of the bulk data
path. AppRun services deferred USB events; AppCheckStatus returns received
bytes and retains partial writes until the transmit FIFO accepts them.

@author	Hoang Nguyen Hoan
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

#include <stdint.h>
#include <string.h>

#include "cfifo.h"
#include "app_evt_handler.h"
#include "usb/usb.h"
#if __has_include("board.h")
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "board.h"
#endif
#include "usb/usbd_bulk.h"
#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

// board.h can select another controller
#ifndef USB_DEVNO
#define USB_DEVNO				0
#endif
#define CUSTOM_STR_INTERFACE	4U

#define CUSTOM_RXFIFO_PKTCNT	4
#define CUSTOM_RXFIFO_MEMSIZE \
	USBD_BULK_RXMEM_SIZE(CUSTOM_RXFIFO_PKTCNT)
#define CUSTOM_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)

#define LOOPBACK_BUFFER_SIZE	USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

alignas(4) static uint8_t s_RxFifoMem[CUSTOM_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_TxFifoMem[CUSTOM_TXFIFO_MEMSIZE];

static UsbdBulk s_CustomBulk;
static uint8_t s_Buffer[LOOPBACK_BUFFER_SIZE];
static int s_Pending;
static int s_Offset;

static const UsbdBulkCfg_t s_BulkCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CUSTOM_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_RxFifoMem,
	.TxFifoMemSize = CUSTOM_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_TxFifoMem,
	.SubClass = 0U,
	.Protocol = 0U,
	.InterfaceString = CUSTOM_STR_INTERFACE,
	.FsMps = 0U,
	.HsMps = 0U,
	.Mode = USBD_BULK_MODE_BYTE,
	.EvtCB = nullptr,
};

// These VID/PID values are for the example. Use IDs assigned to your product
// before shipping a device.
// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0002,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata Custom Bulk Loopback",
	.pSerial = nullptr,
	.pFuncName = "Custom Bulk",
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

bool AppCheckStatus(void)
{
	UsbCheckStatus();
	if (s_Pending == 0)
	{
		s_Pending = s_CustomBulk.RxData(s_Buffer, sizeof(s_Buffer));
		s_Offset = 0;
		if (s_Pending <= 0)
		{
			s_Pending = 0;
			return !AppEvtHandlerPending();
		}
	}
	const int sent = s_CustomBulk.TxData(&s_Buffer[s_Offset], s_Pending);
	if (sent > 0)
	{
		s_Offset += sent;
		s_Pending -= sent;
	}
	// Drain queued RX packets and retry bytes held by TX backpressure.
	return false;
}

int main()
{

	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!UsbInit(&s_UsbCfg))
	{
		return -1;
	}

	// UsbdBulk allocates one interface and one bidirectional endpoint pair.
	// No interface or endpoint number is part of s_BulkCfg.
	if (!s_CustomBulk.Init(s_BulkCfg))
	{
		return -1;
	}

	// A board may start without a cable. UsbProcess retries the connection
	// when VBUS appears, so a false result here is not fatal.
	(void)UsbEnable(USB_DEVNO);

	AppRun();

	return 0;
}
