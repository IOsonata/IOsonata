/**-------------------------------------------------------------------------
@example	usb_custom_bulk_loopback.cpp

@brief	USB custom bulk loopback example

This example creates one custom USB interface with a bulk OUT endpoint and a
bulk IN endpoint. The application does not choose interface or endpoint
numbers. UsbdBulk allocates the USB topology, builds its descriptor fragments
and registers them with the generic USB layer when it is initialized.

The custom interface uses USB class 0xFF. Subclass and protocol are left at
zero so an application can define its own protocol on top of the bulk data
path. The loop below simply returns every byte received from the host.

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
#include "usb/usb.h"
#include "usb/usbd_bulk.h"

#define USB_DEVNO				0
#define CUSTOM_STR_INTERFACE	4U

#define CUSTOM_RXFIFO_PKTCNT	4
#define CUSTOM_RXFIFO_MEMSIZE \
	USBD_BULK_RXMEM_SIZE(CUSTOM_RXFIFO_PKTCNT)
#define CUSTOM_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)

#define LOOPBACK_BUFFER_SIZE	USB_PKT_MAXLEN(USB_DEVNO, BULK)

alignas(4) static uint8_t s_RxFifoMem[CUSTOM_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_TxFifoMem[CUSTOM_TXFIFO_MEMSIZE];

static UsbdBulk g_CustomBulk;

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
	uint8_t buffer[LOOPBACK_BUFFER_SIZE];

	if (!UsbInit(&s_UsbCfg))
	{
		return -1;
	}

	// UsbdBulk allocates one interface and one bidirectional endpoint pair.
	// No interface or endpoint number is part of s_BulkCfg.
	if (!g_CustomBulk.Init(s_BulkCfg))
	{
		return -1;
	}

	// A board may start without a cable. UsbProcess retries the connection
	// when VBUS appears, so a false result here is not fatal.
	(void)UsbEnable(USB_DEVNO);

	int pending = 0;
	int offset = 0;

	while (1)
	{
		UsbProcess(USB_DEVNO);

		if (pending > 0)
		{
			int n = g_CustomBulk.TxData(&buffer[offset], pending);
			if (n > 0)
			{
				offset += n;
				pending -= n;
			}
			continue;
		}

		int len = g_CustomBulk.RxData(buffer, sizeof(buffer));
		if (len > 0)
		{
			pending = len;
			offset = 0;
		}
	}

	return 0;
}
