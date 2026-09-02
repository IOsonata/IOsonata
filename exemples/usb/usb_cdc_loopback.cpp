/**-------------------------------------------------------------------------
@example	usb_cdc_loopback.cpp


@brief	USB CDC loopback test

Demo code using IOsonata library to read from a USB CDC port and send it back
out the same port. The device appears on the host as a serial port. Open it
with any terminal and whatever is typed comes back.

Nothing here is specific to an MCU. The USB device controller, its clock and
its cable detect are behind UsbdInit and friends, and one port file answers
them per MCU family, so the same source builds for every target that has a
USB device controller.

USB RX/TX packet progress is interrupt driven. UsbDevProcess handles device
attach/detach and class housekeeping and should still be called regularly from
the main loop or a thread.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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
#include "usb/usb_dev.h"
#include "usb/usbd_cdc.h"

#define BUFFER_SIZE				64

// RX memory holds four completed full-speed packet blocks. Each block includes
// the generic USB packet header as well as the 64-byte payload capacity. TX
// keeps a byte CFifo so application writes can run ahead of USB IN completion.
#define CDC_RXFIFO_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(4, sizeof(UsbPktHdr_t) + USBD_CDC_BULK_FS_MPS)
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len);

// USB CDC configuration
static const UsbdCdcCfg_t s_CdcCfg = {
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.ItfNo = 0,
	.EvtCB = CdcEvtHandler,
};

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware and is
// what the other USB demo in this tree uses. Put your own vendor and product
// id here before shipping anything : a duplicate pair makes the host reuse a
// driver and a saved COM port from somebody else's board.
static const UsbDevCfg_t s_UsbDevCfg = {
	.Vid = 0x1209,
	.Pid = 0x0001,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata CDC Loopback",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata CDC",
	.NbCdc = 1,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
};

// CDC class/control object. Application data uses g_Cdc.Data().
UsbdCdc g_Cdc;

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_STATECHG:
			// Len is one when the host opened the port. Nothing on the device
			// side can tell that from a cable being plugged in, since a host
			// may enumerate and never open.
			if (Len)
			{
				const char *msg = "\r\nIOsonata USB CDC Loopback\r\n";

				DeviceIntrfTx(g_Cdc.Data(), 0,
							(const uint8_t *)msg, strlen(msg));
			}
			break;

		case DEVINTRF_EVT_RX_DATA:
			// RX completion runs in the USB interrupt. Keep this callback short;
			// the main loop consumes the committed packet-ring data below.
			break;

		default:
			break;
	}

	return 0;
}

int main()
{
	uint8_t buff[BUFFER_SIZE];

	if (UsbDevInit(&s_UsbDevCfg) == false)
	{
		return -1;
	}

	if (g_Cdc.Init(s_CdcCfg) == false)
	{
		return -1;
	}

	DevIntrf_t *pData = g_Cdc.Data();

	// A board on a battery starts with no cable in it, so this failing is
	// not an error. UsbDevProcess notices the attach and comes back to it.
	UsbDevEnable();

	int pending = 0;
	int offset = 0;

	while (1)
	{
	    UsbDevProcess();

	    if (pending > 0)
	    {
	        int n = DeviceIntrfTx(pData, 0, &buff[offset], pending);

	        if (n > 0)
	        {
	            offset += n;
	            pending -= n;
	        }

	        continue;
	    }

	    int l = DeviceIntrfRx(pData, 0, buff, sizeof(buff));

	    if (l > 0)
	    {
	        pending = l;
	        offset = 0;
	    }
	}

	return 0;
}
