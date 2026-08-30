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

UsbDevProcess is what moves data. Call it from the main loop or from a thread,
never from an interrupt, and nothing moves in either direction without it.

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

// FIFO memory belongs to the application, the same as it does for the UART.
// One packet is enough to echo with, but a log or a printf burst wants room
// to run ahead of the host, so give the Tx side more than the Rx side.
#define CDC_RXFIFO_MEMSIZE		CFIFO_MEMSIZE(256)
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len);

// USB CDC interface configuration
static const UsbdCdcIntrfCfg_t s_CdcCfg = {
	.bBlocking = false,
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

// USB CDC object instance
UsbdCdcIntrf g_Cdc;

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

				g_Cdc.Tx(0, (uint8_t*)msg, strlen(msg));
			}
			break;

		case DEVINTRF_EVT_RX_DATA:
			// Data is in the Rx FIFO. Read it from the main loop rather than
			// here, so that the work does not run inside the pump.
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

	// A board on a battery starts with no cable in it, so this failing is
	// not an error. UsbDevProcess notices the attach and comes back to it.
	UsbDevEnable();

	while (1)
	{
		UsbDevProcess();

		int l = g_Cdc.Rx(0, buff, BUFFER_SIZE);

		if (l > 0)
		{
			g_Cdc.Tx(0, buff, l);
		}
	}

	return 0;
}
