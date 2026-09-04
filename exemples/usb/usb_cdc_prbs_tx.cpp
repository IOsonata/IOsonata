/**-------------------------------------------------------------------------
@example	usb_cdc_prbs_tx.cpp


@brief	USB CDC PRBS transmit test

Demo code using IOsonata library to send a PRBS stream out a USB CDC port.
The device appears on the host as a serial port. Read it with the matching
receiver and every dropped or reordered octet shows up as a sequence break,
which a plain text stream would hide.

Nothing here is specific to an MCU. The USB device controller, its clock and
its cable detect are behind UsbdInit and friends, and one port file answers
them per MCU family, so the same source builds for every target that has a
USB device controller.

Tx follows the same model as UART: the application only fills the Tx CFifo.
The first write that finds Tx idle starts the endpoint with whatever data is
available, and every completion interrupt sends whatever has accumulated
since, until the FIFO comes up empty. UsbProcess is needed while waiting for
USB attach and port open, not between transmitted bytes.

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

#include "cfifo.h"
#include "prbs.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"


#define BYTE_MODE

#define USB_DEVNO				0

#define TEST_BUFSIZE			16

// FIFO memory belongs to the application. This test writes as fast as the
// host will take it, so the Tx side is sized to hold several packets and let
// the fill loop run ahead of the bus.
// Rx memory is the USB packet ring itself, one slot per endpoint packet.
#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_PKT_MAXLEN(USB_DEVNO, BULK))
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

// USB CDC configuration.
//
// Match the UART PRBS test: when the FIFO is full, reject new data so the
// caller retries the same byte. This CFifo policy does not wait for hardware;
// Tx still returns immediately while the completion interrupt drains the FIFO.
static const UsbdCdcCfg_t s_CdcCfg = {
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.ItfNo = 0,
	.DevNo = USB_DEVNO,
	.EvtCB = nullptr,
};

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware. Put your
// own vendor and product id here before shipping anything.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0002,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata CDC PRBS Tx",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata CDC",
	.NbCdc = 1,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
};

// CDC class/control object. It inherits the DeviceIntrf transfer methods, so
// the application sends through the object rather than the C interface.
UsbdCdc g_Cdc;

int main()
{
	uint8_t d = 0xff;
#ifndef BYTE_MODE
	uint8_t buff[TEST_BUFSIZE];
#endif

	if (UsbInit(&s_UsbCfg) == false)
	{
		return -1;
	}

	if (g_Cdc.Init(s_CdcCfg) == false)
	{
		return -1;
	}


	// A board on a battery starts with no cable in it, so this failing is
	// not an error. UsbProcess notices the attach and comes back to it.
	UsbEnable(USB_DEVNO);

	while (1)
	{
		// Pump only the USB lifecycle while waiting for the host. Once the port
		// is open, Bulk IN progress is entirely completion-interrupt driven.
		if (g_Cdc.IsPortOpen() == false)
		{
			UsbProcess(USB_DEVNO);
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte. The value advances only when the octet
		// was accepted into the FIFO. If the FIFO is full, retry this same byte
		// while the USB completion interrupt makes room.
		if (g_Cdc.Tx(0, &d, 1) > 0)
		{
			d = Prbs8(d);
		}
#else
		// Demo transfer buffer
		for (int i = 0; i < TEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}

		int len = TEST_BUFSIZE;
		uint8_t *p = buff;

		while (len > 0)
		{
			int l = g_Cdc.Tx(0, p, len);

			len -= l;
			p += l;

			if (g_Cdc.IsPortOpen() == false)
			{
				break;
			}
		}
#endif
	}

	return 0;
}
