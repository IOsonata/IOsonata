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

CDC Tx drains its FIFO from the endpoint-completion interrupt, like UART Tx
drains its FIFO from ENDTX. UsbDevProcess handles cable state and deferred USB
work; it is not part of the steady-state payload path. This test therefore
runs the byte-at-a-time Tx loop until it back-pressures, then gives the USB
runtime a service pass.

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
#include "usb/usb_dev.h"
#include "usb/usbd_cdc.h"

#define BYTE_MODE

#define TEST_BUFSIZE			16

// FIFO memory belongs to the application. This test writes as fast as the
// host will take it, so the Tx side is sized to hold several packets and let
// the fill loop run ahead of the bus.
#define CDC_RXFIFO_MEMSIZE		CFIFO_MEMSIZE(256)
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

// USB CDC interface configuration.
//
// The FIFO must not block. A full Tx FIFO is the point where this test gives
// UsbDevProcess a service pass and then resumes feeding the interrupt-driven
// endpoint path.
static const UsbdCdcIntrfCfg_t s_CdcCfg = {
	.bBlocking = false,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.ItfNo = 0,
	.EvtCB = nullptr,
};

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware. Put your
// own vendor and product id here before shipping anything.
static const UsbDevCfg_t s_UsbDevCfg = {
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

// USB CDC object instance
UsbdCdcIntrf g_Cdc;

int main()
{
	uint8_t d = 0xff;
#ifndef BYTE_MODE
	uint8_t buff[TEST_BUFSIZE];
#endif

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
		// Sending before the host has opened the port fills the FIFO with
		// octets the receiver will never see, and the sequence it does see
		// then starts in the middle. While closed, service cable/control state.
		if (g_Cdc.IsPortOpen() == false)
		{
			UsbDevProcess();
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte, like UartPrbsTx BYTE_MODE. Endpoint
		// completion interrupts drain/refill the FIFO; only back-pressure needs
		// a USB runtime service pass here.
		if (g_Cdc.Tx(0, &d, 1) > 0)
		{
			d = Prbs8(d);
		}
		else
		{
			UsbDevProcess();
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

			if (l > 0)
			{
				len -= l;
				p += l;
				continue;
			}

			// The FIFO is full. Service cable/deferred USB state and stop if
			// the cable went away while this buffer was half sent.
			UsbDevProcess();

			if (g_Cdc.IsPortOpen() == false)
			{
				break;
			}
		}
#endif
	}

	return 0;
}
