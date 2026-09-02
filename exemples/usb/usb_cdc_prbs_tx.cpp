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
available, completion interrupts keep draining the FIFO one full packet at a
time, and a partial tail left behind goes out on its own after one idle USB
frame. UsbDevProcess is needed while waiting for USB attach and port open,
not between transmitted bytes.

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

// FIFO memory belongs to the application. RX uses completed packet blocks,
// including the generic USB packet header. This test writes as fast as the
// host will take it, so the Tx side is sized to hold several packets and let
// the fill loop run ahead of the bus.
#define CDC_RXFIFO_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(4, sizeof(UsbPktHdr_t) + USBD_CDC_BULK_FS_MPS)
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

// CDC class/control object. Application data uses g_Cdc.Data().
UsbdCdc g_Cdc;

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

	DevIntrf_t *pData = g_Cdc.Data();

	// A board on a battery starts with no cable in it, so this failing is
	// not an error. UsbDevProcess notices the attach and comes back to it.
	UsbDevEnable();

	while (1)
	{
		// Pump only the USB lifecycle while waiting for the host. Once the port
		// is open, Bulk IN progress is entirely completion-interrupt driven.
		if (g_Cdc.IsPortOpen() == false)
		{
			UsbDevProcess();
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte. The value advances only when the octet
		// was accepted into the FIFO. If the FIFO is full, retry this same byte
		// while the USB completion interrupt makes room.
		if (DeviceIntrfTx(pData, 0, &d, 1) > 0)
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
			int l = DeviceIntrfTx(pData, 0, p, len);

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
