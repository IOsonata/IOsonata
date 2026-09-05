/**-------------------------------------------------------------------------
@example	usb_dual_cdc_stress.cpp

@brief	Dual USB CDC shared-controller stress test

CDC instance zero echoes a sustained host stream while CDC instance one
transmits PRBS continuously. Running both at the same time exercises the USB
controller arbitration across several endpoint directions: CDC zero bulk OUT
and IN, and CDC one bulk IN.

The application owns a separate RX and TX CFifo for each CDC function. Each
UsbdCdc object owns its endpoint transfer buffers, while the target USB
controller serializes access to shared DMA hardware.

Run Python/usb_dual_cdc_stress.py against the two serial ports exposed by this
firmware.

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2026

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
#include "prbs.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"

#define USB_DEVNO				0
#define BUFFER_SIZE				USB_PKT_MAXLEN(USB_DEVNO, BULK)

#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, BUFFER_SIZE)
#define LOOPBACK_TXFIFO_MEMSIZE	CFIFO_MEMSIZE(1024)
#define PRBS_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

alignas(4) static uint8_t s_LoopbackRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_LoopbackTxFifoMem[LOOPBACK_TXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_PrbsTxFifoMem[PRBS_TXFIFO_MEMSIZE];

static int LoopbackEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
							  uint8_t *pBuffer, int Len);

static const UsbdCdcCfg_t s_LoopbackCfg = {
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_LoopbackRxFifoMem,
	.TxFifoMemSize = LOOPBACK_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_LoopbackTxFifoMem,
	.ItfNo = 0,
	.DevNo = USB_DEVNO,
	.EvtCB = LoopbackEvtHandler,
};

static const UsbdCdcCfg_t s_PrbsCfg = {
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_PrbsRxFifoMem,
	.TxFifoMemSize = PRBS_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_PrbsTxFifoMem,
	.ItfNo = 1,
	.DevNo = USB_DEVNO,
	.EvtCB = nullptr,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Vid = 0x1209,
	.Pid = 0x0003,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata Dual CDC Stress",
	.pSerial = nullptr,
	.pFuncName = "IOsonata CDC",
	.NbCdc = 2,
	.IntPrio = 6,
	.bSelfPowered = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
	.DescHandler = nullptr,
	.pDescContext = nullptr,
};

UsbdCdc g_LoopbackCdc;
UsbdCdc g_PrbsCdc;

static int LoopbackEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
							  uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;

	if (EvtId == DEVINTRF_EVT_STATECHG && Len)
	{
		const char *msg = "\r\nIOsonata USB Dual CDC Loopback\r\n";

		g_LoopbackCdc.Tx(0, reinterpret_cast<const uint8_t *>(msg),
						 (int)strlen(msg));
	}

	return 0;
}

int main()
{
	uint8_t loopbackBuffer[BUFFER_SIZE];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0;
	int loopbackPending = 0;
	int loopbackOffset = 0;

	if (!UsbInit(&s_UsbCfg) ||
		!g_LoopbackCdc.Init(s_LoopbackCfg) ||
		!g_PrbsCdc.Init(s_PrbsCfg))
	{
		return -1;
	}

	// A missing cable is not an initialization failure. UsbProcess detects a
	// later attach and starts the controller then.
	UsbEnable(USB_DEVNO);

	while (1)
	{
		UsbProcess(USB_DEVNO);

		// Service at most one loopback operation per pass so the PRBS producer
		// below always gets a chance to queue data as well.
		if (loopbackPending > 0)
		{
			int length = g_LoopbackCdc.Tx(
				0, &loopbackBuffer[loopbackOffset], loopbackPending);

			if (length > 0)
			{
				loopbackOffset += length;
				loopbackPending -= length;
			}
		}
		else
		{
			int length = g_LoopbackCdc.Rx(
				0, loopbackBuffer, sizeof(loopbackBuffer));

			if (length > 0)
			{
				for (int i = 0; i < length; i++)
				{
					if (loopbackBuffer[i] != loopbackExpected)
					{
						loopbackRxErrorNotify++;
					}
					loopbackExpected = Prbs8(loopbackBuffer[i]);
				}

				loopbackPending = length;
				loopbackOffset = 0;
			}
		}

		// Zero never occurs in this PRBS sequence. Send one zero for every
		// discontinuity already observed at the loopback RX boundary, allowing
		// the host test to distinguish an OUT loss from a later IN loss.
		uint8_t prbsByte = loopbackRxErrorNotify > 0U ? 0U : prbs;
		if (g_PrbsCdc.IsPortOpen() && g_PrbsCdc.Tx(0, &prbsByte, 1) > 0)
		{
			if (loopbackRxErrorNotify > 0U)
			{
				loopbackRxErrorNotify--;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
	}

	return 0;
}
