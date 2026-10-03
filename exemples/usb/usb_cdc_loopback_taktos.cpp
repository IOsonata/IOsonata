/**-------------------------------------------------------------------------
@example	usb_cdc_loopback_taktos.cpp


@brief	USB CDC loopback with TaktOS

One thread owns USB processing and CDC I/O. A second periodic thread updates
an observable counter, demonstrating scheduler progress during USB traffic.
Thread memory and USB FIFOs are static. See usb_taktos/README.md for build
and hardware validation instructions.

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2026

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
#include "app_evt_handler.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "coredev/system_core_clock.h"
#include "TaktOS.h"
#include "TaktOSThread.h"

#define USB_DEVNO				0

#define BUFFER_SIZE				USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

// The application owns queued RX/TX memory. UsbdCdc owns the controller
// transfer buffers and copies completed OUT packets into this RX packet FIFO.
#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE \
	USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK))
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

// USB CDC configuration
static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = nullptr,
};

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware and is
// what the other USB demo in this tree uses. Put your own vendor and product
// id here before shipping anything : a duplicate pair makes the host reuse a
// driver and a saved COM port from somebody else's board.
// Application event queue memory. The USB controller port queues its deferred
// endpoint events there; the library default holds 4.
alignas(4) static uint8_t s_AppEvtQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
const AppEvtHandlerQueCfg_t g_AppEvtHandlerQueCfg = {
	s_AppEvtQueMem, sizeof(s_AppEvtQueMem)
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0001,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata CDC TaktOS",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = true,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static UsbdCdc s_Cdc;

alignas(8) static uint8_t s_UsbThreadMem[TAKTOS_THREAD_MEM_SIZE(2048)];
alignas(8) static uint8_t s_HeartbeatThreadMem[TAKTOS_THREAD_MEM_SIZE(512)];

// Inspect in the debugger. Only HeartbeatThread writes this counter.
volatile uint32_t g_UsbTaktOSHeartbeat = 0;

static void HeartbeatThread(void *pArg)
{
	(void)pArg;
	while (true)
	{
		g_UsbTaktOSHeartbeat = g_UsbTaktOSHeartbeat + 1;
		(void)TaktOSThreadSleep(TaktOSCurrentThread(), 1000);
	}
}

static void UsbThread(void *pArg)
{
	(void)pArg;
	uint8_t buffer[BUFFER_SIZE];
	int pending = 0;
	int offset = 0;

	// Initialize USB after the scheduler starts.
	if (!UsbInit(&s_UsbCfg) || !s_Cdc.Init(s_CdcCfg))
	{
		(void)TaktOSThreadSuspend(TaktOSCurrentThread());
		return;
	}
	(void)UsbEnable(USB_DEVNO);

	while (true)
	{
		// Bound each burst so lower-priority work runs under sustained traffic.
		for (unsigned i = 0; i < 32; i++)
		{
			// nRF52840 also drains deferred completions here. No second
			// AppEvtHandlerExec consumer may run in another thread.
			UsbProcess(USB_DEVNO);
			if (!UsbConfigured(USB_DEVNO))
			{
				pending = 0;
				offset = 0;
				break;
			}
			if (pending == 0)
			{
				int n = s_Cdc.Rx(0, buffer, sizeof(buffer));
				if (n <= 0)
				{
					break;
				}
				pending = n;
				offset = 0;
			}
			int n = s_Cdc.Tx(0, buffer + offset, pending);
			if (n <= 0)
			{
				break;
			}
			offset += n;
			pending -= n;
		}

		// Nonblocking TX preserves partial writes across service passes.
		// Sleep also lets lower-priority threads run; yield alone would not.
		(void)TaktOSThreadSleepTicks(TaktOSCurrentThread(), 1);
	}
}

int main()
{
	const TaktOSCfg_t cfg = {
		.KernClockHz = SystemCoreClockGet(),
		.TickHz = 1000,
		.TickClockSrc = TAKTOS_TICK_CLOCK_PROCESSOR,
	};
	if (TaktOSInit(&cfg) != TAKTOS_OK ||
		TaktOSThreadCreate(s_UsbThreadMem, sizeof(s_UsbThreadMem),
			UsbThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_HeartbeatThreadMem, sizeof(s_HeartbeatThreadMem),
			HeartbeatThread, nullptr, TAKTOS_PRIORITY_LOW) == nullptr)
	{
		while (true) {}
	}
	TaktOSStart();
	while (true) {}
}
