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
#include "coredev/interrupt.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "board.h"
#include "coredev/system_core_clock.h"
#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSSem.h"


#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

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

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len);

// USB CDC configuration
static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = CdcEvtHandler,
};

#ifdef USB_PINS
static const IOPinCfg_t s_UsbPins[] = USB_PINS;
#endif

// USB device configuration.
//
// 0x1209 is the pid.codes vendor id, which exists for open hardware and is
// what the other USB demo in this tree uses. Put your own vendor and product
// id here before shipping anything : a duplicate pair makes the host reuse a
// driver and a saved COM port from somebody else's board.
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
#ifdef USB_PINS
	.pIOPinMap = s_UsbPins,
	.NbIOPins = sizeof(s_UsbPins) / sizeof(IOPinCfg_t),
#else
	.pIOPinMap = nullptr,
	.NbIOPins = 0,
#endif
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

#define USB_WORK_QUE_SIZE		16U
#define USB_WORK_PER_PASS		30U

// Deferred USB work queued with UsbEvtQue, run by UsbThread: the process event
// of the USB stack (bus reset, suspend, resume, cable) and, on a controller
// port that defers them, endpoint completions.
typedef struct {
	uint32_t EvtId;
	void *pCtx;
	UsbEvtQueHandler_t Handler;
} UsbWork_t;

alignas(4) static uint8_t s_UsbWorkMem[
	CFIFO_TOTAL_MEMSIZE(USB_WORK_QUE_SIZE, sizeof(UsbWork_t))];
static hCFifo_t s_hUsbWork;
// Given by every queued event and by the CDC data events, taken by UsbThread
// when it has nothing to do
static TaktOSSem_t s_UsbWake;

alignas(8) static uint8_t s_UsbThreadMem[TAKTOS_THREAD_MEM_SIZE(2048)];
alignas(8) static uint8_t s_HeartbeatThreadMem[TAKTOS_THREAD_MEM_SIZE(512)];

// Inspect in the debugger. Only HeartbeatThread writes this counter.
volatile uint32_t g_UsbTaktOSHeartbeat = 0;

// CDC data events. The nRF52 and nRF54 ports call this from the USB
// interrupt, without UsbEvtQue; a port that defers endpoint events calls it
// from UsbWorkExec. Received data and an emptied TX FIFO wake the USB thread
// from here in both cases.
static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	(void)Len;

	switch (EvtId)
	{
		case DEVINTRF_EVT_RX_DATA:
		case DEVINTRF_EVT_RX_FIFO_FULL:
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			// A full binary semaphore already records a wake for the thread
			(void)TaktOSSemGive(&s_UsbWake, false);
			break;

		default:
			break;
	}

	return 0;
}

// Link-time override of the library default, see usb.h. USB work goes to the
// queue of the thread serving USB, the application event queue is not used.
bool UsbEvtQue(uint32_t EvtId, void *pCtx, UsbEvtQueHandler_t Handler)
{
	// The controller interrupt and the USB thread both queue here, and
	// CFifoPut takes one producer at a time.
	uint32_t state = DisableInterrupt();
	UsbWork_t *p = (UsbWork_t *)CFifoPut(s_hUsbWork);
	if (p != nullptr)
	{
		p->EvtId = EvtId;
		p->pCtx = pCtx;
		p->Handler = Handler;
	}
	EnableInterrupt(state);
	if (p == nullptr)
	{
		return false;
	}
	// A full binary semaphore already records a wake for the thread
	(void)TaktOSSemGive(&s_UsbWake, false);
	return true;
}

// Run queued USB work. The head is copied before it is released: the
// interrupt can reuse the slot as soon as CFifoGet returns.
static void UsbWorkExec(void)
{
	for (unsigned cnt = USB_WORK_PER_PASS; cnt > 0U; cnt--)
	{
		const UsbWork_t *p = (const UsbWork_t *)CFifoPeek(s_hUsbWork);
		if (p == nullptr)
		{
			break;
		}
		const UsbWork_t work = *p;
		(void)CFifoGet(s_hUsbWork);
		work.Handler(work.EvtId, work.pCtx);
	}
}

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

	// Initialize USB after the scheduler starts. The work queue and its wake
	// exist before USB can queue its first event.
	s_hUsbWork = CFifoInit(s_UsbWorkMem, sizeof(s_UsbWorkMem),
						   sizeof(UsbWork_t), true);
	if (s_hUsbWork == nullptr ||
		TaktOSSemInit(&s_UsbWake, 0U, 1U) != TAKTOS_OK ||
		!UsbInit(&s_UsbCfg) || !s_Cdc.Init(s_CdcCfg))
	{
		(void)TaktOSThreadSuspend(TaktOSCurrentThread());
		return;
	}
	(void)UsbEnable(USB_DEVNO);

	while (true)
	{
		// This thread is the only consumer of the USB work queue, which also
		// holds the process event of the USB stack.
		UsbWorkExec();

		// One loopback step: read a packet when none is pending, then send
		// what is pending. Nonblocking TX keeps a partly accepted packet for
		// the next step.
		bool progress = false;
		if (!UsbConfigured(USB_DEVNO))
		{
			pending = 0;
			offset = 0;
		}
		else
		{
			if (pending == 0)
			{
				const int n = s_Cdc.Rx(0, buffer, sizeof(buffer));
				if (n > 0)
				{
					pending = n;
					offset = 0;
					progress = true;
				}
			}
			if (pending > 0)
			{
				const int n = s_Cdc.Tx(0, buffer + offset, pending);
				if (n > 0)
				{
					offset += n;
					pending -= n;
					progress = true;
				}
			}
		}

		// Nothing moved and no event waits: received data and an emptied TX
		// FIFO wake the thread from CdcEvtHandler, queued work from
		// UsbEvtQue, so wait for the next one. The wait is also when the lower
		// priority heartbeat runs. A full TX FIFO is the only reason Tx moves
		// nothing with data pending, and it wakes the thread once emptied.
		if (!progress && CFifoPeek(s_hUsbWork) == nullptr)
		{
			// Work refused by a full queue is queued again now
			UsbCheckStatus();
			(void)TaktOSSemTake(&s_UsbWake, true, TAKTOS_WAIT_FOREVER);
		}
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
