/**-------------------------------------------------------------------------
@example	usb_combo_stress_taktos.cpp

@brief	Multithreaded USB composite stress test with TaktOS.

Runs all USB data paths on one controller at the same time. Two CDC functions
retain the UsbDualCdcStress loopback/PRBS behavior. HID, raw interrupt and
isochronous functions each provide a bidirectional loopback. Interface and
endpoint numbers are assigned by the common allocator; ISO reserves the
controller-supported ISO endpoint.

Host runner: Python/usb_combo_stress.py

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include <atomic>
#include "usb_combo_stress_device.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSSem.h"

alignas(8) static uint8_t s_ServiceMem[TAKTOS_THREAD_MEM_SIZE(2048)];
alignas(8) static uint8_t s_LoopMem[TAKTOS_THREAD_MEM_SIZE(1024)];
alignas(8) static uint8_t s_PrbsMem[TAKTOS_THREAD_MEM_SIZE(1024)];
alignas(8) static uint8_t s_HeartbeatMem[TAKTOS_THREAD_MEM_SIZE(512)];

// LoopThread publishes a cumulative count; PrbsThread tracks how many error
// markers it has sent. Neither thread modifies the other's counter.
static_assert(std::atomic<uint32_t>::is_always_lock_free);
static std::atomic<uint32_t> s_LoopErrors{0};
volatile uint32_t g_UsbComboTaktOSHeartbeat = 0;

// Bound traffic work between yields while keeping the bare-metal per-byte
// PRBS Tx calls. Loopback may receive and echo in the same pass.
#define CDC_PASSES_PER_TURN 4U
#define USB_SERVICE_PASSES_PER_TURN 4U
#define USB_WORK_QUE_SIZE 16U
#define USB_WORK_PER_PASS 30U

// Deferred USB work: queued by the controller interrupt, run by ServiceThread.
typedef struct {
	uint32_t EvtId;
	void *pCtx;
	UsbEvtQueHandler_t Handler;
} UsbWork_t;

alignas(4) static uint8_t s_UsbWorkMem[
	CFIFO_TOTAL_MEMSIZE(USB_WORK_QUE_SIZE, sizeof(UsbWork_t))];
static hCFifo_t s_hUsbWork;
static TaktOSSem_t s_ServiceWake;

// Link-time override of the library default, see usb.h. USB work goes to the
// queue of the thread serving USB instead of the application event queue.
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
	// A full binary semaphore already records a wake for this consumer.
	(void)TaktOSSemGive(&s_ServiceWake, false);
	return true;
}

// Copy the head before releasing it: the interrupt can reuse the slot as soon
// as CFifoGet returns.
static bool UsbWorkGet(UsbWork_t *pWork)
{
	const UsbWork_t *p = (const UsbWork_t *)CFifoPeek(s_hUsbWork);
	if (p == nullptr)
	{
		return false;
	}
	*pWork = *p;
	(void)CFifoGet(s_hUsbWork);
	return true;
}

static void ServiceThread(void *pArg)
{
	(void)pArg;
	(void)UsbEnable(USB_DEVNO);
	while (true)
	{
		// Service completion bursts before paying for another scheduler round.
		// IRQs stay enabled; newly completed DMA can post work between passes.
		// This thread is the only consumer of the USB work queue.
		for (unsigned pass = 0; pass < USB_SERVICE_PASSES_PER_TURN; pass++)
		{
			UsbWork_t work;
			for (unsigned cnt = USB_WORK_PER_PASS;
				 cnt > 0U && UsbWorkGet(&work); cnt--)
			{
				work.Handler(work.EvtId, work.pCtx);
			}
		}
		TaktOSThreadYield();
		// Retain notifications arriving during processing. One tick bounds the
		// wait for any work left by a bounded drain.
		(void)TaktOSSemTake(&s_ServiceWake, true, 1U);
	}
}

static void LoopThread(void *pArg)
{
	(void)pArg;
	uint8_t buffer[CDC_BUFFER_SIZE];
	uint8_t expected = Prbs8(0xff);
	uint32_t errors = 0;
	int pending = 0;
	int offset = 0;
	bool wasOpen = false;
	while (true)
	{
		for (unsigned pass = 0; pass < CDC_PASSES_PER_TURN; pass++)
		{
			const bool open = g_LoopbackCdc.IsPortOpen();
			if (open != wasOpen)
			{
				wasOpen = open;
				pending = 0;
				offset = 0;
				if (open)
				{
					static const char banner[] = "\r\nIOsonata USB Combo Stress\r\n";
					static_assert(sizeof(banner) - 1 <= sizeof(buffer));
					memcpy(buffer, banner, sizeof(banner) - 1);
					pending = sizeof(banner) - 1;
					expected = Prbs8(0xff);
				}
			}
			if (!open)
			{
				break;
			}
			if (pending == 0)
			{
				const int n = g_LoopbackCdc.Rx(0, buffer, sizeof(buffer));
				if (n <= 0)
				{
					break;
				}
				for (int i = 0; i < n; i++)
				{
					if (buffer[i] != expected)
					{
						errors++;
					}
					expected = Prbs8(buffer[i]);
				}
				s_LoopErrors.store(errors, std::memory_order_relaxed);
				pending = n;
				offset = 0;
			}

			// Echo the packet just read in this turn. Waiting for another
			// pass halves the number of packets the turn can service.
			const int n = g_LoopbackCdc.Tx(0, buffer + offset, pending);
			if (n <= 0)
			{
				break;
			}
			offset += n;
			pending -= n;
			if (pending != 0)
			{
				// The TX FIFO filled: let the service thread retire work.
				break;
			}
		}
		TaktOSThreadYield();
	}
}

static void PrbsThread(void *pArg)
{
	(void)pArg;
	uint8_t prbs = 0xff;
	uint32_t reported = 0;
	while (true)
	{
		for (unsigned pass = 0; pass < CDC_PASSES_PER_TURN; pass++)
		{
			const bool error = reported != s_LoopErrors.load(std::memory_order_relaxed);
			const uint8_t byte = error ? 0U : prbs;
			if (!g_PrbsCdc.IsPortOpen() || g_PrbsCdc.Tx(0, &byte, 1) <= 0)
			{
				break;
			}
			if (error)
			{
				reported++;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
		TaktOSThreadYield();
	}
}

static void HeartbeatThread(void *pArg)
{
	(void)pArg;
	while (true)
	{
		g_UsbComboTaktOSHeartbeat = g_UsbComboTaktOSHeartbeat + 1;
		(void)TaktOSThreadSleep(TaktOSCurrentThread(), 1000);
	}
}

int main()
{
	// The work queue and its wake exist before USB can queue its first event.
	s_hUsbWork = CFifoInit(s_UsbWorkMem, sizeof(s_UsbWorkMem),
						   sizeof(UsbWork_t), true);
	if (s_hUsbWork == nullptr ||
		TaktOSSemInit(&s_ServiceWake, 0U, 1U) != TAKTOS_OK)
	{
		return -1;
	}
	// Initialization completes before any traffic thread can access a class.
	if (!UsbInit(&s_UsbCfg) ||
		!g_LoopbackCdc.Init(s_LoopbackCfg) || !g_PrbsCdc.Init(s_PrbsCfg) ||
		!g_Hid.Init(s_HidCfg) || !IntInit() || !IsoInit())
	{
		return -1;
	}
	const TaktOSCfg_t cfg = {
		.KernClockHz = SystemCoreClockGet(),
		.TickHz = 1000,
		.TickClockSrc = TAKTOS_TICK_CLOCK_PROCESSOR,
	};
	if (TaktOSInit(&cfg) != TAKTOS_OK ||
		TaktOSThreadCreate(s_ServiceMem, sizeof(s_ServiceMem),
			ServiceThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_LoopMem, sizeof(s_LoopMem),
			LoopThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_PrbsMem, sizeof(s_PrbsMem),
			PrbsThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_HeartbeatMem, sizeof(s_HeartbeatMem),
			HeartbeatThread, nullptr, TAKTOS_PRIORITY_NORMAL) == nullptr)
	{
		return -1;
	}
	TaktOSStart();
	while (true) {}
}

