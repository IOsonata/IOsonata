#include <atomic>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <thread>

#include "nrf.h"
#include "usb/usbd_ctrlr.h"

TestUsbd_t g_TestUsbd;
volatile bool g_UsbdIrqEnabled;
volatile bool g_UsbdIrqPending;

extern "C" void USBD_IRQHandler(void);

static UsbdCtrlrEvt_t s_LastEvent;
static unsigned int s_EventCount;

static void TestEvent(const UsbdCtrlrEvt_t *pEvt, void *)
{
	s_LastEvent = *pEvt;
	s_EventCount++;
}

static bool Check(bool Condition, const char *pMessage)
{
	if (Condition)
	{
		return true;
	}

	fprintf(stderr, "FAIL: %s\n", pMessage);
	return false;
}

int main(void)
{
	memset(&g_TestUsbd, 0, sizeof(g_TestUsbd));
	g_UsbdIrqEnabled = false;
	g_UsbdIrqPending = false;
	s_EventCount = 0;

	if (!Check(UsbdCtrlrInit(TestEvent, nullptr), "controller init"))
	{
		return 1;
	}

	UsbEndPointDesc_t desc = {};
	desc.bEndpointAddress = 0x81U;
	desc.bmAttributes = USB_ENDPATT_TRANS_BULK;
	desc.wMaxPacketSize = 64U;
	if (!Check(UsbdCtrlrEpOpen(&desc), "open bulk IN endpoint"))
	{
		return 1;
	}

	const uint32_t endIn1Mask =
		1UL << (USBD_INTEN_ENDEPIN0_Pos + 1U);
	NRF_USBD->INTEN = endIn1Mask | USBD_INTEN_EPDATA_Msk;

	uint8_t buffer[64] = {};
	if (!Check(UsbdCtrlrEpXfer(0x81U, buffer, sizeof(buffer)),
			   "queue bulk IN transfer") ||
		!Check(g_UsbdIrqPending, "queue pends the USBD interrupt") ||
		!Check(NRF_USBD->TASKS_STARTEPIN[1] == 0U,
			   "queue does not start EasyDMA from application context"))
	{
		return 1;
	}

	std::atomic<bool> irqDone(false);
	std::thread irq([&irqDone]() {
		USBD_IRQHandler();
		irqDone.store(true, std::memory_order_release);
	});

	while (__atomic_load_n(&NRF_USBD->TASKS_STARTEPIN[1],
						 __ATOMIC_ACQUIRE) == 0U)
	{
		std::this_thread::yield();
	}

	if (!Check(!irqDone.load(std::memory_order_acquire),
			   "USBD interrupt waits for EasyDMA completion"))
	{
		__atomic_store_n(&NRF_USBD->EVENTS_ENDEPIN[1], 1U,
						 __ATOMIC_RELEASE);
		irq.join();
		return 1;
	}

	__atomic_store_n(&NRF_USBD->EVENTS_ENDEPIN[1], 1U,
					 __ATOMIC_RELEASE);
	irq.join();

	if (!Check(irqDone.load(std::memory_order_acquire),
			   "USBD interrupt resumes after ENDEPIN") ||
		!Check(UsbdCtrlrEpBusy(0x81U),
			   "IN transfer remains active until host acknowledgement"))
	{
		return 1;
	}

	// The next IRQ pass consumes ENDEPIN and releases the shared DMA engine.
	USBD_IRQHandler();

	NRF_USBD->EPIN[1].AMOUNT = sizeof(buffer);
	NRF_USBD->EPDATASTATUS = 1UL << 1U;
	NRF_USBD->EVENTS[USBD_INTEN_EPDATA_Pos] = 1U;
	USBD_IRQHandler();

	if (!Check(s_EventCount == 1U, "one transfer completion event") ||
		!Check(s_LastEvent.Type == USBD_CTRLR_EVT_XFER_CMPL,
			   "transfer completion event type") ||
		!Check(s_LastEvent.Xfer.EpAddr == 0x81U,
			   "transfer completion endpoint") ||
		!Check(s_LastEvent.Xfer.Length == sizeof(buffer),
			   "transfer completion length") ||
		!Check(!UsbdCtrlrEpBusy(0x81U), "bulk IN endpoint becomes idle"))
	{
		return 1;
	}

	puts("nRF52 USBD scheduler: 1/1 passed");
	return 0;
}
