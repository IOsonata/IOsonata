/* LPC546xx EP0 and IRQ regression. MIT License, I-SYST, 2026. */
#define main HardwareRegressionMain
#include "hardware546_test.c"
#undef main
#include "usb_control_lpc546xx.h"

#define ACTIVE (1U << 31)
#define DEVICE (1U << 31)
#define SETUP (1U << 8)
#define RESET (1U << 26)

static LpcUsbFsMemory_t *s_Memory;
static UsbCtrlrEvt_t s_Events[32];
static unsigned s_Count;
static bool s_AutoSkip = true;
static bool s_Restart;
static bool s_Stop;

void TestDeviceWrite(uint32_t Value)
{
	const uint32_t old = USB0->DEVCMDSTAT;
	const uint32_t commands = 0x1FAFFU;
	const uint32_t w1c = SETUP | (7U << 24);
	if ((old & Value & SETUP) != 0U && s_Memory != NULL)
	{
		assert((s_Memory->Command[0] & (ACTIVE | (1U << 29))) == 0U);
		assert((s_Memory->Command[2] & (ACTIVE | (1U << 29))) == 0U);
	}
	USB0->DEVCMDSTAT = (old & ~(commands | w1c)) |
		(Value & commands) | (old & w1c & ~Value);
}

void TestInterruptWrite(uint32_t Value)
{
	USB0->INTSTAT &= ~Value;
}

void TestInterruptSet(uint32_t Value)
{
	USB0->INTSTAT |= Value;
}

void TestSkipWrite(uint32_t Value)
{
	USB0->EPSKIP = Value;
	if (s_AutoSkip)
	{
		for (unsigned physical = 0; physical < 10; physical++)
		{
			if (Value & (1U << physical))
				s_Memory->Command[physical * 2] &= ~ACTIVE;
		}
		USB0->EPSKIP = 0;
		USB0->INTSTAT |= Value;
	}
}

static void Event(const UsbCtrlrEvt_t *pEvent)
{
	assert(s_Count < 32);
	s_Events[s_Count++] = *pEvent;
	if (s_Restart || s_Stop)
	{
		assert(Lpc546xxUsbControlRelease());
		if (s_Restart)
		{
			assert(Lpc546xxUsbControlPrepare(6, Event));
			s_Memory = (LpcUsbFsMemory_t *)(uintptr_t)USB0->EPLISTSTART;
		}
	}
}

static void Fresh(void)
{
	assert(Lpc546xxUsbControlRelease());
	Reset();
	s_Memory = NULL;
	assert(Lpc546xxUsbControlPrepare(6, Event));
	assert(!Lpc546xxUsbControlPrepare(6, Event));
	s_Memory = (LpcUsbFsMemory_t *)(uintptr_t)USB0->EPLISTSTART;
	assert(!s_Enabled && (USB0->DEVCMDSTAT & (1U << 16)) == 0);
	s_Count = 0;
	s_Restart = false;
	s_Stop = false;
}

static void Setup(bool In, uint16_t Length)
{
	UsbSetupData_t setup = {0};
	setup.bmRequestType = In ? 0x80 : 0;
	setup.wLength = Length;
	memcpy(s_Memory->Setup, &setup, sizeof(setup));
	USB0->DEVCMDSTAT |= SETUP;
	USB0->INTSTAT |= 1U;
	USB0_IRQHandler();
	assert(s_Events[s_Count - 1].Type == USB_CTRLR_EVT_SETUP);
}

static void Done(bool In, unsigned Remaining)
{
	unsigned physical = In ? 1 : 0;
	assert(s_Memory->Command[physical * 2] & ACTIVE);
	s_Memory->Command[physical * 2] &= ~(ACTIVE | (0x3FFU << 16));
	s_Memory->Command[physical * 2] |= Remaining << 16;
	USB0->INTSTAT |= 1U << physical;
	USB0_IRQHandler();
}

int main(void)
{
	Map(SYSCON_BASE);
	Map(USB0_BASE);
	Map(USBFSH_BASE);
	Reset();
	assert(!Lpc546xxUsbControlPrepare(6, NULL));
	assert(UsbCtrlrEp0Send(0, NULL, 0) == -1);
	USB0_IRQHandler();
	Fresh();
	uint8_t bytes[80] = {1, 2, 3};
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	assert(memcmp(bytes, s_Memory->Packet[1], 64) == 0);
	assert(UsbCtrlrEp0Send(0, bytes, 1) == -1);
	Done(true, 0);
	assert(s_Events[1].Xfer.Length == 64);
	assert(UsbCtrlrEp0Send(0, bytes + 64, 16) == 16);
	Done(true, 0);
	assert(UsbCtrlrEp0Status(0, 0));
	Done(false, 0);
	assert(s_Count == 4 && s_Events[3].Xfer.EpAddr == 0);

	Fresh();
	Setup(false, 80);
	Done(false, 0);
	assert(s_Events[1].Xfer.Length == 64);
	Done(false, 48);
	assert(s_Events[2].Xfer.Length == 16);
	assert(UsbCtrlrEp0Status(0, 0x80));
	Done(true, 0);
	assert(s_Count == 4);

	Fresh();
	Setup(false, 0);
	UsbCtrlrSetAddress(0, 7);
	assert(UsbCtrlrEp0Status(0, 0x80));
	Setup(false, 0);
	assert((USB0->DEVCMDSTAT & 127U) == 0);
	UsbCtrlrSetAddress(0, 9);
	assert(UsbCtrlrEp0Status(0, 0x80));
	s_Memory->Command[2] &= ~ACTIVE;
	USB0->INTSTAT |= 2U;
	Setup(false, 0);
	assert(s_Events[2].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert((USB0->DEVCMDSTAT & 127U) == 9);
	UsbCtrlrEpStall(0, 0, false);
	assert(s_Memory->Command[0] & (1U << 29));
	assert(UsbCtrlrEp0Send(0, NULL, 0) == -1);
	Setup(false, 0);
	assert((s_Memory->Command[0] & (1U << 29)) == 0);

	// Bus reset wins over stale completions, preserving a subsequent SETUP.
	Fresh();
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	USB0->DEVCMDSTAT |= RESET | SETUP;
	USB0->INTSTAT |= DEVICE | 3U;
	USB0_IRQHandler();
	assert(s_Count == 2 && s_Events[1].Type == USB_CTRLR_EVT_RESET);
	assert(USB0->DEVCMDSTAT & SETUP);
	assert(USB0->INTSTAT & 1U);
	USB0_IRQHandler();
	assert(s_Count == 3 && s_Events[2].Type == USB_CTRLR_EVT_SETUP);

	Fresh();
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	USB0->DEVCMDSTAT |= 1U << 24;
	USB0->INTSTAT |= DEVICE | 2U;
	USB0_IRQHandler();
	assert(s_Count == 2 && s_Events[1].Type == USB_CTRLR_EVT_RESET);
	assert((s_Memory->Command[2] & ACTIVE) == 0);

	Fresh();
	USB0->DEVCMDSTAT |= (1U << 17) | (1U << 25);
	USB0->INTSTAT |= DEVICE;
	USB0_IRQHandler();
	assert(s_Events[0].Type == USB_CTRLR_EVT_SUSPEND);
	USB0->DEVCMDSTAT &= ~(1U << 17);
	USB0->DEVCMDSTAT |= 1U << 25;
	USB0->INTSTAT |= DEVICE;
	USB0_IRQHandler();
	assert(s_Events[1].Type == USB_CTRLR_EVT_RESUME);
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	s_Restart = true;
	USB0->DEVCMDSTAT |= (1U << 17) | (1U << 25);
	USB0->INTSTAT |= DEVICE | 3U;
	USB0_IRQHandler();
	assert(s_Count == 4 && s_Events[3].Type == USB_CTRLR_EVT_SUSPEND);
	assert((s_Memory->Command[0] & ACTIVE) == 0);

	Fresh();
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	s_Stop = true;
	Done(true, 0);
	assert(USB0->INTEN == 0);
	USB0_IRQHandler();
	assert(s_Count == 2);

	Fresh();
	Setup(true, 80);
	assert(UsbCtrlrEp0Send(0, bytes, 80) == 64);
	s_AutoSkip = false;
	UsbCtrlrEpStall(0, 0, false);
	assert(USB0->INTEN == 0 && !s_Enabled);
	assert(s_Memory->Command[2] & ACTIVE);
	assert(UsbCtrlrEp0Send(0, bytes, 1) == -1);
	assert(!Lpc546xxUsbControlRelease());
	assert(!Lpc546xxUsbControlPrepare(6, Event));
	s_AutoSkip = true;
	assert(Lpc546xxUsbControlRelease());
	assert(g_IrqMasked == 0);
	puts("LPC546xx EP0, reset, IRQ ordering and DMA ownership: PASS");
	return 0;
}
