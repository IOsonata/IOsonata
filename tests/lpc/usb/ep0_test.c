/* LPC11Uxx EP0 transport regression. MIT License, I-SYST, 2026. */
// Reuse the mapped vendor registers, clock fixture and NVIC recorder.
#define main HardwareRegressionMain
#include "hardware_test.c"
#undef main
#include "usb/usb.h"
#include "ep0_model.h"

#define ACTIVE (1U << 31)
#define STALL (1U << 29)
#define SETUP (1U << 8)
#define DEVICE (1U << 31)
#define RESET (1U << 26)

typedef enum {
	NONE, REPLY_IN, RECEIVE_OUT, ADDRESS, REJECT, STOP_ON_DONE,
} Action_t;

static Action_t s_Action;
static UsbCtrlrEvt_t s_Events[32];
static uint8_t s_Data[128];
static unsigned s_EventCount;
static unsigned s_Queued;
static unsigned s_RemoteWake;
static unsigned s_Received;
static unsigned s_Sent;
static unsigned s_ReplyLength;
static unsigned s_ExpectedOut;
static bool s_AutoSkip;
static bool s_CheckSetupOrder;
static bool s_NeedZlp;
static bool s_ZlpSent;
static bool s_InjectSetupOnAck;
static bool s_RestartOnSuspend;

void TestDeviceWrite(uint32_t Value)
{
	const uint32_t old = LPC_USB->DEVCMDSTAT;
	const uint32_t commands = 0x1FAFFU;
	const uint32_t w1c = SETUP | (7U << 24);
	if ((old & (1U << 16)) != 0U && (old & (1U << 17)) != 0U &&
		(Value & (1U << 17)) == 0U)
		++s_RemoteWake;
	if (s_CheckSetupOrder && (old & Value & SETUP) != 0U)
	{
		assert((s_Memory->Command[0] & (ACTIVE | STALL)) == 0U);
		assert((s_Memory->Command[2] & (ACTIVE | STALL)) == 0U);
	}
	LPC_USB->DEVCMDSTAT = (old & ~(commands | w1c)) |
		(Value & commands) | (old & w1c & ~Value);
}

void TestInterruptWrite(uint32_t Value)
{
	LPC_USB->INTSTAT &= ~Value;
	if (s_InjectSetupOnAck && (Value & 1U) != 0U)
	{
		s_InjectSetupOnAck = false;
		UsbSetupData_t setup = {0};
		memcpy(s_Memory->Setup, &setup, sizeof(setup));
		LPC_USB->DEVCMDSTAT |= SETUP;
		LPC_USB->INTSTAT |= 1U;
	}
}

void TestInterruptSet(uint32_t Value)
{
	LPC_USB->INTSTAT |= Value;
}

void TestSkipWrite(uint32_t Value)
{
	LPC_USB->EPSKIP = Value;
	if (s_AutoSkip)
	{
		for (unsigned physical = 0U; physical < 10U; ++physical)
		{
			if ((Value & (1U << physical)) != 0U)
				s_Memory->Command[physical * 2U] &= ~ACTIVE;
		}
		LPC_USB->EPSKIP = 0U;
		LPC_USB->INTSTAT |= Value;
	}
}

void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
	++s_Queued;
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvt)
{
	assert(DevNo == 0 && s_EventCount < 32);
	s_Events[s_EventCount++] = *pEvt;
	if (pEvt->Type == USB_CTRLR_EVT_SUSPEND && s_RestartOnSuspend)
	{
		assert(Lpc11UsbHwRelease());
		s_Memory = Lpc11UsbHwPrepare(6);
		assert(s_Memory != NULL && Lpc11UsbEp0Enable());
		return;
	}
	if (pEvt->Type == USB_CTRLR_EVT_SETUP)
	{
		if (s_Action == REPLY_IN)
		{
			const int sent = UsbCtrlrEp0Send(0, s_Data, s_ReplyLength);
			assert(sent >= 0);
			s_Sent = sent;
		}
		if (s_Action == ADDRESS)
		{
			UsbCtrlrSetAddress(0, 7);
			assert((LPC_USB->DEVCMDSTAT & 127U) == 7U);
			assert(UsbCtrlrEp0Status(0, 0x80));
		}
		if (s_Action == REJECT)
			UsbCtrlrEpStall(0, 0, false);
	}
	else if (pEvt->Type == USB_CTRLR_EVT_XFER_CMPL)
	{
		if (s_Action == STOP_ON_DONE)
		{
			assert(Lpc11UsbHwRelease());
			return;
		}
		if (pEvt->Xfer.Result != USB_CTRLR_XFER_SUCCESS)
		{
			UsbCtrlrEpStall(0, 0, false);
			return;
		}
		if (s_Action == REPLY_IN && pEvt->Xfer.EpAddr == 0x80)
		{
			if (s_Sent < s_ReplyLength)
			{
				const int sent = UsbCtrlrEp0Send(0, s_Data + s_Sent, s_ReplyLength - s_Sent);
				assert(sent > 0);
				s_Sent += sent;
			}
			else if (s_NeedZlp && !s_ZlpSent)
			{
				s_ZlpSent = true;
				assert(UsbCtrlrEp0Send(0, NULL, 0) == 0);
			}
			else
				assert(UsbCtrlrEp0Status(0, 0));
		}
		if (s_Action == RECEIVE_OUT && pEvt->Xfer.EpAddr == 0)
		{
			assert(pEvt->Xfer.pBuffer == s_Memory->Packet[0]);
			for (unsigned byte = 0U; byte < pEvt->Xfer.Length; ++byte)
				assert(pEvt->Xfer.pBuffer[byte] == (uint8_t)(s_Received + byte));
			s_Received += pEvt->Xfer.Length;
			if (s_Received == s_ExpectedOut || pEvt->Xfer.Length < 64U)
				assert(UsbCtrlrEp0Status(0, 0x80));
		}
	}
}

static void Setup(Action_t Action, bool In, uint16_t Length)
{
	UsbSetupData_t setup = {0};
	setup.bmRequestType = In ? 0x80U : 0U;
	setup.wLength = Length;
	s_Action = Action;
	memcpy(s_Memory->Setup, &setup, sizeof(setup));
	LPC_USB->DEVCMDSTAT |= SETUP;
	LPC_USB->INTSTAT |= 1U;
	USB_IRQHandler();
	assert(s_Events[s_EventCount - 1].Type == USB_CTRLR_EVT_SETUP);
}

static void InDone(void)
{
	assert((s_Memory->Command[2] & ACTIVE) != 0U);
	s_Memory->Command[2] &= ~(ACTIVE | (0x3FFU << 16));
	LPC_USB->INTSTAT |= 2U;
	USB_IRQHandler();
}

static void OutDone(unsigned Length)
{
	assert((s_Memory->Command[0] & ACTIVE) != 0U);
	const unsigned capacity = (s_Memory->Command[0] >> 16) & 0x3FFU;
	assert(Length <= capacity);
	for (unsigned byte = 0U; byte < Length; ++byte)
		s_Memory->Packet[0][byte] = s_Received + byte;
	s_Memory->Command[0] &= ~(ACTIVE | (0x3FFU << 16));
	s_Memory->Command[0] |= (capacity - Length) << 16;
	LPC_USB->INTSTAT |= 1U;
	USB_IRQHandler();
}

static void Fresh(void)
{
	s_AutoSkip = true;
	s_CheckSetupOrder = false;
	assert(Lpc11UsbHwRelease());
	Defaults();
	s_Memory = Lpc11UsbHwPrepare(6);
	assert(s_Memory != NULL);
	assert(Lpc11UsbEp0Enable());
	assert(!Lpc11UsbEp0Enable());
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) == 0U);
	s_CheckSetupOrder = true;
	s_EventCount = 0U;
	s_Queued = 0U;
	s_Received = 0U;
	s_Sent = 0U;
	s_RemoteWake = 0U;
	s_NeedZlp = false;
	s_ZlpSent = false;
	s_InjectSetupOnAck = false;
	s_RestartOnSuspend = false;
	for (unsigned byte = 0U; byte < sizeof(s_Data); ++byte)
		s_Data[byte] = byte;
}

#ifndef LPC11_USB_EP0_FIXTURE_ONLY
int main(void)
{
	assert(mmap((void *)LPC_SYSCON_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_USB_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(!Lpc11UsbEp0Enable());
	assert(UsbCtrlrEp0Send(0, NULL, 0) == -1);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	assert(s_Sent == 64 && memcmp(s_Memory->Packet[1], s_Data, 64) == 0);
	assert(UsbCtrlrEp0Send(0, s_Data, 1) == -1); // still owned
	memset(s_Data, 0xFF, 64); // EP0 copied before returning
	assert(s_Memory->Packet[1][1] == 1);
	InDone();
	assert(s_Sent == 80 && s_Memory->Packet[1][0] == 64);
	InDone();
	assert((s_Memory->Command[0] & ACTIVE) != 0U);
	OutDone(0);
	assert(s_EventCount == 4);

	Fresh();
	s_ReplyLength = 64;
	s_NeedZlp = true;
	Setup(REPLY_IN, true, 128);
	InDone();
	assert(s_ZlpSent && ((s_Memory->Command[2] >> 16) & 0x3FFU) == 0U);
	InDone();
	OutDone(0);
	assert(s_EventCount == 4);

	Fresh();
	s_ExpectedOut = 80;
	Setup(RECEIVE_OUT, false, 80);
	OutDone(64);
	// Host IN retry may NAK just before software sees the last OUT done.
	LPC_USB->INTSTAT |= 2U;
	OutDone(16);
	assert(s_Received == 80 && (s_Memory->Command[2] & ACTIVE) != 0U);
	InDone();
	assert(s_EventCount == 4);

	Fresh();
	s_ExpectedOut = 80;
	Setup(RECEIVE_OUT, false, 80);
	OutDone(7); // short OUT terminates data phase
	InDone();
	assert(s_Received == 7);

	Fresh();
	Setup(ADDRESS, false, 0);
	assert((LPC_USB->DEVCMDSTAT & 127U) == 7U);
	Setup(NONE, false, 0); // supersedes unacknowledged SET_ADDRESS
	assert((LPC_USB->DEVCMDSTAT & 127U) == 0U);
	Setup(ADDRESS, false, 0);
	InDone();
	Setup(NONE, false, 0);
	assert((LPC_USB->DEVCMDSTAT & 127U) == 7U);

	Fresh();
	Setup(ADDRESS, false, 0);
	s_Memory->Command[2] &= ~ACTIVE; // status ACK, IRQ not serviced yet
	LPC_USB->INTSTAT |= 2U;
	Setup(NONE, false, 0);
	assert(s_EventCount == 3);
	assert(s_Events[1].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert(s_Events[2].Type == USB_CTRLR_EVT_SETUP);
	assert((LPC_USB->DEVCMDSTAT & 127U) == 7U);

	// Ordinary no-data writes need the same ACK ordering as SET_ADDRESS.
	Fresh();
	Setup(NONE, false, 0);
	assert(UsbCtrlrEp0Status(0, 0x80));
	s_Memory->Command[2] &= ~ACTIVE;
	LPC_USB->INTSTAT |= 2U;
	Setup(NONE, false, 0);
	assert(s_EventCount == 3);
	assert(s_Events[1].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert(s_Events[1].Xfer.Result == USB_CTRLR_XFER_SUCCESS);

	// A latched IN interrupt with ACTIVE still set is not a status ACK.
	Fresh();
	Setup(NONE, false, 0);
	assert(UsbCtrlrEp0Status(0, 0x80));
	LPC_USB->INTSTAT |= 2U;
	Setup(NONE, false, 0);
	assert(s_EventCount == 2);

	// A completed data-stage ZLP must not commit a control-read request.
	Fresh();
	s_ReplyLength = 0;
	Setup(REPLY_IN, true, 64);
	s_Memory->Command[2] &= ~ACTIVE;
	LPC_USB->INTSTAT |= 2U;
	Setup(NONE, false, 0);
	assert(s_EventCount == 2);

	// Preserve cancellation when the host ended a control write early.
	Fresh();
	Setup(RECEIVE_OUT, false, 80);
	LPC_USB->INTSTAT |= 2U;
	USB_IRQHandler();
	s_Memory->Command[2] &= ~ACTIVE;
	LPC_USB->INTSTAT |= 2U;
	Setup(NONE, false, 0);
	assert(s_EventCount == 3);
	assert(s_Events[1].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert(s_Events[1].Xfer.Result == USB_CTRLR_XFER_CANCELLED);

	Fresh();
	Setup(REJECT, false, 0);
	assert((s_Memory->Command[0] & STALL) != 0U);
	assert((s_Memory->Command[2] & STALL) != 0U);
	assert(UsbCtrlrEp0Send(0, NULL, 0) == -1);
	Setup(NONE, false, 0);
	assert((s_Memory->Command[0] & (ACTIVE | STALL)) == 0U);
	assert(UsbCtrlrEp0Status(0, 0x80));
	InDone();
	assert(!UsbCtrlrEp0Status(1, 0));
	assert(!UsbCtrlrEp0Status(0, 1));
	assert(UsbCtrlrEp0Send(0, NULL, 1) == -1);
	assert(UsbCtrlrEp0Send(0, s_Data, -1) == -1);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	LPC_USB->INTSTAT |= 2U; // old IN completion alongside a new SETUP
	Setup(NONE, false, 0);
	assert(s_EventCount == 2);
	assert((s_Memory->Command[2] & ACTIVE) == 0U);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	LPC_USB->INTSTAT |= 1U; // early status OUT NAK
	USB_IRQHandler();
	assert((s_Memory->Command[2] & ACTIVE) == 0U);
	OutDone(0);
	assert(s_Events[1].Xfer.Result == USB_CTRLR_XFER_CANCELLED);

	Fresh();
	Setup(RECEIVE_OUT, false, 80);
	LPC_USB->INTSTAT |= 2U; // early status IN NAK
	USB_IRQHandler();
	assert((s_Memory->Command[0] & ACTIVE) == 0U);
	InDone();
	assert(s_Events[1].Xfer.Result == USB_CTRLR_XFER_CANCELLED);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	LPC_USB->DEVCMDSTAT |= RESET;
	LPC_USB->INTSTAT |= DEVICE | 3U;
	USB_IRQHandler();
	assert(s_EventCount == 2 && s_Events[1].Type == USB_CTRLR_EVT_RESET);
	assert((s_Memory->Command[2] & ACTIVE) == 0U);
	assert((LPC_USB->DEVCMDSTAT & (RESET | 127U)) == 0U);
	for (unsigned physical = 2; physical < 10; ++physical)
		assert(s_Memory->Command[physical * 2] == (1U << 30));

	Fresh();
	LPC_USB->DEVCMDSTAT |= (1U << 17) | (1U << 25) | (1U << 16);
	LPC_USB->INTSTAT |= DEVICE;
	USB_IRQHandler();
	assert(s_Events[0].Type == USB_CTRLR_EVT_SUSPEND && s_RemoteWake == 0);
	LPC_USB->DEVCMDSTAT &= ~(1U << 17);
	LPC_USB->DEVCMDSTAT |= 1U << 25;
	LPC_USB->INTSTAT |= DEVICE;
	USB_IRQHandler();
	assert(s_Events[1].Type == USB_CTRLR_EVT_RESUME);
	LPC_USB->DEVCMDSTAT |= 1U << 24;
	LPC_USB->INTSTAT |= DEVICE;
	USB_IRQHandler();
	assert(s_Queued == 1);

	Fresh();
	Setup(NONE, true, 64);
	assert(UsbCtrlrEp0Send(0, s_Data, 64) == 64);
	s_Action = STOP_ON_DONE;
	InDone();
	assert(LPC_USB->INTEN == 0);
	USB_IRQHandler(); // must not access clock-gated hardware
	assert(s_EventCount == 2);

	Fresh();
	s_ExpectedOut = 64;
	Setup(RECEIVE_OUT, false, 64);
	s_Action = NONE;
	s_InjectSetupOnAck = true;
	OutDone(64); // New SETUP races the shared OUT interrupt acknowledgment.
	assert(s_EventCount == 2 && s_Events[1].Type == USB_CTRLR_EVT_SETUP);
	assert((s_Memory->Command[0] & ACTIVE) == 0U);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	s_RestartOnSuspend = true;
	LPC_USB->DEVCMDSTAT |= (1U << 17) | (1U << 25);
	LPC_USB->INTSTAT |= DEVICE | 3U;
	USB_IRQHandler();
	assert(s_EventCount == 2 && s_Events[1].Type == USB_CTRLR_EVT_SUSPEND);
	assert((s_Memory->Command[0] & ACTIVE) == 0U);
	assert((s_Memory->Command[2] & ACTIVE) == 0U);

	Fresh();
	Setup(RECEIVE_OUT, false, 7);
	OutDone(8); // Malformed host length must not be reported as success.
	assert(s_Events[1].Xfer.Result == USB_CTRLR_XFER_CANCELLED);
	assert(s_Received == 0U);

	Fresh();
	s_ReplyLength = 80;
	Setup(REPLY_IN, true, 80);
	s_AutoSkip = false;
	UsbCtrlrEpStall(0, 0, false); // bounded skip failure
	assert(LPC_USB->INTEN == 0);
	assert((s_Memory->Command[2] & ACTIVE) != 0U);
	assert(UsbCtrlrEp0Send(0, s_Data, 1) == -1);
	assert(!Lpc11UsbHwRelease());
	s_AutoSkip = true;
	assert(Lpc11UsbHwRelease());
	puts("LPC11Uxx EP0 and USB IRQ model: PASS");
	return 0;
}
#endif
