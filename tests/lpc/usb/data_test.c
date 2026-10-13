/* LPC11Uxx bulk/interrupt regression. MIT License, I-SYST, 2026. */
#define LPC11_USB_EP0_FIXTURE_ONLY
#include "ep0_test.c"

typedef struct {
	unsigned Physical;
	unsigned Ready;
	unsigned Done;
	unsigned Failed;
	unsigned Cancelled;
	uint16_t LastLength;
	bool Rearm;
	bool Reopen;
	bool Stop;
	uint8_t Buffer[66];
} Owner_t;

static Owner_t s_Owner[10];

static void EndpointEvent(UsbCtrlrEvtType_t Event, uint16_t Length, void *pContext)
{
	Owner_t *owner = pContext;
	const uint8_t ep = owner->Physical / 2;
	const bool in = (owner->Physical & 1U) != 0U;
	assert((s_Memory->Command[2 * owner->Physical] & ACTIVE) == 0);
	owner->LastLength = Length;
	if (Event == USB_CTRLR_EVT_DRDY)
	{
		assert(!in);
		++owner->Ready;
		if (owner->Rearm)
			assert(UsbCtrlrEpReceive(0, ep, owner->Buffer + 1, 64));
	}
	if (Event == USB_CTRLR_EVT_XFER_CMPL)
	{
		++owner->Done;
		if (owner->Stop)
		{
			UsbCtrlrStop(0);
			return;
		}
		if (owner->Reopen)
		{
			owner->Reopen = false;
			UsbCtrlrEpClose(0, ep, in);
			assert(UsbCtrlrEpOpenData(0, ep, in, BULK, 64));
		}
		if (in && owner->Rearm)
			assert(UsbCtrlrEpSend(0, ep, owner->Buffer + 1, 64));
	}
	if (Event == USB_CTRLR_EVT_XFER_FAILED)
		++owner->Failed;
	if (Event == USB_CTRLR_EVT_CANCEL)
	{
		++owner->Cancelled;
		assert(!UsbCtrlrEpSend(0, ep, owner->Buffer, 1));
		assert(!UsbCtrlrEpReceive(0, ep, owner->Buffer, 64));
		assert(!UsbCtrlrEpOpenData(0, ep, in, BULK, 64));
		UsbCtrlrStop(0); // reentrant stop must not release twice
	}
}

static void OpenEndpoints(void)
{
	Fresh();
	memset(s_Owner, 0, sizeof(s_Owner));
	for (unsigned physical = 2; physical < 10; ++physical)
	{
		s_Owner[physical].Physical = physical;
		UsbCtrlrEpBind(0, physical / 2, (physical & 1) != 0, false, EndpointEvent, &s_Owner[physical]);
		assert(UsbCtrlrEpOpenData(0, physical / 2, (physical & 1) != 0, BULK, 64));
		assert(!UsbCtrlrEpOpenData(0, physical / 2, (physical & 1) != 0, BULK, 64));
	}
}

static void DataDone(unsigned Physical, unsigned Length)
{
	assert((s_Memory->Command[2 * Physical] & ACTIVE) != 0);
	const unsigned capacity = (s_Memory->Command[2 * Physical] >> 16) & 0x3FF;
	assert(Length <= capacity);
	s_Memory->Command[2 * Physical] &= ~(ACTIVE | (0x3FFU << 16));
	s_Memory->Command[2 * Physical] |= (capacity - Length) << 16;
	LPC_USB->INTSTAT |= 1U << Physical;
	USB_IRQHandler();
}

int main(void)
{
	(void)Setup;
	(void)InDone;
	(void)OutDone;
	assert(mmap((void *)LPC_SYSCON_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_USB_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	OpenEndpoints();
	assert(!UsbCtrlrEpOpenData(0, 0, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(0, 5, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(1, 1, true, BULK, 64));
	assert(!UsbCtrlrIsoInit(0) && !UsbCtrlrIsoOpen(0, 1, true, 64));
	assert(!UsbCtrlrIsoSend(0, 1, NULL, 0));
	uint8_t *trace = s_Data;
	assert(UsbCtrlrIsoTraceSnapshot(0, &trace) == 0 && trace == NULL);
	assert(!UsbCtrlrEpOpen(0, NULL));
	assert(!UsbCtrlrEpReceive(0, 1, s_Data, 63));
	assert(!UsbCtrlrEpSend(0, 1, s_Data, 65));
	USB_IRQHandler(); // deferred initial readiness, no hardware completion
	for (unsigned physical = 2; physical < 10; ++physical)
	{
		assert(s_Owner[physical].Ready == ((physical & 1U) == 0U));
		assert((s_Memory->Command[2 * physical] & ACTIVE) == 0);
	}
	for (unsigned physical = 2; physical < 10; ++physical)
	{
		Owner_t *owner = &s_Owner[physical];
		memset(owner->Buffer, 0xA5, sizeof(owner->Buffer));
		if ((physical & 1U) != 0U)
		{
			assert(UsbCtrlrEpSend(0, physical / 2, owner->Buffer + 1, 64));
			assert(memcmp(s_Memory->Packet[physical], owner->Buffer + 1, 64) == 0);
			assert(!UsbCtrlrEpSend(0, physical / 2, owner->Buffer, 1));
		}
		else
		{
			assert(UsbCtrlrEpReceive(0, physical / 2, owner->Buffer + 1, 64));
			memset(s_Memory->Packet[physical], physical, 64);
		}
	}
	for (unsigned physical = 2; physical < 10; ++physical)
	{
		Owner_t *owner = &s_Owner[physical];
		owner->Rearm = true;
		DataDone(physical, (physical & 1U) != 0U ? 64 : 7);
		assert(owner->Done == 1 && owner->Failed == 0);
		assert((s_Memory->Command[2 * physical] & ACTIVE) != 0);
		if ((physical & 1U) == 0U)
		{
			assert(owner->Buffer[0] == 0xA5 && owner->Buffer[65] == 0xA5);
			assert(owner->Buffer[1] == physical && owner->Buffer[7] == physical);
			assert(owner->Buffer[8] == 0xA5 && owner->Ready == 2);
		}
	}
	UsbCtrlrEpCloseAll(0);
	for (unsigned physical = 2; physical < 10; ++physical)
		assert(s_Owner[physical].Cancelled == 1);
	USB_IRQHandler(); // skip interrupts must not become completions
	for (unsigned physical = 2; physical < 10; ++physical)
		assert(s_Owner[physical].Done == 1);

	OpenEndpoints();
	assert(UsbCtrlrEpSend(0, 1, NULL, 0));
	DataDone(3, 0);
	assert(s_Owner[3].Done == 1 && s_Owner[3].LastLength == 0);
	assert(UsbCtrlrEpReceive(0, 1, s_Owner[2].Buffer + 1, 64));
	DataDone(2, 0);
	assert(s_Owner[2].Done == 1 && s_Owner[2].LastLength == 0);
	assert(UsbCtrlrEpReceive(0, 1, s_Owner[2].Buffer + 1, 64));
	s_Memory->Command[4] &= ~(ACTIVE | (0x3FFU << 16));
	s_Memory->Command[4] |= 65U << 16;
	LPC_USB->INTSTAT |= 4;
	USB_IRQHandler();
	assert(s_Owner[2].Failed == 1 && s_Owner[2].LastLength == 0);

	OpenEndpoints();
	assert(UsbCtrlrEpSend(0, 1, s_Data, 12));
	UsbCtrlrEpStall(0, 1, true);
	assert(s_Owner[3].Failed == 1 && s_Owner[3].Done == 0);
	assert((s_Memory->Command[6] & (ACTIVE | STALL)) == STALL);
	assert(UsbCtrlrEpSend(0, 1, s_Data, 8)); // queued behind HALT
	USB_IRQHandler();
	assert(s_Owner[3].Done == 0);
	UsbCtrlrEpClearStall(0, 1, true);
	assert((s_Memory->Command[6] & (ACTIVE | STALL | (1U << 28))) == (ACTIVE | (1U << 28)));
	DataDone(3, 8);
	assert(s_Owner[3].Done == 1);
	assert(UsbCtrlrEpSend(0, 1, s_Data, 8));
	s_Memory->Command[6] &= ~(ACTIVE | (0x3FFU << 16));
	LPC_USB->INTSTAT |= 8;
	UsbCtrlrEpStall(0, 1, true); // ACK completed before halt
	assert(s_Owner[3].Done == 2 && s_Owner[3].Failed == 1);

	OpenEndpoints();
	s_Owner[2].Reopen = true;
	s_Owner[2].Rearm = true;
	USB_IRQHandler();
	DataDone(2, 4);
	assert(s_Owner[2].Ready == 2 && s_Owner[2].Done == 1);
	assert((s_Memory->Command[4] & ACTIVE) != 0);
	assert(UsbCtrlrEpSend(0, 1, s_Data, 4));
	LPC_USB->DEVCMDSTAT |= RESET;
	LPC_USB->INTSTAT |= DEVICE | 12;
	USB_IRQHandler();
	assert(s_Owner[2].Cancelled == 1 && s_Owner[3].Cancelled == 1);
	assert(s_Events[s_EventCount - 1].Type == USB_CTRLR_EVT_RESET);
	assert(!UsbCtrlrEpSend(0, 1, s_Data, 4));
	assert(UsbCtrlrEpOpenData(0, 1, true, INT, 7));
	assert(UsbCtrlrEpSend(0, 1, s_Data, 7));
	DataDone(3, 7); // binding survives reset
	assert(s_Owner[3].Done == 1);

	OpenEndpoints();
	s_Owner[3].Stop = true;
	assert(UsbCtrlrEpSend(0, 1, s_Data, 8));
	assert(UsbCtrlrEpSend(0, 2, s_Data, 8));
	DataDone(3, 8);
	assert(s_Owner[3].Done == 1 && s_Owner[5].Cancelled == 1);
	assert(LPC_USB->INTEN == 0);

	OpenEndpoints();
	assert(UsbCtrlrEpSend(0, 1, s_Data, 8));
	s_AutoSkip = false;
	UsbCtrlrEpCloseAll(0);
	assert(s_Owner[3].Cancelled == 0 && (s_Memory->Command[6] & ACTIVE) != 0);
	assert(!UsbCtrlrEpSend(0, 1, s_Data, 1));
	s_AutoSkip = true;
	UsbCtrlrStop(0);
	assert(s_Owner[3].Cancelled == 1);
	puts("LPC11Uxx bulk/interrupt ownership, callbacks, halt and reset: PASS");
	return 0;
}
