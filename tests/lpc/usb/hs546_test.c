/* LPC54628 USB1 high-speed regression. MIT License, I-SYST, 2026. */
#define TEST_USB ((USB_Type *)USBHSD)
#define TEST_USB_DEVNO 1
#define TEST_USB_PHYSICAL 12U
// The shared command bits, FORCE_VBUS and the PHY test mode
#define TEST_USB_COMMANDS 0xE001FEFFU
#define TEST_USB_IRQ USB1_IRQn
#define TEST_USB_IRQHandler USB1_IRQHandler
#define TEST_VBUS_PORT 1
#define TEST_VBUS_PIN 5
#define TEST_VBUS_OP IOPINOP_GPIO
#include "integration546_model.c"

#define CONNECT (1U << 16)
#define CONNECT_CHANGE (1U << 24)
#define VBUS_DEBOUNCED (1U << 28)
#define NAK_IN (1U << 13)
#define TEST_MODE (7U << 29)
#define FORCE_ENABLE (5U << 29)
#define HS_LENGTH(Word) (((Word) >> 11) & 0x7FFFU)

typedef struct {
	unsigned Physical;
	unsigned Ready;
	unsigned Done;
	unsigned Failed;
	unsigned Cancelled;
	uint16_t Last;
	uint8_t Buffer[512];
} HsOwner_t;

static unsigned s_Queued;
static HsOwner_t s_Owner[12];
static uint8_t s_Bytes[1024];
static unsigned s_ChirpPhase;
static bool s_ChirpHostEvents;
static bool s_ChirpVbusLoss;

void UsbProcessQue(int DevNo)
{
	assert(DevNo == 1);
	++s_Queued;
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvent)
{
	assert(DevNo == 1);
	assert(s_Count < 32);
	s_Events[s_Count++] = *pEvent;
}

static Lpc546xxUsbHsMemory_t *Hs(void)
{
	return (Lpc546xxUsbHsMemory_t *)(uintptr_t)USBHSD->EPLISTSTART;
}

static void HsEndpoint(UsbCtrlrEvtType_t Event, uint16_t Length, void *pContext)
{
	HsOwner_t *owner = pContext;
	switch (Event)
	{
		case USB_CTRLR_EVT_DRDY:
			++owner->Ready;
			break;
		case USB_CTRLR_EVT_XFER_CMPL:
			owner->Last = Length;
			++owner->Done;
			break;
		case USB_CTRLR_EVT_XFER_FAILED:
			++owner->Failed;
			break;
		case USB_CTRLR_EVT_CANCEL:
			++owner->Cancelled;
			break;
		default:
			assert(false);
	}
}

static void HsBusReset(uint32_t Speed)
{
	USBHSD->DEVCMDSTAT = (USBHSD->DEVCMDSTAT & ~(3U << 22)) | (Speed << 22) | RESET;
	USBHSD->INTSTAT |= DEVICE;
	USB1_IRQHandler();
	assert(s_Events[s_Count - 1].Type == USB_CTRLR_EVT_RESET);
	assert((USBHSD->DEVCMDSTAT & RESET) == 0);
}

static void HsSetup(bool In, uint16_t Length)
{
	UsbSetupData_t setup = {0};
	setup.bmRequestType = In ? 0x80 : 0;
	setup.wLength = Length;
	memcpy(Hs()->Setup, &setup, sizeof(setup));
	USBHSD->DEVCMDSTAT |= SETUP;
	USBHSD->INTSTAT |= 1U;
	USB1_IRQHandler();
	assert(s_Events[s_Count - 1].Type == USB_CTRLR_EVT_SETUP);
}

// The controller moved Length bytes of the armed packet; Remaining is the
// byte count it leaves in the list word.
static void HsMoved(unsigned Physical, unsigned Remaining)
{
	const uint32_t word = Hs()->Command[2 * Physical];
	assert(word & ACTIVE);
	Hs()->Command[2 * Physical] = (word & ~(ACTIVE | (0x7FFFU << 11))) | (Remaining << 11);
	USBHSD->INTSTAT |= 1U << Physical;
	USB1_IRQHandler();
}

static void HsDone(unsigned Physical, unsigned Length)
{
	const unsigned capacity = HS_LENGTH(Hs()->Command[2 * Physical]);
	assert(Length <= capacity);
	HsMoved(Physical, capacity - Length);
}

// An IN token NAKed on a data endpoint
static void HsNak(unsigned Physical)
{
	assert((Hs()->Command[2 * Physical] & ACTIVE) == 0);
	USBHSD->INTSTAT |= 1U << Physical;
	USB1_IRQHandler();
}

static void HsBind(uint8_t EpNo, bool In)
{
	const unsigned physical = 2U * EpNo + In;
	memset(&s_Owner[physical], 0, sizeof(s_Owner[physical]));
	s_Owner[physical].Physical = physical;
	UsbCtrlrEpBind(1, EpNo, In, false, HsEndpoint, &s_Owner[physical]);
}

// EP1 OUT receives Length bytes
static void HsOut(unsigned Length)
{
	HsOwner_t *owner = &s_Owner[2];
	assert(UsbCtrlrEpReceive(1, 1, owner->Buffer, sizeof(owner->Buffer)));
	assert(HS_LENGTH(Hs()->Command[4]) == 512);
	memset(Hs()->Packet[0], (int)Length, Length);
	const unsigned done = owner->Done;
	HsDone(2, Length);
	assert(owner->Done == done + 1 && owner->Last == Length);
	assert(owner->Buffer[0] == (uint8_t)Length && owner->Buffer[Length - 1U] == (uint8_t)Length);
}

// The USB.6 sequence: test mode Force Enable, then disconnected
static void ChirpDelay(uint32_t Us)
{
	const uint32_t status = USBHSD->DEVCMDSTAT;
	assert(!s_Enabled && g_IrqMasked == 0);
	if (s_ChirpPhase == 0)
	{
		assert(Us == 100000U);
		assert((status & TEST_MODE) == FORCE_ENABLE && (status & CONNECT) != 0);
		// Release is refused during the sequence
		assert(!Lpc546xxUsbControlRelease(1));
	}
	else
	{
		assert(s_ChirpPhase == 1 && Us == 510000U);
		assert((status & (TEST_MODE | CONNECT)) == 0);
		// Events of the bus during the sequence are acknowledged after it
		if (s_ChirpHostEvents)
			USBHSD->DEVCMDSTAT |= RESET | (1U << 24);
		if (s_ChirpVbusLoss)
			USBHSD->DEVCMDSTAT &= ~VBUS_DEBOUNCED;
	}
	++s_ChirpPhase;
}

static void HsProcess(bool Chirp)
{
	s_DelayUs = 0;
	s_ChirpPhase = 0;
	s_DelayHook = ChirpDelay;
	UsbCtrlrProcess(1);
	s_DelayHook = NULL;
	assert(s_ChirpPhase == (Chirp ? 2U : 0U));
	assert(s_DelayUs == (Chirp ? 610000U : 0U));
	assert((USBHSD->DEVCMDSTAT & TEST_MODE) == 0);
}

static void HsChirpTest(void)
{
	// A full-speed bus at the first reset of an attachment: once
	HsBusReset(1);
	assert(!UsbCtrlrHighSpeed(1));
	assert(UsbCtrlrEpOpenData(1, 1, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(1, 1, false, BULK, 512));
	assert(!UsbCtrlrEpOpenData(1, 2, true, INT, 65));
	s_ChirpHostEvents = true;
	HsProcess(true);
	s_ChirpHostEvents = false;
	assert(s_Enabled);
	assert((USBHSD->DEVCMDSTAT & (CONNECT | RESET | (1U << 24))) == CONNECT);
	HsBusReset(1);
	HsProcess(false);

	// Detached: checked again at the next attachment
	TestUsbCable(false, false);
	const unsigned count = s_Count;
	HsProcess(false);
	assert(s_Count == count + 1 && s_Events[count].Type == USB_CTRLR_EVT_RESET);
	TestUsbCable(true, false);
	HsProcess(false);
	HsBusReset(2);
	HsProcess(false);
	assert(UsbCtrlrHighSpeed(1));
	HsBusReset(1);
	HsProcess(true);
	HsBusReset(2);
	HsProcess(false);

	// The interrupt stays off when it was off; no connection without VBUS
	TestUsbCable(false, false);
	HsProcess(false);
	TestUsbCable(true, false);
	HsProcess(false);
	HsBusReset(1);
	UsbCtrlrIntDisable(1);
	s_ChirpPhase = 0;
	s_DelayHook = ChirpDelay;
	s_DelayUs = 0;
	GPIO->B[TEST_VBUS_PORT][TEST_VBUS_PIN] = 1;
	UsbCtrlrProcess(1);
	s_DelayHook = NULL;
	assert(s_ChirpPhase == 2 && !s_Enabled);
	assert(USBHSD->DEVCMDSTAT & CONNECT);
	UsbCtrlrIntEnable(1);

	// A restart is a new attachment
	UsbCtrlrStop(1);
	assert(UsbCtrlrStart(1));
	UsbCtrlrIntEnable(1);
	UsbCtrlrConnect(1);
	HsBusReset(1);
	HsProcess(true);
}

// Errata USB.15 and USB.2 at high speed
static void HsErrataTest(void)
{
	HsBusReset(2);
	assert(UsbCtrlrHighSpeed(1));
	assert(!UsbCtrlrEpOpenData(1, 1, true, BULK, 64));
	assert(UsbCtrlrEpOpenData(1, 1, true, BULK, 512));
	assert(UsbCtrlrEpOpenData(1, 1, false, BULK, 512));
	assert(UsbCtrlrEpOpenData(1, 5, true, INT, 512));
	assert(UsbCtrlrEpOpenData(1, 4, false, INT, 8));
	assert(!UsbCtrlrEpOpenData(1, 2, true, INT, 513));
	assert(!UsbCtrlrEpOpenData(1, 6, true, INT, 8));
	HsBind(1, true);
	HsBind(1, false);
	HsBind(5, true);
	USB1_IRQHandler();
	assert(s_Owner[2].Ready == 1);

	// An IN after an OUT of 4 + 16 x N bytes waits for an IN NAK
	HsOut(20);
	for (unsigned i = 0; i < sizeof(s_Bytes); i++)
		s_Bytes[i] = (uint8_t)(i + 1);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 600));
	assert((Hs()->Command[6] & ACTIVE) == 0 && HS_LENGTH(Hs()->Command[6]) == 512);
	assert(memcmp(Hs()->Packet[1], s_Bytes, 512) == 0);
	assert(USBHSD->DEVCMDSTAT & NAK_IN);
	HsNak(3);
	assert((Hs()->Command[6] & ACTIVE) && (USBHSD->DEVCMDSTAT & NAK_IN) == 0);
	// USB.2: the byte count left after an IN packet is not checked
	HsMoved(3, 77);
	assert(s_Owner[3].Done == 0);
	assert((Hs()->Command[6] & ACTIVE) && HS_LENGTH(Hs()->Command[6]) == 88);
	assert(memcmp(Hs()->Packet[1], s_Bytes + 512, 88) == 0);
	HsMoved(3, 5);
	assert(s_Owner[3].Done == 1 && s_Owner[3].Last == 600);

	// Other OUT lengths arm the IN at once
	HsOut(512);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert(Hs()->Command[6] & ACTIVE);
	HsDone(3, 4);
	assert(s_Owner[3].Done == 2);
	HsOut(36);
	HsOut(21);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert(Hs()->Command[6] & ACTIVE);
	HsDone(3, 4);

	// A zero length IN packet keeps the condition; a SETUP clears it
	HsOut(4);
	HsSetup(false, 20);
	HsDone(0, 20);
	assert(s_Events[s_Count - 1].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert(UsbCtrlrEp0Status(1, 0x80));
	assert(HS_LENGTH(Hs()->Command[2]) == 0);
	HsDone(1, 0);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert((Hs()->Command[6] & ACTIVE) == 0 && (USBHSD->DEVCMDSTAT & NAK_IN));
	HsNak(3);
	HsDone(3, 4);
	HsOut(52);
	HsSetup(true, 70);
	assert(UsbCtrlrEp0Send(1, s_Bytes, 70) == 64);
	assert(HS_LENGTH(Hs()->Command[2]) == 64 && memcmp(Hs()->Packet0[1], s_Bytes, 64) == 0);
	HsDone(1, 64);
	assert(UsbCtrlrEp0Send(1, s_Bytes + 64, 6) == 6);
	HsDone(1, 6);
	assert(UsbCtrlrEp0Status(1, 0));
	HsDone(0, 0);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert(Hs()->Command[6] & ACTIVE);
	HsDone(3, 4);

	// Two waiting endpoints: the NAK interrupts stay on until both go
	HsOut(20);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert(UsbCtrlrEpSend(1, 5, s_Bytes, 8));
	assert((Hs()->Command[6] & ACTIVE) == 0 && (Hs()->Command[22] & ACTIVE) == 0);
	HsNak(3);
	assert((Hs()->Command[6] & ACTIVE) && (USBHSD->DEVCMDSTAT & NAK_IN));
	// The waiting EP5 IN does not go before its own NAK
	assert((Hs()->Command[22] & ACTIVE) == 0);
	HsNak(11);
	assert((Hs()->Command[22] & ACTIVE) && (USBHSD->DEVCMDSTAT & NAK_IN) == 0);
	HsDone(3, 4);
	HsDone(11, 8);
	assert(s_Owner[11].Done == 1 && s_Owner[11].Last == 8);

	// A halt or a close of a waiting IN
	HsOut(20);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	UsbCtrlrEpStall(1, 1, true);
	assert(s_Owner[3].Failed == 1 && (USBHSD->DEVCMDSTAT & NAK_IN) == 0);
	assert(Hs()->Command[6] & (1U << 29));
	UsbCtrlrEpClearStall(1, 1, true);
	assert(UsbCtrlrEpSend(1, 1, s_Bytes, 4));
	assert((Hs()->Command[6] & ACTIVE) == 0 && (USBHSD->DEVCMDSTAT & NAK_IN));
	UsbCtrlrEpClose(1, 1, true);
	assert(s_Owner[3].Cancelled == 1 && (USBHSD->DEVCMDSTAT & NAK_IN) == 0);
	assert(Hs()->Command[6] & (1U << 30));
	UsbCtrlrEpCloseAll(1);
}

// The default: no VBUS pin, the controller reads USB1_VBUS while it runs
static void HsNoPinTest(void)
{
	UsbCtrlrCfg_t cfg = {6, NULL, 0, false};
	assert(!UsbCtrlrInit(0, &cfg));
	IOPinCfg_t two[2] = {0};
	cfg.pIOPinMap = two;
	cfg.NbIOPins = 2;
	assert(!UsbCtrlrInit(1, &cfg));
	cfg.NbIOPins = 1;
	cfg.pIOPinMap = NULL;
	assert(!UsbCtrlrInit(1, &cfg));
	cfg.NbIOPins = 0;
	assert(s_VbusHandler != NULL);
	assert(UsbCtrlrInit(1, &cfg));
	// The GPIO interrupt of the override is released
	assert(s_VbusHandler == NULL);
	GPIO->B[TEST_VBUS_PORT][TEST_VBUS_PIN] = 1;
	assert(!UsbCtrlrVbusDetected(1));

	// Started without a cable: connection requested, VBUS read from the controller
	USBHSD->DEVCMDSTAT = 0;
	assert(UsbCtrlrStart(1));
	assert(!UsbCtrlrVbusDetected(1));
	UsbCtrlrIntEnable(1);
	UsbCtrlrConnect(1);
	assert(s_Enabled && (USBHSD->DEVCMDSTAT & CONNECT));
	unsigned count = s_Count;
	HsProcess(false);
	assert(s_Count == count + 1 && s_Events[count].Type == USB_CTRLR_EVT_RESET);
	HsProcess(false);
	assert(s_Count == count + 1 && (USBHSD->DEVCMDSTAT & CONNECT));
	assert(!UsbCtrlrEpOpenData(1, 1, true, BULK, 512));

	// Attachment: VBUS on USB1_VBUS, then the reset of the host
	USBHSD->DEVCMDSTAT |= VBUS_DEBOUNCED;
	assert(UsbCtrlrVbusDetected(1));
	HsBusReset(2);
	assert(UsbCtrlrHighSpeed(1));
	HsProcess(false);
	assert(UsbCtrlrEpOpenData(1, 1, true, BULK, 512));

	// Detachment: VBUS falls with the connection set. The controller may drop
	// DCON; it is requested again for the next attachment.
	unsigned queued = s_Queued;
	count = s_Count;
	USBHSD->DEVCMDSTAT &= ~(VBUS_DEBOUNCED | CONNECT);
	USBHSD->DEVCMDSTAT |= CONNECT_CHANGE;
	USBHSD->INTSTAT |= DEVICE;
	USB1_IRQHandler();
	assert(s_Count == count + 1 && s_Events[count].Type == USB_CTRLR_EVT_RESET);
	assert(s_Queued == queued + 1 && !UsbCtrlrHighSpeed(1));
	assert((USBHSD->DEVCMDSTAT & (CONNECT | CONNECT_CHANGE)) == CONNECT);
	assert(!UsbCtrlrEpOpenData(1, 1, true, BULK, 512));
	HsProcess(false);
	assert(s_Count == count + 1);

	// Attached again at full speed: USB.6 again; the connection is requested
	// at its end even when VBUS went away meanwhile
	USBHSD->DEVCMDSTAT |= VBUS_DEBOUNCED;
	HsBusReset(1);
	HsProcess(true);
	assert(USBHSD->DEVCMDSTAT & CONNECT);
	HsBusReset(1);
	HsProcess(false);
	USBHSD->DEVCMDSTAT &= ~VBUS_DEBOUNCED;
	count = s_Count;
	HsProcess(false);
	assert(s_Count == count + 1 && (USBHSD->DEVCMDSTAT & CONNECT));
	USBHSD->DEVCMDSTAT |= VBUS_DEBOUNCED;
	HsBusReset(1);
	s_ChirpVbusLoss = true;
	HsProcess(true);
	s_ChirpVbusLoss = false;
	assert(USBHSD->DEVCMDSTAT & CONNECT);

	// A disconnection by the application stays through a detachment
	USBHSD->DEVCMDSTAT |= VBUS_DEBOUNCED;
	HsBusReset(2);
	UsbCtrlrDisconnect(1);
	USBHSD->DEVCMDSTAT &= ~VBUS_DEBOUNCED;
	HsProcess(false);
	assert((USBHSD->DEVCMDSTAT & CONNECT) == 0);

	// Stopped: nothing reads VBUS until the next start
	USBHSD->DEVCMDSTAT |= VBUS_DEBOUNCED;
	UsbCtrlrStop(1);
	assert(!UsbCtrlrVbusDetected(1) && !s_Enabled);
	assert((SYSCON->AHBCLKCTRL[2] & SYSCON_AHBCLKCTRL_USB1D_MASK) == 0);
	assert(UsbCtrlrStart(1) && UsbCtrlrVbusDetected(1));
	UsbCtrlrStop(1);
}

int main(void)
{
	(void)Fresh;
	(void)Setup;
	(void)Done;
	TestUsbInitialize();
	s_Memory = NULL;
	IOPinCfg_t pin = {TEST_VBUS_PORT, TEST_VBUS_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL};
	UsbCtrlrCfg_t cfg = {6, &pin, 1, false};
	assert(!UsbCtrlrInit(1, &cfg) && !UsbCtrlrStart(1));
	pin.PinDir = IOPINDIR_INPUT;
	assert(UsbCtrlrInit(1, &cfg));
	assert(UsbCtrlrVbusDetected(1) && !UsbCtrlrVbusDetected(0));
	TestUsbCable(false, false);
	assert(!UsbCtrlrVbusDetected(1) && !UsbCtrlrStart(1));
	TestUsbCable(true, false);
	TestUsbVbusInterrupt();
	assert(s_Queued == 1);
	assert(UsbCtrlrStart(1));
	assert(s_Priority == 6 && !UsbCtrlrHighSpeed(1) && !UsbCtrlrHighSpeed(0));
	assert(!Lpc546xxUsbControlPrepare(0, 6, Event));
	assert(!UsbCtrlrInit(1, &cfg));
	UsbCtrlrIntEnable(1);
	UsbCtrlrConnect(1);
	assert(s_Enabled && (USBHSD->DEVCMDSTAT & CONNECT));
	char serial[33];
	assert(UsbCtrlrGetSerial(1, serial, sizeof(serial)) == 32);

	// The GPIO override above; high speed at the first reset: no USB.6 sequence
	HsBusReset(2);
	assert(UsbCtrlrHighSpeed(1));
	HsProcess(false);
	HsErrataTest();
	HsChirpTest();

	UsbCtrlrStop(1);
	assert(!s_Enabled && USBHSD->INTEN == 0 && !UsbCtrlrHighSpeed(1));
	assert((SYSCON->AHBCLKCTRL[2] & SYSCON_AHBCLKCTRL_USB1D_MASK) == 0);
	assert(UsbCtrlrEp0Send(1, NULL, 0) == -1);
	HsNoPinTest();
	assert(g_IrqMasked == 0);
	puts("LPC54628 USB1 VBUS from the controller or a GPIO, speed, high-speed list, "
		"errata USB.2, USB.6 and USB.15: PASS");
	return 0;
}
