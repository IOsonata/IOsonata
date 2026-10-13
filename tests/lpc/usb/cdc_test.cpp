/* LPC11Uxx real CDC enumeration, control and data regression. MIT License, I-SYST, 2026. */
#include <cassert>
#include <cstdio>
#include <cstring>
#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb_hw_lpc11uxx.h"

static UsbdCdc s_Cdc;
static unsigned s_PortOpened;
static unsigned s_PortClosed;
static unsigned s_RxEvents;
static const uint8_t s_LineCoding[] = {0x00, 0xE1, 0, 0, 0, 0, 8};
alignas(4) static uint8_t s_Events[APPEVT_HANDLER_QUE_MEMSIZE(16)];
alignas(4) static uint8_t s_Rx[USB_INTRF_RXMEM_SIZE(2, 64)];
alignas(4) static uint8_t s_Tx[CFIFO_MEMSIZE(128)];

extern "C" void TestUsbInitialize(void);
extern "C" Lpc11UsbMemory_t *TestUsbMemory(void);
extern "C" void TestUsbPacketDone(unsigned Physical, unsigned Length);
extern "C" void TestUsbSetup(const UsbSetupData_t *pSetup);
extern "C" void TestUsbCable(bool Connected, bool DeviceInterrupt);
extern "C" void TestUsbVbusInterrupt(void);

static void ProcessEvents(void)
{
	for (unsigned pass = 0; pass < 8 && AppEvtHandlerPending(); ++pass)
		AppEvtHandlerExec();
	assert(!AppEvtHandlerPending());
}

static int CdcEvent(DevIntrf_t *, DEVINTRF_EVT Event, uint8_t *, int Length)
{
	if (Event == DEVINTRF_EVT_STATECHG)
	{
		assert(s_Cdc.IsPortOpen() == (Length != 0));
		if (Length != 0)
			++s_PortOpened;
		else
			++s_PortClosed;
	}
	if (Event == DEVINTRF_EVT_RX_DATA)
		++s_RxEvents;
	return 0;
}

static unsigned ControlRead(uint8_t Type, uint8_t Request, uint16_t Value,
	uint16_t Index, uint16_t Length, uint8_t *pData, unsigned Capacity)
{
	const UsbSetupData_t setup = {Type, Request, Value, Index, Length};
	TestUsbSetup(&setup);
	Lpc11UsbMemory_t *memory = TestUsbMemory();
	unsigned count = 0;
	unsigned packets = 0;
	while ((memory->Command[2] & (1U << 31)) != 0)
	{
		const unsigned packet = (memory->Command[2] >> 16) & 0x3FF;
		assert(++packets <= 8 && packet <= 64 && count + packet <= Capacity);
		std::memcpy(pData + count, memory->Packet[1], packet);
		count += packet;
		TestUsbPacketDone(1, packet);
	}
	if ((memory->Command[0] & (1U << 31)) == 0)
		std::fprintf(stderr, "ControlRead type=%x request=%x value=%x bytes=%u packets=%u open=%u address=%u\n", Type, Request, Value, count, packets, s_PortOpened, UsbGetAddress(0));
	assert((memory->Command[0] & (1U << 31)) != 0);
	TestUsbPacketDone(0, 0);
	return count;
}

static void Enumerate(void)
{
	uint8_t descriptor[256];
	assert(ControlRead(0x80, 6, 0x0100, 0, 8, descriptor, sizeof(descriptor)) == 8);
	assert(descriptor[0] == 18 && descriptor[1] == 1 && descriptor[7] == 64);
	assert(ControlRead(0x80, 6, 0x0100, 0, 18, descriptor, sizeof(descriptor)) == 18);
	assert(descriptor[8] == 0x34 && descriptor[9] == 0x12);
	assert(descriptor[10] == 0x78 && descriptor[11] == 0x56);
	const UsbSetupData_t address = {0, 5, 7, 0, 0};
	TestUsbSetup(&address);
	assert(UsbGetAddress(0) == 0);
	TestUsbPacketDone(1, 0);
	assert(UsbGetAddress(0) == 7);
	assert(ControlRead(0x80, 6, 0x0200, 0, 9, descriptor, sizeof(descriptor)) == 9);
	const unsigned total = descriptor[2] | (descriptor[3] << 8);
	assert(total == 9 + sizeof(UsbdCdcDesc_t) && descriptor[4] == 2);
	assert(ControlRead(0x80, 6, 0x0200, 0, total, descriptor, sizeof(descriptor)) == total);
	unsigned endpoints = 0;
	for (unsigned offset = 0; offset < total; offset += descriptor[offset])
	{
		assert(descriptor[offset] >= 2 && offset + descriptor[offset] <= total);
		if (descriptor[offset + 1] != USB_DESCTYPE_ENDPOINT)
			continue;
		++endpoints;
		const unsigned mps = descriptor[offset + 4] | (descriptor[offset + 5] << 8);
		assert(mps == (descriptor[offset + 3] == INT ? 8U : 64U));
	}
	assert(endpoints == 3);
	// A 31-character UTF-16 string is exactly 64 bytes, requiring a ZLP
	// because the host asked for more. The UID string takes 64+2 bytes.
	assert(ControlRead(0x80, 6, 0x0301, 0x0409, 255, descriptor, sizeof(descriptor)) == 64);
	assert(descriptor[0] == 64 && descriptor[1] == USB_DESCTYPE_STRING);
	assert(ControlRead(0x80, 6, 0x0303, 0x0409, 255, descriptor, sizeof(descriptor)) == 66);
	const UsbSetupData_t config = {0, 9, 1, 0, 0};
	TestUsbSetup(&config);
	TestUsbPacketDone(1, 0);
	assert(UsbConfigured(0));
}

static void OpenPort(void)
{
	const UsbdCdcDev_t *cdc = s_Cdc;
	uint8_t coding[7];
	assert(ControlRead(0xA1, 0x21, 0, cdc->CtrlIfNo, 7, coding, sizeof(coding)) == 7);
	assert(coding[0] == 0 && coding[1] == 0xC2 && coding[2] == 1); // 115200
	const UsbSetupData_t line = {0x21, 0x20, 0, cdc->CtrlIfNo, 7};
	TestUsbSetup(&line);
	std::memcpy(TestUsbMemory()->Packet[0], s_LineCoding, sizeof(s_LineCoding));
	TestUsbPacketDone(0, 7);
	assert(s_Cdc.LineCoding()->dwDTERate == 115200);
	TestUsbPacketDone(1, 0); // commit only after the status ACK
	assert(s_Cdc.LineCoding()->dwDTERate == 57600);
	assert(ControlRead(0xA1, 0x21, 0, cdc->CtrlIfNo, 7, coding, sizeof(coding)) == 7);
	assert(std::memcmp(coding, s_LineCoding, sizeof(coding)) == 0);
	const UsbSetupData_t open = {0x21, 0x22, 3, cdc->CtrlIfNo, 0};
	TestUsbSetup(&open);
	assert(!s_Cdc.IsPortOpen());
	TestUsbPacketDone(1, 0);
	assert(s_Cdc.IsPortOpen());
}

static void Loopback(void)
{
	const UsbdCdcDev_t *cdc = s_Cdc;
	Lpc11UsbMemory_t *memory = TestUsbMemory();
	const unsigned out = 2U * cdc->DataEpNo;
	uint8_t sent[80];
	uint8_t received[80];
	for (unsigned i = 0; i < sizeof(sent); ++i)
		sent[i] = (uint8_t)(i ^ 0xA5);
	USB_IRQHandler(); // initial OUT buffer request after class configuration
	std::memcpy(memory->Packet[out], sent, 64);
	TestUsbPacketDone(out, 64);
	std::memcpy(memory->Packet[out], sent + 64, 16);
	TestUsbPacketDone(out, 16);
	assert(s_Cdc.Rx(0, received, sizeof(received)) == 80);
	assert(std::memcmp(sent, received, sizeof(sent)) == 0);
	assert(s_Cdc.Tx(0, received, sizeof(received)) == 80);
	assert(std::memcmp(memory->Packet[out + 1], sent, 64) == 0);
	TestUsbPacketDone(out + 1, 64);
	assert(std::memcmp(memory->Packet[out + 1], sent + 64, 16) == 0);
	TestUsbPacketDone(out + 1, 16);
	assert((memory->Command[2 * (out + 1)] & (1U << 31)) == 0);
}

int main()
{
	TestUsbInitialize();
	assert(AppEvtHandlerInit(s_Events, sizeof(s_Events)));
	const IOPinCfg_t pin = {0, 3, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	UsbCfg_t cfg = {};
	cfg.Vid = 0x1234;
	cfg.Pid = 0x5678;
	cfg.pIOPinMap = &pin;
	cfg.NbIOPins = 1;
	cfg.pManufacturer = "1234567890123456789012345678901";
	assert(UsbInit(&cfg));
	UsbdCdcCfg_t cdcCfg = {};
	cdcCfg.bBlocking = true; // retain unread packets, like the loopback example
	cdcCfg.pRxFifoMem = s_Rx;
	cdcCfg.RxFifoMemSize = sizeof(s_Rx);
	cdcCfg.pTxFifoMem = s_Tx;
	cdcCfg.TxFifoMemSize = sizeof(s_Tx);
	cdcCfg.EvtCB = CdcEvent;
	assert(s_Cdc.Init(cdcCfg));
	assert(UsbEnable(0));
	ProcessEvents();
	Enumerate();
	OpenPort();
	Loopback();
	assert(s_PortOpened == 1 && s_RxEvents == 2);
	UsbdCdcDev_t *cdc = s_Cdc;
	Lpc11UsbMemory_t *memory = TestUsbMemory();
	const unsigned physical = 2U * cdc->NotifyEpNo + 1U;
	const uint8_t header[] = {0xA1, 0x20, 0, 0, cdc->CtrlIfNo, 0, 2, 0};
	assert(cdc->SerialStateActive && !cdc->SerialStatePending);
	assert(std::memcmp(memory->Packet[physical], header, sizeof(header)) == 0);
	assert(((memory->Command[2 * physical] >> 16) & 0x3FF) == 8);
	TestUsbPacketDone(physical, 8);
	assert(cdc->SerialStateActive); // first packet must not complete notification
	assert(((memory->Command[2 * physical] >> 16) & 0x3FF) == 2);
	assert(memory->Packet[physical][0] == 0 && memory->Packet[physical][1] == 0);

	// A new state during the old transfer must not overwrite its tail. The
	// completion callback immediately starts the next complete notification.
	s_Cdc.SetSerialState(USB_CDC_SERIAL_STATE_RX_CARRIER);
	assert(cdc->SerialStatePending);
	TestUsbPacketDone(physical, 2);
	assert(cdc->SerialStateActive && !cdc->SerialStatePending);
	assert(std::memcmp(memory->Packet[physical], header, sizeof(header)) == 0);
	TestUsbPacketDone(physical, 8);
	assert(memory->Packet[physical][0] == USB_CDC_SERIAL_STATE_RX_CARRIER);
	TestUsbPacketDone(physical, 2);
	assert(!cdc->SerialStateActive);
	assert((memory->Command[2 * physical] & (1U << 31)) == 0);

	s_Cdc.SetSerialState(0);
	TestUsbPacketDone(physical, 8);
	TestUsbCable(false, true);
	ProcessEvents();
	assert(!UsbConfigured(0) && !cdc->SerialStateActive);
	assert((memory->Command[2 * physical] & (1U << 31)) == 0);
	assert(s_PortClosed == 1 && !s_Cdc.IsPortOpen());
	TestUsbCable(true, false);
	TestUsbVbusInterrupt();
	ProcessEvents();
	Enumerate();
	OpenPort();
	Loopback();
	assert(s_PortOpened == 2 && s_RxEvents == 4);
	UsbDisable(0);
	std::puts("LPC11Uxx CDC enumeration, line coding, loopback, notifications and reconnect: PASS");
}
