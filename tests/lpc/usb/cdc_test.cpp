/* LPC11Uxx real CDC enumeration, control and data regression. MIT License, I-SYST, 2026. */
#include <cassert>
#include <cstdio>
#include <cstring>
#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_hid.h"
#include "usb/usbd_bulk.h"
#include "usb_hw_lpc11uxx.h"

class TestHid : public UsbdHid {
public:
	unsigned ReadComplete = 0;
	unsigned ReadAbort = 0;

	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
		uint8_t **ppData, uint16_t *pLength) override;
};

static UsbdCdc s_Cdc;
static TestHid s_Hid;
static bool s_Composite;
static bool s_VendorBulk;
static UsbdBulk s_Bulk;
alignas(4) static uint8_t s_BulkRx[USBD_BULK_RXMEM_SIZE(2)];
alignas(4) static uint8_t s_BulkTx[CFIFO_MEMSIZE(128)];
static unsigned s_HidTx;
static const uint8_t s_ReportDesc[] = {
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01,
	0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02,
	0x09, 0x01, 0x91, 0x02, 0xC0
};
alignas(4) static uint8_t s_HidRx[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
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
extern "C" void TestUsbStatusAckPending(unsigned Physical);
extern "C" void TestUsbSetup(const UsbSetupData_t *pSetup);
extern "C" void TestUsbCable(bool Connected, bool DeviceInterrupt);
extern "C" void TestUsbVbusInterrupt(void);

bool TestHid::Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
	uint8_t **ppData, uint16_t *pLength)
{
	if (pSetup->bmRequestType == 0xA1 && pSetup->bRequest == USB_HID_REQ_GET_IDLE)
	{
		if (Stage == USB_CTRL_COMPLETE)
			++ReadComplete;
		if (Stage == USB_CTRL_ABORT)
			++ReadAbort;
	}
	return UsbdHid::Control(pSetup, Stage, ppData, pLength);
}

static int HidEvent(DevIntrf_t *, DEVINTRF_EVT Event, uint8_t *, int)
{
	if (Event == DEVINTRF_EVT_TX_FIFO_EMPTY)
		++s_HidTx;
	return 0; // Retain reports for foreground Rx().
}

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
	assert(total == 9 + sizeof(UsbdCdcDesc_t) + (s_Composite ? sizeof(UsbdHidDesc_t) : 0) +
		(s_VendorBulk ? sizeof(UsbdBulkDesc_t) : 0));
	assert(descriptor[4] == (s_VendorBulk ? 4 : (s_Composite ? 3 : 2)));
	assert(ControlRead(0x80, 6, 0x0200, 0, total, descriptor, sizeof(descriptor)) == total);
	unsigned endpoints = 0;
	unsigned addresses = 0;
	for (unsigned offset = 0; offset < total; offset += descriptor[offset])
	{
		assert(descriptor[offset] >= 2 && offset + descriptor[offset] <= total);
		if (descriptor[offset + 1] != USB_DESCTYPE_ENDPOINT)
			continue;
		++endpoints;
		const unsigned mps = descriptor[offset + 4] | (descriptor[offset + 5] << 8);
		const UsbdCdcDev_t *cdc = s_Cdc;
		const unsigned address = descriptor[offset + 2];
		const unsigned ep = address & 0x0F;
		const unsigned physical = 2 * ep + ((address >> 7) & 1);
		assert(ep > 0 && ep <= 4 && (addresses & (1U << physical)) == 0);
		addresses |= 1U << physical;
		assert(mps == (address == (0x80U | cdc->NotifyEpNo) ? 8U : 64U));
	}
	assert(endpoints == (s_VendorBulk ? 7 : (s_Composite ? 5 : 3)));
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
	TestUsbStatusAckPending(1); // ACK and next SETUP share one IRQ
	assert(s_Cdc.LineCoding()->dwDTERate == 115200);
	assert(ControlRead(0xA1, 0x21, 0, cdc->CtrlIfNo, 7, coding, sizeof(coding)) == 7);
	assert(std::memcmp(coding, s_LineCoding, sizeof(coding)) == 0);
	assert(s_Cdc.LineCoding()->dwDTERate == 57600);
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
	for (unsigned offset = 0; offset < sizeof(sent);)
	{
		const unsigned packet = (memory->Command[2 * (out + 1)] >> 16) & 0x3FF;
		assert(packet > 0 && packet <= 64 && offset + packet <= sizeof(sent));
		assert(std::memcmp(memory->Packet[out + 1], sent + offset, packet) == 0);
		TestUsbPacketDone(out + 1, packet);
		offset += packet;
	}
	assert((memory->Command[2 * (out + 1)] & (1U << 31)) == 0);
}

static void BulkLoopback(void)
{
	if (!s_VendorBulk)
		return;
	const UsbdBulkDev_t *bulk = s_Bulk;
	assert(bulk->EpNo == 4 && bulk->ItfNo == 3);
	const unsigned out = 2U * bulk->EpNo;
	uint8_t sent[64];
	uint8_t received[64];
	for (unsigned i = 0; i < sizeof(sent); ++i)
		sent[i] = (uint8_t)(i ^ 0x69);
	USB_IRQHandler();
	std::memcpy(TestUsbMemory()->Packet[out], sent, sizeof(sent));
	TestUsbPacketDone(out, sizeof(sent));
	assert(s_Bulk.Rx(0, received, sizeof(received)) == sizeof(received));
	assert(std::memcmp(received, sent, sizeof(sent)) == 0);
	assert(s_Bulk.Tx(0, received, sizeof(received)) == sizeof(received));
	assert(std::memcmp(TestUsbMemory()->Packet[out + 1], sent, sizeof(sent)) == 0);
	TestUsbPacketDone(out + 1, sizeof(sent));
}

static void HidReports(void)
{
	if (!s_Composite)
		return;
	const UsbdHidDev_t *hid = s_Hid;
	const UsbdCdcDev_t *cdc = s_Cdc;
	assert(hid->Configured && hid->ItfNo != cdc->CtrlIfNo);
	assert(hid->EpNo != cdc->DataEpNo && hid->EpNo != cdc->NotifyEpNo);
	uint8_t descriptor[64];
	assert(ControlRead(0x81, 6, 0x2200, hid->ItfNo, 64,
		descriptor, sizeof(descriptor)) == sizeof(s_ReportDesc));
	assert(std::memcmp(descriptor, s_ReportDesc, sizeof(s_ReportDesc)) == 0);
	const UsbSetupData_t idle = {0x21, USB_HID_REQ_SET_IDLE, 7U << 8,
		(uint16_t)hid->ItfNo, 0};
	TestUsbSetup(&idle);
	assert(hid->Idle == 0);
	TestUsbStatusAckPending(1);
	assert(ControlRead(0xA1, USB_HID_REQ_GET_IDLE, 0, hid->ItfNo, 1,
		descriptor, sizeof(descriptor)) == 1);
	assert(descriptor[0] == 7 && hid->Idle == 7);
	const unsigned completed = s_Hid.ReadComplete;
	const unsigned aborted = s_Hid.ReadAbort;
	const UsbSetupData_t getIdle = {0xA1, USB_HID_REQ_GET_IDLE, 0,
		(uint16_t)hid->ItfNo, 1};
	TestUsbSetup(&getIdle);
	TestUsbPacketDone(1, 1);
	TestUsbStatusAckPending(0);
	// The next request must not abort the successfully completed read.
	assert(ControlRead(0xA1, USB_HID_REQ_GET_IDLE, 0, hid->ItfNo, 1,
		descriptor, sizeof(descriptor)) == 1);
	assert(s_Hid.ReadComplete == completed + 2 && s_Hid.ReadAbort == aborted);
	const unsigned out = 2U * hid->EpNo;
	uint8_t report[64];
	uint8_t received[64];
	for (unsigned i = 0; i < sizeof(report); ++i)
		report[i] = (uint8_t)(i ^ 0x3C);
	USB_IRQHandler();
	std::memcpy(TestUsbMemory()->Packet[out], report, sizeof(report));
	TestUsbPacketDone(out, sizeof(report));
	assert(s_Hid.Rx(0, received, sizeof(received)) == sizeof(received));
	assert(std::memcmp(report, received, sizeof(report)) == 0);
	assert(s_Hid.Tx(0, received, sizeof(received)) == sizeof(received));
	const unsigned before = s_HidTx;
	Loopback(); // Other classes progress while the HID IN report is active.
	BulkLoopback();
	assert(s_HidTx == before);
	assert(std::memcmp(TestUsbMemory()->Packet[out + 1], report, sizeof(report)) == 0);
	TestUsbPacketDone(out + 1, sizeof(report));
	assert(s_HidTx == before + 1);
}

static void Reconfigure(void)
{
	const UsbdCdcDev_t *cdc = s_Cdc;
	const UsbdHidDev_t *hid = s_Hid;
	const UsbdBulkDev_t *bulk = s_Bulk;
	uint8_t oldPacket[64];
	uint8_t newPacket[64];
	std::memset(oldPacket, 0xD3, sizeof(oldPacket));
	std::memset(newPacket, 0x6C, sizeof(newPacket));
	const unsigned closed = s_PortClosed;
	const unsigned opened = s_PortOpened;
	for (unsigned pass = 0; pass < 4; ++pass)
	{
		USB_IRQHandler(); // Arm all configured OUT destinations.
		assert(s_Cdc.Tx(0, oldPacket, sizeof(oldPacket)) == sizeof(oldPacket));
		if (s_Composite)
			assert(s_Hid.Tx(0, oldPacket, sizeof(oldPacket)) == sizeof(oldPacket));
		if (s_VendorBulk)
			assert(s_Bulk.Tx(0, oldPacket, sizeof(oldPacket)) == sizeof(oldPacket));
		assert((TestUsbMemory()->Command[2 * (2 * cdc->DataEpNo + 1)] & (1U << 31)) != 0);

		// Alternate explicit unconfigure/reconfigure and repeated selection
		// of configuration 1. Every old IN packet is still awaiting its ACK.
		if ((pass & 1U) == 0)
		{
			const UsbSetupData_t clear = {0, USB_REQ_SET_CONFIGURATION, 0, 0, 0};
			TestUsbSetup(&clear);
			TestUsbPacketDone(1, 0);
			assert(!UsbConfigured(0));
			for (unsigned physical = 2; physical < 10; ++physical)
				assert((TestUsbMemory()->Command[2 * physical] & (1U << 31)) == 0);
			// FIFO transports may accept bytes while unconfigured, but must
			// not arm hardware. Selecting the next configuration flushes them.
			assert(s_Cdc.Tx(0, newPacket, sizeof(newPacket)) == sizeof(newPacket));
			if (s_Composite)
				assert(s_Hid.Tx(0, newPacket, sizeof(newPacket)) == 0);
			if (s_VendorBulk)
				assert(s_Bulk.Tx(0, newPacket, sizeof(newPacket)) == sizeof(newPacket));
			for (unsigned physical = 2; physical < 10; ++physical)
				assert((TestUsbMemory()->Command[2 * physical] & (1U << 31)) == 0);
		}
		const UsbSetupData_t select = {0, USB_REQ_SET_CONFIGURATION, 1, 0, 0};
		TestUsbSetup(&select);
		TestUsbPacketDone(1, 0);
		assert(UsbConfigured(0) && !s_Cdc.IsPortOpen());
		assert(UsbGetAddress(0) == 7);
		assert(s_PortClosed == closed + pass + 1);
		const UsbSetupData_t open = {0x21, 0x22, 3, cdc->CtrlIfNo, 0};
		TestUsbSetup(&open);
		TestUsbPacketDone(1, 0);
		assert(s_Cdc.IsPortOpen() && s_PortOpened == opened + pass + 1);
		Loopback(); // Old queued bytes must not precede the new echo.
		BulkLoopback();
		if (s_Composite)
		{
			assert(s_Hid.Tx(0, newPacket, sizeof(newPacket)) == sizeof(newPacket));
			const unsigned physical = 2U * hid->EpNo + 1;
			assert(std::memcmp(TestUsbMemory()->Packet[physical], newPacket, sizeof(newPacket)) == 0);
			TestUsbPacketDone(physical, sizeof(newPacket));
			assert(hid->Configured);
		}
		if (s_VendorBulk)
			assert(bulk->pData->Mps == 64);
	}
	std::puts("LPC11Uxx active-transfer unconfigure/reconfigure and repeated configuration: PASS");
}

int main(int argc, char **)
{
	s_Composite = argc > 1;
	s_VendorBulk = argc > 2;
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
	if (s_Composite)
	{
		UsbdHidCfg_t hidCfg = {};
		hidCfg.pReportDesc = s_ReportDesc;
		hidCfg.ReportDescLength = sizeof(s_ReportDesc);
		hidCfg.pRxBuffer = s_HidRx;
		hidCfg.pTxBuffer = s_HidTxBuffer;
		hidCfg.EvtCB = HidEvent;
		assert(s_Hid.Init(hidCfg));
	}
	if (s_VendorBulk)
	{
		UsbdBulkCfg_t bulkCfg = {};
		bulkCfg.bBlocking = true;
		bulkCfg.pRxFifoMem = s_BulkRx;
		bulkCfg.RxFifoMemSize = sizeof(s_BulkRx);
		bulkCfg.pTxFifoMem = s_BulkTx;
		bulkCfg.TxFifoMemSize = sizeof(s_BulkTx);
		assert(s_Bulk.Init(bulkCfg));
	}
	assert(UsbEnable(0));
	ProcessEvents();
	Enumerate();
	OpenPort();
	Loopback();
	HidReports();
	assert(s_PortOpened == 1 && s_RxEvents == (s_Composite ? 4U : 2U));
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
	assert(!s_Composite || !static_cast<UsbdHidDev_t *>(s_Hid)->Configured);
	assert((memory->Command[2 * physical] & (1U << 31)) == 0);
	assert(s_PortClosed == 1 && !s_Cdc.IsPortOpen());
	TestUsbCable(true, false);
	TestUsbVbusInterrupt();
	ProcessEvents();
	Enumerate();
	OpenPort();
	Loopback();
	HidReports();
	assert(s_PortOpened == 2 && s_RxEvents == (s_Composite ? 8U : 4U));
	Reconfigure();
	UsbDisable(0);
	if (s_VendorBulk)
		std::puts("LPC11Uxx CDC/HID/vendor bulk, seven endpoint addresses and reconnect: PASS");
	if (s_Composite)
		std::puts("LPC11Uxx composite CDC/HID descriptors, reports and reconnect: PASS");
	std::puts("LPC11Uxx CDC enumeration, line coding, loopback, notifications and reconnect: PASS");
}
