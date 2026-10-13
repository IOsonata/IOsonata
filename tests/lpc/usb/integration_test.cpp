/* Real generic USB core/interface with LPC port. MIT License, I-SYST, 2026. */
#include <cassert>
#include <cstdio>
#include <cstring>
#include "app_evt_handler.h"
#include "usb/usb_intrf.h"
#include "usb_hw_lpc11uxx.h"

static const uint8_t s_Config[] = {9, 2, 9, 0, 0, 1, 0, 0xA0, 50};
static const uint8_t s_Device[] = {18, 1, 0, 2, 0, 0, 0, 64, 0x34, 0x12,
	0x78, 0x56, 0, 1, 0, 0, 0, 1};
static unsigned s_Queued;
static unsigned s_DetachEvents;

extern "C" void TestUsbInitialize(void);
extern "C" Lpc11UsbMemory_t *TestUsbMemory(void);
extern "C" void TestUsbPacketDone(unsigned Physical, unsigned Length);
extern "C" void TestUsbSetup(const UsbSetupData_t *pSetup);
extern "C" void TestUsbSuspend(bool Suspended);
extern "C" void TestUsbFrame(void);
extern "C" unsigned TestUsbWakeCount(void);
extern "C" void TestUsbCable(bool Connected, bool DeviceInterrupt);

static void CableEvent(int DevNo, UsbEvt_t Event)
{
	assert(DevNo == 0);
	if (Event != USB_EVT_DETACHED)
		return;
	assert(!UsbConfigured(0) && UsbGetAddress(0) == 0);
	const Lpc11UsbMemory_t *memory = TestUsbMemory();
	for (unsigned physical = 0; physical < 10; ++physical)
		assert((memory->Command[2 * physical] & (1U << 31)) == 0);
	++s_DetachEvents;
}

extern "C" bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t)
{
	++s_Queued;
	return true;
}

// This transport fixture needs no device class. Supply a minimal static
// configuration through the generic core's supported descriptor override.
const uint8_t *UsbGetDescriptor(int, uint8_t Type, uint8_t, uint16_t, UsbSpeed_t, uint16_t *pLength)
{
	if (Type == USB_DESCTYPE_DEVICE)
	{
		*pLength = sizeof(s_Device);
		return s_Device;
	}
	if (Type != USB_DESCTYPE_CONFIGURATION)
		return nullptr;
	*pLength = sizeof(s_Config);
	return s_Config;
}

int main()
{
	TestUsbInitialize();
	const IOPinCfg_t pin = {0, 3, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	UsbCfg_t cfg = {};
	cfg.Vid = 0x1234;
	cfg.Pid = 0x5678;
	cfg.pIOPinMap = &pin;
	cfg.NbIOPins = 1;
	cfg.bRemoteWakeup = true;
	cfg.EvtHandler = CableEvent;
	assert(UsbInit(&cfg));
	assert(std::strlen(UsbGetSerial(0)) == 32);
	assert(UsbEnable(0) && s_Queued != 0);
	UsbProcess(0); // establish the attached level in the real core
	Lpc11UsbMemory_t *memory = TestUsbMemory();
	UsbSetupData_t setup = {};
	setup.bRequest = 5; // SET_ADDRESS stays in the real Chapter 9 engine.
	setup.wValue = 7;
	TestUsbSetup(&setup);
	assert(UsbGetAddress(0) == 0);
	TestUsbPacketDone(1, 0);
	assert(UsbGetAddress(0) == 7);
	setup.bRequest = 9; // SET_CONFIGURATION
	setup.wValue = 1;
	TestUsbSetup(&setup);
	TestUsbPacketDone(1, 0);
	assert(UsbGetConfiguration(0) == 1);
	TestUsbSuspend(true);
	assert(UsbSuspended(0) && !UsbRemoteWakeup(0)); // host has not permitted it
	TestUsbFrame();
	assert(TestUsbWakeCount() == 0);
	TestUsbSuspend(false);
	setup.bRequest = 3; // SET_FEATURE DEVICE_REMOTE_WAKEUP
	TestUsbSetup(&setup);
	TestUsbPacketDone(1, 0);
	assert(UsbRemoteWakeupEnabled(0));
	assert(!UsbRemoteWakeup(0)); // permission alone does not mean suspended
	TestUsbSuspend(true);
	assert(UsbRemoteWakeup(0));
	TestUsbFrame();
	TestUsbFrame();
	assert(TestUsbWakeCount() == 0);
	TestUsbFrame();
	assert(TestUsbWakeCount() == 1);
	TestUsbSuspend(false);
	assert(!UsbSuspended(0));
	setup.bRequest = 1; // CLEAR_FEATURE
	TestUsbSetup(&setup);
	TestUsbPacketDone(1, 0);
	TestUsbSuspend(true);
	assert(!UsbRemoteWakeupEnabled(0) && !UsbRemoteWakeup(0));
	TestUsbSuspend(false);

	alignas(4) uint8_t rx[CFIFO_TOTAL_MEMSIZE(2, USB_INTRF_PKT_BLKSIZE(64))];
	alignas(4) uint8_t tx[CFIFO_TOTAL_MEMSIZE(160, 1)];
	UsbIntrfCfg_t intrfCfg = {};
	intrfCfg.EpNo = 1;
	intrfCfg.bBlocking = true;
	intrfCfg.Mode = USB_INTRF_MODE_BYTE;
	intrfCfg.BufferSize = 64;
	intrfCfg.pRxFifoMem = rx;
	intrfCfg.RxFifoMemSize = sizeof(rx);
	intrfCfg.pTxFifoMem = tx;
	intrfCfg.TxFifoMemSize = sizeof(tx);
	intrfCfg.TxFifoBlkSize = 1;
	UsbDevIntrf_t intrf = {};
	assert(UsbIntrfInit(&intrf, &intrfCfg));
	assert(UsbCtrlrEpOpenData(0, 1, false, BULK, 64));
	assert(UsbCtrlrEpOpenData(0, 1, true, BULK, 64));
	assert(UsbIntrfConfigure(&intrf, 64));
	USB_IRQHandler();
	uint8_t payload[80];
	for (unsigned i = 0; i < sizeof(payload); ++i)
		payload[i] = i;
	assert(intrf.DevIntrf.TxData(&intrf.DevIntrf, payload, sizeof(payload)) == 80);
	assert(std::memcmp(memory->Packet[3], payload, 64) == 0);
	TestUsbPacketDone(3, 64);
	assert(std::memcmp(memory->Packet[3], payload + 64, 16) == 0);
	TestUsbPacketDone(3, 16);
	assert(CFifoUsed(intrf.hTxFifo) == 0);

	std::memcpy(memory->Packet[2], payload, 64);
	TestUsbPacketDone(2, 64);
	std::memcpy(memory->Packet[2], payload + 64, 16);
	TestUsbPacketDone(2, 16);
	assert((memory->Command[4] & (1U << 31)) == 0 && intrf.RxPending != 0);
	uint8_t received[80];
	assert(intrf.DevIntrf.RxData(&intrf.DevIntrf, received, sizeof(received)) == 80);
	assert(std::memcmp(received, payload, sizeof(payload)) == 0);
	assert((memory->Command[4] & (1U << 31)) != 0 && intrf.RxPending == 0);
	for (unsigned path = 0; path < 2; ++path)
	{
		assert(intrf.DevIntrf.TxData(&intrf.DevIntrf, payload, 64) == 64);
		setup = {};
		setup.bmRequestType = 0x80;
		setup.bRequest = 0; // GET_STATUS still pending when the cable falls
		setup.wLength = 2;
		TestUsbSetup(&setup);
		assert((memory->Command[2] & (1U << 31)) != 0);
		TestUsbCable(false, path != 0); // GPIO process path and device IRQ path
		UsbProcess(0);
		assert(s_DetachEvents == path + 1);
		assert(!UsbConfigured(0) && !UsbSuspended(0));
		assert(!UsbCtrlrEpSend(0, 1, payload, 8));
		assert(!UsbCtrlrEpReceive(0, 1, received, sizeof(received)));
		UsbProcess(0); // repeated low level must not notify twice
		TestUsbCable(false, true); // delayed debounced interrupt
		UsbProcess(0);
		assert(s_DetachEvents == path + 1);
		TestUsbCable(true, false);
		UsbProcess(0);
		setup = {};
		setup.bRequest = 5;
		setup.wValue = 7;
		TestUsbSetup(&setup);
		TestUsbPacketDone(1, 0);
		setup.bRequest = 9;
		setup.wValue = 1;
		TestUsbSetup(&setup);
		TestUsbPacketDone(1, 0);
		assert(UsbConfigured(0));
		assert(UsbCtrlrEpOpenData(0, 1, false, BULK, 64));
		assert(UsbCtrlrEpOpenData(0, 1, true, BULK, 64));
		assert(UsbIntrfConfigure(&intrf, 64));
		USB_IRQHandler();
	}
	assert(intrf.DevIntrf.TxData(&intrf.DevIntrf, payload, 64) == 64);
	TestUsbCable(true, true); // latched VBUS loss serviced after reattachment
	UsbProcess(0);
	assert(!UsbConfigured(0) && UsbGetAddress(0) == 0 && s_DetachEvents == 2);
	assert((memory->Command[4] & (1U << 31)) == 0);
	assert((memory->Command[6] & (1U << 31)) == 0);
	UsbDisable(0);
	assert((memory->Command[4] & (1U << 31)) == 0);
	std::puts("LPC11Uxx real generic core/interface link, address, FIFO transfers: PASS");
}
