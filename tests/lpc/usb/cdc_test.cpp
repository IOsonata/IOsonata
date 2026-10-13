/* LPC11Uxx real CDC notification regression. MIT License, I-SYST, 2026. */
#include <cassert>
#include <cstdio>
#include <cstring>
#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb_hw_lpc11uxx.h"

static UsbdCdc s_Cdc;
alignas(4) static uint8_t s_Rx[USB_INTRF_RXMEM_SIZE(2, 64)];
alignas(4) static uint8_t s_Tx[CFIFO_MEMSIZE(128)];

extern "C" void TestUsbInitialize(void);
extern "C" Lpc11UsbMemory_t *TestUsbMemory(void);
extern "C" void TestUsbPacketDone(unsigned Physical, unsigned Length);
extern "C" void TestUsbSetup(const UsbSetupData_t *pSetup);
extern "C" void TestUsbCable(bool Connected, bool DeviceInterrupt);

extern "C" bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t)
{
	return true;
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
	assert(UsbInit(&cfg));
	UsbdCdcCfg_t cdcCfg = {};
	cdcCfg.pRxFifoMem = s_Rx;
	cdcCfg.RxFifoMemSize = sizeof(s_Rx);
	cdcCfg.pTxFifoMem = s_Tx;
	cdcCfg.TxFifoMemSize = sizeof(s_Tx);
	assert(s_Cdc.Init(cdcCfg));
	assert(UsbEnable(0));
	UsbProcess(0);
	UsbSetupData_t setup = {};
	setup.bRequest = 5;
	setup.wValue = 7;
	TestUsbSetup(&setup);
	TestUsbPacketDone(1, 0);
	setup.bRequest = 9;
	setup.wValue = 1;
	TestUsbSetup(&setup);
	TestUsbPacketDone(1, 0);
	assert(UsbConfigured(0));
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
	UsbProcess(0);
	assert(!UsbConfigured(0) && !cdc->SerialStateActive);
	assert((memory->Command[2 * physical] & (1U << 31)) == 0);
	UsbDisable(0);
	std::puts("LPC11Uxx real CDC 8+2 notification, resubmission and detach: PASS");
}
