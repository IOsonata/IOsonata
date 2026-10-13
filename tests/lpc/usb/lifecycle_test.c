/* LPC11Uxx USB lifecycle regression. MIT License, I-SYST, 2026. */
#define main HardwareRegressionMain
#include "hardware_test.c"
#undef main
#include "usb/usb.h"
#include "lpc_iap.h"

static IOPinEvtHandler_t s_VbusHandler;
static unsigned s_PinCalls;
static unsigned s_PinReleases;
static unsigned s_IrqEnables;
#ifndef LPC11_USB_INTEGRATION
static unsigned s_ProcessCalls;
static unsigned s_SofCalls;
#endif
static unsigned s_UidCalls;
#ifndef LPC11_USB_INTEGRATION
static uint16_t s_Frame;
#endif
static bool s_NoPinSlot;
static bool s_UidFailure;

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	++s_IrqEnables;
}

void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	assert(Port == 0 && (Pin == 3 || Pin == 6));
	assert(Op == IOPINOP_FUNC1 && Res == IOPINRES_NONE && Type == IOPINTYPE_NORMAL);
	assert(Dir == (Pin == 3 ? IOPINDIR_INPUT : IOPINDIR_OUTPUT));
	++s_PinCalls;
}

int IOPinAllocateInterrupt(int Priority, int Port, int Pin, IOPINSENSE Sense,
	IOPinEvtHandler_t Handler, void *pContext)
{
	assert(Priority >= 0 && Priority <= 3);
	assert(Port == 0 && Pin == 3 && Sense == IOPINSENSE_TOGGLE && pContext == NULL);
	s_VbusHandler = s_NoPinSlot ? NULL : Handler;
	return s_NoPinSlot ? -1 : 5;
}

void IOPinDisableInterrupt(int IntNo)
{
	assert(IntNo == 5);
	s_VbusHandler = NULL;
	++s_PinReleases;
}

IAPSTATUS IAPReadUID(IAP_CMD_STATUS *pResult)
{
	++s_UidCalls;
	pResult->Data[0] = 0x12345678;
	pResult->Data[1] = 0x9ABCDEF0;
	pResult->Data[2] = 0;
	pResult->Data[3] = 0xFEDCBA98;
	return s_UidFailure ? IAPSTATUS_BUSY : IAPSTATUS_SUCCESS;
}

#ifndef LPC11_USB_INTEGRATION
void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
	++s_ProcessCalls;
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvt)
{
	assert(DevNo == 0 && pEvt->Type == USB_CTRLR_EVT_SOF);
	s_Frame = pEvt->FrameNo;
	++s_SofCalls;
}
#endif

void TestDeviceWrite(uint32_t Value)
{
	const uint32_t commands = 0x1FAFFU;
	const uint32_t w1c = (1U << 8) | (7U << 24);
	const uint32_t old = LPC_USB->DEVCMDSTAT;
	// Lifecycle operations must never request remote wakeup.
	assert((Value & ((1U << 17) | (1U << 19))) == ((1U << 17) | (1U << 19)));
	LPC_USB->DEVCMDSTAT = (old & ~(commands | w1c)) |
		(Value & commands) | (old & w1c & ~Value);
}

void TestInterruptWrite(uint32_t Value)
{
	LPC_USB->INTSTAT &= ~Value;
}

void TestInterruptSet(uint32_t Value)
{
	LPC_USB->INTSTAT |= Value;
}

void TestSkipWrite(uint32_t Value)
{
	LPC_USB->EPSKIP = Value;
}

#ifndef LPC11_USB_INTEGRATION
int main(void)
{
	char serial[40];
	const char expected[] = "123456789ABCDEF000000000FEDCBA98";
	const IOPinCfg_t pin = {0, 3, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	UsbCtrlrCfg_t cfg = {INT_MAX, &pin, 1, false};
	// No USB/GPIO registers mapped yet: these calls must not touch hardware.
	assert(!UsbCtrlrStart(0) && !UsbCtrlrVbusDetected(0));
	UsbCtrlrConnect(0);
	UsbCtrlrDisconnect(0);
	UsbCtrlrStop(0);
	UsbCtrlrIntEnable(0);
	UsbCtrlrSofEnable(0, true);
	UsbCtrlrProcess(0);
	assert(s_IrqEnables == 0 && !UsbCtrlrHighSpeed(0));
	for (size_t size = 0; size <= sizeof(serial); ++size)
	{
		memset(serial, '!', sizeof(serial));
		const size_t count = size == 0 ? 0 : (size > 32 ? 32 : size - 1);
		assert(UsbCtrlrGetSerial(0, serial, size) == count);
		assert(memcmp(serial, expected, count) == 0);
		if (size != 0)
			assert(serial[count] == '\0');
		if (size < sizeof(serial))
			assert(serial[size] == '!');
	}
	assert(UsbCtrlrGetSerial(0, NULL, 40) == 0);
	const unsigned calls = s_UidCalls;
	assert(UsbCtrlrGetSerial(1, serial, sizeof(serial)) == 0 && serial[0] == '\0');
	assert(s_UidCalls == calls);
	s_UidFailure = true;
	assert(UsbCtrlrGetSerial(0, serial, sizeof(serial)) == 0 && serial[0] == '\0');
	assert(!UsbCtrlrInit(0, NULL) && !UsbCtrlrInit(1, &cfg));
	cfg.bLowPowerSuspend = true;
	assert(!UsbCtrlrInit(0, &cfg));
	cfg.bLowPowerSuspend = false;
	cfg.NbIOPins = 2;
	assert(!UsbCtrlrInit(0, &cfg));
	cfg.NbIOPins = 1;
	IOPinCfg_t bad = pin;
	bad.PinNo = 4;
	cfg.pIOPinMap = &bad;
	assert(!UsbCtrlrInit(0, &cfg) && s_PinCalls == 0);
	cfg.pIOPinMap = &pin;
	s_NoPinSlot = true;
	assert(!UsbCtrlrInit(0, &cfg));
	assert(!UsbCtrlrStart(0));
	s_NoPinSlot = false;
	assert(UsbCtrlrInit(0, &cfg) && s_PinCalls == 2);
	assert(s_VbusHandler != NULL);
	assert(mmap((void *)LPC_GPIO_BASE, 0x3000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(!UsbCtrlrStart(0)); // no VBUS, still no USB register access
	LPC_GPIO->PIN[0] = 1U << 3;
	s_VbusHandler(5, NULL);
	assert(s_ProcessCalls == 1 && UsbCtrlrVbusDetected(0));
	assert(mmap((void *)LPC_SYSCON_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_USB_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	Defaults();
	LPC_SYSCON->USBCLKDIV = 0;
	assert(!UsbCtrlrStart(0));
	LPC_SYSCON->USBCLKDIV = 1;
	assert(UsbCtrlrStart(0) && s_Priority == 3 && s_IrqEnables == 0);
	s_Memory = (void *)0x20004000;
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) == 0);
	const unsigned pins = s_PinCalls;
	assert(UsbCtrlrStart(0) && s_PinCalls == pins);
	assert(!UsbCtrlrInit(0, &cfg));
	UsbCtrlrIntEnable(0);
	assert(s_IrqEnables == 1);
	UsbCtrlrConnect(1);
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) == 0);
	UsbCtrlrConnect(0);
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) != 0);
	LPC_USB->DEVCMDSTAT |= (1U << 17) | (1U << 25);
	UsbCtrlrDisconnect(0);
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) == 0);
	assert((LPC_USB->DEVCMDSTAT & (1U << 25)) != 0); // preserve pending W1C
	LPC_USB->DEVCMDSTAT &= ~((1U << 17) | (1U << 25));
	LPC_GPIO->PIN[0] = 0;
	s_VbusHandler(5, NULL);
	UsbCtrlrConnect(0);
	assert((LPC_USB->DEVCMDSTAT & (1U << 16)) == 0 && s_ProcessCalls == 2);
	LPC_GPIO->PIN[0] = 1U << 3;
	UsbCtrlrConnect(0);
	LPC_USB->INTSTAT |= 1U << 30;
	UsbCtrlrSofEnable(0, true);
	assert((LPC_USB->INTSTAT & (1U << 30)) == 0);
	LPC_USB->INFO = 0xFFFF;
	LPC_USB->INTSTAT |= 1U << 30;
	USB_IRQHandler();
	assert(s_SofCalls == 1 && s_Frame == 0x7FF);
	UsbCtrlrSofEnable(0, false);
	LPC_USB->INTSTAT |= 1U << 30;
	USB_IRQHandler();
	assert(s_SofCalls == 1);
	assert(UsbCtrlrEp0Send(0, NULL, 0) == 0);
	UsbCtrlrStop(0); // skip never completes
	assert(LPC_USB->INTEN == 0 && (s_Memory->Command[2] & (1U << 31)) != 0);
	assert(!UsbCtrlrStart(0) && !UsbCtrlrInit(0, &cfg));
	UsbCtrlrConnect(0);
	UsbCtrlrIntEnable(0);
	assert(s_IrqEnables == 1 && (LPC_USB->DEVCMDSTAT & (1U << 16)) == 0);
	s_CompleteSkip = true;
	UsbCtrlrStop(0); // confirmed retirement allows reuse
	assert((LPC_SYSCON->SYSAHBCLKCTRL & ((1U << 14) | (1U << 27))) == 0);
	s_VbusHandler(5, NULL); // cable IRQ remains allocated across Stop
	assert(s_ProcessCalls == 3 && s_PinReleases == 0);
	assert(UsbCtrlrStart(0));
	UsbCtrlrStop(0);
	cfg.IntPrio = INT_MIN;
	assert(UsbCtrlrInit(0, &cfg) && s_PinReleases == 1);
	assert(UsbCtrlrStart(0) && s_Priority == 0);
	UsbCtrlrStop(0);
	puts("LPC11Uxx USB lifecycle, VBUS, SOF and UID: PASS");
	return 0;
}
#endif
