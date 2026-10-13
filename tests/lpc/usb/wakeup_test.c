/* LPC11Uxx remote-wakeup regression. MIT License, I-SYST, 2026. */
#include "integration_model.c"

static unsigned s_SuspendEvents;
static unsigned s_ResumeEvents;
static unsigned s_SofEvents;

void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvent)
{
	assert(DevNo == 0);
	if (pEvent->Type == USB_CTRLR_EVT_SUSPEND)
		++s_SuspendEvents;
	if (pEvent->Type == USB_CTRLR_EVT_RESUME)
		++s_ResumeEvents;
	if (pEvent->Type == USB_CTRLR_EVT_SOF)
		++s_SofEvents;
}

int main(void)
{
	UsbCtrlrRemoteWakeup(0); // no register access before preparation
	TestUsbInitialize();
	const IOPinCfg_t pin = {0, 3, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	const UsbCtrlrCfg_t cfg = {2, &pin, 1, false};
	assert(UsbCtrlrInit(0, &cfg) && UsbCtrlrStart(0));
	UsbCtrlrConnect(0);
	UsbCtrlrRemoteWakeup(1);
	UsbCtrlrRemoteWakeup(0); // active bus
	assert((LPC_USB->INTEN & (1U << 30)) == 0 && s_WakeCount == 0);
	TestUsbSuspend(true);
	assert(s_SuspendEvents == 1);
	LPC_SYSCON->USBCLKCTRL = 2; // preserve wake polarity
	LPC_USB->INTSTAT |= 1U << 30; // stale tick cannot count
	UsbCtrlrRemoteWakeup(0);
	assert((LPC_USB->INTSTAT & (1U << 30)) == 0);
	assert(s_WakeCount == 0 && LPC_SYSCON->USBCLKCTRL == 2);
	TestUsbFrame();
	UsbCtrlrRemoteWakeup(0); // duplicate request must not restart the delay
	TestUsbFrame();
	assert(s_WakeCount == 0);
	TestUsbFrame();
	assert(s_WakeCount == 1 && LPC_SYSCON->USBCLKCTRL == 3);
	assert(s_SofEvents == 0 && (LPC_USB->INTEN & (1U << 30)) == 0);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	assert(s_WakeCount == 1); // no repeated signalling while waiting for host
	TestUsbSuspend(false);
	assert(s_ResumeEvents == 1 && LPC_SYSCON->USBCLKCTRL == 2);

	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	TestUsbSuspend(false); // host beat the pending request
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 1);

	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	LPC_GPIO->PIN[0] = 0; // raw VBUS falls before debounced DCON_C
	TestUsbFrame();
	LPC_GPIO->PIN[0] = 1U << 3;
	TestUsbFrame();
	assert(s_WakeCount == 1 && LPC_SYSCON->USBCLKCTRL == 2);
	UsbCtrlrRemoteWakeup(0);
	LPC_USB->DEVCMDSTAT |= 1U << 26;
	LPC_USB->INTSTAT |= 1U << 31;
	USB_IRQHandler();
	TestUsbFrame();
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 1);

	TestUsbSuspend(true);
	UsbCtrlrSofEnable(0, true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	UsbCtrlrSofEnable(0, false); // must keep internal wake timing alive
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 2 && s_SofEvents == 1);
	UsbCtrlrDisconnect(0);
	assert(LPC_SYSCON->USBCLKCTRL == 2);
	UsbCtrlrRemoteWakeup(0); // disconnected
	TestUsbFrame();
	assert(s_WakeCount == 2);

	UsbCtrlrConnect(0);
	TestUsbSuspend(true);
	LPC_SYSCON->USBCLKCTRL = 3; // caller already owns AP_CLK
	UsbCtrlrSofEnable(0, true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 3);
	TestUsbSuspend(false);
	assert(LPC_SYSCON->USBCLKCTRL == 3 && (LPC_USB->INTEN & (1U << 30)) != 0);
	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	UsbCtrlrStop(0);
	assert(LPC_SYSCON->USBCLKCTRL == 3 && LPC_USB->INTEN == 0);
	assert(UsbCtrlrStart(0));
	UsbCtrlrConnect(0);
	TestUsbSuspend(true);
	LPC_SYSCON->USBCLKCTRL = 2;
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 4);
	UsbCtrlrStop(0);
	assert(LPC_SYSCON->USBCLKCTRL == 2);
	puts("LPC11Uxx remote wake delay, cancellation and clock ownership: PASS");
}
