/* LPC54628 remote wake regression. MIT License, I-SYST, 2026. */
#include "integration546_model.c"

static unsigned s_SofEvents;

void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvent)
{
	assert(DevNo == 0);
	if (pEvent->Type == USB_CTRLR_EVT_SOF)
		++s_SofEvents;
}

static void WakeTicks(void)
{
	TestUsbFrame();
	TestUsbFrame();
	TestUsbFrame();
}

int main(void)
{
	(void)Fresh;
	(void)Setup;
	(void)Done;
	UsbCtrlrRemoteWakeup(0);
	TestUsbInitialize();
	const IOPinCfg_t pin = {0, 22, IOPINOP_FUNC7, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	const UsbCtrlrCfg_t cfg = {6, &pin, 1, false};
	assert(UsbCtrlrInit(0, &cfg) && UsbCtrlrStart(0));
	UsbCtrlrConnect(0);
	const uint32_t clock = SYSCON->USB0CLKCTRL;
	// Preparation owns FORCE_NEEDCLK by default. Exercise temporary wake
	// ownership too; AP_FS_DEV_CLK keeps the clock running in this case.
	assert(USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK);
	USB0->DEVCMDSTAT &= ~USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK;
	UsbCtrlrRemoteWakeup(1);
	UsbCtrlrRemoteWakeup(0);
	assert((USB0->INTEN & (1U << 30)) == 0);
	TestUsbSuspend(true);
	USB0->INTSTAT |= 1U << 30;
	UsbCtrlrRemoteWakeup(0);
	assert((USB0->INTSTAT & (1U << 30)) == 0);
	TestUsbFrame();
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	assert(s_WakeCount == 0);
	TestUsbFrame();
	assert(s_WakeCount == 1 && s_SofEvents == 0);
	assert(USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK);
	assert(SYSCON->USB0CLKCTRL == clock);
	UsbCtrlrRemoteWakeup(0);
	WakeTicks();
	assert(s_WakeCount == 1);
	TestUsbSuspend(false);
	assert((USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK) == 0);

	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	TestUsbSuspend(false);
	WakeTicks();
	assert(s_WakeCount == 1);
	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbCable(false, false);
	TestUsbFrame();
	TestUsbCable(true, false);
	WakeTicks();
	assert(s_WakeCount == 1);
	UsbCtrlrRemoteWakeup(0);
	USB0->DEVCMDSTAT |= RESET;
	USB0->INTSTAT |= DEVICE;
	USB0_IRQHandler();
	WakeTicks();
	assert(s_WakeCount == 1);

	TestUsbSuspend(true);
	UsbCtrlrSofEnable(0, true);
	UsbCtrlrRemoteWakeup(0);
	TestUsbFrame();
	UsbCtrlrSofEnable(0, false);
	TestUsbFrame();
	TestUsbFrame();
	assert(s_WakeCount == 2 && s_SofEvents == 1);
	UsbCtrlrDisconnect(0);
	assert((USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK) == 0);
	UsbCtrlrRemoteWakeup(0);
	WakeTicks();
	assert(s_WakeCount == 2);

	UsbCtrlrConnect(0);
	TestUsbSuspend(true);
	USB0->DEVCMDSTAT |= USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK;
	UsbCtrlrSofEnable(0, true);
	UsbCtrlrRemoteWakeup(0);
	WakeTicks();
	TestUsbSuspend(false);
	assert(s_WakeCount == 3);
	assert(USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK);
	assert(USB0->INTEN & (1U << 30));
	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	UsbCtrlrStop(0);
	WakeTicks();
	assert(s_WakeCount == 3 && USB0->INTEN == 0);
	assert(UsbCtrlrStart(0));
	UsbCtrlrConnect(0);
	TestUsbSuspend(true);
	UsbCtrlrRemoteWakeup(0);
	WakeTicks();
	assert(s_WakeCount == 4);
	UsbCtrlrStop(0);
	assert((USB0->DEVCMDSTAT & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK) == 0);
	assert(g_IrqMasked == 0);
	puts("LPC54628 remote wake delay, cancellation and clock ownership: PASS");
}
