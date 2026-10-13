/* LPC54628 USB lifecycle regression. MIT License, I-SYST, 2026. */
#include <limits.h>
#include "integration546_model.c"

static unsigned s_Queued;

void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
	++s_Queued;
}

void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvent)
{
	assert(DevNo == 0);
	Event(pEvent);
}

int main(void)
{
	(void)Fresh;
	(void)Setup;
	(void)Done;
	assert(!UsbCtrlrStart(0));
	assert(!UsbCtrlrVbusDetected(0));
	UsbCtrlrProcess(0);
	UsbCtrlrStop(0);
	TestUsbInitialize();
	SYSCON->PDRUNCFG[0] |= SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK;
	const uint32_t power = SYSCON->PDRUNCFG[0];
	char serial[33];
	assert(UsbCtrlrGetSerial(0, serial, sizeof(serial)) == 32);
	assert(strcmp(serial, "12345678123456791234567A1234567B") == 0);
	assert(SYSCON->PDRUNCFG[0] == power && SYSCON->AHBCLKCTRL[0] == 0);
	IOPinCfg_t pin = {0, 22, IOPINOP_FUNC7, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	UsbCtrlrCfg_t cfg = {INT_MAX, &pin, 1, false};
	assert(!UsbCtrlrInit(1, &cfg) && !UsbCtrlrInit(0, NULL));
	pin.PinNo = 21;
	assert(!UsbCtrlrInit(0, &cfg));
	pin.PinNo = 22;
	cfg.bLowPowerSuspend = true;
	assert(!UsbCtrlrInit(0, &cfg));
	cfg.bLowPowerSuspend = false;
	assert(UsbCtrlrInit(0, &cfg));
	TestUsbCable(false, false);
	assert(!UsbCtrlrStart(0));
	assert((SYSCON->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_USB0D_MASK) == 0);
	TestUsbCable(true, false);
	TestUsbVbusInterrupt();
	assert(s_Queued == 1 && UsbCtrlrStart(0));
	assert(s_Priority == (1U << __NVIC_PRIO_BITS) - 1U);
	assert(!UsbCtrlrInit(0, &cfg));
	UsbCtrlrIntEnable(0);
	UsbCtrlrConnect(0);
	assert(s_Enabled && (USB0->DEVCMDSTAT & (1U << 16)) != 0);
	UsbCtrlrSofEnable(0, true);
	USB0->INTSTAT |= 1U << 30;
	USB0_IRQHandler();
	assert(s_Count == 1 && s_Events[0].Type == USB_CTRLR_EVT_SOF);
	UsbCtrlrSofEnable(0, false);
	TestUsbCable(false, true);
	assert(s_Count == 2 && s_Events[1].Type == USB_CTRLR_EVT_RESET);
	assert(UsbCtrlrEp0Send(0, NULL, 0) == -1);
	UsbCtrlrStop(0);
	assert(!s_Enabled && USB0->INTEN == 0);
	assert((SYSCON->AHBCLKCTRL[1] & SYSCON_AHBCLKCTRL_USB0D_MASK) == 0);
	TestUsbCable(true, false);
	assert(UsbCtrlrStart(0));
	UsbCtrlrStop(0);
	assert(g_IrqMasked == 0);
	puts("LPC54628 USB VBUS, lifecycle, SOF and UID resource restoration: PASS");
	return 0;
}
