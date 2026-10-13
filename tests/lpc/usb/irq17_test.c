/* LPC1769 top-level USB IRQ regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "LPC17xx.h"
#include "usb_control_lpc17xx.h"

static Lpc17UsbControl_t s_Control;
static uint32_t s_Mask;
static int s_Result;
static uint8_t s_Status;
static unsigned s_Events;
static unsigned s_Faults;
static unsigned s_Endpoints;
static unsigned s_Resets;
static unsigned s_Clears;
static unsigned s_Sets;
static UsbCtrlrEvtType_t s_LastEvent;
static bool s_Unbind;
static bool s_Disable;
static bool s_Arrival;
static bool s_KeepPending;

void USB_IRQHandler(void);
uint32_t TestGetPrimask(void)
{
	return s_Mask;
}
void TestDisableIRQ(void)
{
	s_Mask = 1;
}
void TestSetPrimask(uint32_t State)
{
	s_Mask = State;
}
static void SetReg(volatile const uint32_t *pReg, uint32_t Value)
{
	*(volatile uint32_t *)pReg = Value;
}
void TestIrqClear(uint32_t Value)
{
	assert(s_Mask == 1 && Value == 4 && LPC_USB->USBEpIntSt == 0);
	++s_Clears;
	SetReg(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt & ~Value);
	if (s_Arrival)
		SetReg(&LPC_USB->USBEpIntSt, 1);
}
void TestIrqSet(uint32_t Value)
{
	assert(s_Mask == 1 && Value == 4);
	++s_Sets;
	SetReg(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | Value);
}
static void Event(const UsbCtrlrEvt_t *pEvent)
{
	assert(s_Mask == 1);
	++s_Events;
	s_LastEvent = pEvent->Type;
	if (s_Unbind)
		Lpc17UsbIrqUnbind();
	if (s_Disable)
		s_Control.Enabled = false;
}
static void Fault(void)
{
	assert(s_Mask == 1 && !s_Control.Enabled);
	assert(LPC_USB->USBDevIntEn == 0 && LPC_USB->USBEpIntEn == 0);
	++s_Faults;
}
void Lpc17UsbControlReset(Lpc17UsbControl_t *pControl, bool Enable)
{
	pControl->Enabled = Enable;
	++pControl->Generation;
}
int Lpc17UsbDeviceStatus(uint8_t *pStatus)
{
	assert(s_Mask == 1);
	SetReg(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt & ~8U);
	*pStatus = s_Status;
	return s_Result;
}
void Lpc17UsbControlBusReset(Lpc17UsbControl_t *pControl)
{
	assert(pControl == &s_Control && s_Mask == 1);
	++s_Resets;
	SetReg(&LPC_USB->USBEpIntSt, 0);
	const UsbCtrlrEvt_t event = {.Type = USB_CTRLR_EVT_RESET};
	Event(&event);
}
void Lpc17UsbControlInterrupt(Lpc17UsbControl_t *pControl)
{
	assert(pControl == &s_Control && s_Mask == 1);
	++s_Endpoints;
	if (!s_KeepPending)
		SetReg(&LPC_USB->USBEpIntSt, 0);
	if (s_Unbind)
		Lpc17UsbIrqUnbind();
}
static void Prepare(void)
{
	Lpc17UsbIrqUnbind();
	s_Control.Enabled = true;
	s_Control.Event = Event;
	s_Control.Fault = Fault;
	s_Mask = 0;
	assert(!Lpc17UsbIrqBind(NULL));
	assert(Lpc17UsbIrqBind(&s_Control) && s_Mask == 0);
	assert(!Lpc17UsbIrqBind(&s_Control));
	LPC_SC->PCONP = 1UL << 31;
	LPC_USB->USBDevIntEn = 0x0C;
	LPC_USB->USBEpIntEn = 3;
	SetReg(&LPC_USB->USBDevIntSt, 0x3C); // includes SIE handshake flags
	SetReg(&LPC_USB->USBEpIntSt, 3);
	s_Result = 1;
	s_Status = 1;
	s_Events = 0;
	s_Faults = 0;
	s_Endpoints = 0;
	s_Resets = 0;
	s_Clears = 0;
	s_Sets = 0;
	s_Unbind = false;
	s_Disable = false;
	s_Arrival = false;
	s_KeepPending = false;
}
int main(void)
{
	assert(mmap((void *)0x5000C000, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x5000C000);
	assert(mmap((void *)0x400FC000, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x400FC000);
	Prepare();
	s_Status = 0x1D; // reset and suspend reported together
	USB_IRQHandler();
	assert(s_Resets == 1 && s_Endpoints == 0 && s_Events == 1);
	assert(s_LastEvent == USB_CTRLR_EVT_RESET && s_Clears == 1 && s_Mask == 0);
	assert(LPC_USB->USBDevIntSt == 0x30); // handshake flags are untouched
	Prepare();
	s_Status = 0x0D;
	USB_IRQHandler();
	assert(s_LastEvent == USB_CTRLR_EVT_SUSPEND && s_Endpoints == 1);
	Prepare();
	s_Status = 9;
	s_Mask = 1;
	USB_IRQHandler();
	assert(s_LastEvent == USB_CTRLR_EVT_RESUME && s_Mask == 1);
	Prepare();
	s_Status = 9;
	s_Unbind = true;
	USB_IRQHandler();
	assert(s_Events == 1 && s_Endpoints == 0 && s_Clears == 0);
	USB_IRQHandler();
	assert(s_Events == 1);
	Prepare();
	s_Status = 9;
	s_Disable = true;
	USB_IRQHandler();
	assert(s_Endpoints == 0 && s_Clears == 0);
	Prepare();
	s_Arrival = true;
	USB_IRQHandler();
	assert(s_Clears == 1 && s_Sets == 1 && (LPC_USB->USBDevIntSt & 4) != 0);
	Prepare();
	s_KeepPending = true;
	USB_IRQHandler();
	assert(s_Endpoints == 1 && s_Clears == 0);
	Prepare();
	s_Result = -1;
	USB_IRQHandler();
	assert(s_Faults == 1 && s_Endpoints == 0);
	Prepare();
	s_Status = 2; // VBUS loss
	USB_IRQHandler();
	assert(s_Faults == 1 && s_Endpoints == 0);
	Prepare();
	LPC_USB->USBDevIntEn |= 0x10; // handshake IRQs are not supported
	USB_IRQHandler();
	assert(s_Faults == 1);
	Prepare();
	LPC_SC->PCONP = 0;
	USB_IRQHandler();
	assert(s_Faults == 0 && s_Endpoints == 0 && s_Events == 0);
	puts("LPC1769 USB IRQ reset priority, bus events, callback stop and W1C race: PASS");
}
