/* LPC1769 top-level USB IRQ regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "LPC17xx.h"
#include "usb_control_lpc17xx.h"
#include "usb_data_lpc17xx.h"
#include <string.h>

static Lpc17UsbControl_t s_Control;
static Lpc17UsbDataEndpoint_t s_Data[30];
static unsigned s_DataCalls;
static unsigned s_Forgotten;
static unsigned s_DataAction;
static bool s_CollectFail;
static bool s_ResetUnbind;
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
		SetReg(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt & ~3U);
	if (s_Unbind)
		Lpc17UsbIrqUnbind();
}
bool Lpc17UsbDataCollect(uint32_t *pMask, uint8_t Status[32])
{
	assert(s_Mask == 1);
	if (s_CollectFail)
		return false;
	*pMask = LPC_USB->USBEpIntSt & LPC_USB->USBEpIntEn & ~3U;
	memset(Status, 0, 32);
	SetReg(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt & ~*pMask);
	return true;
}
void Lpc17UsbDataDispatch(Lpc17UsbDataEndpoint_t *pEp, uint32_t Generation, uint8_t Status)
{
	assert(s_Mask == 1 && pEp->Generation == Generation && Status == 0);
	++s_DataCalls;
	if (s_DataAction == 1)
		++s_Data[29].Generation; // Callback rearmed another captured endpoint.
	if (s_DataAction == 2)
		Lpc17UsbIrqUnbind();
	if (s_DataAction == 3)
		SetReg(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | 8);
	if (s_DataAction == 5)
		++s_Control.Generation; // Same binding, restarted controller.
	if (s_DataAction == 4)
		SetReg(&LPC_USB->USBEpIntSt, 4); // OUT Kick requires another IRQ pass.
}
void Lpc17UsbDataForget(Lpc17UsbDataEndpoint_t *pEp)
{
	assert(s_Mask == 1 && !s_Control.Enabled && (LPC_USB->USBReEp & ~3U) == 0);
	++s_Forgotten;
	pEp->Enabled = false;
	pEp->Busy = false;
	++pEp->Generation;
	if (s_ResetUnbind)
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
	s_DataCalls = 0;
	s_Forgotten = 0;
	s_DataAction = 0;
	s_CollectFail = false;
	s_ResetUnbind = false;
	LPC_USB->USBReEp = 3;
	SetReg(&LPC_USB->USBEpDMASt, 0);
	LPC_USB->USBDMAIntEn = 0;
}
static void BindData(void)
{
	memset(s_Data, 0, sizeof(s_Data));
	for (unsigned i = 0; i < 30; ++i)
	{
		s_Data[i].Enabled = true;
		s_Data[i].EpNo = (i + 2) / 2;
		s_Data[i].In = (i & 1) != 0;
	}
	assert(!Lpc17UsbIrqBindData(NULL));
	assert(Lpc17UsbIrqBindData(s_Data));
	assert(!Lpc17UsbIrqBindData(s_Data));
	LPC_USB->USBEpIntEn = 0x80000007;
	SetReg(&LPC_USB->USBEpIntSt, 0x80000007);
}
static void DataIrqTests(void)
{
	Prepare();
	BindData();
	s_Mask = 1;
	USB_IRQHandler();
	assert(s_DataCalls == 2 && s_Endpoints == 1 && s_Clears == 1 && s_Mask == 1);
	Prepare();
	BindData();
	s_DataAction = 1;
	USB_IRQHandler();
	assert(s_DataCalls == 1 && s_Clears == 1);
	Prepare();
	BindData();
	s_DataAction = 2;
	USB_IRQHandler();
	assert(s_DataCalls == 1 && s_Clears == 0);
	assert(!Lpc17UsbIrqBindData(s_Data));
	Prepare();
	BindData();
	s_DataAction = 5;
	USB_IRQHandler();
	assert(s_DataCalls == 1);
	Prepare();
	BindData();
	s_DataAction = 3;
	USB_IRQHandler();
	assert(s_DataCalls == 1 && s_Faults == 1 && s_Clears == 0);
	Prepare();
	BindData();
	s_DataAction = 4;
	USB_IRQHandler();
	assert(s_DataCalls == 2 && s_Clears == 0 && LPC_USB->USBEpIntSt == 4);
	Prepare();
	BindData();
	s_CollectFail = true;
	USB_IRQHandler();
	assert(s_DataCalls == 0 && s_Faults == 1);
	Prepare();
	BindData();
	s_Data[29].Enabled = false;
	USB_IRQHandler();
	assert(s_DataCalls == 0 && s_Faults == 1);
	Prepare();
	BindData();
	s_Status = 0x11;
	USB_IRQHandler();
	assert(s_Forgotten == 30 && s_Resets == 1 && s_DataCalls == 0);
	assert(LPC_USB->USBEpIntEn == 3 && s_LastEvent == USB_CTRLR_EVT_RESET);
	Prepare();
	BindData();
	s_Status = 0x11;
	s_ResetUnbind = true;
	USB_IRQHandler();
	assert(s_Forgotten == 1 && s_Resets == 0 && s_DataCalls == 0);
	Prepare();
	BindData();
	s_Status = 0x11;
	LPC_USB->USBReEp = 7; // Hardware retirement is not confirmed.
	USB_IRQHandler();
	assert(s_Forgotten == 0 && s_Resets == 0 && s_Faults == 1);
	Prepare();
	LPC_USB->USBEpIntEn |= 4; // No data binding.
	USB_IRQHandler();
	assert(s_Faults == 1);
	puts("LPC1769 IRQ data dispatch, callback generation, reset retirement and faults: PASS");
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
	DataIrqTests();
	puts("LPC1769 USB IRQ reset priority, bus events, callback stop and W1C race: PASS");
}
