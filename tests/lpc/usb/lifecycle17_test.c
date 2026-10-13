/* LPC1769 public USB lifecycle regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "usb/usb.h"
#include "usb_control_lpc17xx.h"
#include "coredev/system_core_clock.h"
#include "LPC17xx.h"

McuOsc_t g_McuOsc = {{OSC_TYPE_XTAL, 16000000, 0, 0}, {0}, false};
static uint32_t s_Mask;
static unsigned s_Priority;
static unsigned s_PinWrites;
static unsigned s_Queued;
static bool s_Irq;
static bool s_InitFail;
static bool s_SieFail;
static bool s_Prepared;
static bool s_Bound;
static Lpc17UsbControl_t *s_pControl;
static int s_ConnectMux;
static const IOPinCfg_t s_Pins[] = {
	{1, 30, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 29, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 30, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 9, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
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
void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
}
void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Irq == USB_IRQn);
	s_Priority = Priority;
}
void NVIC_EnableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn && s_Bound);
	s_Irq = true;
}
void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	s_Irq = false;
}
void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	(void)Dir;
	assert(Res == IOPINRES_NONE && Type == IOPINTYPE_NORMAL);
	++s_PinWrites;
	if (Port == 2 && Pin == 9)
		s_ConnectMux = Op;
}
bool Lpc17UsbHwPrepare(uint32_t Frequency)
{
	assert(Frequency == 16000000 && !s_Prepared);
	s_Prepared = true;
	return true;
}
bool Lpc17UsbDeviceInit(void)
{
	return !s_InitFail;
}
bool Lpc17UsbHwRelease(void)
{
	assert(!s_Irq && LPC_USB->USBReEp == 0 && s_ConnectMux == IOPINOP_GPIO);
	s_Prepared = false;
	return true;
}
bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data)
{
	assert(Command == 0xFE && Data <= 1);
	return !s_SieFail;
}
void Lpc17UsbControlReset(Lpc17UsbControl_t *pControl, bool Enable)
{
	pControl->Enabled = Enable;
}
bool Lpc17UsbIrqBind(Lpc17UsbControl_t *pControl)
{
	assert(!s_Bound && pControl->Enabled);
	s_pControl = pControl;
	s_Bound = true;
	return true;
}
void Lpc17UsbIrqUnbind(void)
{
	assert(!s_Irq);
	s_Bound = false;
}
void UsbProcessQue(int DevNo)
{
	assert(DevNo == 0);
	++s_Queued;
}
void UsbDevProcessEvent(int DevNo, const UsbCtrlrEvt_t *pEvent)
{
	(void)DevNo;
	(void)pEvent;
}
void UsbDisable(int DevNo)
{
	UsbCtrlrStop(DevNo);
}
int main(void)
{
	assert(mmap((void *)0x5000C000, 4096, 3, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
		-1, 0) == (void *)0x5000C000);
	assert(mmap((void *)0x400FC000, 4096, 3, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
		-1, 0) == (void *)0x400FC000);
	assert(mmap((void *)0x2009C000, 4096, 3, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
		-1, 0) == (void *)0x2009C000);
	UsbCtrlrCfg_t config = {.IntPrio = 999, .pIOPinMap = s_Pins, .NbIOPins = 4};
	assert(!UsbCtrlrInit(1, &config) && !UsbCtrlrInit(0, NULL));
	LPC_SC->PCONP = 1UL << 31;
	assert(!UsbCtrlrInit(0, &config) && s_PinWrites == 0);
	LPC_SC->PCONP = 0;
	assert(UsbCtrlrInit(0, &config) && s_Mask == 0);
	assert(!UsbCtrlrStart(0));
	LPC_GPIO1->FIOPIN = 1UL << 30;
	assert(UsbCtrlrStart(0) && s_Prepared && s_Bound && !s_Irq);
	assert(s_Priority == 31 && LPC_USB->USBEpIntEn == 3 && LPC_USB->USBDevIntEn == 12);
	assert(!UsbCtrlrInit(0, &config));
	UsbCtrlrIntEnable(0);
	UsbCtrlrConnect(0);
	assert(s_Irq && s_ConnectMux == IOPINOP_FUNC1);
	UsbCtrlrDisconnect(0);
	assert(s_ConnectMux == IOPINOP_GPIO);
	UsbCtrlrStop(0);
	assert(!s_Prepared && !s_Bound && !s_Irq);
	s_InitFail = true;
	assert(!UsbCtrlrStart(0) && !s_Prepared && !s_Bound);
	s_InitFail = false;
	assert(UsbCtrlrStart(0));
	s_SieFail = true;
	UsbCtrlrConnect(0);
	assert(s_Queued == 1 && !s_pControl->Enabled && s_ConnectMux == IOPINOP_GPIO);
	UsbCtrlrProcess(0);
	assert(!s_Prepared && !s_Bound);
	config.IntPrio = -1;
	assert(UsbCtrlrInit(0, &config));
	s_SieFail = false;
	assert(UsbCtrlrStart(0) && s_Priority == 0);
	UsbCtrlrStop(0);
	puts("LPC1769 public lifecycle, pin ownership, priority clamp and failure cleanup: PASS");
}
