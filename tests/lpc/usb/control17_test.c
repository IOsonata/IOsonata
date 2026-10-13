/* LPC1769 control event regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC17xx.h"
#include "usb_control_lpc17xx.h"
#include "usb_hw_lpc17xx.h"

static Lpc17UsbControl_t s_Control;
static uint32_t s_Mask;
static uint8_t s_Status[2];
static uint8_t s_Current;
static uint8_t s_Packet[64];
static int s_Pending;
static int s_Length;
static int s_Written;
static unsigned s_Events;
static unsigned s_Faults;
static unsigned s_Stalls;
static UsbCtrlrEvt_t s_Event[8];
static uint8_t s_Received[64];
static bool s_FailRead;
static bool s_FailWrite;
static bool s_ResetCallback;
static bool s_ArmStatus;
static bool s_SendCallback;

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
static void Event(const UsbCtrlrEvt_t *pEvent)
{
	assert(s_Mask == 1 && s_Events < 8);
	s_Event[s_Events++] = *pEvent;
	if (pEvent->Type == USB_CTRLR_EVT_XFER_CMPL && pEvent->Xfer.pBuffer != NULL)
		memcpy(s_Received, pEvent->Xfer.pBuffer, pEvent->Xfer.Length);
	if (s_ResetCallback)
		Lpc17UsbControlReset(&s_Control, false);
	else if (s_ArmStatus)
		assert(Lpc17UsbControlStatus(&s_Control, 0));
	else if (s_SendCallback)
		assert(Lpc17UsbControlSend(&s_Control, s_Packet, 7) == 7);
}
static void Fault(void)
{
	assert(!s_Control.Enabled && s_Mask == 1);
	++s_Faults;
}
bool Lpc17UsbSieRead(uint8_t Command, uint8_t *pData)
{
	assert(Command == 0 && s_Mask == 1);
	*pData = s_Current;
	return !s_FailRead;
}
bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data)
{
	assert((Command == 0x40 || Command == 0x41) && Data == 1 && s_Mask == 1);
	++s_Stalls;
	return !s_FailWrite;
}
bool Lpc17UsbEp0Ack(bool In, uint8_t *pStatus)
{
	assert(In && s_Mask == 1);
	*pStatus = 0;
	return !s_FailRead;
}
bool Lpc17UsbEp0Write(const uint8_t *pData, int Length)
{
	assert(s_Mask == 1 && Length <= 64 && (Length == 0 || pData != NULL));
	s_Written = Length;
	return !s_FailWrite;
}
int Lpc17UsbEp0Read(uint8_t *pData, int Capacity)
{
	assert(s_Mask == 1 && Capacity == 64);
	if (s_Length >= 0)
		memcpy(pData, s_Packet, s_Length);
	return s_Length;
}
int Lpc17UsbEp0Collect(uint8_t Status[2])
{
	assert(s_Mask == 1);
	memcpy(Status, s_Status, 2);
	const int pending = s_Pending;
	s_Pending = 0;
	return pending;
}
static void Reset(void)
{
	s_Events = 0;
	s_Faults = 0;
	s_Stalls = 0;
	s_Current = 0;
	s_Pending = 0;
	s_Status[0] = 0;
	s_Status[1] = 0;
	s_FailRead = false;
	s_FailWrite = false;
	s_ResetCallback = false;
	s_ArmStatus = false;
	s_SendCallback = false;
	s_Mask = 0;
	Lpc17UsbControlReset(&s_Control, true);
	assert(s_Control.Enabled && s_Mask == 0);
}
static void Setup(bool In, uint16_t Length)
{
	UsbSetupData_t setup = {0};
	setup.bmRequestType = In ? 0x80 : 0;
	setup.wLength = Length;
	memcpy(s_Packet, &setup, sizeof(setup));
	s_Length = sizeof(setup);
	s_Status[0] = 5;
	s_Pending |= 1;
	Lpc17UsbControlInterrupt(&s_Control);
}
int main(void)
{
	assert(mmap((void *)0x5000C000, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x5000C000);
	s_Control.Event = Event;
	s_Control.Fault = Fault;
	Reset();
	Setup(true, 128);
	assert(s_Events == 1 && s_Event[0].Type == USB_CTRLR_EVT_SETUP);
	assert(s_Control.DirectionIn && s_Control.RemainingOut == 0);
	assert(Lpc17UsbControlSend(&s_Control, s_Packet, 128) == 64 && s_Written == 64);
	assert(Lpc17UsbControlSend(&s_Control, s_Packet, 1) == -1);
	s_Pending = 2;
	s_Status[1] = 0x10; // NAK is not completion
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 1 && s_Control.InBusy);
	s_Pending = 2;
	s_Status[1] = 1; // full buffer is not completion
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 1 && s_Control.InBusy);
	s_Pending = 2;
	s_Status[1] = 0;
	s_SendCallback = true;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 2 && s_Event[1].Xfer.Length == 64 && s_Control.InLength == 7);
	s_SendCallback = false;
	s_ArmStatus = true;
	s_Pending = 3;
	s_Status[0] = 1;
	s_Length = 0;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 4 && s_Event[2].Xfer.EpAddr == 0x80);
	assert(s_Event[3].Xfer.EpAddr == 0 && s_Event[3].Xfer.Result == USB_CTRLR_XFER_SUCCESS);
	Reset();
	Setup(false, 65);
	memset(s_Packet, 0xA5, sizeof(s_Packet));
	s_Status[0] = 1;
	s_Pending = 1;
	s_Length = 64;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Control.RemainingOut == 1 && s_Received[63] == 0xA5);
	s_Pending = 1;
	s_Length = 1;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Control.RemainingOut == 0 && s_Event[2].Xfer.Length == 1);
	assert(Lpc17UsbControlStatus(&s_Control, 0x80));
	s_Pending = 2;
	Setup(true, 8); // acknowledged status and next SETUP share one IRQ
	assert(s_Events == 5 && s_Event[3].Type == USB_CTRLR_EVT_XFER_CMPL);
	assert(s_Event[4].Type == USB_CTRLR_EVT_SETUP && !s_Control.InBusy);
	assert(Lpc17UsbControlSend(&s_Control, NULL, 0) == 0);
	s_Pending = 2;
	Setup(false, 0); // data ZLP invalidation must not become status completion
	assert(s_Events == 6 && s_Event[5].Type == USB_CTRLR_EVT_SETUP);
	Reset();
	Setup(false, 1);
	s_Status[0] = 1;
	s_Length = 2;
	s_Pending = 1;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Event[1].Xfer.Result == USB_CTRLR_XFER_CANCELLED);
	Reset();
	s_Length = -2; // overwritten during FIFO copy
	s_Status[0] = 5;
	s_Pending = 3;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 0 && s_Faults == 0);
	Setup(true, 1);
	assert(s_Events == 1);
	Reset();
	Setup(false, 0);
	assert(Lpc17UsbControlStatus(&s_Control, 0x80));
	s_Current = 4; // SETUP arrives after the interrupt snapshot
	s_Pending = 2;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 2 && s_Event[1].Xfer.Result == USB_CTRLR_XFER_SUCCESS);
	assert(!s_Control.InBusy);
	s_Current = 0;
	Setup(true, 8);
	assert(s_Events == 3 && s_Event[2].Type == USB_CTRLR_EVT_SETUP);
	Reset();
	s_Length = 7; // malformed SETUP
	s_Status[0] = 5;
	s_Pending = 1;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Stalls == 2 && s_Control.Halted && s_Events == 0);
	Setup(true, 1); // next SETUP recovers the control stall
	assert(!s_Control.Halted && s_Events == 1);
	Reset();
	assert(Lpc17UsbControlStatus(&s_Control, 0x81) == false);
	assert(Lpc17UsbControlSend(&s_Control, NULL, 1) == -1);
	assert(Lpc17UsbControlSend(&s_Control, NULL, -1) == -1);
	s_Current = 4; // newer SETUP while previous interrupt is being serviced
	s_Length = 8;
	s_Status[0] = 5;
	s_Pending = 1;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 0 && Lpc17UsbControlSend(&s_Control, NULL, 0) == -1);
	Reset();
	Setup(true, 1);
	assert(Lpc17UsbControlSend(&s_Control, s_Packet, 1) == 1);
	s_ResetCallback = true;
	s_Pending = 3;
	s_Status[0] = 1;
	s_Length = 0;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Events == 2 && !s_Control.Enabled); // no OUT after callback reset
	Reset();
	s_Pending = -1;
	s_Mask = 1;
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Faults == 1 && !s_Control.Enabled && s_Mask == 1);
	Lpc17UsbControlInterrupt(&s_Control);
	assert(s_Faults == 1);
	Reset();
	s_FailWrite = true;
	assert(Lpc17UsbControlSend(&s_Control, NULL, 0) == -1 && s_Faults == 1);
	Reset();
	s_FailRead = true;
	assert(Lpc17UsbControlSend(&s_Control, NULL, 0) == -1 && s_Faults == 1);
	puts("LPC1769 control SETUP, IN/OUT events, callback reset, overwrite and faults: PASS");
}
