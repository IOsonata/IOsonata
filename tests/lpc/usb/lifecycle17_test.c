/* LPC1769 public USB lifecycle regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lpc_iap.h"
#include <sys/mman.h>
#include "usb/usb.h"
#include "usb_control_lpc17xx.h"
#include "coredev/system_core_clock.h"
#include "LPC17xx.h"
#include "usb_data_lpc17xx.h"

McuOsc_t g_McuOsc = {{OSC_TYPE_XTAL, 16000000, 0, 0}, {0}, false};
typedef void (*IAPENTRY)(IAP_CMD_STATUS *, IAP_CMD_STATUS *);
extern IAPENTRY IapEntry;
static unsigned s_UidCalls;
static bool s_UidFail;
static Lpc17UsbDataEndpoint_t *s_pData;
static unsigned s_Realized;
static unsigned s_StatusWrites;
static unsigned s_Failed;
static unsigned s_Cancelled;
static unsigned s_Kicks;
static bool s_StatusFail;
static bool s_RearmFailed;
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
static void ReadUid(IAP_CMD_STATUS *pCommand, IAP_CMD_STATUS *pResult)
{
	assert(s_Mask == 1 && pCommand->CmdStatus == 58);
	++s_UidCalls;
	pResult->CmdStatus = s_UidFail ? IAPSTATUS_BUSY : IAPSTATUS_SUCCESS;
	pResult->Data[0] = 0x01234567;
	pResult->Data[1] = 0x89ABCDEF;
	pResult->Data[2] = 0x00000000;
	pResult->Data[3] = 0xFFFFFFFF;
}
static void SerialTest(void)
{
	char buffer[40];
	const char *expected = "0123456789ABCDEF00000000FFFFFFFF";

	assert((uintptr_t)IapEntry == 0x1FFF1FF1);
	IapEntry = ReadUid;
	assert(UsbCtrlrGetSerial(0, NULL, 40) == 0 && s_UidCalls == 0);
	for (unsigned mask = 0; mask <= 1; ++mask)
	{
		s_Mask = mask;
		for (size_t length = 0; length < sizeof(buffer); ++length)
		{
			memset(buffer, '?', sizeof(buffer));
			size_t count = length > 32 ? 32 : length ? length - 1 : 0;
			s_UidCalls = 0;
			assert(UsbCtrlrGetSerial(0, buffer, length) == count);
			assert(s_UidCalls == (length > 1) && s_Mask == mask);
			assert(memcmp(buffer, expected, count) == 0);
			if (length != 0)
				assert(buffer[count] == 0);
			for (size_t i = length ? count + 1 : 0; i < sizeof(buffer); ++i)
				assert(buffer[i] == '?');
		}
		s_UidCalls = 0;
		assert(UsbCtrlrGetSerial(-1, buffer, sizeof(buffer)) == 0 && buffer[0] == 0);
		assert(UsbCtrlrGetSerial(1, buffer, sizeof(buffer)) == 0 && s_UidCalls == 0);
		s_UidFail = true;
		memset(buffer, '?', sizeof(buffer));
		assert(UsbCtrlrGetSerial(0, buffer, sizeof(buffer)) == 0);
		assert(buffer[0] == 0 && buffer[1] == '?' && s_Mask == mask);
		s_UidFail = false;
	}
	s_Mask = 0;
	assert(s_PinWrites == 0 && !s_Prepared);
	puts("LPC1769 ROM UID serial, truncation, failure and interrupt restoration: PASS");
}
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
void NVIC_SetPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	++s_Kicks;
}
bool Lpc17UsbDataRealize(uint8_t EpNo, bool In, uint8_t Type, uint16_t Size)
{
	assert(EpNo > 0 && EpNo < 16 && Size == 64);
	assert(Type == (EpNo % 3 == 1 ? INT : BULK));
	assert(LPC_USB->USBDevIntEn == 0 && LPC_USB->USBEpIntEn == 0);
	LPC_USB->USBReEp |= 1UL << (EpNo * 2 + In);
	++s_Realized;
	return true;
}
bool Lpc17UsbDataSetStatus(uint8_t EpNo, bool In, uint8_t Status, bool Initializing)
{
	assert((LPC_USB->USBEpIntEn & (1UL << (EpNo * 2 + In))) == 0);
	assert(Status == 0 || Status == 1 || Status == 0x20);
	if (Initializing)
		assert(Status == 0x20 && LPC_USB->USBEpIntEn == 0 && LPC_USB->USBDevIntEn == 0);
	++s_StatusWrites;
	return !s_StatusFail;
}
bool Lpc17UsbIrqBindData(Lpc17UsbDataEndpoint_t Endpoint[30])
{
	assert(s_Bound);
	s_pData = Endpoint;
	return true;
}
bool Lpc17UsbDataWrite(uint8_t EpNo, const uint8_t *pData, int Length)
{
	assert(EpNo == 2 && Length <= 64 && (pData != NULL || Length == 0));
	return true;
}
int Lpc17UsbDataRead(uint8_t EpNo, uint8_t *pData, int Capacity)
{
	(void)EpNo;
	(void)pData;
	(void)Capacity;
	return -2;
}
static void DataEvent(UsbCtrlrEvtType_t Event, uint16_t Length, void *pContext)
{
	(void)Length;
	assert(pContext == s_pData);
	if (Event == USB_CTRLR_EVT_XFER_FAILED)
		{
		++s_Failed;
		if (s_RearmFailed)
			assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	}
	else
	{
		assert(Event == USB_CTRLR_EVT_CANCEL);
		++s_Cancelled;
		assert(!UsbCtrlrEpSend(0, 2, NULL, 0));
	}
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
	assert((Command == 0xFE || Command == 0xD8) && Data <= 1);
	return !s_SieFail;
}
void Lpc17UsbControlStall(Lpc17UsbControl_t *pControl)
{
	pControl->Halted = true;
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
	// No peripheral mappings or controller initialization are needed for UID.
	SerialTest();
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
	assert(s_Realized >= 22);
	UsbCtrlrEpBind(0, 2, true, false, DataEvent, s_pData);
	assert(!UsbCtrlrEpOpenData(0, 1, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(0, 3, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(0, 2, true, BULK, 7));
	assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
	assert(!UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
	assert(UsbCtrlrEpOpenData(0, 1, false, INT, 13));
	assert((LPC_USB->USBEpIntEn & 0x24) == 0x24);
	assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	UsbCtrlrEpStall(0, 2, true);
	assert(s_Failed == 1 && s_pData[3].Halted && !s_pData[3].Busy);
	assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	UsbCtrlrEpClearStall(0, 2, true);
	assert(!s_pData[3].Halted && s_pData[3].Busy);
	s_RearmFailed = true;
	const unsigned writes = s_StatusWrites;
	UsbCtrlrEpClearStall(0, 2, true);
	assert(s_StatusWrites == writes + 1 && s_Failed == 2);
	assert(!s_pData[3].Halted && s_pData[3].Busy);
	s_RearmFailed = false;
	UsbCtrlrEpClose(0, 2, true);
	assert(s_Cancelled == 1 && !s_pData[3].Enabled);
	assert((LPC_USB->USBEpIntEn & 0x20) == 0);
	assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 8));
	assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	UsbCtrlrEpCloseAll(0);
	assert(s_Cancelled == 2 && LPC_USB->USBEpIntEn == 3);
	assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
	assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	UsbCtrlrStop(0);
	assert(s_Cancelled == 3 && !s_pData[3].Enabled);
	assert(UsbCtrlrStart(0));
	assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
	assert(UsbCtrlrEpSend(0, 2, NULL, 0));
	s_StatusFail = true;
	UsbCtrlrEpClose(0, 2, true);
	assert(s_Cancelled == 3 && s_pData[3].Busy && !s_pControl->Enabled);
	UsbCtrlrProcess(0);
	assert(s_Cancelled == 4 && !s_pData[3].Busy);
	s_StatusFail = false;
	for (unsigned clear = 0; clear < 2; ++clear)
	{
		assert(UsbCtrlrStart(0));
		assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
		assert(UsbCtrlrEpSend(0, 2, NULL, 0));
		const unsigned statuses = s_StatusWrites;
		const unsigned cancelled = s_Cancelled;
		*(volatile uint32_t *)&LPC_USB->USBDevIntSt = 8;
		if (clear)
			UsbCtrlrEpClearStall(0, 2, true);
		else
			UsbCtrlrEpStall(0, 2, true);
		assert(!s_pControl->Enabled && !s_Irq);
		assert(s_StatusWrites == statuses && s_pData[3].Busy);
		assert(s_Cancelled == cancelled && s_ConnectMux == IOPINOP_GPIO);
		UsbCtrlrProcess(0);
		assert(s_Cancelled == cancelled + 1 && !s_pData[3].Busy);
		*(volatile uint32_t *)&LPC_USB->USBDevIntSt = 0;
	}
	puts("LPC1769 public data open/close, halt/clear, stop and fault retirement: PASS");
	puts("LPC1769 public lifecycle, pin ownership, priority clamp and failure cleanup: PASS");
}
