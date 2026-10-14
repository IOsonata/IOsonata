/**-------------------------------------------------------------------------
@file	integration17_model.c

@brief	LPC1769 class integration hardware boundary model.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <assert.h>
#include <string.h>
#include <sys/mman.h>
#include "usb/usb.h"
#include "usb_control_lpc17xx.h"
#include "usb_data_lpc17xx.h"
#include "coredev/system_core_clock.h"
#include "lpc_iap.h"
#include "LPC17xx.h"

McuOsc_t g_McuOsc = {{OSC_TYPE_XTAL, 16000000, 0, 0}, {0}, false};
static uint32_t s_Mask;
static Lpc17UsbDataEndpoint_t *s_Data;
static Lpc17UsbControl_t *s_Control;
static uint8_t s_Rx[16][64];
static int s_RxLength[16];
static uint8_t s_Tx[16][256];
static unsigned s_TxLength[16];
static bool s_InPending[16];
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
void NVIC_SetPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
}
void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
}
void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Irq == USB_IRQn && Priority <= 31);
}
void NVIC_EnableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
}
void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
}
void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	(void)Port;
	(void)Pin;
	(void)Op;
	(void)Dir;
	assert(Res == IOPINRES_NONE && Type == IOPINTYPE_NORMAL);
}
bool Lpc17UsbHwPrepare(uint32_t Frequency)
{
	return Frequency == 16000000;
}
bool Lpc17UsbDeviceInit(void)
{
	LPC_USB->USBReEp = 3;
	return true;
}
bool Lpc17UsbHwRelease(void)
{
	assert(LPC_USB->USBReEp == 0);
	return true;
}
bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data)
{
	assert((Command == 0xFE || Command == 0xD8) && Data <= 1);
	return true;
}
bool Lpc17UsbDataRealize(uint8_t EpNo, bool In, uint8_t Type, uint16_t Size)
{
	assert(EpNo > 0 && EpNo < 16 && Size == 64);
	assert((USB_EPIN_TYPE_MASK(0, Type) & (1U << EpNo)) != 0);
	LPC_USB->USBReEp |= 1UL << (EpNo * 2 + In);
	return true;
}
bool Lpc17UsbDataSetStatus(uint8_t EpNo, bool In, uint8_t Status, bool Initializing)
{
	assert((LPC_USB->USBEpIntEn & (1UL << (EpNo * 2 + In))) == 0);
	assert(Status == 0 || Status == 1 || Status == 0x20);
	(void)Initializing;
	if (In)
		s_InPending[EpNo] = false;
	else
		s_RxLength[EpNo] = -2;
	return true;
}
bool Lpc17UsbDataWrite(uint8_t EpNo, const uint8_t *pData, int Length)
{
	assert(s_Mask == 1 && EpNo > 0 && EpNo < 16 && Length >= 0 && Length <= 64);
	assert(!s_InPending[EpNo] && s_TxLength[EpNo] + Length <= 256);
	if (Length != 0)
		memcpy(s_Tx[EpNo] + s_TxLength[EpNo], pData, Length);
	s_TxLength[EpNo] += Length;
	s_InPending[EpNo] = true;
	return true;
}
int Lpc17UsbDataRead(uint8_t EpNo, uint8_t *pData, int Capacity)
{
	assert(s_Mask == 1);
	const int length = s_RxLength[EpNo];
	if (length > Capacity)
		return -1;
	if (length >= 0)
	{
		memcpy(pData, s_Rx[EpNo], length);
		s_RxLength[EpNo] = -2;
	}
	return length;
}
bool Lpc17UsbIrqBind(Lpc17UsbControl_t *pControl)
{
	s_Control = pControl;
	return true;
}
bool Lpc17UsbIrqBindData(Lpc17UsbDataEndpoint_t Endpoint[30])
{
	s_Data = Endpoint;
	return true;
}
void Lpc17UsbIrqUnbind(void)
{
	s_Control = NULL;
}
void Lpc17UsbControlReset(Lpc17UsbControl_t *pControl, bool Enable)
{
	pControl->Enabled = Enable;
	pControl->Halted = false;
}
void Lpc17UsbControlStall(Lpc17UsbControl_t *pControl)
{
	pControl->Halted = true;
}
int Lpc17UsbControlSend(Lpc17UsbControl_t *pControl, const uint8_t *pData, int Length)
{
	(void)pData;
	assert(pControl->Enabled);
	return Length < 64 ? Length : 64;
}
bool Lpc17UsbControlStatus(Lpc17UsbControl_t *pControl, uint8_t EpAddr)
{
	(void)EpAddr;
	return pControl->Enabled;
}
bool Lpc17UsbControlSetAddress(Lpc17UsbControl_t *pControl, uint8_t Address)
{
	(void)Address;
	return pControl->Enabled;
}
IAPSTATUS IAPReadUID(IAP_CMD_STATUS *pResult)
{
	memset(pResult->Data, 0x12, 16);
	pResult->CmdStatus = IAPSTATUS_SUCCESS;
	return IAPSTATUS_SUCCESS;
}
void Test17Init(UsbCfg_t *pConfig)
{
	const uintptr_t regions[] = {0x5000C000, 0x400FC000, 0x2009C000};
	for (unsigned i = 0; i < 3; ++i)
	{
		void *p = (void *)regions[i];
		assert(mmap(p, 4096, 3, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == p);
	}
	LPC_GPIO1->FIOPIN = 1UL << 30;
	pConfig->pIOPinMap = s_Pins;
	pConfig->NbIOPins = 4;
	memset(s_TxLength, 0, sizeof(s_TxLength));
	memset(s_InPending, 0, sizeof(s_InPending));
}
void Test17Setup(const UsbSetupData_t *pSetup)
{
	UsbCtrlrEvt_t event = {.Type = USB_CTRLR_EVT_SETUP};
	event.Setup = *pSetup;
	s_Control->Halted = false;
	s_Control->Event(&event);
	assert(!s_Control->Halted);
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = 0x80;
	event.Xfer.Length = 0;
	event.Xfer.Result = USB_CTRLR_XFER_SUCCESS;
	s_Control->Event(&event);
}
void Test17Out(unsigned EpNo, const uint8_t *pData, unsigned Length)
{
	assert(Length <= 64 && s_RxLength[EpNo] == -2);
	memcpy(s_Rx[EpNo], pData, Length);
	s_RxLength[EpNo] = Length;
	Lpc17UsbDataEndpoint_t *ep = &s_Data[EpNo * 2 - 2];
	Lpc17UsbDataDispatch(ep, ep->Generation, 1);
}
void Test17InDrain(unsigned EpNo, const uint8_t *pExpected, unsigned Length)
{
	for (unsigned i = 0; i < 8 && s_InPending[EpNo]; ++i)
	{
		s_InPending[EpNo] = false;
		Lpc17UsbDataEndpoint_t *ep = &s_Data[EpNo * 2 - 1];
		Lpc17UsbDataDispatch(ep, ep->Generation, 0);
	}
	assert(!s_InPending[EpNo] && s_TxLength[EpNo] == Length);
	assert(memcmp(s_Tx[EpNo], pExpected, Length) == 0);
	s_TxLength[EpNo] = 0;
}
