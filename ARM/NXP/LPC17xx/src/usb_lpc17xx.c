/**-------------------------------------------------------------------------
@file	usb_lpc17xx.c

@brief	LPC1769 EP0-only public USB controller lifecycle.

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
#include "LPC17xx.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "lpc_iap.h"
#include "usb/usb.h"
#include "usb_hw_lpc17xx.h"
#include "usb_control_lpc17xx.h"

static bool s_Initialized;
static bool s_Prepared;
static bool s_Faulted;
static bool s_Bound;
static int s_IntPrio;
static Lpc17UsbControl_t s_Control;
static const IOPinCfg_t s_Pins[] = {
	{1, 30, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 29, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 30, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 9, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static void Lpc17UsbDisconnectPin(void)
{
	// Active-low external SoftConnect: GPIO can disconnect even after a
	// timed-out SIE command. Set the latch before switching the pin mux.
	IOPinSet(2, 9);
	IOPinConfig(2, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

static void Lpc17UsbFault(void)
{
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->USBDevIntEn = 0U;
	LPC_USB->USBEpIntEn = 0U;
	Lpc17UsbControlReset(&s_Control, false);
	Lpc17UsbDisconnectPin();
	s_Faulted = true;
	UsbProcessQue(0);
}

static void Lpc17UsbEvent(const UsbCtrlrEvt_t *pEvent)
{
	UsbDevProcessEvent(0, pEvent);
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == NULL || pCfg->bLowPowerSuspend ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 4)
		return false;
	for (int i = 0; i < 4; ++i)
	{
		const IOPinCfg_t *pin = &pCfg->pIOPinMap[i];
		const IOPinCfg_t *expected = &s_Pins[i];
		if (pin->PortNo != expected->PortNo || pin->PinNo != expected->PinNo ||
			pin->PinOp != expected->PinOp || pin->PinDir != expected->PinDir ||
			pin->Res != expected->Res || pin->Type != expected->Type)
			return false;
	}
	const uint32_t state = DisableInterrupt();
	if (s_Prepared || (LPC_SC->PCONP & (1UL << 31)) != 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	s_IntPrio = pCfg->IntPrio < 0 ? 0 : (pCfg->IntPrio > maxprio ? maxprio : pCfg->IntPrio);
	NVIC_DisableIRQ(USB_IRQn);
	Lpc17UsbDisconnectPin();
	IOPinCfg(s_Pins, 3);
	s_Control.Event = Lpc17UsbEvent;
	s_Control.Fault = Lpc17UsbFault;
	s_Initialized = true;
	s_Faulted = false;
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	return DevNo == 0 && s_Initialized && IOPinRead(1, 30) != 0;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || !UsbCtrlrVbusDetected(0))
		return false;
	const uint32_t state = DisableInterrupt();
	if (s_Prepared)
	{
		const bool ready = s_Control.Enabled && !s_Faulted;
		EnableInterrupt(state);
		return ready;
	}
	bool ready = Lpc17UsbHwPrepare(GetHighFreqOscFreq());
	if (ready)
	{
		s_Prepared = true;
		ready = Lpc17UsbDeviceInit();
		if (ready)
		{
			Lpc17UsbControlReset(&s_Control, true);
			ready = Lpc17UsbIrqBind(&s_Control);
			s_Bound = ready;
		}
		if (ready)
		{
			NVIC_ClearPendingIRQ(USB_IRQn);
			NVIC_SetPriority(USB_IRQn, s_IntPrio);
			LPC_USB->USBEpIntEn = 3U;
			LPC_USB->USBDevIntEn = 0x0CU;
			s_Faulted = false;
		}
		else
			UsbCtrlrStop(0);
	}
	EnableInterrupt(state);
	return ready;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0 || !s_Initialized)
		return;
	const uint32_t state = DisableInterrupt();
	NVIC_DisableIRQ(USB_IRQn);
	Lpc17UsbDisconnectPin();
	if (s_Bound)
		Lpc17UsbIrqUnbind();
	s_Bound = false;
	Lpc17UsbControlReset(&s_Control, false);
	if (s_Prepared)
	{
		LPC_USB->USBDevIntEn = 0U;
		LPC_USB->USBEpIntEn = 0U;
		// This port never enables DMA. Do not power down someone else's DMA.
		if (LPC_USB->USBEpDMASt == 0U && LPC_USB->USBDMAIntEn == 0U)
		{
			LPC_USB->USBCtrl = 0U;
			LPC_USB->USBReEp = 0U;
			if (Lpc17UsbHwRelease())
				s_Prepared = false;
		}
	}
	NVIC_ClearPendingIRQ(USB_IRQn);
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (DevNo == 0 && s_Faulted)
	{
		s_Faulted = false;
		UsbDisable(0);
	}
}

void UsbCtrlrConnect(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Control.Enabled && UsbCtrlrVbusDetected(0))
	{
		if (Lpc17UsbSieWrite(0xFEU, 1U))
			IOPinCfg(&s_Pins[3], 1);
		else
			Lpc17UsbFault();
	}
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo != 0 || !s_Initialized)
		return;
	const uint32_t state = DisableInterrupt();
	Lpc17UsbDisconnectPin();
	if (s_Prepared)
		(void)Lpc17UsbSieWrite(0xFEU, 0U);
	EnableInterrupt(state);
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (DevNo == 0 && s_Prepared && s_Control.Enabled)
		NVIC_EnableIRQ(USB_IRQn);
}
void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
		NVIC_DisableIRQ(USB_IRQn);
}
int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	return DevNo == 0 ? Lpc17UsbControlSend(&s_Control, pBuffer, Length) : -1;
}
bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	return DevNo == 0 && Lpc17UsbControlStatus(&s_Control, EpAddr);
}
void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (DevNo == 0)
		(void)Lpc17UsbControlSetAddress(&s_Control, Address);
}
bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return DevNo == 0 && pDesc != NULL && s_Control.Enabled &&
		(pDesc->bEndpointAddress & 0x7FU) == 0U &&
		pDesc->bmAttributes == USB_ENDPATT_TRANS_CONTROL && pDesc->wMaxPacketSize == 64;
}
bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool In, uint8_t Type, uint16_t Size)
{
	(void)DevNo;
	(void)EpNo;
	(void)In;
	(void)Type;
	(void)Size;
	return false;
}
void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool In)
{
	(void)In;
	if (DevNo == 0 && EpNo == 0)
		Lpc17UsbControlStall(&s_Control);
}
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool In)
{
	// EP0 stall is cleared by the next SETUP, not endpoint CLEAR_FEATURE.
	(void)DevNo;
	(void)EpNo;
	(void)In;
}
void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool In)
{
	(void)DevNo;
	(void)EpNo;
	(void)In;
}
void UsbCtrlrEpCloseAll(int DevNo)
{
	(void)DevNo;
}
void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool In, bool Blocking,
		UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)DevNo;
	(void)EpNo;
	(void)In;
	(void)Blocking;
	(void)Handler;
	(void)pContext;
}
bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNo;
	(void)pBuffer;
	(void)Length;
	return false;
}
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNo;
	(void)pBuffer;
	(void)Length;
	return false;
}
void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool In, UsbCtrlrEvtType_t Event, uint16_t Value)
{
	(void)DevNo;
	(void)EpNo;
	(void)In;
	(void)Event;
	(void)Value;
}
bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}
void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	(void)DevNo;
	(void)Enable;
}
void UsbCtrlrRemoteWakeup(int DevNo)
{
	(void)DevNo;
}
bool UsbCtrlrIsoInit(int DevNo)
{
	(void)DevNo;
	return false;
}
bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool In, uint16_t Size)
{
	(void)DevNo;
	(void)EpNo;
	(void)In;
	(void)Size;
	return false;
}
bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNo;
	(void)pBuffer;
	(void)Length;
	return false;
}
uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != NULL)
		*ppData = NULL;
	return 0;
}
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (pBuff == NULL || BuffLen == 0U)
		return 0U;
	pBuff[0] = '\0';
	if (DevNo != 0 || BuffLen == 1U)
		return 0U;
	IAP_CMD_STATUS result;
	if (IAPReadUID(&result) != IAPSTATUS_SUCCESS)
		return 0U;
	// Four ROM UID words, eight uppercase hexadecimal digits per word.
	// Usable before Init: the generic core asks for its serial first.
	const size_t count = BuffLen > 32U ? 32U : BuffLen - 1U;
	for (size_t i = 0U; i < count; ++i)
	{
		const uint8_t digit = (result.Data[i / 8U] >> (28U - (i % 8U) * 4U)) & 15U;
		pBuff[i] = digit < 10U ? '0' + digit : 'A' + digit - 10;
	}
	pBuff[count] = '\0';
	return count;
}
