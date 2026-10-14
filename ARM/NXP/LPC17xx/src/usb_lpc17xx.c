/**-------------------------------------------------------------------------
@file	usb_lpc17xx.c

@brief	LPC1769 full-speed USB controller lifecycle.

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
#include "usb_data_lpc17xx.h"

static bool s_Closing;
static Lpc17UsbDataEndpoint_t s_Data[30];
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

static bool Lpc17UsbDataReady(void)
{
	return s_Prepared && s_Control.Enabled && !s_Closing && !s_Faulted &&
		(LPC_USB->USBDevIntSt & 8U) == 0U;
}

static void Lpc17UsbDataKick(uint8_t EpNo)
{
	if (Lpc17UsbDataReady())
	{
		LPC_USB->USBEpIntSet = 1UL << (EpNo * 2U);
		LPC_USB->USBDevIntSet = 4U;
		NVIC_SetPendingIRQ(USB_IRQn);
	}
}

static bool Lpc17UsbPrepareData(void)
{
	const uint32_t deviceEnable = LPC_USB->USBDevIntEn;
	const uint32_t endpointEnable = LPC_USB->USBEpIntEn;
	LPC_USB->USBDevIntEn = 0U;
	LPC_USB->USBEpIntEn = 0U;
	bool ready = true;
	// Reserve fixed 64-byte hardware buffers before any data traffic. Logical
	// MPS remains per endpoint; later opens never relocate live endpoint RAM.
	for (unsigned ep = 1; ep < 16U && ready; ++ep)
	{
		if (ep % 3U == 0U && ep != 15U)
			continue;
		for (unsigned in = 0; in < 2U && ready; ++in)
			ready = Lpc17UsbDataRealize(ep, in != 0, ep % 3U == 1U ? INT : BULK, 64) &&
				Lpc17UsbDataSetStatus(ep, in != 0, 0x20, true);
	}
	if (ready)
	{
		LPC_USB->USBEpIntEn = endpointEnable;
		LPC_USB->USBDevIntEn = deviceEnable;
	}
	return ready;
}

static Lpc17UsbDataEndpoint_t *Lpc17UsbEndpoint(int DevNo, uint8_t EpNo, bool In)
{
	if (DevNo != 0 || EpNo == 0 || EpNo > 15 || (EpNo % 3 == 0 && EpNo != 15))
		return NULL;
	return &s_Data[EpNo * 2U + (In ? 1U : 0U) - 2U];
}

static void Lpc17UsbEvent(const UsbCtrlrEvt_t *pEvent)
{
	if (pEvent->Type == USB_CTRLR_EVT_RESET && !Lpc17UsbPrepareData())
	{
		Lpc17UsbFault();
		return;
	}
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
	for (unsigned i = 0; i < 30U; ++i)
	{
		s_Data[i].Ready = Lpc17UsbDataReady;
		s_Data[i].Fault = Lpc17UsbFault;
		s_Data[i].Kick = Lpc17UsbDataKick;
	}
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
	if (DevNo != 0 || s_Closing || !UsbCtrlrVbusDetected(0))
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
		ready = Lpc17UsbDeviceInit() && Lpc17UsbPrepareData();
		if (ready)
		{
			Lpc17UsbControlReset(&s_Control, true);
			ready = Lpc17UsbIrqBind(&s_Control);
			s_Bound = ready;
			if (ready)
				ready = Lpc17UsbIrqBindData(s_Data);
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
	if (DevNo != 0 || !s_Initialized || s_Closing)
		return;
	const uint32_t state = DisableInterrupt();
	s_Closing = true;
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
	if (!s_Prepared)
	{
		for (unsigned i = 0; i < 30U; ++i)
			Lpc17UsbDataForget(&s_Data[i]);
	}
	s_Closing = false;
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
	if (pDesc == NULL || (pDesc->bEndpointAddress & 0x70U) != 0U)
		return false;
	if ((pDesc->bEndpointAddress & 15U) == 0U)
		return DevNo == 0 && s_Control.Enabled &&
			pDesc->bmAttributes == USB_ENDPATT_TRANS_CONTROL && pDesc->wMaxPacketSize == 64;
	return UsbCtrlrEpOpenData(DevNo, pDesc->bEndpointAddress & 15U,
		(pDesc->bEndpointAddress & 0x80U) != 0U, pDesc->bmAttributes & 3U, pDesc->wMaxPacketSize);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool In, uint8_t Type, uint16_t Size)
{
	Lpc17UsbDataEndpoint_t *ep = Lpc17UsbEndpoint(DevNo, EpNo, In);
	if (ep == NULL || Type != (EpNo % 3U == 1U ? INT : BULK))
		return false;
	const uint32_t state = DisableInterrupt();
	bool ready = Lpc17UsbDataOpen(ep, EpNo, In, Size);
	if (ready)
	{
		ready = Lpc17UsbDataSetStatus(EpNo, In, 0, false) && Lpc17UsbSieWrite(0xD8, 1);
		if (ready)
			LPC_USB->USBEpIntEn |= 1UL << (EpNo * 2U + (In ? 1U : 0U));
		else
			Lpc17UsbFault();
	}
	EnableInterrupt(state);
	return ready;
}

static void Lpc17UsbHalt(int DevNo, uint8_t EpNo, bool In, bool Halt)
{
	Lpc17UsbDataEndpoint_t *ep = Lpc17UsbEndpoint(DevNo, EpNo, In);
	if (ep == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	// The public halt API cannot report failure. A bus event racing SETUP
	// must abort control processing rather than acknowledge an unapplied halt.
	if (ep->Enabled && s_Prepared && s_Control.Enabled && !s_Closing &&
		!s_Faulted && (LPC_USB->USBDevIntSt & 8U) != 0U)
	{
		Lpc17UsbFault();
	}
	else if (Lpc17UsbDataReady() && ep->Enabled)
	{
		const uint32_t bit = 1UL << (EpNo * 2U + (In ? 1U : 0U));
		LPC_USB->USBEpIntEn &= ~bit;
		if (Lpc17UsbDataSetStatus(EpNo, In, Halt ? 1 : 0, false))
		{
			Lpc17UsbDataHalt(ep, Halt);
			if (Lpc17UsbDataReady() && ep->Enabled)
				LPC_USB->USBEpIntEn |= bit;
		}
		else
			Lpc17UsbFault();
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool In)
{
	if (DevNo == 0 && EpNo == 0)
		Lpc17UsbControlStall(&s_Control);
	else
		Lpc17UsbHalt(DevNo, EpNo, In, true);
}
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool In)
{
	Lpc17UsbHalt(DevNo, EpNo, In, false);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool In)
{
	Lpc17UsbDataEndpoint_t *ep = Lpc17UsbEndpoint(DevNo, EpNo, In);
	if (ep == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	if (Lpc17UsbDataReady() && ep->Enabled)
	{
		s_Closing = true;
		LPC_USB->USBEpIntEn &= ~(1UL << (EpNo * 2U + (In ? 1U : 0U)));
		if (Lpc17UsbDataSetStatus(EpNo, In, 0x20, false))
			Lpc17UsbDataForget(ep);
		else
			Lpc17UsbFault();
		s_Closing = false;
	}
	EnableInterrupt(state);
}
void UsbCtrlrEpCloseAll(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	if (Lpc17UsbDataReady())
	{
		s_Closing = true;
		LPC_USB->USBEpIntEn &= 3U;
		bool ready = Lpc17UsbSieWrite(0xD8, 0);
		for (unsigned i = 0; i < 30U && ready; ++i)
		{
			if (s_Data[i].Enabled)
				ready = Lpc17UsbDataSetStatus(s_Data[i].EpNo, s_Data[i].In, 0x20, false);
		}
		if (ready)
		{
			for (unsigned i = 0; i < 30U; ++i)
				Lpc17UsbDataForget(&s_Data[i]);
		}
		else
			Lpc17UsbFault();
		s_Closing = false;
	}
	EnableInterrupt(state);
}
void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool In, bool Blocking,
		UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)Blocking;
	Lpc17UsbDataEndpoint_t *ep = Lpc17UsbEndpoint(DevNo, EpNo, In);
	if (ep == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	if (!s_Closing && !ep->Busy)
	{
		ep->Handler = Handler;
		ep->pContext = pContext;
		++ep->Generation;
		if (!In && ep->Enabled && !ep->Halted)
			Lpc17UsbDataKick(EpNo);
	}
	EnableInterrupt(state);
}
bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	return Lpc17UsbDataReceive(Lpc17UsbEndpoint(DevNo, EpNo, false), pBuffer, Length);
}
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Length)
{
	return Lpc17UsbDataSend(Lpc17UsbEndpoint(DevNo, EpNo, true), pBuffer, Length);
}
void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool In, UsbCtrlrEvtType_t Event, uint16_t Value)
{
	Lpc17UsbDataEndpoint_t *ep = Lpc17UsbEndpoint(DevNo, EpNo, In);
	const uint32_t state = DisableInterrupt();
	if (ep != NULL && Lpc17UsbDataReady() && ep->Enabled && ep->Handler != NULL)
		ep->Handler(Event, Value, ep->pContext);
	EnableInterrupt(state);
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
