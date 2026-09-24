/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52_iso.cpp

@brief	Optional nRF52 USBD isochronous endpoint support.

This implementation is kept in its own archive member so applications without
UsbIsoIntrf do not pull the ISO scheduler, SOF processing or deferred
completion path.

@author	Hoang Nguyen Hoan
@date	Sep. 19, 2026

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
#include <stddef.h>
#include <stdint.h>

#include "nrf.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"

#include "usb_ctrlr.h"

static_assert(offsetof(USBD_ISOOUT_Type, MAXCNT) ==
	offsetof(USBD_ISOIN_Type, MAXCNT), "ISO register layout");

static __attribute__((noinline))
void nRFIsoHwEnable(bool In, bool Enable)
{
	volatile uint32_t *pEnd = In ?
		&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	volatile uint32_t *pEnable = In ? &NRF_USBD->EPINEN : &NRF_USBD->EPOUTEN;
	const uint32_t msk = 1UL << NRFX_USBD_ISO_EP_NO;
	const uint32_t endMsk = In ?
		USBD_INTEN_ENDISOIN_Msk : USBD_INTEN_ENDISOOUT_Msk;

	*pEnd = 0U;
	if (Enable)
	{
		NRF_USBD->INTENSET = endMsk;
		*pEnable |= msk;
	}
	else
	{
		NRF_USBD->INTENCLR = endMsk;
		*pEnable &= ~msk;
	}
}



// The shared scheduler already owns the channel lock.
bool nRFUsbdIsoStart(void)
{
	if (!s_Usbd.IsoOpen)
		return false;

	if ((s_Usbd.IsoBusy & NRFUSBD_ISO_IN_BUSY) != 0U)
	{
		nRFUsbEpReg_t *pReg =
			&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][1];
		NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)pReg->pBuffer;
		NRF_USBD->ISOIN.MAXCNT = s_Usbd.IsoInDmaLen;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOIN,
			&NRF_USBD->EVENTS_ENDISOIN);
		return true;
	}

	if ((s_Usbd.IsoBusy & NRFUSBD_ISO_OUT_BUSY) == 0U)
		return false;

	const uint32_t size = NRF_USBD->SIZE.ISOOUT;
	if (size == 0U)
	{
		s_Usbd.IsoBusy &= (uint8_t)~NRFUSBD_ISO_OUT_BUSY;
		return false;
	}

	nRFUsbEpReg_t *pReg =
		&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0];
	const uint16_t len = (size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ?
		0U : (uint16_t)size;
	if (len > pReg->MaxPacketSize)
	{
		s_Usbd.IsoBusy &= (uint8_t)~NRFUSBD_ISO_OUT_BUSY;
		return false;
	}

	NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)pReg->pBuffer;
	NRF_USBD->ISOOUT.MAXCNT = len;
	nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOOUT,
		&NRF_USBD->EVENTS_ENDISOOUT);
	return true;
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
	uint16_t Length)
{
	(void)DevNo;
	(void)EpNum;

	if (!s_Usbd.IsoOpen)
		return false;

	bool send = false;
	if ((s_Usbd.IsoBusy & NRFUSBD_ISO_OUT_BUSY) == 0U)
	{
		s_Usbd.IsoBusy |= NRFUSBD_ISO_OUT_BUSY;
		send = true;
	}

	if (pBuffer != nullptr &&
		(s_Usbd.IsoBusy & NRFUSBD_ISO_IN_BUSY) == 0U)
	{
		nRFUsbEpReg_t *pIn =
			&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][1];
		pIn->pBuffer = pBuffer;
		s_Usbd.IsoInDmaLen = Length;
		s_Usbd.IsoBusy |= NRFUSBD_ISO_IN_BUSY;
		send = true;
	}

	if (send)
		nRFUsbdResumeQueuedDmaLocked();
	return send;
}

static bool nRFUsbdFinishIsoDma(bool In, bool Notify)
{
	volatile uint32_t *pEnd = In ?
		&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	if (*pEnd == 0U)
		return false;

	const uint8_t dir = In ? 1U : 0U;
	const uint8_t busy = (uint8_t)NRFUSBD_ISO_OUT_BUSY << dir;
	const uint16_t amount = (uint16_t)(In ?
		NRF_USBD->ISOIN.AMOUNT : NRF_USBD->ISOOUT.AMOUNT);

	*pEnd = 0U;
	NRF_USBD->EPSTATUS = In ? (1UL << 8U) : (1UL << 24U);
	__DSB();

	if (!Notify)
		return true;

	if (In)
	{
		s_Usbd.IsoBusy &= (uint8_t)~busy;
	}

	nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, dir,
		USB_CTRLR_EVT_XFER_CMPL, amount);

	if (!In)
	{
		s_Usbd.IsoBusy &= (uint8_t)~busy;
	}
	return true;
}

bool nRFUsbdIsoFinishDma(void)
{
	return nRFUsbdFinishIsoDma(true, s_Usbd.IsoOpen) ||
		nRFUsbdFinishIsoDma(false, s_Usbd.IsoOpen);
}

bool UsbCtrlrIsoInit(int DevNo)
{
	(void)DevNo;
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	(void)DevNo;
	const uint8_t epAddr = pDesc->bEndpointAddress;
	const bool in = USB_ENDPADDR_IS_IN(epAddr);
	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][in].MaxPacketSize = pDesc->wMaxPacketSize;
	NRF_USBD->ISOSPLIT =
		USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
	NRF_USBD->ISOINCONFIG =
		USBD_ISOINCONFIG_RESPONSE_ZeroData << USBD_ISOINCONFIG_RESPONSE_Pos;

	nRFIsoHwEnable(in, true);

	const uint8_t dir = in ? 1U : 0U;
	const uint32_t state = DisableInterrupt();
	s_Usbd.IsoBusy &=
		(uint8_t)~((uint8_t)NRFUSBD_ISO_OUT_BUSY << dir);
	s_Usbd.IsoOpen =
		s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0].MaxPacketSize != 0U &&
		s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][1].MaxPacketSize != 0U;
	EnableInterrupt(state);

	__DSB();
	return true;
}

void nRFUsbdIsoEpClose(bool bIn)
{
	const uint8_t dir = bIn ? 1U : 0U;
	const uint32_t state = DisableInterrupt();

	// ISO is one bidirectional path. Closing either side stops scheduling and
	// makes a polled END a cancellation rather than a normal completion.
	s_Usbd.IsoOpen = false;
	nRFUsbdDmaWait();
	s_Usbd.IsoBusy = 0U;

	nRFIsoHwEnable(bIn, false);

	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].MaxPacketSize = 0U;
	__DSB();
	EnableInterrupt(state);
}
