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

	if (Enable)
	{
		*pEnd = 0U;
		NRF_USBD->INTENSET = endMsk;
		*pEnable |= msk;
	}
	else
	{
		NRF_USBD->INTENCLR = endMsk;
		*pEnable &= ~msk;
		*pEnd = 0U;
	}
}

static inline __attribute__((always_inline)) bool nRFIsoOpen(void)
{
	return s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0].MaxPacketSize != 0U;
}


// The shared scheduler already owns the channel lock.
bool nRFUsbdIsoStart(void)
{
	if (!nRFIsoOpen())
	{
		return false;
	}

	const uint8_t state = s_Usbd.IsoBufState;
	for (int8_t dir = 1; dir >= 0; dir--)
	{
		const uint8_t need = (uint8_t)(NRFUSBD_ISO_OUT_READY |
			NRFUSBD_ISO_OUT_BUSY) << dir;
		if ((state & need) != need)
		{
			continue;
		}

		uint16_t len = s_Usbd.IsoDmaLen[dir];
		volatile USBD_ISOIN_Type *pEp;
		volatile uint32_t *pTask;
		volatile uint32_t *pEnd;
		if (dir != 0)
		{
			pEp = &NRF_USBD->ISOIN;
			pTask = &NRF_USBD->TASKS_STARTISOIN;
			pEnd = &NRF_USBD->EVENTS_ENDISOIN;
		}
		else
		{
			pEp = (volatile USBD_ISOIN_Type *)&NRF_USBD->ISOOUT;
			pTask = &NRF_USBD->TASKS_STARTISOOUT;
			pEnd = &NRF_USBD->EVENTS_ENDISOOUT;
		}

		s_Usbd.IsoBufState =
			state & (uint8_t)~((uint8_t)NRFUSBD_ISO_OUT_READY << dir);
		pEp->PTR = (uint32_t)(uintptr_t)
			s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].pBuffer;
		pEp->MAXCNT = len;
		nRFUsbdDmaStartLocked(pTask, pEnd);
		return true;
	}
	return false;
}

void nRFUsbdIsoService(void)
{
	if ((s_Usbd.IsoBufState &
		(NRFUSBD_ISO_IN_READY | NRFUSBD_ISO_OUT_READY)) != 0U)
	{
		nRFUsbdResumeQueuedDmaLocked();
	}
}

bool nRFUsbdIsoXfer(uint8_t Dir, uint16_t Length)
{
	nRFUsbEpReg_t *pReg = &s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][Dir];
	const uint8_t busy = (uint8_t)NRFUSBD_ISO_OUT_BUSY << Dir;
	const uint32_t state = DisableInterrupt();
	if (!nRFIsoOpen() || (s_Usbd.IsoBufState & busy) != 0U ||
		pReg->pBuffer == NULL || pReg->Handler == NULL ||
		Length > pReg->MaxPacketSize)
	{
		EnableInterrupt(state);
		return false;
	}

	s_Usbd.IsoDmaLen[Dir] = Length;
	s_Usbd.IsoBufState |= busy;
	nRFUsbdIsoService();
	EnableInterrupt(state);
	return true;
}

static bool nRFUsbdFinishIsoDma(bool In, bool Notify)
{
	volatile uint32_t *pEnd = In ?
		&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	if (*pEnd == 0U)
	{
		return false;
	}

	const uint8_t dir = In ? 1U : 0U;
	const uint8_t busy = (uint8_t)NRFUSBD_ISO_OUT_BUSY << dir;
	const uint16_t amount = (uint16_t)(In ?
		NRF_USBD->ISOIN.AMOUNT : NRF_USBD->ISOOUT.AMOUNT);

	*pEnd = 0U;
	NRF_USBD->EPSTATUS = In ? (1UL << 8U) : (1UL << 24U);
	__DSB();

	if (!Notify)
	{
		return true;
	}

	// IN may queue its next buffer from the callback. OUT keeps ownership
	// through the callback so its buffer cannot be reused while copied.
	if (In)
	{
		s_Usbd.IsoBufState &= (uint8_t)~busy;
	}

	nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, dir,
		USB_CTRLR_EVT_XFER_CMPL, amount);

	if (!In)
	{
		s_Usbd.IsoBufState &= (uint8_t)~busy;
	}
	return true;
}

bool nRFUsbdIsoFinishDma(uint32_t DmaStatus)
{
	const bool notify = DmaStatus != 0U || nRFIsoOpen();
	return (DmaStatus != 0x01000000U && nRFUsbdFinishIsoDma(true, notify)) ||
		(DmaStatus != 0x00000100U && nRFUsbdFinishIsoDma(false, notify));
}

void nRFUsbdIsoSof(void)
{
	if (!nRFIsoOpen())
	{
		return;
	}

	const uint8_t state = s_Usbd.IsoBufState;
	s_Usbd.IsoBufState = state | NRFUSBD_ISO_IN_READY;

	const uint32_t size = NRF_USBD->SIZE.ISOOUT;
	const bool waiting = (state & NRFUSBD_ISO_OUT_READY) != 0U;
	if ((state & NRFUSBD_ISO_OUT_BUSY) == 0U || waiting)
	{
		if (size != 0U)
		{
			const uint16_t len =
				(size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ? 0U : (uint16_t)size;
			s_Usbd.IsoBufState |= NRFUSBD_ISO_OUT_READY;
			if (waiting)
			{
				s_Usbd.IsoDmaLen[0] = len;
			}
			else
			{
				nRFUsbEpReg_t *pReg =
					&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0];
				if (pReg->bBlocking)
				{
					nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, 0U,
						USB_CTRLR_EVT_DRDY, 0U);
				}
				(void)nRFUsbdIsoXfer(0U, len);
			}
		}
		else
		{
			s_Usbd.IsoBufState &= (uint8_t)~NRFUSBD_ISO_OUT_READY;
		}
	}
}

bool UsbCtrlrIsoInit(int DevNo)
{
	return DevNo == 0;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	(void)DevNo;
	const uint8_t epAddr = pDesc->bEndpointAddress;
	const bool in = USB_ENDPADDR_IS_IN(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;
	if (type != USB_ENDPATT_TRANS_ISO ||
		pDesc->wMaxPacketSize == 0U ||
		pDesc->wMaxPacketSize > NRFX_USBD_ISO_MAX_PACKET_SIZE)
	{
		return false;
	}

	NRF_USBD->ISOSPLIT =
		USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
	NRF_USBD->ISOINCONFIG =
		USBD_ISOINCONFIG_RESPONSE_ZeroData << USBD_ISOINCONFIG_RESPONSE_Pos;

	nRFIsoHwEnable(in, true);

	const uint8_t dir = in ? 1U : 0U;
	const uint32_t state = DisableInterrupt();
	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].MaxPacketSize =
		pDesc->wMaxPacketSize;
	s_Usbd.IsoBufState &=
		(uint8_t)~((uint8_t)NRFUSBD_ISO_OUT_READY << dir);
	EnableInterrupt(state);

	nRFUsbdSofAcquire();
	__DSB();
	return true;
}

void nRFUsbdIsoEpClose(bool bIn)
{
	const uint8_t dir = bIn ? 1U : 0U;
	const uint32_t state = DisableInterrupt();

	// ISO is one bidirectional path. Closing either side clears endpoint
	// state first so a polled END is cancellation, not normal completion.
	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].MaxPacketSize = 0U;
	nRFUsbdDmaWait();
	s_Usbd.IsoBufState = 0U;

	nRFIsoHwEnable(bIn, false);
	nRFUsbdSofRelease();

	__DSB();
	EnableInterrupt(state);
}
