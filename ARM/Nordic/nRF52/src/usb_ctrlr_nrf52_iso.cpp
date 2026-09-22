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



// The shared scheduler already owns the channel lock.
bool nRFUsbdIsoStart(void)
{
	const uint32_t flags = s_Usbd.Flags;

	for (int8_t dir = 1; dir >= 0; dir--)
	{
		const uint32_t need = (uint32_t)(USBD_FLAG_ISO_OUT_OPEN |
			USBD_FLAG_ISO_OUT_READY) << dir;
		if ((flags & need) != need || s_Usbd.IsoDmaLen[dir] < 0)
		{
			continue;
		}

		uint16_t len = (uint16_t)s_Usbd.IsoDmaLen[dir];
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
			if (s_Usbd.IsoOutSize < len)
			{
				len = s_Usbd.IsoOutSize;
			}
			pEp = (volatile USBD_ISOIN_Type *)&NRF_USBD->ISOOUT;
			pTask = &NRF_USBD->TASKS_STARTISOOUT;
			pEnd = &NRF_USBD->EVENTS_ENDISOOUT;
		}

		s_Usbd.Flags = flags & ~((uint32_t)USBD_FLAG_ISO_OUT_READY << dir);
		pEp->PTR = (uint32_t)(uintptr_t)
			s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].pBuffer;
		pEp->MAXCNT = len;
		s_Usbd.IsoDmaLen[dir] = -1;
		nRFUsbdDmaStartLocked(pTask, pEnd);
		return true;
	}
	return false;
}

void nRFUsbdIsoService(void)
{
	if ((s_Usbd.Flags & (USBD_FLAG_ISO_IN_READY | USBD_FLAG_ISO_OUT_READY)) != 0U)
	{
		nRFUsbdResumeQueuedDmaLocked();
	}
}

bool nRFUsbdIsoXfer(uint8_t Dir, uint16_t Length)
{
	const uint32_t openBusy = (uint32_t)(USBD_FLAG_ISO_OUT_OPEN |
		USBD_FLAG_ISO_OUT_BUSY) << Dir;
	nRFUsbEpReg_t *pReg = &s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][Dir];
	const uint32_t state = DisableInterrupt();
	if ((s_Usbd.Flags & openBusy) != (uint32_t)USBD_FLAG_ISO_OUT_OPEN << Dir ||
		pReg->pBuffer == NULL || pReg->Handler == NULL ||
		Length > pReg->MaxPacketSize)
	{
		EnableInterrupt(state);
		return false;
	}

	s_Usbd.IsoDmaLen[Dir] = (int16_t)Length;
	s_Usbd.Flags |= (uint32_t)USBD_FLAG_ISO_OUT_BUSY << Dir;
	nRFUsbdIsoService();
	EnableInterrupt(state);
	return true;
}

static void nRFUsbdProcessIsoComplete(uint32_t Evt, void *pContext)
{
	(void)pContext;
	const uint8_t dir = (uint8_t)(Evt & 1U);
	const uint32_t busyBit = (uint32_t)USBD_FLAG_ISO_OUT_BUSY << dir;
	const uint32_t openBusy = ((uint32_t)USBD_FLAG_ISO_OUT_OPEN << dir) | busyBit;
	const uint32_t generation = Evt >> 1U;
	uint32_t state = DisableInterrupt();
	if (generation != (s_Usbd.IsoGeneration[dir] & 0x7FFFFFFFUL) ||
		(s_Usbd.Flags & openBusy) != openBusy)
	{
		EnableInterrupt(state);
		return;
	}

	// BUSY prevents another DMA on this endpoint until its callback.
	const uint16_t amount = (uint16_t)(dir != 0U ?
		NRF_USBD->ISOIN.AMOUNT : NRF_USBD->ISOOUT.AMOUNT);
	if (dir != 0U)
	{
		s_Usbd.Flags &= ~busyBit;
	}
	EnableInterrupt(state);

	nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, dir,
		USB_CTRLR_EVT_XFER_CMPL, amount);

	state = DisableInterrupt();
	if (dir == 0U &&
		generation == (s_Usbd.IsoGeneration[dir] & 0x7FFFFFFFUL))
	{
		s_Usbd.Flags &= ~busyBit;
	}
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

static void nRFUsbdRetryIsoComplete(void)
{
	if (s_Usbd.IsoDmaLen[0] != -2 && s_Usbd.IsoDmaLen[1] != -2)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	for (uint8_t dir = 0U; dir < 2U; ++dir)
	{
		if (s_Usbd.IsoDmaLen[dir] == -2 &&
			AppEvtHandlerQue((s_Usbd.IsoGeneration[dir] << 1U) | dir,
				NULL, nRFUsbdProcessIsoComplete))
		{
			s_Usbd.IsoDmaLen[dir] = -1;
		}
	}
	EnableInterrupt(state);
}

static bool nRFUsbdFinishIsoDma(bool In)
{
	volatile uint32_t *pEnd = In ?
		&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	if (*pEnd == 0U)
	{
		return false;
	}

	const uint8_t dir = In ? 1U : 0U;
	*pEnd = 0U;
	NRF_USBD->EPSTATUS = In ? (1UL << 8U) : (1UL << 24U);
	__DSB();
	// Retain only AppEvt publication state; DMA handoff is already independent.
	s_Usbd.IsoDmaLen[dir] = -2;
	nRFUsbdRetryIsoComplete();
	return true;
}

bool nRFUsbdIsoFinishDma(uint32_t DmaStatus)
{
	return (DmaStatus != 0x01000000U && nRFUsbdFinishIsoDma(true)) ||
		(DmaStatus != 0x00000100U && nRFUsbdFinishIsoDma(false));
}

void nRFUsbdIsoSof(void)
{
	const uint32_t flags = s_Usbd.Flags;
	if ((flags & USBD_FLAG_ISO_IN_OPEN) != 0U)
	{
		s_Usbd.Flags |= USBD_FLAG_ISO_IN_READY;
	}

	if ((flags & USBD_FLAG_ISO_OUT_OPEN) != 0U)
	{
		const uint32_t size = NRF_USBD->SIZE.ISOOUT;
		const bool waiting = s_Usbd.IsoDmaLen[0] >= 0;
		if ((flags & USBD_FLAG_ISO_OUT_BUSY) == 0U || waiting)
		{
			if (size != 0U)
			{
				s_Usbd.Flags |= USBD_FLAG_ISO_OUT_READY;
				s_Usbd.IsoOutSize =
					(size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ? 0U : (uint16_t)size;
				if (!waiting)
				{
					nRFUsbEpReg_t *pReg =
						&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0];
					if (pReg->bBlocking)
					{
						nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, 0U,
							USB_CTRLR_EVT_DRDY, 0U);
					}
					(void)nRFUsbdIsoXfer(0U, pReg->MaxPacketSize);
				}
			}
			else
			{
				s_Usbd.Flags &= ~(uint32_t)USBD_FLAG_ISO_OUT_READY;
			}
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

	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][in].MaxPacketSize = pDesc->wMaxPacketSize;
	if (!AppEvtHandlerIdleRegister(nRFUsbdRetryIsoComplete))
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
	s_Usbd.Flags = (s_Usbd.Flags &
		~((uint32_t)USBD_FLAG_ISO_OUT_READY << dir)) |
		((uint32_t)USBD_FLAG_ISO_OUT_OPEN << dir);
	EnableInterrupt(state);

	nRFUsbdSofAcquire();
	__DSB();
	return true;
}

void nRFUsbdIsoEpClose(bool bIn)
{
	const uint8_t dir = bIn ? 1U : 0U;
	const uint32_t state = DisableInterrupt();
	nRFUsbdDmaWait();

	++s_Usbd.IsoGeneration[dir];
	s_Usbd.Flags &= ~((uint32_t)(USBD_FLAG_ISO_OUT_BUSY |
		USBD_FLAG_ISO_OUT_OPEN | USBD_FLAG_ISO_OUT_READY) << dir);
	s_Usbd.IsoDmaLen[dir] = -1;

	nRFIsoHwEnable(bIn, false);
	nRFUsbdSofRelease();

	s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][dir].MaxPacketSize = 0U;
	__DSB();
	EnableInterrupt(state);
}
