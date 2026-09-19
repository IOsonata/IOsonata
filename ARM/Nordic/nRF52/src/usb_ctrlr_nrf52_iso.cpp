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

#define NRFX_USBD_EASYDMA_BUSY_REG			(*((volatile uint32_t *)0x40027C1CUL))
#define NRFX_USBD_EASYDMA_BUSY_REG_BUSY		0x82UL
#define NRFX_USBD_EASYDMA_BUSY_REG_CLEAR	0UL

typedef struct __nRF_Usbd_Iso_State
{
	uint32_t Generation[2];
	nRFUsbdXfer_t Xfer[2];
	nRFUsbEpReg_t EpReg[2];
	uint16_t OutSize;
} nRFUsbdIsoState_t;

static nRFUsbdIsoState_t s_Iso;

static inline __attribute__((always_inline))
uint8_t nRFIsoDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
nRFUsbEpReg_t *nRFIsoReg(uint8_t EpAddr)
{
	return &s_Iso.EpReg[nRFIsoDir(EpAddr)];
}



static bool nRFUsbdStartIsoNow(void)
{
	const uint32_t flags = s_Usbd.Flags;
	static_assert(offsetof(USBD_ISOOUT_Type, MAXCNT) ==
		offsetof(USBD_ISOIN_Type, MAXCNT), "ISO register layout");

	for (int8_t dir = 1; dir >= 0; dir--)
	{
		const uint32_t need = (uint32_t)(USBD_FLAG_ISO_OUT_OPEN |
			USBD_FLAG_ISO_OUT_READY) << dir;
		nRFUsbdXfer_t *pXfer = &s_Iso.Xfer[dir];
		if ((flags & need) != need || pXfer->pBuffer == NULL)
		{
			continue;
		}

		uint16_t len = pXfer->TotalLen;
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
			if (s_Iso.OutSize < len)
			{
				len = s_Iso.OutSize;
			}
			pEp = (volatile USBD_ISOIN_Type *)&NRF_USBD->ISOOUT;
			pTask = &NRF_USBD->TASKS_STARTISOOUT;
			pEnd = &NRF_USBD->EVENTS_ENDISOOUT;
		}

		s_Usbd.Flags = flags & ~((uint32_t)USBD_FLAG_ISO_OUT_READY << dir);
		pEp->PTR = (uint32_t)(uintptr_t)
			s_Iso.EpReg[dir].pBuffer;
		pEp->MAXCNT = len;
		pXfer->pBuffer = NULL;
		nRFUsbdDmaStartLocked(pTask, pEnd);
		return true;
	}
	return false;
}

static void nRFUsbdServiceIso(void)
{
	const uint32_t flags = s_Usbd.Flags;
	if ((flags & (USBD_FLAG_ISO_IN_READY | USBD_FLAG_ISO_OUT_READY)) == 0U)
	{
		return;
	}

	const uint32_t gate = flags &
		(USBD_FLAG_HOST_RESUME | USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND);
	if ((gate & USBD_FLAG_HOST_RESUME) == 0U &&
		gate != USBD_FLAG_SUSPENDED &&
		NRFX_USBD_EASYDMA_BUSY_REG != NRFX_USBD_EASYDMA_BUSY_REG_BUSY)
	{
		(void)nRFUsbdStartIsoNow();
	}
}

static bool nRFUsbRegIsoXfer(uint8_t EpAddr, uint16_t Length)
{
	const uint8_t dir = nRFIsoDir(EpAddr);
	const uint32_t openBusy = (uint32_t)(USBD_FLAG_ISO_OUT_OPEN |
		USBD_FLAG_ISO_OUT_BUSY) << dir;
	nRFUsbEpReg_t *pReg = nRFIsoReg(EpAddr);
	const uint32_t state = DisableInterrupt();
	if ((s_Usbd.Flags & openBusy) != (uint32_t)USBD_FLAG_ISO_OUT_OPEN << dir ||
		pReg->pBuffer == NULL || pReg->Handler == NULL || Length > pReg->Mps)
	{
		EnableInterrupt(state);
		return false;
	}

	nRFUsbdXfer_t *pXfer = &s_Iso.Xfer[dir];
	pXfer->pBuffer = pReg->pBuffer;
	pXfer->TotalLen = Length;
	pXfer->ActualLen = 0U;
	s_Usbd.Flags |= (uint32_t)USBD_FLAG_ISO_OUT_BUSY << dir;
	nRFUsbdServiceIso();
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
	if (generation != (s_Iso.Generation[dir] & 0x7FFFFFFFUL) ||
		(s_Usbd.Flags & openBusy) != openBusy)
	{
		EnableInterrupt(state);
		return;
	}

	const uint8_t epAddr = dir != 0U ?
		USB_ENDPADDR_DIRIN(NRFX_USBD_ISO_EP_NO) : (uint8_t)NRFX_USBD_ISO_EP_NO;
	const uint16_t amount = s_Iso.Xfer[dir].ActualLen;
	if (dir != 0U)
	{
		s_Usbd.Flags &= ~busyBit;
	}
	EnableInterrupt(state);

	nRFUsbEpRegisteredEvent(epAddr, USB_CTRLR_EVT_XFER_CMPL,
		amount, USB_CTRLR_XFER_SUCCESS);

	state = DisableInterrupt();
	if (dir == 0U &&
		generation == (s_Iso.Generation[dir] & 0x7FFFFFFFUL))
	{
		s_Usbd.Flags &= ~busyBit;
	}
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

static void nRFUsbdRetryIsoComplete(void)
{
	if ((s_Usbd.Flags &
		(USBD_FLAG_ISO_OUT_CMPL | USBD_FLAG_ISO_IN_CMPL)) == 0U)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	for (uint8_t dir = 0U; dir < 2U; ++dir)
	{
		const uint32_t bit = (uint32_t)USBD_FLAG_ISO_OUT_CMPL << dir;
		if ((s_Usbd.Flags & bit) != 0U &&
			AppEvtHandlerQue((s_Iso.Generation[dir] << 1U) | dir,
				NULL, nRFUsbdProcessIsoComplete))
		{
			s_Usbd.Flags &= ~bit;
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
	const volatile USBD_ISOIN_Type *pEp = In ? &NRF_USBD->ISOIN :
		(const volatile USBD_ISOIN_Type *)&NRF_USBD->ISOOUT;
	s_Iso.Xfer[dir].ActualLen = (uint16_t)pEp->AMOUNT;
	*pEnd = 0U;
	NRF_USBD->EPSTATUS = In ? (1UL << 8U) : (1UL << 24U);
	__DSB();
	nRFUsbdDmaUnlock();
	s_Usbd.Flags |= (uint32_t)USBD_FLAG_ISO_OUT_CMPL << dir;
	nRFUsbdRetryIsoComplete();
	return true;
}

bool nRFUsbdIsoStart(void)
{
	return nRFUsbdStartIsoNow();
}

void nRFUsbdIsoService(void)
{
	nRFUsbdServiceIso();
}

bool nRFUsbdIsoFinishDma(uint32_t DmaStatus)
{
	if (DmaStatus == 0x00000100U)
	{
		return nRFUsbdFinishIsoDma(true);
	}
	if (DmaStatus == 0x01000000U)
	{
		return nRFUsbdFinishIsoDma(false);
	}
	if (nRFUsbdFinishIsoDma(true))
	{
		return true;
	}
	return nRFUsbdFinishIsoDma(false);
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
		nRFUsbdXfer_t *pOut = &s_Iso.Xfer[0];
		const bool waiting = pOut->pBuffer != NULL;
		if ((flags & USBD_FLAG_ISO_OUT_BUSY) == 0U || waiting)
		{
			if (size != 0U)
			{
				s_Usbd.Flags |= USBD_FLAG_ISO_OUT_READY;
				s_Iso.OutSize =
					(size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ? 0U : (uint16_t)size;
				if (!waiting)
				{
					nRFUsbEpReg_t *pReg =
						&s_Iso.EpReg[0];
					if (pReg->bBlocking)
					{
						nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO,
							USB_CTRLR_EVT_DRDY, 0U, USB_CTRLR_XFER_SUCCESS);
					}
					else
					{
						(void)nRFUsbRegIsoXfer(NRFX_USBD_ISO_EP_NO, pReg->Mps);
					}
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

bool nRFUsbdIsoEpOpen(const UsbEndPointDesc_t *pDesc)
{
	const uint8_t epAddr = pDesc->bEndpointAddress;
	const bool in = USB_ENDPADDR_IS_IN(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;
	if (type != USB_ENDPATT_TRANS_ISO ||
		pDesc->wMaxPacketSize == 0U ||
		pDesc->wMaxPacketSize > NRFX_USBD_ISO_MAX_PACKET_SIZE)
	{
		return false;
	}

	nRFIsoReg(epAddr)->Mps = pDesc->wMaxPacketSize;
	if (!AppEvtHandlerIdleRegister(nRFUsbdRetryIsoComplete))
	{
		return false;
	}

	NRF_USBD->ISOSPLIT =
		USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
	NRF_USBD->ISOINCONFIG =
		USBD_ISOINCONFIG_RESPONSE_ZeroData << USBD_ISOINCONFIG_RESPONSE_Pos;

	nRFUsbdEpHwEnable(NRFX_USBD_ISO_EP_NO, in, true);

	const uint8_t dir = in ? 1U : 0U;
	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = &s_Iso.Xfer[dir];
	++s_Iso.Generation[dir];
	pXfer->pBuffer = NULL;
	pXfer->TotalLen = 0U;
	pXfer->ActualLen = 0U;
	s_Usbd.Flags = (s_Usbd.Flags &
		~((uint32_t)(USBD_FLAG_ISO_OUT_BUSY | USBD_FLAG_ISO_OUT_CMPL |
			USBD_FLAG_ISO_OUT_READY) << dir)) |
		((uint32_t)USBD_FLAG_ISO_OUT_OPEN << dir);
	EnableInterrupt(state);

	nRFUsbdSofAcquire();
	__ISB();
	__DSB();
	return true;
}

void nRFUsbdIsoEpClose(uint8_t EpAddr)
{
	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	const uint8_t dir = in ? 1U : 0U;
	const uint32_t state = DisableInterrupt();
	if (NRFX_USBD_EASYDMA_BUSY_REG == NRFX_USBD_EASYDMA_BUSY_REG_BUSY)
	{
		nRFUsbdDmaWait();
	}

	nRFUsbdXfer_t *pXfer = &s_Iso.Xfer[dir];
	++s_Iso.Generation[dir];
	s_Usbd.Flags &= ~((uint32_t)(USBD_FLAG_ISO_OUT_BUSY |
		USBD_FLAG_ISO_OUT_CMPL | USBD_FLAG_ISO_OUT_OPEN |
		USBD_FLAG_ISO_OUT_READY) << dir);
	pXfer->pBuffer = NULL;
	pXfer->TotalLen = 0U;
	pXfer->ActualLen = 0U;

	nRFUsbdEpHwEnable(NRFX_USBD_ISO_EP_NO, in, false);
	nRFUsbdSofRelease();

	nRFIsoReg(EpAddr)->Mps = 0U;
	__DSB();
	EnableInterrupt(state);
}

bool nRFUsbdIsoEpRegister(uint8_t EpAddr, uint8_t *pBuffer,
	bool bBlocking, UsbCtrlrEpHandler_t Handler, void *pContext)
{
	nRFUsbEpReg_t *pReg = nRFIsoReg(EpAddr);
	pReg->pBuffer = pBuffer;
	pReg->Handler = Handler;
	pReg->pContext = pContext;
	pReg->bBlocking = bBlocking;
	return true;
}

bool nRFUsbdIsoXfer(uint8_t EpAddr, uint16_t Length)
{
	return nRFUsbRegIsoXfer(EpAddr, Length);
}
