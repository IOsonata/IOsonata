/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52_iso.cpp

@brief	Optional nRF52 USBD isochronous endpoint support.

This implementation is kept in its own archive member so applications without
UsbIsoIntrf do not pull the ISO scheduler, SOF processing or ISR
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
#include <string.h>

#include "nrf.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"

#include "usb_ctrlr.h"

static_assert(offsetof(USBD_ISOOUT_Type, MAXCNT) ==
	offsetof(USBD_ISOIN_Type, MAXCNT), "ISO register layout");

// Enable explicitly in the library build for bench diagnostics.
#ifndef NRFUSBD_ISO_TRACE
#define NRFUSBD_ISO_TRACE			0
#endif

#if NRFUSBD_ISO_TRACE
// Per-frame trace. One entry per SOF: when the IN DMA started and ended
// relative to SOF, what the scheduler saw when the frame was offered, and
// what came in on OUT. Read by the bench through the application's vendor
// request; the application copies UsbCtrlrIsoTraceSnapshot().
#define NRFUSBD_ISO_TRACE_CNT		128U
#define NRFUSBD_ISO_TRACE_MASK		(NRFUSBD_ISO_TRACE_CNT - 1U)
#define NRFUSBD_ISO_CYC_PER_US		64U

enum
{
	ISO_TRACE_IN_OFFERED	= 0x01U,	// IsoSend called with an IN buffer
	ISO_TRACE_IN_REFUSED	= 0x02U,	// previous IN still pending at offer
	ISO_TRACE_DMA_BUSY		= 0x04U,	// channel held by another DMA at offer
	ISO_TRACE_EP0_HELD		= 0x08U,	// that DMA was EP0
	ISO_TRACE_IN_START		= 0x10U,	// STARTISOIN issued this frame
	ISO_TRACE_IN_END		= 0x20U,	// ENDISOIN retired this frame
	ISO_TRACE_OUT_START		= 0x40U,	// STARTISOOUT issued this frame
	ISO_TRACE_OUT_END		= 0x80U,	// ENDISOOUT retired this frame
};

typedef struct __nRF_Usbd_Iso_Trace {
	uint16_t Frame;			// FRAMECNTR at SOF
	uint16_t SofDeltaUs;	// time since the previous SOF mark, 65535 = more
	uint8_t StartUs;		// SOF to STARTISOIN, 255 = later than that
	uint8_t EndUs;			// SOF to ENDISOIN retire
	uint8_t InLen;			// bytes offered for IN
	uint8_t Flags;			// ISO_TRACE_*
	uint16_t OutLen;		// bytes read on OUT
	uint16_t Reserved;
} nRFUsbdIsoTrace_t;

typedef struct __nRF_Usbd_Iso_Trace_Snap {
	uint16_t Next;			// index of the entry the next SOF will write
	uint16_t Count;			// NRFUSBD_ISO_TRACE_CNT
	uint16_t EntrySize;		// sizeof(nRFUsbdIsoTrace_t)
	uint16_t Version;		// layout version
	nRFUsbdIsoTrace_t Entry[NRFUSBD_ISO_TRACE_CNT];
} nRFUsbdIsoTraceSnap_t;

static nRFUsbdIsoTrace_t s_IsoTrace[NRFUSBD_ISO_TRACE_CNT];
static nRFUsbdIsoTraceSnap_t s_IsoTraceSnap;
static uint32_t s_IsoSofCyc;
static uint16_t s_IsoTraceIdx;

static inline uint8_t nRFUsbdIsoUsSinceSof(void)
{
	const uint32_t us = (DWT->CYCCNT - s_IsoSofCyc) / NRFUSBD_ISO_CYC_PER_US;
	return us > 255U ? 255U : (uint8_t)us;
}

static inline nRFUsbdIsoTrace_t *nRFUsbdIsoTraceCur(void)
{
	return &s_IsoTrace[s_IsoTraceIdx];
}

// Called from the USBD interrupt at SOF, before the core sees the frame.
// FRAMECNTR can still be changing when SOF asserts; read until two reads
// agree. The delta to the previous mark separates a SOF never received
// (2000 us, FRAMECNTR skips one) from an interrupt held off past the next
// SOF (2000 us plus the delay, the two events serviced as one).
void nRFUsbdIsoSofMark(void)
{
	const uint32_t now = DWT->CYCCNT;
	const uint32_t delta = (now - s_IsoSofCyc) / NRFUSBD_ISO_CYC_PER_US;
	s_IsoSofCyc = now;

	uint32_t frame = NRF_USBD->FRAMECNTR;
	for (uint32_t i = 0U; i < 4U; i++)
	{
		const uint32_t again = NRF_USBD->FRAMECNTR;
		if (again == frame)
			break;
		frame = again;
	}

	s_IsoTraceIdx = (uint16_t)((s_IsoTraceIdx + 1U) & NRFUSBD_ISO_TRACE_MASK);
	nRFUsbdIsoTrace_t *p = &s_IsoTrace[s_IsoTraceIdx];
	p->Frame = (uint16_t)frame;
	p->SofDeltaUs = delta > 65535U ? 65535U : (uint16_t)delta;
	p->StartUs = 0U;
	p->EndUs = 0U;
	p->InLen = 0U;
	p->Flags = 0U;
	p->OutLen = 0U;
	p->Reserved = 0U;
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	const uint32_t state = DisableInterrupt();
	s_IsoTraceSnap.Next =
		(uint16_t)((s_IsoTraceIdx + 1U) & NRFUSBD_ISO_TRACE_MASK);
	s_IsoTraceSnap.Count = NRFUSBD_ISO_TRACE_CNT;
	s_IsoTraceSnap.EntrySize = sizeof(nRFUsbdIsoTrace_t);
	s_IsoTraceSnap.Version = 2U;
	memcpy(s_IsoTraceSnap.Entry, s_IsoTrace, sizeof(s_IsoTrace));
	EnableInterrupt(state);
	*ppData = (uint8_t *)&s_IsoTraceSnap;
	return (uint16_t)sizeof(s_IsoTraceSnap);
}

#define ISO_TRACE_FLAG(f)		(nRFUsbdIsoTraceCur()->Flags |= (uint8_t)(f))
#else
#define ISO_TRACE_FLAG(f)		((void)0)

void nRFUsbdIsoSofMark(void)
{
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	*ppData = nullptr;
	return 0U;
}
#endif

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
//
// IN goes first whenever it is ready. The host schedules its periodic
// transactions at the start of the frame, and the ISOIN buffer answers the
// IN token with whatever EasyDMA has delivered by then (ZeroData otherwise),
// so the IN staging has a hard deadline a few tens of microseconds after
// SOF. Putting the OUT DMA and its ISR completion round trip ahead of it on
// alternate frames pushes IN past that deadline under load; those frames are
// the empty IN slots the combo stress reports. OUT follows as soon as the IN
// DMA ends, still well inside the frame.
bool nRFUsbdIsoStart(void)
{
	const uint8_t dataFlag = s_Usbd.IsoDataFlag;
	if (dataFlag == 0U)
		return false;

	if ((dataFlag & NRFUSBD_ISO_IN_READY) != 0U)
	{
		NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)s_Usbd.pIsoBuffer[1];
		NRF_USBD->ISOIN.MAXCNT = s_Usbd.IsoInDmaLen;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOIN,
			&NRF_USBD->EVENTS_ENDISOIN);
#if NRFUSBD_ISO_TRACE
		nRFUsbdIsoTraceCur()->StartUs = nRFUsbdIsoUsSinceSof();
		ISO_TRACE_FLAG(ISO_TRACE_IN_START);
#endif
		return true;
	}

	// OUT: only inspect the hardware once OUT is the selected transfer.
	const uint32_t size = NRF_USBD->SIZE.ISOOUT;
	const uint16_t len = (size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ?
		0U : (uint16_t)size;

	if (len != 0U && len <= s_Usbd.IsoMaxPacketSize[0])
	{
		// Ask the owner to submit this frame's destination through EpReceive.
		// Completion releases the destination before the next interval.
		nRFUsbEpReg_t *pReg =
			&s_Usbd.EpReg[NRFX_USBD_ISO_EP_NO - 1U][0];
		if (s_Usbd.pIsoBuffer[0] == NULL && pReg->Handler != NULL)
			pReg->Handler(USB_CTRLR_EVT_DRDY, 0U, pReg->pContext);

		if (s_Usbd.pIsoBuffer[0] != NULL)
		{
			NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)s_Usbd.pIsoBuffer[0];
			NRF_USBD->ISOOUT.MAXCNT = len;
			nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOOUT,
				&NRF_USBD->EVENTS_ENDISOOUT);
			ISO_TRACE_FLAG(ISO_TRACE_OUT_START);
			return true;
		}
	}

	// Empty, zero-length, oversize, or no destination: drop this request.
	// The next SOF service queues a fresh one.
	s_Usbd.IsoDataFlag &= (uint8_t)~NRFUSBD_ISO_OUT_READY;
	return false;
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
	uint16_t Length)
{
	(void)DevNo;
	(void)EpNum;

	if (!s_Usbd.IsoOpen)
		return false;

#if NRFUSBD_ISO_TRACE
	if (pBuffer != nullptr)
	{
		nRFUsbdIsoTrace_t *pt = nRFUsbdIsoTraceCur();
		pt->InLen = Length > 255U ? 255U : (uint8_t)Length;
		pt->Flags |= ISO_TRACE_IN_OFFERED;
		if ((s_Usbd.IsoDataFlag & NRFUSBD_ISO_IN_READY) != 0U)
			pt->Flags |= ISO_TRACE_IN_REFUSED;
		// EPSTATUS holds the captured DMA until software retires it, so a
		// nonzero value is a channel the scheduler cannot take yet.
		const uint32_t epstatus = NRF_USBD->EPSTATUS;
		if (epstatus != 0U)
		{
			pt->Flags |= ISO_TRACE_DMA_BUSY;
			if ((epstatus & 0x00010001UL) != 0U)
				pt->Flags |= ISO_TRACE_EP0_HELD;
		}
	}
#endif

	uint8_t dataFlag = s_Usbd.IsoDataFlag | NRFUSBD_ISO_OUT_READY;

	if (pBuffer != nullptr &&
		(dataFlag & NRFUSBD_ISO_IN_READY) == 0U)
	{
		s_Usbd.pIsoBuffer[1] = pBuffer;
		s_Usbd.IsoInDmaLen = Length;
		dataFlag |= NRFUSBD_ISO_IN_READY;
	}

	if (dataFlag == s_Usbd.IsoDataFlag)
		return false;

	s_Usbd.IsoDataFlag = dataFlag;
	nRFUsbdResumeQueuedDmaLocked();
	return true;
}

void nRFUsbdIsoComplete(uint8_t In)
{
	const uint8_t flag = In ?
		NRFUSBD_ISO_IN_READY : NRFUSBD_ISO_OUT_READY;
	const uint16_t amount = (uint16_t)(In ?
		NRF_USBD->ISOIN.AMOUNT : NRF_USBD->ISOOUT.AMOUNT);

	s_Usbd.IsoDataFlag &= (uint8_t)~flag;
	if (!In)
		s_Usbd.pIsoBuffer[0] = nullptr;

#if NRFUSBD_ISO_TRACE
	if (In)
	{
		nRFUsbdIsoTraceCur()->EndUs = nRFUsbdIsoUsSinceSof();
		ISO_TRACE_FLAG(ISO_TRACE_IN_END);
	}
	else
	{
		nRFUsbdIsoTraceCur()->OutLen = amount;
		ISO_TRACE_FLAG(ISO_TRACE_OUT_END);
	}
#endif

	if (s_Usbd.IsoOpen)
	{
		nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO, In,
			USB_CTRLR_EVT_XFER_CMPL, amount);
	}
}

bool UsbCtrlrIsoInit(int DevNo)
{
	(void)DevNo;
#if NRFUSBD_ISO_TRACE
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
	return true;
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn,
	uint16_t MaxPacketSize)
{
	(void)DevNo;
	(void)EpNo;
	if (MaxPacketSize > NRFX_USBD_ISO_MAX_PACKET_SIZE)
		return false;
	s_Usbd.IsoMaxPacketSize[bIn] = MaxPacketSize;
	NRF_USBD->ISOSPLIT =
		USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
	NRF_USBD->ISOINCONFIG =
		USBD_ISOINCONFIG_RESPONSE_ZeroData << USBD_ISOINCONFIG_RESPONSE_Pos;

	nRFIsoHwEnable(bIn, true);

	s_Usbd.IsoOpen =
		s_Usbd.IsoMaxPacketSize[!bIn] != 0U;

	__DSB();
	return true;
}

void nRFUsbdIsoEpClose(bool bIn)
{
	// The base close path has stopped ISO scheduling and retired DMA with
	// interrupts excluded. Only direction-specific hardware/state remains.
	s_Usbd.IsoDataFlag = 0U;
	s_Usbd.pIsoBuffer[0] = nullptr;

	nRFIsoHwEnable(bIn, false);
	s_Usbd.IsoMaxPacketSize[bIn] = 0U;
}
