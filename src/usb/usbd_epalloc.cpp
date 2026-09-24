/**-------------------------------------------------------------------------
@file	usbd_epalloc.cpp

@brief	Internal USB device interface/endpoint allocator.

Recursive backtracking search over the endpoint space, lowest numbers
first, IN only endpoints placed before bidirectional pairs, pairs before
OUT only. Controller-constrained endpoints requested through the fixed
masks are excluded from the search and merged into the registered masks.
Each complete candidate is offered to the core; the first placement the
core accepts is returned to the caller.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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
#include <string.h>

#include "usb/usbd_epalloc.h"

typedef struct __Usbd_EpAlloc_State {
	int DevNo;
	const UsbdEpAllocReq_t *pReq;
	UsbDeviceClass *pClass;
	UsbdEpAllocRes_t *pRes;
	uint8_t FirstInterface;
	uint8_t InLimit;
	uint8_t OutLimit;
	uint8_t PairLimit;
} UsbdEpAllocState_t;

static void EpAllocStore(uint8_t *pEp, unsigned Count, uint32_t Mask)
{
	while (Count-- != 0U)
	{
		*pEp++ = (uint8_t)__builtin_ctz((unsigned)Mask);
		Mask &= Mask - 1U;
	}
}

enum
{
	EPALLOC_OUT,
	EPALLOC_PAIR,
	EPALLOC_IN,
};

static bool EpAllocTry(const UsbdEpAllocState_t *pState, unsigned Phase,
					   unsigned Needed, unsigned StartEp,
					   uint32_t InMask, uint32_t PairMask, uint32_t OutMask)
{
	if (Needed == 0U)
	{
		if (Phase != EPALLOC_OUT)
		{
			const unsigned next = Phase == EPALLOC_IN ?
				pState->pReq->BidirectionalCount : pState->pReq->OutCount;
			return EpAllocTry(pState, Phase - 1U, next, 1U,
				InMask, PairMask, OutMask);
		}

		if (!UsbClassRegister(pState->DevNo, pState->pClass,
			pState->FirstInterface, pState->pReq->InterfaceCount,
			InMask | PairMask, OutMask | PairMask))
		{
			return false;
		}

		pState->pRes->FirstInterface = pState->FirstInterface;
		EpAllocStore(pState->pRes->Bidirectional,
			pState->pReq->BidirectionalCount, PairMask);
		EpAllocStore(pState->pRes->In, pState->pReq->InCount,
			InMask & ~pState->pReq->FixedInMask);
		EpAllocStore(pState->pRes->Out, pState->pReq->OutCount,
			OutMask & ~pState->pReq->FixedOutMask);
		return true;
	}

	unsigned limit;
	uint32_t used;
	if (Phase == EPALLOC_IN)
	{
		limit = pState->InLimit;
		used = InMask;
	}
	else if (Phase == EPALLOC_PAIR)
	{
		limit = pState->PairLimit;
		used = InMask | OutMask;
	}
	else
	{
		limit = pState->OutLimit;
		used = PairMask | OutMask;
	}

	for (unsigned ep = StartEp; ep < limit; ep++)
	{
		const uint32_t bit = 1UL << ep;
		if ((used & bit) != 0U)
		{
			continue;
		}

		bool accepted;
		if (Phase == EPALLOC_IN)
		{
			accepted = EpAllocTry(pState, Phase, Needed - 1U, ep + 1U,
				InMask | bit, PairMask, OutMask);
		}
		else if (Phase == EPALLOC_PAIR)
		{
			accepted = EpAllocTry(pState, Phase, Needed - 1U, ep + 1U,
				InMask, PairMask | bit, OutMask);
		}
		else
		{
			accepted = EpAllocTry(pState, Phase, Needed - 1U, ep + 1U,
				InMask, PairMask, OutMask | bit);
		}
		if (accepted)
		{
			return true;
		}
	}

	return false;
}

bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 UsbDeviceClass *pClass, UsbdEpAllocRes_t *pRes)
{
	if (pReq == nullptr || pRes == nullptr ||
		pReq->InterfaceCount > 16U ||
		pReq->BidirectionalCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->InCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->OutCount > USBD_EPALLOC_EP_MAXCNT)
	{
		return false;
	}

	const uint8_t inLimit = USB_EPIN_CNT(DevNo) < 16 ?
		USB_EPIN_CNT(DevNo) : 16U;
	const uint8_t outLimit = USB_EPOUT_CNT(DevNo) < 16 ?
		USB_EPOUT_CNT(DevNo) : 16U;
	const uint8_t pairLimit = inLimit < outLimit ? inLimit : outLimit;
	if (inLimit < 1U || outLimit < 1U)
	{
		return false;
	}

	UsbdEpAllocState_t state = {};
	state.DevNo = DevNo;
	state.pReq = pReq;
	state.pClass = pClass;
	state.pRes = pRes;
	state.InLimit = inLimit;
	state.OutLimit = outLimit;
	state.PairLimit = pairLimit;

	const unsigned lastFirst = 16U - pReq->InterfaceCount;
	for (unsigned first = 0; first <= lastFirst; first++)
	{
		state.FirstInterface = (uint8_t)first;
		if (EpAllocTry(&state, EPALLOC_IN, pReq->InCount, 1U,
			pReq->FixedInMask, 0U, pReq->FixedOutMask))
		{
			return true;
		}
	}

	return false;
}
