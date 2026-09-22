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

static void EpAllocStore(uint8_t *pEp, unsigned Count, uint16_t Mask)
{
	while (Count-- != 0U)
	{
		*pEp++ = (uint8_t)__builtin_ctz((unsigned)Mask);
		Mask &= (uint16_t)(Mask - 1U);
	}
}

static bool EpAllocTryOut(const UsbdEpAllocState_t *pState, uint8_t Needed,
						  uint8_t StartEp, uint16_t InMask,
						  uint16_t PairMask, uint16_t OutMask)
{
	if (Needed == 0U)
	{
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
			(uint16_t)(InMask & ~pState->pReq->FixedInMask));
		EpAllocStore(pState->pRes->Out, pState->pReq->OutCount,
			(uint16_t)(OutMask & ~pState->pReq->FixedOutMask));
		return true;
	}

	for (uint8_t ep = StartEp; ep < pState->OutLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((PairMask & bit) != 0U ||
			(pState->pReq->FixedOutMask & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryOut(pState, (uint8_t)(Needed - 1U),
						  (uint8_t)(ep + 1U), InMask, PairMask,
						  OutMask | bit))
		{
			return true;
		}
	}

	return false;
}

static bool EpAllocTryPair(const UsbdEpAllocState_t *pState, uint8_t Needed,
						   uint8_t StartEp, uint16_t InMask,
						   uint16_t PairMask)
{
	if (Needed == 0U)
	{
		return EpAllocTryOut(pState, pState->pReq->OutCount, 1U,
							 InMask, PairMask, pState->pReq->FixedOutMask);
	}

	for (uint8_t ep = StartEp; ep < pState->PairLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if (((InMask | pState->pReq->FixedOutMask) & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryPair(pState, (uint8_t)(Needed - 1U),
						   (uint8_t)(ep + 1U), InMask, PairMask | bit))
		{
			return true;
		}
	}

	return false;
}

static bool EpAllocTryIn(const UsbdEpAllocState_t *pState, uint8_t Needed,
						 uint8_t StartEp, uint16_t InMask)
{
	if (Needed == 0U)
	{
		return EpAllocTryPair(pState, pState->pReq->BidirectionalCount,
							  1U, InMask, 0U);
	}

	for (uint8_t ep = StartEp; ep < pState->InLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);
		if ((InMask & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryIn(pState, (uint8_t)(Needed - 1U),
						 (uint8_t)(ep + 1U), InMask | bit))
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
		if (EpAllocTryIn(&state, pReq->InCount, 1U, pReq->FixedInMask))
		{
			return true;
		}
	}

	return false;
}
